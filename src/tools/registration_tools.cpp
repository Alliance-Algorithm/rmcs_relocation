/**
 * @file registration_tools.cpp
 * @brief 点云配准工具实现
 *
 * 提供初始重定位和丢失重定位两套 GICP 点云配准流程。
 * 核心为 MultiStageGicp （coarse → refine → precise），
 * 通过 yaw 角度搜索 + top-k 候选 + map consistency filter 提升鲁棒性。
 *
 * @author RMCS Development Team
 */

#include "tools/registration_tools.hpp"

#include "tools/numeric_tools.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <ranges>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <rclcpp/logging.hpp>

namespace rmcs::location::tools {

namespace {

using GicpRegistrator = small_gicp::RegistrationPCL<Point, Point>;
using PointCloudPtr = std::shared_ptr<PointCloud>;

struct StageParams {
    int coarse_iterations = 12;
    int refine_iterations = 8;
    int precise_iterations = 20;

    double yaw_window_deg = 15.0;
    double coarse_step_deg = 15.0;
    double refine_step_deg = 15.0;
    double refine_window_deg = 15.0;

    std::size_t coarse_top_k = 1;
    double coarse_score_threshold = std::numeric_limits<double>::infinity();
    double max_correspondence_distance = 0.5;
    double submap_radius_m = 3.5;
    double max_distance_from_prior_m = 3.0;

    static auto from_initial(const InitialRegistrationConfig& config) -> StageParams {
        auto stage = StageParams {};
        stage.coarse_iterations = sanitize_iterations(config.coarse_iterations, 12);
        stage.refine_iterations = sanitize_iterations(config.refine_iterations, 8);
        stage.precise_iterations = sanitize_iterations(config.precise_iterations, 20);

        stage.yaw_window_deg = sanitize_non_negative(config.yaw_search_window_deg, 0.0);
        stage.coarse_step_deg = sanitize_step(config.coarse_yaw_step_deg, 1.0);
        stage.refine_step_deg = sanitize_step(config.refine_yaw_step_deg, 1.0);
        stage.refine_window_deg = std::max(
            sanitize_step(config.coarse_yaw_step_deg, 15.0),
            sanitize_step(config.refine_yaw_step_deg, 15.0));

        stage.coarse_top_k = std::max<std::size_t>(1, config.coarse_top_k);
        stage.coarse_score_threshold = sanitize_non_negative(config.score_threshold, 0.04);
        stage.max_correspondence_distance =
            sanitize_non_negative(config.max_correspondence_distance_m, 0.5);
        return stage;
    }

    static auto from_lost(const LostRegistrationConfig& config, LostTier tier) -> StageParams {
        auto stage = StageParams {};

        const auto coarse_iterations = tier == LostTier::LOCAL ? config.coarse_iterations_local
                                                                : config.coarse_iterations_wide;
        const auto refine_iterations = tier == LostTier::LOCAL ? config.refine_iterations_local
                                                                : config.refine_iterations_wide;
        const auto precise_iterations = tier == LostTier::LOCAL ? config.precise_iterations_local
                                                                 : config.precise_iterations_wide;
        const auto yaw_window_deg = tier == LostTier::LOCAL ? config.local_yaw_window_deg
                                                             : config.wide_yaw_window_deg;
        const auto coarse_step_deg = tier == LostTier::LOCAL ? config.local_coarse_yaw_step_deg
                                                              : config.wide_coarse_yaw_step_deg;
        const auto submap_radius_m = tier == LostTier::LOCAL ? config.submap_radius_local_m
                                                              : config.submap_radius_wide_m;
        const auto max_distance_from_prior_m =
            tier == LostTier::LOCAL ? config.max_distance_from_prior_local_m
                                    : config.max_distance_from_prior_wide_m;
        const auto default_coarse_iterations = tier == LostTier::LOCAL ? 10 : 12;
        const auto default_refine_iterations = tier == LostTier::LOCAL ? 5 : 8;
        const auto default_precise_iterations = tier == LostTier::LOCAL ? 15 : 20;
        const auto default_coarse_step_deg = tier == LostTier::LOCAL ? 15.0 : 22.5;

        stage.coarse_iterations = sanitize_iterations(coarse_iterations, default_coarse_iterations);
        stage.refine_iterations = sanitize_iterations(refine_iterations, default_refine_iterations);
        stage.precise_iterations =
            sanitize_iterations(precise_iterations, default_precise_iterations);

        stage.yaw_window_deg = sanitize_non_negative(yaw_window_deg, 0.0);
        stage.coarse_step_deg = sanitize_step(coarse_step_deg, 1.0);
        stage.refine_step_deg = sanitize_step(config.refine_yaw_step_deg, 1.0);
        stage.refine_window_deg = std::max(
            sanitize_step(coarse_step_deg, default_coarse_step_deg),
            sanitize_step(config.refine_yaw_step_deg, 15.0));

        stage.coarse_top_k = std::max<std::size_t>(1, config.max_candidate_count);
        stage.coarse_score_threshold = tier == LostTier::LOCAL
            ? sanitize_non_negative(config.coarse_score_threshold_local, 0.3)
            : sanitize_non_negative(config.coarse_score_threshold_wide, 0.15);
        stage.max_correspondence_distance =
            sanitize_non_negative(config.max_correspondence_distance_m, 0.9);
        stage.submap_radius_m = submap_radius_m;
        stage.max_distance_from_prior_m = max_distance_from_prior_m;
        return stage;
    }
};

/// 单次 GICP 对齐结果：score + 变换矩阵
struct ScoredTransform {
    double score = std::numeric_limits<double>::infinity();
    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
};

/// 三阶段流水线最终产出：变换 + score + inlier 比例
struct PipelineResult {
    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
    double score = std::numeric_limits<double>::infinity();
    double inlier_ratio = 0.0;
};

/**
 * @brief 丢失重定位候选评分结构
 *
 * 包含配准 score、inlier 比例、先验距离等，通过 compute_ranking() 计算综合排名代价。
 * ranking_cost = score + rank_weight_inlier * (1 - inlier_ratio)
 *               + rank_weight_distance * prior_distance / max_distance_from_prior
 */
struct RankedCandidate {
    double score = std::numeric_limits<double>::infinity();
    double inlier_ratio = 0.0;
    double prior_distance_m = 0.0;
    double ranking_cost = std::numeric_limits<double>::infinity();
    Eigen::Isometry3f world_to_odom = Eigen::Isometry3f::Identity();

    /**
     * @brief 计算综合排名代价
     *
     * inlier 惩罚: 1 - inlier_ratio
     * 距离惩罚: min(1, prior_distance / max_distance_from_prior)（仅当先验存在）
     */
    void compute_ranking(
        const LostPrior& prior,
        const StageParams& stage,
        const LostRegistrationConfig& config) {
        const auto inlier_penalty = 1.0 - std::clamp(inlier_ratio, 0.0, 1.0);

        auto distance_penalty = 0.0;
        if (prior.has_prior) {
            const auto denom =
                std::max(1e-6, sanitize_non_negative(stage.max_distance_from_prior_m, 1.0));
            distance_penalty = std::min(1.0, prior_distance_m / denom);
        }

        ranking_cost = score + sanitize_non_negative(config.rank_weight_inlier, 0.5) * inlier_penalty
            + sanitize_non_negative(config.rank_weight_distance, 0.3) * distance_penalty;
    }

    //从 PipelineResult 构造 RankedCandidate 并计算排名
    static auto from_pipeline(
        const PipelineResult& pipeline_result,
        const LostPrior& prior,
        const StageParams& stage,
        const LostRegistrationConfig& config) -> RankedCandidate {
        auto candidate = RankedCandidate {};
        candidate.score = pipeline_result.score;
        candidate.inlier_ratio = pipeline_result.inlier_ratio;
        candidate.world_to_odom = pipeline_result.transform;

        if (prior.has_prior) {
            const auto world_to_base_estimated = candidate.world_to_odom * prior.odom_to_base;
            candidate.prior_distance_m =
                (world_to_base_estimated.translation() - prior.world_to_base.translation()).norm();
        }

        candidate.compute_ranking(prior, stage, config);
        return candidate;
    }
};

/**
 * @brief RAII 风格的 GICP 配准器封装
 *
 * 构造函数完成配置（类型/迭代次数/对应距离/epsilon），
 * try_align() 执行对齐并自动检查收敛性和 score 合法性。
 */
class GicpAligner {
    mutable GicpRegistrator registrator_;
    double max_correspondence_distance_m_ = 5.0;

public:
    GicpAligner(
        int iterations,
        double max_correspondence_distance_m,
        PointCloudPtr source,
        PointCloudPtr target,
        double epsilon = 1e-6)
        : max_correspondence_distance_m_(max_correspondence_distance_m) {
        // registrator_.setRegistrationType("GICP");
        // VGICP //
        registrator_.setRegistrationType("VGICP");
        registrator_.setVoxelResolution(0.2);
        registrator_.setMaximumIterations(iterations);
        registrator_.setMaxCorrespondenceDistance(max_correspondence_distance_m_);
        registrator_.setTransformationEpsilon(epsilon);
        registrator_.setEuclideanFitnessEpsilon(epsilon);
        registrator_.setInputSource(source);
        registrator_.setInputTarget(target);
    }

    //以给定初值执行 GICP 对齐，收敛且 score 有限时返回 ScoredTransform，否则返回 nullopt
    [[nodiscard]] auto try_align(const Eigen::Isometry3f& guess) const
        -> std::optional<ScoredTransform> {
        auto aligned = PointCloud {};
        registrator_.align(aligned, guess.matrix());
        if (!registrator_.hasConverged())
            return std::nullopt;

        const auto score = registrator_.getFitnessScore(max_correspondence_distance_m_);
        if (!std::isfinite(score))
            return std::nullopt;

        return ScoredTransform {
            .score = score,
            .transform = Eigen::Isometry3f { registrator_.getFinalTransformation() },
        };
    }

    //获取最近一次对齐的 inlier 数量
    [[nodiscard]] auto num_inliers() const -> std::size_t {
        const auto result = registrator_.getRegistrationResult();
        if (result.num_inliers <= 0)
            return 0;
        return static_cast<std::size_t>(result.num_inliers);
    }
};

/**
 * @brief 从 world→base 和 odom→base 推导 world→odom 变换
 *
 * world_to_odom = world_to_base * odom_to_base⁻¹
 */
auto world_to_odom_from_world_to_base(
    const Eigen::Isometry3f& world_to_base,
    const Eigen::Isometry3f& odom_to_base) -> Eigen::Isometry3f {
    return world_to_base * odom_to_base.inverse();
}

/**
 * @brief 生成对称 yaw 偏移候选位姿
 *
 * 以 base_pose 为中心，在 [0, ±step, ±2*step, ..., ±window] 范围内生成绕 Z 轴旋转的候选位姿。
 * 顺序: 0°, +step°, -step°, +2*step°, -2*step°, ... 便于 coarse 阶段尽早遇到低 score 并 early break。
 */
[[nodiscard]] auto generate_yaw_candidates(
    const Eigen::Isometry3f& base_pose, double window_deg, double step_deg)
    -> std::vector<Eigen::Isometry3f> {
    auto offsets = std::vector<double> { 0.0 };
    for (double delta = step_deg; delta <= (window_deg + 1e-6); delta += step_deg) {
        offsets.push_back(delta);
        offsets.push_back(-delta);
    }

    auto candidates = std::vector<Eigen::Isometry3f> {};
    candidates.reserve(offsets.size());

    for (const auto yaw_delta_deg : offsets) {
        const auto yaw_delta_radian = static_cast<float>(yaw_delta_deg * std::numbers::pi / 180.0);
        const auto yaw_rotation = Eigen::AngleAxisf { yaw_delta_radian, Eigen::Vector3f::UnitZ() };

        auto candidate = base_pose;
        candidate.linear() = (yaw_rotation * Eigen::Quaternionf { base_pose.rotation() })
                                 .normalized()
                                 .toRotationMatrix();
        candidates.push_back(candidate);
    }

    return candidates;
}

/**
 * @brief 点云预处理：体素降采样 + 可选离群点去除
 *
 * 1. VoxelGrid 降采样（leaf = voxel_leaf_m）
 * 2. 若 with_outlier=true 且点数足够，StatisticalOutlierRemoval 去除离群点
 */
auto preprocess_cloud(
    const PointCloudPtr& cloud,
    const InitialRegistrationConfig& initial_config,
    bool with_outlier) -> PointCloudPtr {
    auto downsampled = std::make_shared<PointCloud>();
    if (!cloud || cloud->empty())
        return downsampled;

    const auto leaf =
        static_cast<float>(std::max(1e-3, sanitize_non_negative(initial_config.voxel_leaf_m, 0.2)));
    auto voxel = pcl::VoxelGrid<Point> {};
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*downsampled);

    const auto mean_k = std::max(1, initial_config.outlier_mean_k);
    if (!with_outlier || downsampled->size() <= static_cast<std::size_t>(mean_k))
        return downsampled;

    auto filtered = std::make_shared<PointCloud>();
    auto outlier = pcl::StatisticalOutlierRemoval<Point> {};
    outlier.setInputCloud(downsampled);
    outlier.setMeanK(mean_k);
    outlier.setStddevMulThresh(sanitize_non_negative(initial_config.outlier_stddev_mul_thresh, 0.5));
    outlier.filter(*filtered);
    return filtered;
}

/**
 * @brief 地图一致性过滤
 *
 * 将 query 点云按 world_to_odom_guess 变换到世界系，剔除与地图点云最邻近距离超出阈值的点。
 * 若保留比例 < min_retained_fraction 则回退到原始点云。
 */
auto apply_map_consistency_filter(
    const PointCloudPtr& query_odom_cloud,
    const PointCloudPtr& map_world_cloud,
    const Eigen::Isometry3f& world_to_odom_guess,
    double map_consistency_distance_m,
    double min_retained_fraction) -> PointCloudPtr {
    if (!query_odom_cloud || query_odom_cloud->empty() || !map_world_cloud || map_world_cloud->empty())
        return query_odom_cloud;

    auto transformed_query_world = std::make_shared<PointCloud>();
    pcl::transformPointCloud(*query_odom_cloud, *transformed_query_world, world_to_odom_guess.matrix());

    auto kdtree = pcl::KdTreeFLANN<Point> {};
    kdtree.setInputCloud(map_world_cloud);

    const auto max_distance = sanitize_non_negative(map_consistency_distance_m, 0.8);
    const auto max_distance_sq = static_cast<float>(max_distance * max_distance);

    auto filtered = std::make_shared<PointCloud>();
    filtered->reserve(query_odom_cloud->size());

    std::vector<int> indices(1);
    std::vector<float> distance_sq(1);
    for (std::size_t i = 0; i < transformed_query_world->size(); ++i) {
        if (kdtree.nearestKSearch(transformed_query_world->points[i], 1, indices, distance_sq) <= 0)
            continue;

        if (distance_sq[0] <= max_distance_sq)
            filtered->points.push_back(query_odom_cloud->points[i]);
    }

    const auto retained_fraction = static_cast<double>(filtered->size())
        / static_cast<double>(std::max<std::size_t>(1, query_odom_cloud->size()));
    if (retained_fraction < sanitize_non_negative(min_retained_fraction, 0.15))
        return query_odom_cloud;

    filtered->width = static_cast<std::uint32_t>(filtered->size());
    filtered->height = 1;
    filtered->is_dense = query_odom_cloud->is_dense;
    return filtered;
}

/**
 * @brief 根据先验不确定性选择丢失配准档位
 *
 * prior.sigma_xy_m ≤ local_sigma_xy_m 且 prior.sigma_yaw_deg ≤ local_sigma_yaw_deg → LOCAL，否则 → WIDE。
 */
[[nodiscard]] auto select_lost_tier(const LostPrior& prior, const LostRegistrationConfig& config)
    -> LostTier {
    const auto prior_sigma_xy_m = sanitize_non_negative(prior.sigma_xy_m, 0.0);
    const auto prior_sigma_yaw_deg = sanitize_non_negative(prior.sigma_yaw_deg, 0.0);
    if (prior_sigma_xy_m <= config.local_sigma_xy_m && prior_sigma_yaw_deg <= config.local_sigma_yaw_deg)
        return LostTier::LOCAL;
    return LostTier::WIDE;
}

/**
 * @brief 为 WIDE 模式生成多种子候选
 *
 * 以 prior.world_to_base 为中心，在 XY 四方向各偏移 offset。
 * offset = clamp(2 * max(0.5, sigma_xy_m), 0.5, submap_radius)。
 * 共生成 5 个种子：原位置 + 4 个偏移方向。
 */
[[nodiscard]] auto create_wide_seeds(const LostPrior& prior, double submap_radius_m)
    -> std::vector<Eigen::Isometry3f> {
    auto seeds = std::vector<Eigen::Isometry3f> { prior.world_to_base };

    const auto raw_offset = 2.0 * std::max(0.5, sanitize_non_negative(prior.sigma_xy_m, 0.5));
    const auto offset = std::clamp(raw_offset, 0.5, std::max(0.5, submap_radius_m));
    const auto deltas = std::vector<Eigen::Vector3f> {
        { static_cast<float>(offset), 0.0F, 0.0F },
        { static_cast<float>(-offset), 0.0F, 0.0F },
        { 0.0F, static_cast<float>(offset), 0.0F },
        { 0.0F, static_cast<float>(-offset), 0.0F },
    };

    for (const auto& delta : deltas) {
        auto seed = prior.world_to_base;
        seed.translation() += delta;
        seeds.push_back(seed);
    }

    return seeds;
}

/**
 * @brief 三阶段 GICP 配准流水线
 *
 * 封装 coarse → refine → precise 的顺序执行：
 * - coarse: yaw 窗口内大步长搜索，top-k 截断，遇低 score 可 early break
 * - refine: 对 top-k 结果在小窗口内细搜索，取全局最优
 * - precise: 对最优结果做高精度收敛（epsilon=1e-5），计算 inlier 比例
 *
 * @param require_refine_convergence  refine 失败时是否直接放弃流水线；
 *                                    false 则 fallback 到 coarse best（用于 run_initial）；
 *                                    true 则返回 nullopt（用于 run_lost 单种子评估）
 */
class MultiStageGicp {
    StageParams params_;
    GicpAligner coarse_;
    GicpAligner refine_;
    GicpAligner precise_;
    std::size_t source_size_ = 0;

    /**
     * @brief coarse 阶段：yaw 大窗口扫描 + early break + top-k 截断
     *
     * 对初始位姿在 [0, ±coarse_step, ...] 范围内做 yaw 搜索，收集收敛候选。
     * 若某候选 score ≤ coarse_score_threshold 则提前退出扫描。
     * 结果按 score 升序排序并截断到 coarse_top_k。
     */
    [[nodiscard]] auto run_coarse(const Eigen::Isometry3f& guess)
        -> std::optional<std::vector<ScoredTransform>> {
        auto coarse_results = std::vector<ScoredTransform> {};
        for (const auto& candidate_guess :
             generate_yaw_candidates(guess, params_.yaw_window_deg, params_.coarse_step_deg)) {
            if (auto candidate = coarse_.try_align(candidate_guess); candidate.has_value()) {
                coarse_results.push_back(candidate.value());
                if (candidate->score <= params_.coarse_score_threshold)
                    break;
            }
        }

        if (coarse_results.empty())
            return std::nullopt;

        std::ranges::sort(coarse_results, {}, &ScoredTransform::score);
        if (coarse_results.size() > params_.coarse_top_k)
            coarse_results.resize(params_.coarse_top_k);

        return coarse_results;
    }

    /**
     * @brief refine 阶段：对 top-k 候选在小窗口内做细粒度搜索
     *
     * 对每个 coarse 候选在 [0, ±refine_step, ..., refine_window] 做 yaw 搜索，跟踪全局最优。
     * 返回最优 ScoredTransform 或 nullopt。
     */
    [[nodiscard]] auto run_refine(const std::vector<ScoredTransform>& coarse_results)
        -> std::optional<ScoredTransform> {
        auto best_refine = std::optional<ScoredTransform> {};

        for (const auto& coarse_result : coarse_results) {
            for (const auto& candidate_guess : generate_yaw_candidates(
                     coarse_result.transform, params_.refine_window_deg, params_.refine_step_deg)) {
                if (auto candidate = refine_.try_align(candidate_guess); candidate.has_value()) {
                    if (!best_refine || candidate->score < best_refine->score)
                        best_refine = candidate;
                }
            }
        }

        return best_refine;
    }

    /**
     * @brief precise 阶段：严格 epsilon 下的最终精配
     *
     * 使用 GICP (epsilon=1e-5) 做高精度收敛，从 getRegistrationResult() 计算 inlier_ratio。
     */
    [[nodiscard]] auto run_precise(const Eigen::Isometry3f& guess) -> std::optional<PipelineResult> {
        auto precise_result = precise_.try_align(guess);
        if (!precise_result)
            return std::nullopt;

        const auto query_size = std::max<std::size_t>(1, source_size_);
        const auto inlier_ratio = std::clamp(
            static_cast<double>(precise_.num_inliers()) / static_cast<double>(query_size), 0.0, 1.0);

        return PipelineResult {
            .transform = precise_result->transform,
            .score = precise_result->score,
            .inlier_ratio = inlier_ratio,
        };
    }

public:
    MultiStageGicp(const StageParams& params, PointCloudPtr source, PointCloudPtr target)
        : params_(params),
          coarse_(params_.coarse_iterations, params_.max_correspondence_distance, source, target),
          refine_(params_.refine_iterations, params_.max_correspondence_distance, source, target),
          precise_(params_.precise_iterations, params_.max_correspondence_distance, source, target, 1e-5),
          source_size_(source ? source->size() : 0) {}

    /**
     * @brief 执行完整三阶段流水线
     *
     * @param guess 初始 guess 变换
     * @param require_refine_convergence refine 失败时是否直接放弃；
     *                                    false 则 fallback 到 coarse best
     * @return 配准结果或 nullopt
     */
    [[nodiscard]] auto run(const Eigen::Isometry3f& guess, bool require_refine_convergence)
        -> std::optional<PipelineResult> {
        auto coarse_results = run_coarse(guess);
        if (!coarse_results)
            return std::nullopt;

        auto refined_result = run_refine(coarse_results.value());
        if (!refined_result) {
            if (require_refine_convergence)
                return std::nullopt;
            refined_result = coarse_results->front();
        }

        return run_precise(refined_result->transform);
    }
};

/**
 * @brief 对单个丢失重定位种子执行三阶段配准
 *
 * 调用 MultiStageGicp (require_refine=true)，从 PipelineResult 构造 RankedCandidate。
 */
auto run_lost_seed(
    const PointCloudPtr& filtered_query,
    const PointCloudPtr& filtered_map,
    const Eigen::Isometry3f& seed_world_to_odom,
    const LostPrior& prior,
    const LostRegistrationConfig& config,
    const StageParams& stage) -> std::optional<RankedCandidate> {
    auto pipeline = MultiStageGicp { stage, filtered_query, filtered_map };
    auto pipeline_result = pipeline.run(seed_world_to_odom, true);
    if (!pipeline_result)
        return std::nullopt;

    return RankedCandidate::from_pipeline(pipeline_result.value(), prior, stage, config);
}

} // namespace


auto from_ros_pointcloud(const sensor_msgs::msg::PointCloud2& message, PointCloud& cloud) -> bool {
    pcl::fromROSMsg(message, cloud);
    return !cloud.empty();
}

auto transform_pointcloud(
    const PointCloud& source, const Eigen::Isometry3f& transform, PointCloud& target) -> bool {
    pcl::transformPointCloud(source, target, transform.matrix());
    return !target.empty();
}

auto extract_submap_radius(
    const pcl::KdTreeFLANN<Point>& kdtree,
    const std::shared_ptr<PointCloud>& map_world_cloud,
    const Eigen::Vector3f& center,
    double radius_m,
    double fallback_radius_m) -> std::shared_ptr<PointCloud> {
    auto submap = std::make_shared<PointCloud>();
    if (!map_world_cloud || map_world_cloud->empty())
        return submap;

    auto center_point = Point {};
    center_point.x = center.x();
    center_point.y = center.y();
    center_point.z = center.z();

    std::vector<int> indices;
    std::vector<float> distances;
    kdtree.radiusSearch(
        center_point,
        static_cast<float>(std::max(0.1, sanitize_non_negative(radius_m, fallback_radius_m))),
        indices,
        distances);

    submap->reserve(indices.size());
    for (const auto index : indices)
        submap->points.push_back(map_world_cloud->points[static_cast<std::size_t>(index)]);
    submap->width = static_cast<std::uint32_t>(submap->size());
    submap->height = 1;
    submap->is_dense = map_world_cloud->is_dense;
    return submap;
}

auto extract_submap_radius(
    const std::shared_ptr<PointCloud>& map_world_cloud,
    const Eigen::Vector3f& center,
    double radius_m,
    double fallback_radius_m) -> std::shared_ptr<PointCloud> {
    if (!map_world_cloud || map_world_cloud->empty())
        return std::make_shared<PointCloud>();

    auto kdtree = pcl::KdTreeFLANN<Point> {};
    kdtree.setInputCloud(map_world_cloud);
    return extract_submap_radius(kdtree, map_world_cloud, center, radius_m, fallback_radius_m);
}

/**
 * @brief 初始重定位（MODE_INITIAL）
 *
 * 将 query odom 点云与 global map 直接配准，输出 world→odom 变换。
 * 流程：预处理 → MultiStageGicp (require_refine=false) → 输出结果。
 */
auto run_initial(
    const InitialRegistrationConfig& initial_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_world_cloud,
    const Eigen::Isometry3f& world_to_odom_guess,
    Eigen::Isometry3f& world_to_odom_result,
    double& score) -> bool {
    auto filtered_query = preprocess_cloud(query_odom_cloud, initial_config, true);
    auto filtered_map = preprocess_cloud(map_world_cloud, initial_config, true);
    if (filtered_query->empty() || filtered_map->empty())
        return false;

    auto pipeline = MultiStageGicp { StageParams::from_initial(initial_config), filtered_query, filtered_map };
    auto pipeline_result = pipeline.run(world_to_odom_guess, false);
    if (!pipeline_result)
        return false;

    world_to_odom_result = pipeline_result->transform;
    score = pipeline_result->score;
    return std::isfinite(score);
}

/**
 * @brief 丢失重定位（MODE_LOST）
 *
 * 根据先验不确定性选择 LOCAL / WIDE 档位，生成多种子候选，
 * 对每个种子提取子地图 → 过滤 → 三阶段配准 → 按 ranking_cost 排序取最优。
 *
 * 流程:
 * 1. 预处理 query 点云
 * 2. select_lost_tier: 根据 sigma 选择 LOCAL / WIDE
 * 3. StageParams::from_lost: 加载对应档位参数
 * 4. create_wide_seeds (仅 WIDE): 生成多方向种子
 * 5. 对每个种子: extract_submap → preprocess → (可选 map_consistency_filter) → run_lost_seed
 * 6. 按 ranking_cost 排序，取最优
 */
auto run_lost(
    const InitialRegistrationConfig& initial_config,
    const LostRegistrationConfig& lost_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_world_cloud,
    const pcl::KdTreeFLANN<Point>& map_kdtree,
    const LostPrior& prior,
    LostResult& result) -> bool {
    result = LostResult {};

    if (!prior.has_prior)
        return false;

    auto filtered_query = preprocess_cloud(query_odom_cloud, initial_config, true);
    if (!filtered_query || filtered_query->empty())
        return false;

    const auto tier = select_lost_tier(prior, lost_config);
    const auto stage = StageParams::from_lost(lost_config, tier);
    auto seeds_world_to_base = tier == LostTier::WIDE ? create_wide_seeds(prior, stage.submap_radius_m)
                                                        : std::vector<Eigen::Isometry3f> { prior.world_to_base };

    auto candidates = std::vector<RankedCandidate> {};
    candidates.reserve(seeds_world_to_base.size());
    const auto logger = rclcpp::get_logger("rmcs_relocation");

    for (const auto& seed_world_to_base : seeds_world_to_base) {
        const auto submap_radius_fallback = tier == LostTier::LOCAL ? 3.5 : 5.0;
        auto map_submap =
            extract_submap_radius(
                map_kdtree,
                map_world_cloud,
                seed_world_to_base.translation(),
                stage.submap_radius_m,
                submap_radius_fallback);
        if (!map_submap || map_submap->empty()) {
            RCLCPP_WARN(
                logger,
                "lost seed submap empty around (%.3f, %.3f, %.3f), skip",
                seed_world_to_base.translation().x(),
                seed_world_to_base.translation().y(),
                seed_world_to_base.translation().z());
            continue;
        }

        auto filtered_map = preprocess_cloud(map_submap, initial_config, false);
        if (!filtered_map || filtered_map->empty())
            continue;

        auto seed_filtered_query = filtered_query;
        if (lost_config.enable_map_consistency_filter) {
            const auto world_to_odom_prior =
                world_to_odom_from_world_to_base(seed_world_to_base, prior.odom_to_base);
            seed_filtered_query = apply_map_consistency_filter(
                filtered_query,
                filtered_map,
                world_to_odom_prior,
                lost_config.map_consistency_distance_m,
                lost_config.min_retained_fraction);
            if (!seed_filtered_query || seed_filtered_query->empty())
                continue;
        }

        const auto seed_world_to_odom =
            world_to_odom_from_world_to_base(seed_world_to_base, prior.odom_to_base);
        auto candidate =
            run_lost_seed(seed_filtered_query, filtered_map, seed_world_to_odom, prior, lost_config, stage);
        if (!candidate.has_value())
            continue;

        candidates.push_back(candidate.value());
    }

    if (candidates.empty())
        return false;

    std::ranges::sort(candidates, {}, &RankedCandidate::ranking_cost);
    result = LostResult {
        .world_to_odom = candidates.front().world_to_odom,
        .score = candidates.front().score,
        .inlier_ratio = candidates.front().inlier_ratio,
        .tier_used = tier,
    };

    return std::isfinite(result.score);
}

} // namespace rmcs::location::tools
