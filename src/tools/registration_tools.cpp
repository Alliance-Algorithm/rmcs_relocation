/**
 * @file registration_tools.cpp
 * @brief 点云配准工具实现
 *
 * 提供 INITIAL / LOCAL / WIDE 三个独立配准入口。共用 MultiStageGicp（coarse → refine → precise）
 * 与 yaw 搜索 + top-k 候选 + 可选 map consistency filter。
 */

#include "tools/registration_tools.hpp"

#include "server/validator.hpp"
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

/// refine 窗口取 coarse / refine yaw step 的较大值，加大默认下限保证窗口非零
auto refine_window(double coarse_step_deg, double refine_step_deg, double default_floor) -> double {
    return std::max(
        sanitize_step(coarse_step_deg, default_floor), sanitize_step(refine_step_deg, 15.0));
}

/// 多阶段 GICP 的统一参数包（INITIAL / LOCAL / WIDE 三处复用）
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

    static auto from_initial(const InitialRegistrationConfig& c) -> StageParams {
        return StageParams{
            .coarse_iterations           = sanitize_iterations(c.coarse_iterations, 12),
            .refine_iterations           = sanitize_iterations(c.refine_iterations, 8),
            .precise_iterations          = sanitize_iterations(c.precise_iterations, 20),
            .yaw_window_deg              = sanitize_non_negative(c.yaw_search_window_deg, 0.0),
            .coarse_step_deg             = sanitize_step(c.coarse_yaw_step_deg, 1.0),
            .refine_step_deg             = sanitize_step(c.refine_yaw_step_deg, 1.0),
            .refine_window_deg           = refine_window(c.coarse_yaw_step_deg, c.refine_yaw_step_deg, 15.0),
            .coarse_top_k                = std::max<std::size_t>(1, c.coarse_top_k),
            .coarse_score_threshold      = sanitize_non_negative(c.score_threshold, 0.04),
            .max_correspondence_distance = sanitize_non_negative(c.max_correspondence_distance_m, 0.5),
        };
    }

    static auto from_local(const LocalRegistrationConfig& c) -> StageParams {
        return StageParams{
            .coarse_iterations           = sanitize_iterations(c.coarse_iterations, 10),
            .refine_iterations           = sanitize_iterations(c.refine_iterations, 5),
            .precise_iterations          = sanitize_iterations(c.precise_iterations, 15),
            .yaw_window_deg              = sanitize_non_negative(c.yaw_window_deg, 0.0),
            .coarse_step_deg             = sanitize_step(c.coarse_yaw_step_deg, 1.0),
            .refine_step_deg             = sanitize_step(c.refine_yaw_step_deg, 1.0),
            .refine_window_deg           = refine_window(c.coarse_yaw_step_deg, c.refine_yaw_step_deg, 15.0),
            .coarse_top_k                = 1,
            .coarse_score_threshold      = sanitize_non_negative(c.coarse_score_threshold, 0.3),
            .max_correspondence_distance = sanitize_non_negative(c.max_correspondence_distance_m, 0.9),
            .submap_radius_m             = sanitize_non_negative(c.submap_radius_m, 3.5),
        };
    }

    static auto from_wide(const WideRegistrationConfig& c) -> StageParams {
        return StageParams{
            .coarse_iterations           = sanitize_iterations(c.coarse_iterations, 12),
            .refine_iterations           = sanitize_iterations(c.refine_iterations, 8),
            .precise_iterations          = sanitize_iterations(c.precise_iterations, 20),
            .yaw_window_deg              = sanitize_non_negative(c.yaw_window_deg, 0.0),
            .coarse_step_deg             = sanitize_step(c.coarse_yaw_step_deg, 1.0),
            .refine_step_deg             = sanitize_step(c.refine_yaw_step_deg, 1.0),
            .refine_window_deg           = refine_window(c.coarse_yaw_step_deg, c.refine_yaw_step_deg, 22.5),
            .coarse_top_k                = std::max<std::size_t>(1, c.max_candidate_count),
            .coarse_score_threshold      = sanitize_non_negative(c.coarse_score_threshold, 0.15),
            .max_correspondence_distance = sanitize_non_negative(c.max_correspondence_distance_m, 0.9),
            .submap_radius_m             = sanitize_non_negative(c.submap_radius_m, 5.0),
            .max_distance_from_prior_m   = sanitize_non_negative(c.max_distance_from_prior_m, 10.0),
        };
    }
};

struct ScoredTransform {
    double score = std::numeric_limits<double>::infinity();
    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
};

struct PipelineResult {
    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
    double score = std::numeric_limits<double>::infinity();
    double inlier_ratio = 0.0;
};

/**
 * @brief WIDE 多 seed 候选评分
 *
 * ranking_cost = score + w_inlier * (1 - inlier_ratio) + w_distance * normalized_prior_distance
 */
struct RankedCandidate {
    double score = std::numeric_limits<double>::infinity();
    double inlier_ratio = 0.0;
    double prior_distance_m = 0.0;
    double ranking_cost = std::numeric_limits<double>::infinity();
    Eigen::Isometry3f world_to_odom = Eigen::Isometry3f::Identity();

    void compute_ranking(
        const RegistrationPrior& prior, const StageParams& stage,
        const WideRegistrationConfig& config) {
        const auto inlier_penalty = 1.0 - std::clamp(inlier_ratio, 0.0, 1.0);

        auto distance_penalty = 0.0;
        if (prior.has_prior) {
            const auto denom =
                std::max(1e-6, sanitize_non_negative(stage.max_distance_from_prior_m, 1.0));
            distance_penalty = std::min(1.0, prior_distance_m / denom);
        }

        ranking_cost = score
            + sanitize_non_negative(config.rank_weight_inlier, 0.5) * inlier_penalty
            + sanitize_non_negative(config.rank_weight_distance, 0.3) * distance_penalty;
    }

    static auto from_pipeline(
        const PipelineResult& pipeline_result, const RegistrationPrior& prior,
        const StageParams& stage, const WideRegistrationConfig& config) -> RankedCandidate {
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

class GicpAligner {
    mutable GicpRegistrator registrator_;
    double max_correspondence_distance_m_ = 5.0;

public:
    GicpAligner(
        int iterations, double max_correspondence_distance_m, PointCloudPtr source,
        PointCloudPtr target, double epsilon = 1e-6)
        : max_correspondence_distance_m_(max_correspondence_distance_m) {
        registrator_.setRegistrationType("VGICP");
        registrator_.setVoxelResolution(0.2);
        registrator_.setMaximumIterations(iterations);
        registrator_.setMaxCorrespondenceDistance(max_correspondence_distance_m_);
        registrator_.setTransformationEpsilon(epsilon);
        registrator_.setEuclideanFitnessEpsilon(epsilon);
        registrator_.setInputSource(source);
        registrator_.setInputTarget(target);
    }

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

    [[nodiscard]] auto num_inliers() const -> std::size_t {
        const auto result = registrator_.getRegistrationResult();
        if (result.num_inliers <= 0)
            return 0;
        return static_cast<std::size_t>(result.num_inliers);
    }
};

auto world_to_odom_from_world_to_base(
    const Eigen::Isometry3f& world_to_base, const Eigen::Isometry3f& odom_to_base)
    -> Eigen::Isometry3f {
    return world_to_base * odom_to_base.inverse();
}

/**
 * @brief 在给定 base_pose 周围生成对称 yaw 偏移候选，顺序 0, +step, -step, +2step, -2step, ...
 *        便于 coarse 阶段尽早遇低 score 时早退。
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

auto preprocess_cloud(
    const PointCloudPtr& cloud, const InitialRegistrationConfig& initial_config, bool with_outlier)
    -> PointCloudPtr {
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

/// 三个 run_* 入口共享：voxel + outlier 滤波 → 空检查。空时返回 nullptr，调用方应直接 false。
auto prepare_filtered_query(
    const PointCloudPtr& query_odom_cloud, const InitialRegistrationConfig& initial_config)
    -> PointCloudPtr {
    auto filtered = preprocess_cloud(query_odom_cloud, initial_config, true);
    if (!filtered || filtered->empty())
        return nullptr;
    return filtered;
}

auto apply_map_consistency_filter(
    const PointCloudPtr& query_odom_cloud, const PointCloudPtr& map_world_cloud,
    const Eigen::Isometry3f& world_to_odom_guess, double map_consistency_distance_m,
    double min_retained_fraction) -> PointCloudPtr {
    if (!query_odom_cloud || query_odom_cloud->empty() || !map_world_cloud
        || map_world_cloud->empty())
        return query_odom_cloud;

    auto transformed_query_world = std::make_shared<PointCloud>();
    pcl::transformPointCloud(
        *query_odom_cloud, *transformed_query_world, world_to_odom_guess.matrix());

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

class MultiStageGicp {
    StageParams params_;
    GicpAligner coarse_;
    GicpAligner refine_;
    GicpAligner precise_;
    std::size_t source_size_ = 0;

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
        : params_(params)
        , coarse_(params_.coarse_iterations, params_.max_correspondence_distance, source, target)
        , refine_(params_.refine_iterations, params_.max_correspondence_distance, source, target)
        , precise_(
              params_.precise_iterations, params_.max_correspondence_distance, source, target, 1e-5)
        , source_size_(source ? source->size() : 0) {}

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
 * @brief 单 seed 配准：extract_submap → preprocess → optional consistency filter → MultiStageGicp
 *        LOCAL 用 LocalRegistrationConfig 的 enable_map_consistency_filter / map_consistency_distance_m,
 *        WIDE 用 WideRegistrationConfig 的对应字段；通过两个 bool/double 参数显式传入解耦。
 */
auto run_seed_pipeline(
    const PointCloudPtr& filtered_query, const PointCloudPtr& map_world_cloud,
    const pcl::KdTreeFLANN<Point>& map_kdtree, const Eigen::Isometry3f& seed_world_to_base,
    const RegistrationPrior& prior, const InitialRegistrationConfig& initial_config,
    const StageParams& stage, bool enable_consistency_filter, double consistency_distance_m,
    double min_retained_fraction) -> std::optional<PipelineResult> {
    constexpr auto submap_radius_fallback = 5.0;
    auto map_submap = extract_submap_radius(
        map_kdtree, map_world_cloud, seed_world_to_base.translation(), stage.submap_radius_m,
        submap_radius_fallback);
    if (!map_submap || map_submap->empty()) {
        const auto logger = rclcpp::get_logger("rmcs_relocation");
        RCLCPP_WARN(
            logger, "seed submap empty around (%.3f, %.3f, %.3f), skip",
            seed_world_to_base.translation().x(), seed_world_to_base.translation().y(),
            seed_world_to_base.translation().z());
        return std::nullopt;
    }

    auto filtered_map = preprocess_cloud(map_submap, initial_config, false);
    if (!filtered_map || filtered_map->empty())
        return std::nullopt;

    auto seed_filtered_query = filtered_query;
    if (enable_consistency_filter) {
        const auto world_to_odom_prior =
            world_to_odom_from_world_to_base(seed_world_to_base, prior.odom_to_base);
        seed_filtered_query = apply_map_consistency_filter(
            filtered_query, filtered_map, world_to_odom_prior, consistency_distance_m,
            min_retained_fraction);
        if (!seed_filtered_query || seed_filtered_query->empty())
            return std::nullopt;
    }

    const auto seed_world_to_odom =
        world_to_odom_from_world_to_base(seed_world_to_base, prior.odom_to_base);
    auto pipeline = MultiStageGicp { stage, seed_filtered_query, filtered_map };
    return pipeline.run(seed_world_to_odom, true);
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
    const pcl::KdTreeFLANN<Point>& kdtree, const std::shared_ptr<PointCloud>& map_world_cloud,
    const Eigen::Vector3f& center, double radius_m, double fallback_radius_m)
    -> std::shared_ptr<PointCloud> {
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
        indices, distances);

    submap->reserve(indices.size());
    for (const auto index : indices)
        submap->points.push_back(map_world_cloud->points[static_cast<std::size_t>(index)]);
    submap->width = static_cast<std::uint32_t>(submap->size());
    submap->height = 1;
    submap->is_dense = map_world_cloud->is_dense;
    return submap;
}

auto extract_submap_radius(
    const std::shared_ptr<PointCloud>& map_world_cloud, const Eigen::Vector3f& center,
    double radius_m, double fallback_radius_m) -> std::shared_ptr<PointCloud> {
    if (!map_world_cloud || map_world_cloud->empty())
        return std::make_shared<PointCloud>();

    auto kdtree = pcl::KdTreeFLANN<Point> {};
    kdtree.setInputCloud(map_world_cloud);
    return extract_submap_radius(kdtree, map_world_cloud, center, radius_m, fallback_radius_m);
}

auto run_initial(
    const InitialRegistrationConfig& initial_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_world_cloud,
    const Eigen::Isometry3f& world_to_odom_guess, Eigen::Isometry3f& world_to_odom_result,
    double& score) -> bool {
    auto filtered_query = prepare_filtered_query(query_odom_cloud, initial_config);
    auto filtered_map = preprocess_cloud(map_world_cloud, initial_config, true);
    if (!filtered_query || !filtered_map || filtered_map->empty())
        return false;

    auto pipeline =
        MultiStageGicp { StageParams::from_initial(initial_config), filtered_query, filtered_map };
    auto pipeline_result = pipeline.run(world_to_odom_guess, false);
    if (!pipeline_result)
        return false;

    world_to_odom_result = pipeline_result->transform;
    score = pipeline_result->score;
    return std::isfinite(score);
}

auto run_local(
    const InitialRegistrationConfig& initial_config, const LocalRegistrationConfig& local_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_world_cloud, const pcl::KdTreeFLANN<Point>& map_kdtree,
    const RegistrationPrior& prior, RegistrationResult& result) -> bool {
    result = RegistrationResult {};
    if (!prior.has_prior)
        return false;

    auto filtered_query = prepare_filtered_query(query_odom_cloud, initial_config);
    if (!filtered_query)
        return false;

    const auto stage = StageParams::from_local(local_config);
    auto pipeline_result = run_seed_pipeline(
        filtered_query, map_world_cloud, map_kdtree, prior.world_to_base, prior, initial_config,
        stage, local_config.enable_map_consistency_filter, local_config.map_consistency_distance_m,
        local_config.min_retained_fraction);
    if (!pipeline_result)
        return false;

    result = RegistrationResult {
        .world_to_odom = pipeline_result->transform,
        .score = pipeline_result->score,
        .inlier_ratio = pipeline_result->inlier_ratio,
    };
    return std::isfinite(result.score);
}

auto run_wide(
    const InitialRegistrationConfig& initial_config, const WideRegistrationConfig& wide_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_world_cloud, const pcl::KdTreeFLANN<Point>& map_kdtree,
    const RegistrationPrior& prior, const std::vector<Eigen::Isometry3f>& seeds_world_to_base,
    RegistrationResult& result) -> bool {
    result = RegistrationResult {};
    if (seeds_world_to_base.empty())
        return false;

    auto filtered_query = prepare_filtered_query(query_odom_cloud, initial_config);
    if (!filtered_query)
        return false;

    const auto stage = StageParams::from_wide(wide_config);

    auto candidates = std::vector<RankedCandidate> {};
    candidates.reserve(seeds_world_to_base.size());

    for (const auto& seed_world_to_base : seeds_world_to_base) {
        auto pipeline_result = run_seed_pipeline(
            filtered_query, map_world_cloud, map_kdtree, seed_world_to_base, prior, initial_config,
            stage, wide_config.enable_map_consistency_filter,
            wide_config.map_consistency_distance_m, wide_config.min_retained_fraction);
        if (!pipeline_result)
            continue;

        candidates.push_back(
            RankedCandidate::from_pipeline(pipeline_result.value(), prior, stage, wide_config));
    }

    if (candidates.empty())
        return false;

    std::ranges::sort(candidates, {}, &RankedCandidate::ranking_cost);
    result = RegistrationResult {
        .world_to_odom = candidates.front().world_to_odom,
        .score = candidates.front().score,
        .inlier_ratio = candidates.front().inlier_ratio,
    };
    return std::isfinite(result.score);
}

} // namespace rmcs::location::tools
