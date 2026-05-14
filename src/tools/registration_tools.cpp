/**
 * @file registration_tools.cpp
 * @brief 点云配准工具实现
 *
 * 提供 INITIAL / LOCAL / WIDE 三个独立配准入口。共用 TwoStageGicp（coarse → precise）
 * 与 yaw 搜索 + top-k 候选。
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
#include <string_view>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/logging.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>

namespace rmcs::location::tools {

namespace {

using GicpRegistrator = small_gicp::RegistrationPCL<Point, Point>;
using PointCloudPtr = std::shared_ptr<PointCloud>;

/// 两阶段 GICP 的统一参数包（INITIAL / LOCAL / WIDE 三处复用）
struct StageParams {
    int coarse_iterations = 12;
    int precise_iterations = 20;

    double yaw_window_deg = 15.0;
    double coarse_step_deg = 15.0;

    std::size_t coarse_top_k = 1;
    double coarse_score_threshold = std::numeric_limits<double>::infinity();
    double max_correspondence_distance = 0.5;

    static auto from_initial(const InitialRegistrationConfig& c) -> StageParams {
        return StageParams{
            .coarse_iterations = sanitize_iterations(c.coarse_iterations, 12),
            .precise_iterations = sanitize_iterations(c.precise_iterations, 20),
            .yaw_window_deg = sanitize_non_negative(c.yaw_search_window_deg, 0.0),
            .coarse_step_deg = sanitize_step(c.coarse_yaw_step_deg, 1.0),
            .coarse_top_k = std::max<std::size_t>(1, c.coarse_top_k),
            .coarse_score_threshold = sanitize_non_negative(c.score_threshold, 0.04),
            .max_correspondence_distance =
                sanitize_non_negative(c.max_correspondence_distance_m, 0.5),
        };
    }

    static auto from_local(const LocalRegistrationConfig& c) -> StageParams {
        return StageParams{
            .coarse_iterations = sanitize_iterations(c.coarse_iterations, 10),
            .precise_iterations = sanitize_iterations(c.precise_iterations, 15),
            .yaw_window_deg = sanitize_non_negative(c.yaw_window_deg, 0.0),
            .coarse_step_deg = sanitize_step(c.coarse_yaw_step_deg, 1.0),
            .coarse_top_k = 1,
            .coarse_score_threshold = sanitize_non_negative(c.coarse_score_threshold, 0.3),
            .max_correspondence_distance =
                sanitize_non_negative(c.max_correspondence_distance_m, 0.9),
        };
    }

    static auto from_wide(const WideRegistrationConfig& c) -> StageParams {
        return StageParams{
            .coarse_iterations = sanitize_iterations(c.coarse_iterations, 12),
            .precise_iterations = sanitize_iterations(c.precise_iterations, 20),
            .yaw_window_deg = sanitize_non_negative(c.yaw_window_deg, 0.0),
            .coarse_step_deg = sanitize_step(c.coarse_yaw_step_deg, 1.0),
            .coarse_top_k = std::max<std::size_t>(1, c.max_candidate_count),
            .coarse_score_threshold = sanitize_non_negative(c.coarse_score_threshold, 0.15),
            .max_correspondence_distance =
                sanitize_non_negative(c.max_correspondence_distance_m, 0.9),
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

auto relocation_logger() -> rclcpp::Logger { return rclcpp::get_logger("rmcs_relocation"); }

struct LogPrefix {
    int size = 0;
    const char* data = "";
};

auto log_prefix(std::string_view label) -> LogPrefix {
    return LogPrefix{.size = static_cast<int>(label.size()), .data = label.data()};
}

/**
 * @brief WIDE 多 seed 候选评分
 *
 * ranking_cost = score + w_inlier * (1 - inlier_ratio)
 */
struct RankedCandidate {
    double score = std::numeric_limits<double>::infinity();
    double inlier_ratio = 0.0;
    double ranking_cost = std::numeric_limits<double>::infinity();
    Eigen::Isometry3f world_to_odom = Eigen::Isometry3f::Identity();

    void compute_ranking(const WideRegistrationConfig& config) {
        const auto inlier_penalty = 1.0 - std::clamp(inlier_ratio, 0.0, 1.0);
        ranking_cost =
            score + sanitize_non_negative(config.rank_weight_inlier, 0.5) * inlier_penalty;
    }

    static auto
        from_pipeline(const PipelineResult& pipeline_result, const WideRegistrationConfig& config)
            -> RankedCandidate {
        auto candidate = RankedCandidate{};
        candidate.score = pipeline_result.score;
        candidate.inlier_ratio = pipeline_result.inlier_ratio;
        candidate.world_to_odom = pipeline_result.transform;
        candidate.compute_ranking(config);
        return candidate;
    }
};

class GicpAligner {
    mutable GicpRegistrator registrator_;
    double max_correspondence_distance_m_ = 5.0;

public:
    GicpAligner(
        int iterations, double max_correspondence_distance_m, PointCloudPtr source,
        PointCloudPtr target, const GicpConfig& config, double transformation_epsilon)
        : max_correspondence_distance_m_(max_correspondence_distance_m) {
        registrator_.setNumThreads(sanitize_positive_int(config.num_threads, 4));
        registrator_.setNumNeighborsForCovariance(
            sanitize_positive_int(config.num_neighbors_for_covariance, 20));
        registrator_.setRegistrationType("VGICP");
        registrator_.setVoxelResolution(
            std::max(1e-3, sanitize_non_negative(config.voxel_resolution, 0.2)));
        registrator_.setRotationEpsilon(sanitize_non_negative(config.rotation_epsilon, 2e-3));
        registrator_.setMaximumIterations(iterations);
        registrator_.setMaxCorrespondenceDistance(max_correspondence_distance_m_);
        registrator_.setTransformationEpsilon(transformation_epsilon);
        registrator_.setEuclideanFitnessEpsilon(transformation_epsilon);
        registrator_.setInputSource(source);
        registrator_.setInputTarget(target);
    }

    [[nodiscard]] auto try_align(const Eigen::Isometry3f& guess) const
        -> std::optional<ScoredTransform> {
        auto aligned = PointCloud{};
        registrator_.align(aligned, guess.matrix());
        if (!registrator_.hasConverged())
            return std::nullopt;

        const auto score = registrator_.getFitnessScore(max_correspondence_distance_m_);
        if (!std::isfinite(score))
            return std::nullopt;

        return ScoredTransform{
            .score = score,
            .transform = Eigen::Isometry3f{registrator_.getFinalTransformation()},
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
[[nodiscard]] auto
    generate_yaw_candidates(const Eigen::Isometry3f& base_pose, double window_deg, double step_deg)
        -> std::vector<Eigen::Isometry3f> {
    auto offsets = std::vector<double>{0.0};
    for (double delta = step_deg; delta <= (window_deg + 1e-6); delta += step_deg) {
        offsets.push_back(delta);
        offsets.push_back(-delta);
    }

    auto candidates = std::vector<Eigen::Isometry3f>{};
    candidates.reserve(offsets.size());

    for (const auto yaw_delta_deg : offsets) {
        const auto yaw_delta_radian = static_cast<float>(yaw_delta_deg * std::numbers::pi / 180.0);
        const auto yaw_rotation = Eigen::AngleAxisf{yaw_delta_radian, Eigen::Vector3f::UnitZ()};

        auto candidate = base_pose;
        candidate.linear() = (yaw_rotation * Eigen::Quaternionf{base_pose.rotation()})
                                 .normalized()
                                 .toRotationMatrix();
        candidates.push_back(candidate);
    }

    return candidates;
}

class TwoStageGicp {
    StageParams params_;
    GicpAligner coarse_;
    GicpAligner precise_;
    std::size_t source_size_ = 0;
    std::size_t target_size_ = 0;
    std::string_view label_;

    [[nodiscard]] auto run_coarse(const Eigen::Isometry3f& guess)
        -> std::optional<std::vector<ScoredTransform>> {
        auto coarse_results = std::vector<ScoredTransform>{};
        const auto candidates =
            generate_yaw_candidates(guess, params_.yaw_window_deg, params_.coarse_step_deg);
        auto tried_count = std::size_t{};
        auto converged_count = std::size_t{};
        for (const auto& candidate_guess : candidates) {
            ++tried_count;
            if (auto candidate = coarse_.try_align(candidate_guess); candidate.has_value()) {
                coarse_results.push_back(candidate.value());
                ++converged_count;
                if (candidate->score <= params_.coarse_score_threshold)
                    break;
            }
        }

        const auto prefix = log_prefix(label_);
        if (coarse_results.empty()) {
            RCLCPP_WARN(
                relocation_logger(),
                "%.*s GICP coarse failed: tried=%zu converged=0 yaw_window=%.1f step=%.1f "
                "iter=%d max_corr=%.2f source=%zu target=%zu",
                prefix.size, prefix.data, tried_count, params_.yaw_window_deg,
                params_.coarse_step_deg, params_.coarse_iterations,
                params_.max_correspondence_distance, source_size_, target_size_);
            return std::nullopt;
        }

        std::ranges::sort(coarse_results, {}, &ScoredTransform::score);
        if (coarse_results.size() > params_.coarse_top_k)
            coarse_results.resize(params_.coarse_top_k);

        RCLCPP_INFO(
            relocation_logger(),
            "%.*s GICP coarse ok: tried=%zu converged=%zu kept=%zu best_score=%.4f "
            "score_break<=%.4f",
            prefix.size, prefix.data, tried_count, converged_count, coarse_results.size(),
            coarse_results.front().score, params_.coarse_score_threshold);

        return coarse_results;
    }

    [[nodiscard]] auto run_precise(const Eigen::Isometry3f& guess)
        -> std::optional<PipelineResult> {
        const auto prefix = log_prefix(label_);
        auto precise_result = precise_.try_align(guess);
        if (!precise_result) {
            RCLCPP_WARN(
                relocation_logger(),
                "%.*s GICP precise failed: iter=%d max_corr=%.2f source=%zu target=%zu",
                prefix.size, prefix.data, params_.precise_iterations,
                params_.max_correspondence_distance, source_size_, target_size_);
            return std::nullopt;
        }

        const auto query_size = std::max<std::size_t>(1, source_size_);
        const auto num_inliers = precise_.num_inliers();
        const auto inlier_ratio = std::clamp(
            static_cast<double>(num_inliers) / static_cast<double>(query_size), 0.0, 1.0);
        RCLCPP_INFO(
            relocation_logger(), "%.*s GICP precise ok: score=%.4f inlier=%zu/%zu ratio=%.3f",
            prefix.size, prefix.data, precise_result->score, num_inliers, query_size, inlier_ratio);

        return PipelineResult{
            .transform = precise_result->transform,
            .score = precise_result->score,
            .inlier_ratio = inlier_ratio,
        };
    }

public:
    TwoStageGicp(
        const StageParams& params, PointCloudPtr source, PointCloudPtr target,
        const GicpConfig& gicp_config, std::string_view label)
        : params_(params)
        , coarse_(
              params_.coarse_iterations, params_.max_correspondence_distance, source, target,
              gicp_config, sanitize_non_negative(gicp_config.coarse_transformation_epsilon, 1e-3))
        , precise_(
              params_.precise_iterations, params_.max_correspondence_distance, source, target,
              gicp_config, sanitize_non_negative(gicp_config.precise_transformation_epsilon, 1e-4))
        , source_size_(source ? source->size() : 0)
        , target_size_(target ? target->size() : 0)
        , label_(label) {}

    [[nodiscard]] auto run(const Eigen::Isometry3f& guess) -> std::optional<PipelineResult> {
        auto coarse_results = run_coarse(guess);
        if (!coarse_results)
            return std::nullopt;

        auto best_precise = std::optional<PipelineResult>{};
        auto tried_count = std::size_t{};
        auto converged_count = std::size_t{};
        for (const auto& coarse_result : coarse_results.value()) {
            ++tried_count;
            auto precise_result = run_precise(coarse_result.transform);
            if (!precise_result)
                continue;

            ++converged_count;
            if (!best_precise || precise_result->score < best_precise->score)
                best_precise = precise_result;
        }

        const auto prefix = log_prefix(label_);
        if (!best_precise) {
            RCLCPP_WARN(
                relocation_logger(),
                "%.*s GICP precise failed for all coarse candidates: "
                "tried=%zu converged=0 iter=%d max_corr=%.2f",
                prefix.size, prefix.data, tried_count, params_.precise_iterations,
                params_.max_correspondence_distance);
            return std::nullopt;
        }

        RCLCPP_INFO(
            relocation_logger(),
            "%.*s GICP two-stage ok: precise_tried=%zu precise_converged=%zu best_score=%.4f "
            "best_inlier=%.3f",
            prefix.size, prefix.data, tried_count, converged_count, best_precise->score,
            best_precise->inlier_ratio);
        return best_precise;
    }
};

auto preprocess_cloud(
    const PointCloudPtr& cloud, const PointCloudPreprocessConfig& preprocess_config,
    bool with_outlier) -> PointCloudPtr {
    auto downsampled = std::make_shared<PointCloud>();
    if (!cloud || cloud->empty())
        return downsampled;

    const auto leaf = static_cast<float>(
        std::max(1e-3, sanitize_non_negative(preprocess_config.voxel_leaf_m, 0.2)));
    auto voxel = pcl::VoxelGrid<Point>{};
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*downsampled);

    const auto mean_k = std::max(1, preprocess_config.outlier_mean_k);
    if (!with_outlier || downsampled->size() <= static_cast<std::size_t>(mean_k))
        return downsampled;

    auto filtered = std::make_shared<PointCloud>();
    auto outlier = pcl::StatisticalOutlierRemoval<Point>{};
    outlier.setInputCloud(downsampled);
    outlier.setMeanK(mean_k);
    outlier.setStddevMulThresh(
        sanitize_non_negative(preprocess_config.outlier_stddev_mul_thresh, 0.5));
    outlier.filter(*filtered);
    return filtered;
}

/// 三个 run_* 入口共享：voxel + outlier 滤波 → 空检查。空时返回 nullptr，调用方应直接 false。
auto prepare_filtered_query(
    const PointCloudPtr& query_odom_cloud, const PointCloudPreprocessConfig& preprocess_config)
    -> PointCloudPtr {
    auto filtered = preprocess_cloud(query_odom_cloud, preprocess_config, true);
    if (!filtered || filtered->empty())
        return nullptr;
    return filtered;
}

/**
 * @brief 单 seed 配准：TwoStageGicp
 *
 * map_target_cloud 必须由 preprocess_map() 预处理过；run_seed_pipeline 不再切 submap，
 * 直接把全图作为 GICP target。这样 query 里的远处/场外点也能找到对应，
 * 是 RM 这种镜像对称场地里消歧的关键信号。
 */
auto run_seed_pipeline(
    const PointCloudPtr& filtered_query, const PointCloudPtr& map_target_cloud,
    const Eigen::Isometry3f& seed_world_to_base, const RegistrationPrior& prior,
    const StageParams& stage, const GicpConfig& gicp_config, std::string_view label)
    -> std::optional<PipelineResult> {
    const auto prefix = log_prefix(label);
    if (!map_target_cloud || map_target_cloud->empty()) {
        RCLCPP_WARN(
            relocation_logger(), "%.*s map_target_cloud empty, skip", prefix.size, prefix.data);
        return std::nullopt;
    }
    const auto& filtered_map = map_target_cloud;

    // 调参诊断日志：记录进入单 seed GICP 前的 query/map 点数和关键 local 参数。
    RCLCPP_INFO(
        relocation_logger(),
        "%.*s seed setup: query_filtered=%zu map_filtered=%zu max_corr=%.2f iter=%d/%d yaw=%.1f "
        "step=%.1f seed=(%.2f,%.2f,%.1fdeg)",
        prefix.size, prefix.data, filtered_query ? filtered_query->size() : 0, filtered_map->size(),
        stage.max_correspondence_distance, stage.coarse_iterations, stage.precise_iterations,
        stage.yaw_window_deg, stage.coarse_step_deg, seed_world_to_base.translation().x(),
        seed_world_to_base.translation().y(),
        std::atan2(
            static_cast<double>(seed_world_to_base.rotation()(1, 0)),
            static_cast<double>(seed_world_to_base.rotation()(0, 0)))
            * 180.0 / std::numbers::pi);

    const auto seed_filtered_query = filtered_query;

    const auto seed_world_to_odom =
        world_to_odom_from_world_to_base(seed_world_to_base, prior.odom_to_base);
    auto pipeline = TwoStageGicp{stage, seed_filtered_query, filtered_map, gicp_config, label};
    return pipeline.run(seed_world_to_odom);
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

auto preprocess_map(
    const std::shared_ptr<PointCloud>& raw_map_cloud,
    const PointCloudPreprocessConfig& preprocess_config) -> std::shared_ptr<PointCloud> {
    return preprocess_cloud(raw_map_cloud, preprocess_config, true);
}

auto run_initial(
    const CommonRegistrationConfig& common_config, const InitialRegistrationConfig& initial_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_target_cloud,
    const Eigen::Isometry3f& world_to_odom_guess, Eigen::Isometry3f& world_to_odom_result,
    double& score) -> bool {
    auto filtered_query = prepare_filtered_query(query_odom_cloud, common_config.preprocess);
    if (!filtered_query || !map_target_cloud || map_target_cloud->empty())
        return false;

    auto pipeline = TwoStageGicp{
        StageParams::from_initial(initial_config), filtered_query, map_target_cloud,
        common_config.gicp, "initial"};
    auto pipeline_result = pipeline.run(world_to_odom_guess);
    if (!pipeline_result)
        return false;

    world_to_odom_result = pipeline_result->transform;
    score = pipeline_result->score;
    return std::isfinite(score);
}

auto run_local(
    const CommonRegistrationConfig& common_config, const LocalRegistrationConfig& local_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_target_cloud, const RegistrationPrior& prior,
    RegistrationResult& result) -> bool {
    result = RegistrationResult{};
    if (!prior.has_prior)
        return false;

    auto filtered_query = prepare_filtered_query(query_odom_cloud, common_config.preprocess);
    if (!filtered_query) {
        // 调参诊断日志：记录 local 输入点云预处理后为空。
        RCLCPP_WARN(
            relocation_logger(), "local query preprocess empty: raw=%zu",
            query_odom_cloud ? query_odom_cloud->size() : 0);
        return false;
    }

    const auto stage = StageParams::from_local(local_config);
    // 调参诊断日志：记录 local 输入点云 raw/voxel/outlier 之后的点数。
    RCLCPP_INFO(
        relocation_logger(),
        "local query preprocess: raw=%zu filtered=%zu voxel=%.3f outlier_mean_k=%d "
        "outlier_stddev=%.3f",
        query_odom_cloud ? query_odom_cloud->size() : 0, filtered_query->size(),
        common_config.preprocess.voxel_leaf_m, common_config.preprocess.outlier_mean_k,
        common_config.preprocess.outlier_stddev_mul_thresh);
    auto pipeline_result = run_seed_pipeline(
        filtered_query, map_target_cloud, prior.world_to_base, prior, stage, common_config.gicp,
        "local");
    if (!pipeline_result)
        return false;

    result = RegistrationResult{
        .world_to_odom = pipeline_result->transform,
        .score = pipeline_result->score,
        .inlier_ratio = pipeline_result->inlier_ratio,
    };
    return std::isfinite(result.score);
}

WideSeedRunner::WideSeedRunner(
    const CommonRegistrationConfig& common_config, const WideRegistrationConfig& wide_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_target_cloud, const RegistrationPrior& prior)
    : common_config_(common_config)
    , wide_config_(wide_config)
    , filtered_query_(prepare_filtered_query(query_odom_cloud, common_config.preprocess))
    , map_target_cloud_(map_target_cloud)
    , prior_(prior) {}

auto WideSeedRunner::valid() const -> bool { return static_cast<bool>(filtered_query_); }

auto WideSeedRunner::run(
    const Eigen::Isometry3f& seed_world_to_base, RegistrationResult& result) const -> bool {
    result = RegistrationResult{};
    if (!valid())
        return false;

    const auto stage = StageParams::from_wide(wide_config_);
    auto pipeline_result = run_seed_pipeline(
        filtered_query_, map_target_cloud_, seed_world_to_base, prior_, stage, common_config_.gicp,
        "wide");
    if (!pipeline_result)
        return false;

    result = RegistrationResult{
        .world_to_odom = pipeline_result->transform,
        .score = pipeline_result->score,
        .inlier_ratio = pipeline_result->inlier_ratio,
    };
    return std::isfinite(result.score);
}

auto run_wide_seed(
    const CommonRegistrationConfig& common_config, const WideRegistrationConfig& wide_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_target_cloud, const RegistrationPrior& prior,
    const Eigen::Isometry3f& seed_world_to_base, RegistrationResult& result) -> bool {
    const auto runner =
        WideSeedRunner{common_config, wide_config, query_odom_cloud, map_target_cloud, prior};
    return runner.run(seed_world_to_base, result);
}

auto run_wide(
    const CommonRegistrationConfig& common_config, const WideRegistrationConfig& wide_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_target_cloud, const RegistrationPrior& prior,
    const std::vector<Eigen::Isometry3f>& seeds_world_to_base, RegistrationResult& result) -> bool {
    result = RegistrationResult{};
    if (seeds_world_to_base.empty())
        return false;

    auto filtered_query = prepare_filtered_query(query_odom_cloud, common_config.preprocess);
    if (!filtered_query)
        return false;

    const auto stage = StageParams::from_wide(wide_config);

    auto candidates = std::vector<RankedCandidate>{};
    candidates.reserve(seeds_world_to_base.size());

    for (const auto& seed_world_to_base : seeds_world_to_base) {
        auto pipeline_result = run_seed_pipeline(
            filtered_query, map_target_cloud, seed_world_to_base, prior, stage, common_config.gicp,
            "wide");
        if (!pipeline_result)
            continue;

        candidates.push_back(RankedCandidate::from_pipeline(pipeline_result.value(), wide_config));
    }

    if (candidates.empty())
        return false;

    std::ranges::sort(candidates, {}, &RankedCandidate::ranking_cost);
    result = RegistrationResult{
        .world_to_odom = candidates.front().world_to_odom,
        .score = candidates.front().score,
        .inlier_ratio = candidates.front().inlier_ratio,
    };
    return std::isfinite(result.score);
}

} // namespace rmcs::location::tools
