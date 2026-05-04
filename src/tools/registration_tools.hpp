#pragma once

#include <cstddef>
#include <limits>
#include <memory>

#include <Eigen/Geometry>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rmcs::location {

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

struct InitialRegistrationConfig {
    int coarse_iterations = 12;
    int refine_iterations = 8;
    int precise_iterations = 20;

    double max_correspondence_distance_m = 0.5;
    double score_threshold = 0.04;
    double yaw_search_window_deg = 15.0;
    double coarse_yaw_step_deg = 15.0;
    double refine_yaw_step_deg = 15.0;
    std::size_t coarse_top_k = 1;

    double voxel_leaf_m = 0.2;
    int outlier_mean_k = 30;
    double outlier_stddev_mul_thresh = 0.5;
};

/**
 * @brief LOCAL 模式配准参数（单 seed，prior 准确时使用）
 */
struct LocalRegistrationConfig {
    int coarse_iterations = 12;
    int refine_iterations = 6;
    int precise_iterations = 12;

    double max_correspondence_distance_m = 4.0;
    double coarse_score_threshold = 0.5;

    double yaw_window_deg = 30.0;
    double coarse_yaw_step_deg = 15.0;
    double refine_yaw_step_deg = 12.0;

    double submap_radius_m = 5.0;

    bool enable_map_consistency_filter = false;
    double map_consistency_distance_m = 0.8;
    double min_retained_fraction = 0.15;
};

/**
 * @brief WIDE 模式配准参数（多 seed，全局兜底；Phase 5 将由 ScanContext 取代 seed 生成）
 */
struct WideRegistrationConfig {
    int coarse_iterations = 15;
    int refine_iterations = 10;
    int precise_iterations = 25;

    double max_correspondence_distance_m = 4.0;
    double coarse_score_threshold = 0.35;

    double yaw_window_deg = 60.0;
    double coarse_yaw_step_deg = 18.0;
    double refine_yaw_step_deg = 12.0;

    double submap_radius_m = 6.0;

    bool enable_map_consistency_filter = false;
    double map_consistency_distance_m = 0.8;
    double min_retained_fraction = 0.15;

    /// fallback seed 生成的位置偏移半径（仅在 ScanContext 不可用时用）
    double seed_offset_m = 1.5;

    /// 从 ScanContext 描述子库取前 sc_top_k 个候选作为 wide handler 的 seed
    std::size_t sc_top_k = 5;

    std::size_t max_candidate_count = 1;
    double rank_weight_inlier = 0.5;
    double rank_weight_distance = 0.3;
    double max_distance_from_prior_m = 12.0;
};

/**
 * @brief LOCAL/WIDE 共用先验上下文
 */
struct RegistrationPrior {
    bool has_prior = false;
    Eigen::Isometry3f world_to_base = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f odom_to_base = Eigen::Isometry3f::Identity();
};

/**
 * @brief LOCAL/WIDE 共用配准结果
 */
struct RegistrationResult {
    Eigen::Isometry3f world_to_odom = Eigen::Isometry3f::Identity();
    double score = std::numeric_limits<double>::infinity();
    double inlier_ratio = 0.0;
};

} // namespace rmcs::location

namespace rmcs::location::tools {

auto from_ros_pointcloud(const sensor_msgs::msg::PointCloud2& message, PointCloud& cloud) -> bool;

auto transform_pointcloud(
    const PointCloud& source, const Eigen::Isometry3f& transform, PointCloud& target) -> bool;

auto extract_submap_radius(
    const std::shared_ptr<PointCloud>& map_world_cloud,
    const Eigen::Vector3f& center,
    double radius_m,
    double fallback_radius_m) -> std::shared_ptr<PointCloud>;

auto extract_submap_radius(
    const pcl::KdTreeFLANN<Point>& kdtree,
    const std::shared_ptr<PointCloud>& map_world_cloud,
    const Eigen::Vector3f& center,
    double radius_m,
    double fallback_radius_m) -> std::shared_ptr<PointCloud>;

auto run_initial(
    const InitialRegistrationConfig& initial_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_world_cloud,
    const Eigen::Isometry3f& world_to_odom_guess,
    Eigen::Isometry3f& world_to_odom_result,
    double& score) -> bool;

/**
 * @brief LOCAL 重定位：单 seed 走多阶段 GICP，依赖 prior 准确
 */
auto run_local(
    const InitialRegistrationConfig& initial_config,
    const LocalRegistrationConfig& local_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_world_cloud,
    const pcl::KdTreeFLANN<Point>& map_kdtree,
    const RegistrationPrior& prior,
    RegistrationResult& result) -> bool;

/**
 * @brief WIDE 重定位：对外部传入的 seed 列表逐个跑 TwoStageGicp，按 ranking 选最优
 *
 * seed 来源由 caller 决定：
 *   - SC 主路径：MapDescriptorDB::query 的 top-K 候选转为 world_to_base seed
 *   - SC 不可用：build_wide_fallback_seeds(prior, config)
 */
auto run_wide(
    const InitialRegistrationConfig& initial_config,
    const WideRegistrationConfig& wide_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_world_cloud,
    const pcl::KdTreeFLANN<Point>& map_kdtree,
    const RegistrationPrior& prior,
    const std::vector<Eigen::Isometry3f>& seeds_world_to_base,
    RegistrationResult& result) -> bool;

/**
 * @brief 构造 wide fallback seed 集（5 位置 × 8 yaw = 40 seed），围绕 prior.world_to_base
 *
 * - 位置偏移：(0,0), (±offset, 0), (0, ±offset)
 * - yaw 偏移（叠加 prior.yaw）：0, ±45°, ±90°, ±135°, 180°
 * 仅在 SC 不可用时使用。
 */
auto build_wide_fallback_seeds(
    const RegistrationPrior& prior, const WideRegistrationConfig& wide_config)
    -> std::vector<Eigen::Isometry3f>;

} // namespace rmcs::location::tools
