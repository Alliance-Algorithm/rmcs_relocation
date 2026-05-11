#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

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
    int precise_iterations = 20;

    double max_correspondence_distance_m = 0.5;
    double score_threshold = 0.04;
    double yaw_search_window_deg = 15.0;
    double coarse_yaw_step_deg = 15.0;
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
    int precise_iterations = 12;

    double max_correspondence_distance_m = 4.0;
    double coarse_score_threshold = 0.5;

    double yaw_window_deg = 30.0;
    double coarse_yaw_step_deg = 15.0;

    double submap_radius_m = 5.0;

    bool enable_map_consistency_filter = false;
    double map_consistency_distance_m = 0.8;
    double min_retained_fraction = 0.15;

    /// 镜像对称场地兜底；早停：seed 1 通过即返回，seed 2 仅当 seed 1 失败时才跑
    std::size_t sc_top_k = 2;
};

/**
 * @brief WIDE 模式配准参数（多 seed，全局兜底；seed 来源由 runtime 决定）
 */
struct WideRegistrationConfig {
    int coarse_iterations = 15;
    int precise_iterations = 25;

    double max_correspondence_distance_m = 4.0;
    double coarse_score_threshold = 0.35;

    double yaw_window_deg = 60.0;
    double coarse_yaw_step_deg = 18.0;

    double submap_radius_m = 6.0;

    bool enable_map_consistency_filter = false;
    double map_consistency_distance_m = 0.8;
    double min_retained_fraction = 0.15;

    /// 从 ScanContext 描述子库取前 sc_top_k 个候选作为 wide handler 的 seed
    std::size_t sc_top_k = 5;

    /// fallback 路径（SC 不可用 / 无匹配时）：以中心 + ±radius/0/0 + 0/±radius/0 共 5 个位置
    /// 每个位置均匀 yaw_count 个朝向，做无 prior 兜底搜索
    double fallback_position_radius_m = 3.5;
    int fallback_yaw_count = 8;

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
    const std::shared_ptr<PointCloud>& map_world_cloud, const Eigen::Vector3f& center,
    double radius_m, double fallback_radius_m) -> std::shared_ptr<PointCloud>;

auto extract_submap_radius(
    const pcl::KdTreeFLANN<Point>& kdtree, const std::shared_ptr<PointCloud>& map_world_cloud,
    const Eigen::Vector3f& center, double radius_m, double fallback_radius_m)
    -> std::shared_ptr<PointCloud>;

auto run_initial(
    const InitialRegistrationConfig& initial_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_world_cloud,
    const Eigen::Isometry3f& world_to_odom_guess, Eigen::Isometry3f& world_to_odom_result,
    double& score) -> bool;

/**
 * @brief LOCAL 重定位：单 seed 走多阶段 GICP，依赖 prior 准确
 */
auto run_local(
    const InitialRegistrationConfig& initial_config, const LocalRegistrationConfig& local_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_world_cloud, const pcl::KdTreeFLANN<Point>& map_kdtree,
    const RegistrationPrior& prior, RegistrationResult& result) -> bool;

/**
 * @brief WIDE 单 seed runner：构造时预处理 query，run() 逐 seed 复用。
 *
 * runtime 的 SC / fallback 路径需要逐 seed 早停；用 runner 避免每个 seed 重复 voxel/outlier。
 */
class WideSeedRunner {
    InitialRegistrationConfig initial_config_;
    WideRegistrationConfig wide_config_;
    std::shared_ptr<PointCloud> filtered_query_;
    std::shared_ptr<PointCloud> map_world_cloud_;
    const pcl::KdTreeFLANN<Point>& map_kdtree_;
    RegistrationPrior prior_;

public:
    WideSeedRunner(
        const InitialRegistrationConfig& initial_config, const WideRegistrationConfig& wide_config,
        const std::shared_ptr<PointCloud>& query_odom_cloud,
        const std::shared_ptr<PointCloud>& map_world_cloud,
        const pcl::KdTreeFLANN<Point>& map_kdtree, const RegistrationPrior& prior);

    [[nodiscard]] auto valid() const -> bool;

    [[nodiscard]] auto
        run(const Eigen::Isometry3f& seed_world_to_base, RegistrationResult& result) const -> bool;
};

/**
 * @brief WIDE 重定位：单 seed 跑完整 GICP 管线。
 */
auto run_wide_seed(
    const InitialRegistrationConfig& initial_config, const WideRegistrationConfig& wide_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_world_cloud, const pcl::KdTreeFLANN<Point>& map_kdtree,
    const RegistrationPrior& prior, const Eigen::Isometry3f& seed_world_to_base,
    RegistrationResult& result) -> bool;

/**
 * @brief WIDE 重定位：多 seed 跑 GICP 后按 ranking 选最优。
 *
 * 保留给需要“一次性评估一组 seed”的调用方；SC / fallback 早停路径通常用 run_wide_seed。
 */
auto run_wide(
    const InitialRegistrationConfig& initial_config, const WideRegistrationConfig& wide_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_world_cloud, const pcl::KdTreeFLANN<Point>& map_kdtree,
    const RegistrationPrior& prior, const std::vector<Eigen::Isometry3f>& seeds_world_to_base,
    RegistrationResult& result) -> bool;

} // namespace rmcs::location::tools
