#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rmcs::location {

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

/**
 * @brief INITIAL / LOCAL / WIDE 共用点云预处理参数
 */
struct PointCloudPreprocessConfig {
    double voxel_leaf_m = 0.2;
    int outlier_mean_k = 30;
    double outlier_stddev_mul_thresh = 0.5;
};

/**
 * @brief small_gicp PCL adapter 共用参数
 */
struct GicpConfig {
    int num_threads = 4;
    int num_neighbors_for_covariance = 20;
    double rotation_epsilon = 2e-3;
    double voxel_resolution = 0.2;
    double coarse_transformation_epsilon = 1e-3;
    double precise_transformation_epsilon = 1e-4;
};

/**
 * @brief 三种模式共用的配准底层参数
 */
struct CommonRegistrationConfig {
    PointCloudPreprocessConfig preprocess{};
    GicpConfig gicp{};
};

struct InitialRegistrationConfig {
    int coarse_iterations = 12;
    int precise_iterations = 20;

    double max_correspondence_distance_m = 0.5;
    double score_threshold = 0.04;
    double yaw_search_window_deg = 15.0;
    double coarse_yaw_step_deg = 15.0;
    std::size_t coarse_top_k = 1;
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

    /// 镜像对称场地兜底；早停：seed 1 通过即返回，seed 2 仅当 seed 1 失败时才跑
    std::size_t sc_top_k = 2;
};

/**
 * @brief WIDE 模式配准参数（SC 全局 seed，seed 来源由 runtime 决定）
 */
struct WideRegistrationConfig {
    int coarse_iterations = 15;
    int precise_iterations = 25;

    double max_correspondence_distance_m = 4.0;
    double coarse_score_threshold = 0.35;

    double yaw_window_deg = 60.0;
    double coarse_yaw_step_deg = 18.0;

    /// 从 ScanContext 描述子库取前 sc_top_k 个候选作为 wide handler 的 seed
    std::size_t sc_top_k = 5;

    std::size_t max_candidate_count = 1;
    double rank_weight_inlier = 0.5;
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

/**
 * @brief 启动时一次性预处理全局地图（voxel 下采样 + statistical outlier 移除）
 *
 * 处理后的 cloud 直接作为 initial / local / wide 各路径的 GICP target。
 * 让 query 里的"场外/远处"点找到对应（RM 镜像消歧的关键信号），
 * 同时省掉 per-seed 的 voxel + outlier 开销。
 */
auto preprocess_map(
    const std::shared_ptr<PointCloud>& raw_map_cloud,
    const PointCloudPreprocessConfig& preprocess_config) -> std::shared_ptr<PointCloud>;

/**
 * @brief INITIAL 重定位：单 seed，由用户提供的 initial_guess 驱动。
 *
 * @param map_target_cloud 由 preprocess_map() 产出的全图 target（不再切 submap）
 */
auto run_initial(
    const CommonRegistrationConfig& common_config, const InitialRegistrationConfig& initial_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_target_cloud,
    const Eigen::Isometry3f& world_to_odom_guess, Eigen::Isometry3f& world_to_odom_result,
    double& score) -> bool;

/**
 * @brief LOCAL 重定位：单 seed 走多阶段 GICP，依赖 prior 准确
 *
 * @param map_target_cloud 由 preprocess_map() 产出的全图 target（不再切 submap）
 */
auto run_local(
    const CommonRegistrationConfig& common_config, const LocalRegistrationConfig& local_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_target_cloud, const RegistrationPrior& prior,
    RegistrationResult& result) -> bool;

/**
 * @brief WIDE 单 seed runner：构造时预处理 query，run() 逐 seed 复用。
 *
 * runtime 的 SC 路径需要逐 seed 早停；用 runner 避免每个 seed 重复 voxel/outlier。
 * map_target_cloud 由 preprocess_map() 产出，所有 seed 复用同一个 target。
 */
class WideSeedRunner {
    CommonRegistrationConfig common_config_;
    WideRegistrationConfig wide_config_;
    std::shared_ptr<PointCloud> filtered_query_;
    std::shared_ptr<PointCloud> map_target_cloud_;
    RegistrationPrior prior_;

public:
    WideSeedRunner(
        const CommonRegistrationConfig& common_config, const WideRegistrationConfig& wide_config,
        const std::shared_ptr<PointCloud>& query_odom_cloud,
        const std::shared_ptr<PointCloud>& map_target_cloud, const RegistrationPrior& prior);

    [[nodiscard]] auto valid() const -> bool;

    [[nodiscard]] auto
        run(const Eigen::Isometry3f& seed_world_to_base, RegistrationResult& result) const -> bool;
};

/**
 * @brief WIDE 重定位：单 seed 跑完整 GICP 管线。
 */
auto run_wide_seed(
    const CommonRegistrationConfig& common_config, const WideRegistrationConfig& wide_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_target_cloud, const RegistrationPrior& prior,
    const Eigen::Isometry3f& seed_world_to_base, RegistrationResult& result) -> bool;

/**
 * @brief WIDE 重定位：多 seed 跑 GICP 后按 ranking 选最优。
 *
 * 保留给需要"一次性评估一组 seed"的调用方；SC 早停路径通常用 run_wide_seed。
 */
auto run_wide(
    const CommonRegistrationConfig& common_config, const WideRegistrationConfig& wide_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_target_cloud, const RegistrationPrior& prior,
    const std::vector<Eigen::Isometry3f>& seeds_world_to_base, RegistrationResult& result) -> bool;

} // namespace rmcs::location::tools
