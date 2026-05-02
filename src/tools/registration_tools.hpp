#pragma once

#include <cstddef>
#include <cstdint>
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

enum class LostTier : std::uint8_t {
    LOCAL = 0,
    WIDE = 1,
};

struct LostRegistrationConfig {
    double local_sigma_xy_m = 5.0;
    double local_sigma_yaw_deg = 60.0;

    double submap_radius_local_m = 3.5;
    double submap_radius_wide_m = 5.0;

    int coarse_iterations_local = 10;
    int refine_iterations_local = 5;
    int precise_iterations_local = 15;

    int coarse_iterations_wide = 12;
    int refine_iterations_wide = 8;
    int precise_iterations_wide = 20;

    double max_correspondence_distance_m = 0.9;
    double score_threshold_wide = 0.08;
    double coarse_score_threshold_local = 0.3;
    double coarse_score_threshold_wide = 0.15;

    double local_yaw_window_deg = 15.0;
    double wide_yaw_window_deg = 45.0;

    double local_coarse_yaw_step_deg = 15.0;
    double wide_coarse_yaw_step_deg = 22.5;
    double refine_yaw_step_deg = 15.0;

    bool enable_map_consistency_filter = false;
    double map_consistency_distance_m = 0.8;
    double min_retained_fraction = 0.15;

    std::size_t max_candidate_count = 1;
    double rank_weight_inlier = 0.5;
    double rank_weight_distance = 0.3;
    double max_distance_from_prior_local_m = 3.0;
    double max_distance_from_prior_wide_m = 10.0;
};

struct LostPrior {
    bool has_prior = false;
    Eigen::Isometry3f world_to_base = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f odom_to_base = Eigen::Isometry3f::Identity();
    double sigma_xy_m = 0.0;
    double sigma_yaw_deg = 0.0;
};

struct LostResult {
    Eigen::Isometry3f world_to_odom = Eigen::Isometry3f::Identity();
    double score = std::numeric_limits<double>::infinity();
    double inlier_ratio = 0.0;
    LostTier tier_used = LostTier::WIDE;
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

auto run_lost(
    const InitialRegistrationConfig& initial_config,
    const LostRegistrationConfig& lost_config,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_world_cloud,
    const pcl::KdTreeFLANN<Point>& map_kdtree,
    const LostPrior& prior,
    LostResult& result) -> bool;

} // namespace rmcs::location::tools
