#pragma once

#include <memory>

#include <Eigen/Geometry>
#include <pcl/kdtree/kdtree_flann.h>
#include <rclcpp/time.hpp>
#include <rmcs_msgs/msg/location_health.hpp>

#include "common/pimpl.hpp"
#include "tools/registration_tools.hpp"

namespace rmcs::location {

struct HealthRuntimeConfig {
    double rate_hz = 5.0;
    int sample_points = 500;

    double warn_threshold_m = 0.25;
    double lost_threshold_m = 0.45;
    double min_inlier_ratio = 0.30;

    double warn_dwell_sec = 0.6;
    double lost_dwell_sec = 1.0;

    double recover_margin_m = 0.05;
    double recover_dwell_sec = 2.0;

    double inlier_distance_m = 0.5;
};

class HealthMonitor final {
public:
    explicit HealthMonitor(HealthRuntimeConfig config);
    ~HealthMonitor();

    RMCS_LOCATION_DELETE_COPY(HealthMonitor)

    void ingest_cloud_odom(const std::shared_ptr<PointCloud>& cloud_odom);

    auto evaluate(
        const pcl::KdTreeFLANN<Point>& map_kdtree,
        bool map_kdtree_ready,
        const Eigen::Isometry3f& world_to_odom,
        const rclcpp::Time& stamp) -> rmcs_msgs::msg::LocationHealth;

private:
    RMCS_LOCATION_DECLARE_PIMPL(HealthMonitor)
};

} // namespace rmcs::location
