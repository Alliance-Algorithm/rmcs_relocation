#pragma once

#include <string>

#include <rclcpp/node.hpp>

#include "common/pimpl.hpp"

namespace rmcs::location {

struct InitialRuntimeConfig {
    std::string pointcloud_topic = "/cloud_registered_undistort";
    double collect_duration_sec = 2.0;
    int min_accumulated_points = 2500;
    double submap_radius_m = 4.0;
};

struct LocalRuntimeConfig {
    std::string pointcloud_topic = "/cloud_registered_undistort";
    double collect_duration_sec = 0.8;
    int min_accumulated_points = 1500;
};

struct WideRuntimeConfig {
    std::string pointcloud_topic = "/cloud_registered_undistort";
    double collect_duration_sec = 1.5;
    int min_accumulated_points = 2000;
};

class RelocalizationServer final
    : public rclcpp::Node {
public:
    RelocalizationServer();
    ~RelocalizationServer() override;

    RMCS_LOCATION_DELETE_COPY(RelocalizationServer)

private:
    RMCS_LOCATION_DECLARE_PIMPL(RelocalizationServer)
};

} // namespace rmcs::location
