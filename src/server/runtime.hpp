#pragma once

#include <string>

#include <rclcpp/node.hpp>

#include "common/pimpl.hpp"

namespace rmcs::location {

struct InitialRuntimeConfig {
    std::string pointcloud_topic = "/cloud_registered_undistort";
    double collect_duration_sec = 2.0;
    int min_accumulated_points = 2500;
    double submap_radius_m = 25.0;
};

struct LostRuntimeConfig {
    std::string pointcloud_topic = "/cloud_registered_undistort";
    double collect_duration_sec = 2.0;
    int min_accumulated_points = 2500;
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
