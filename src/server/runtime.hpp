#pragma once

#include <string>

#include <rclcpp/node.hpp>

#include "common/pimpl.hpp"

namespace rmcs::location {

struct InitialRuntimeConfig {
    std::string pointcloud_topic = "/cloud_registered_undistort";
    double collect_duration_sec = 2.0;
    int min_accumulated_points = 2500;
};

struct LocalRuntimeConfig {
    std::string pointcloud_topic = "/cloud_registered_undistort";
    double collect_duration_sec = 0.8;
    int min_accumulated_points = 1500;
};

/// local 模式的导航安全门控参数。
///
/// 这些参数控制 local 在 GICP/validator 之后是否真的发布 world->odom：
/// - 任何单次发布相对当前 world->odom 的增量必须落在 max_tf_correction_* 内；
/// - 两次成功发布之间必须至少间隔 min_accept_interval_sec；
/// - 按参考时刻查 odom_to_base 时允许 tf_lookup_timeout_sec 等待。
struct LocalSafetyConfig {
    double max_tf_correction_m = 0.5;
    double max_tf_correction_yaw_deg = 10.0;
    double min_accept_interval_sec = 0.5;
    double tf_lookup_timeout_sec = 0.05;
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
