#pragma once

#include <string>
#include <string_view>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/node.hpp>

#include "server/health_monitor.hpp"
#include "server/runtime.hpp"
#include "server/validator.hpp"
#include "tools/registration_tools.hpp"

namespace rmcs::location::tools {

class ParamReader final {
public:
    explicit ParamReader(rclcpp::Node& node);

    auto read_string(std::string_view key, std::string_view default_value) const -> std::string;
    auto read_double(std::string_view key, double default_value) const -> double;
    auto read_int(std::string_view key, int default_value) const -> int;
    auto read_bool(std::string_view key, bool default_value) const -> bool;
    auto read_positive_int(std::string_view key, int default_value) const -> int;
    auto read_bounds(std::string_view prefix, const FieldBoundsConfig& defaults) const
        -> FieldBoundsConfig;
    auto read_pose(std::string_view prefix, const geometry_msgs::msg::Pose& defaults) const
        -> geometry_msgs::msg::Pose;

private:
    rclcpp::Node& node_;
};

auto read_pose_parameter(
    rclcpp::Node& node,
    const std::string& prefix,
    const geometry_msgs::msg::Pose& defaults = geometry_msgs::msg::Pose {}) -> geometry_msgs::msg::Pose;

struct RuntimeParamsBundle {
    std::string map_path;
    std::string service_name;
    std::string world_frame;
    std::string odom_frame;
    std::string base_frame;
    double publish_tf_rate_hz = 10.0;

    InitialRuntimeConfig initial_runtime_config {};
    LostRuntimeConfig lost_runtime_config {};
    HealthRuntimeConfig health_runtime_config {};

    InitialRegistrationConfig initial_registration_config {};
    LostRegistrationConfig lost_registration_config {};

    InitialValidationConfig initial_validation_config {};
    LostValidationConfig lost_validation_config {};
};

auto load_runtime_params(rclcpp::Node& node) -> RuntimeParamsBundle;

} // namespace rmcs::location::tools
