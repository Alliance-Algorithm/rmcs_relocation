#pragma once

#include <string>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/node.hpp>
#include <tf2_ros/buffer.h>

#include "common/pimpl.hpp"
#include "tools/registration_tools.hpp"

namespace rmcs::location {

class Collector final {
public:
    explicit Collector(std::string odom_frame);
    ~Collector();

    RMCS_LOCATION_DELETE_COPY(Collector)

    auto collect(
        rclcpp::Node& node,
        tf2_ros::Buffer& tf_buffer,
        const rclcpp::CallbackGroup::SharedPtr& callback_group,
        const std::string& topic_name,
        double duration_sec) const -> std::shared_ptr<PointCloud>;

private:
    RMCS_LOCATION_DECLARE_PIMPL(Collector)
};

} // namespace rmcs::location
