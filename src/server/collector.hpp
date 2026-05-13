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

    /**
     * @brief 采集点云直到时长到期 或 累积点数达到 min_points（取先到者）
     *
     * duration_sec 仍是上界，避免话题阻塞时一直等下去。
     * min_points <= 0 表示禁用早停，等满 duration（旧行为）。
     */
    auto collect(
        rclcpp::Node& node,
        tf2_ros::Buffer& tf_buffer,
        const rclcpp::CallbackGroup::SharedPtr& callback_group,
        const std::string& topic_name,
        double duration_sec,
        int min_points = 0) const -> std::shared_ptr<PointCloud>;

private:
    RMCS_LOCATION_DECLARE_PIMPL(Collector)
};

} // namespace rmcs::location
