#include "collector.hpp"
#include "tools/geometry_tools.hpp"
#include "tools/numeric_tools.hpp"
#include "tools/registration_tools.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <Eigen/Geometry>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace rmcs::location {

/**
 * @brief 点云收集器内部实现
 *
 * 负责从指定 ROS 话题订阅点云，持续采集指定时长，将每帧点云转换到 odom 坐标系后累加，
 * 最终返回累积后的完整点云。采集期间阻塞调用线程。
 */
struct Collector::Impl {
    explicit Impl(std::string odom_frame)
        : odom_frame_(std::move(odom_frame)) {}

    auto lookup_frame_to_odom(
        tf2_ros::Buffer& tf_buffer, const std::string& frame_id, Eigen::Isometry3f& transform) const
        -> bool {
        if (frame_id.empty())
            return false;
        try {
            const auto stamp = tf_buffer.lookupTransform(odom_frame_, frame_id, tf2::TimePointZero);
            transform = tools::transform_to_isometry(stamp.transform);
            return true;
        } catch (const tf2::TransformException&) {
            return false;
        }
    }

    /**
     * @brief 采集指定时长内的点云并转换到 odom 坐标系
     *
     * 创建临时订阅，每收到一帧点云即通过 TF 转换到 odom 坐标系并累加到累积点云中。
     * 采集时长到期后销毁订阅并返回结果。
     *
     */
    auto collect(
        rclcpp::Node& node, tf2_ros::Buffer& tf_buffer,
        const rclcpp::CallbackGroup::SharedPtr& callback_group, const std::string& topic_name,
        double duration_sec) const -> std::shared_ptr<PointCloud> {
        auto accumulated = std::make_shared<PointCloud>();
        auto cloud_mutex = std::make_shared<std::mutex>();

        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group;
        auto subscription = node.create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, rclcpp::SensorDataQoS(),
            [this, accumulated, cloud_mutex,
             &tf_buffer](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
                if (!message)
                    return;

                auto frame_to_odom = Eigen::Isometry3f::Identity();
                if (!lookup_frame_to_odom(tf_buffer, message->header.frame_id, frame_to_odom))
                    return;

                auto cloud = PointCloud{};
                if (!tools::from_ros_pointcloud(*message, cloud))
                    return;

                auto transformed = PointCloud{};
                if (!tools::transform_pointcloud(cloud, frame_to_odom, transformed))
                    return;

                auto lock = std::scoped_lock{*cloud_mutex};
                *accumulated += transformed;
            },
            options);

        const auto duration = std::max(0.1, duration_sec);
        const auto finish = std::chrono::steady_clock::now() + tools::as_steady_duration(duration);

        while (rclcpp::ok() && std::chrono::steady_clock::now() < finish)
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

        subscription.reset();
        return accumulated;
    }

    std::string odom_frame_;
};

Collector::Collector(std::string odom_frame)
    : pimpl_(std::make_unique<Impl>(std::move(odom_frame))) {}

Collector::~Collector() = default;

auto Collector::collect(
    rclcpp::Node& node, tf2_ros::Buffer& tf_buffer,
    const rclcpp::CallbackGroup::SharedPtr& callback_group, const std::string& topic_name,
    double duration_sec) const -> std::shared_ptr<PointCloud> {
    return pimpl_->collect(node, tf_buffer, callback_group, topic_name, duration_sec);
}

} // namespace rmcs::location
