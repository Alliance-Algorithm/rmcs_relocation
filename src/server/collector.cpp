#include "collector.hpp"
#include "tools/numeric_tools.hpp"
#include "tools/registration_tools.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rmcs::location {

namespace {

/// 累积过程中需要在订阅回调和主线程之间共享的可变状态。
struct CollectorState {
    std::shared_ptr<PointCloud> accumulated;
    rclcpp::Time first_stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
    std::size_t frame_count = 0;
};

} // namespace

/**
 * @brief 点云收集器内部实现
 *
 * 负责从指定 ROS 话题订阅 odom 系点云，持续采集指定时长，将每帧点云累加到累积点云中，
 * 同时记录首末帧 stamp 与帧数。采集期间阻塞调用线程。
 */
struct Collector::Impl {
    /**
     * @brief 采集指定时长内的 odom 系点云
     *
     * 创建临时订阅，每收到一帧 frame_id 匹配的点云即累加到累积点云中并更新时间戳。
     * 采集时长到期或累积点数达到 min_points 后销毁订阅并返回结果。
     */
    auto collect(
        rclcpp::Node& node, const rclcpp::CallbackGroup::SharedPtr& callback_group,
        const std::string& topic_name, const std::string& expected_frame_id, double duration_sec,
        int min_points) const -> CollectedCloud {
        auto state = std::make_shared<CollectorState>();
        state->accumulated = std::make_shared<PointCloud>();
        auto state_mutex = std::make_shared<std::mutex>();

        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group;
        auto subscription = node.create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, rclcpp::SensorDataQoS(),
            [state, state_mutex,
             expected_frame_id](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
                if (!message)
                    return;
                if (!expected_frame_id.empty() && message->header.frame_id != expected_frame_id)
                    return;

                auto cloud = PointCloud{};
                if (!tools::from_ros_pointcloud(*message, cloud))
                    return;

                const auto stamp = rclcpp::Time{message->header.stamp, RCL_ROS_TIME};

                auto lock = std::scoped_lock{*state_mutex};
                *state->accumulated += cloud;
                if (state->frame_count == 0)
                    state->first_stamp = stamp;
                state->last_stamp = stamp;
                state->frame_count += 1;
            },
            options);

        const auto duration = std::max(0.1, duration_sec);
        const auto finish = std::chrono::steady_clock::now() + tools::as_steady_duration(duration);
        const auto early_stop_threshold =
            min_points > 0 ? static_cast<std::size_t>(min_points) : std::size_t{0};

        while (rclcpp::ok() && std::chrono::steady_clock::now() < finish) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if (early_stop_threshold == 0)
                continue;
            auto current_size = std::size_t{0};
            {
                auto lock = std::scoped_lock{*state_mutex};
                current_size = state->accumulated->size();
            }
            if (current_size >= early_stop_threshold)
                break;
        }

        subscription.reset();

        auto result = CollectedCloud{};
        auto lock = std::scoped_lock{*state_mutex};
        result.cloud = state->accumulated;
        result.frame_count = state->frame_count;
        if (state->frame_count > 0) {
            // 取首末帧 stamp 的中点作为本次累积点云的参考时刻。
            const auto half = (state->last_stamp - state->first_stamp) * 0.5;
            result.reference_stamp = state->first_stamp + half;
        }
        return result;
    }
};

Collector::Collector()
    : pimpl_(std::make_unique<Impl>()) {}

Collector::~Collector() = default;

auto Collector::collect(
    rclcpp::Node& node, const rclcpp::CallbackGroup::SharedPtr& callback_group,
    const std::string& topic_name, const std::string& expected_frame_id, double duration_sec,
    int min_points) const -> CollectedCloud {
    return pimpl_->collect(
        node, callback_group, topic_name, expected_frame_id, duration_sec, min_points);
}

} // namespace rmcs::location
