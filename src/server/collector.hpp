#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include "common/pimpl.hpp"
#include "tools/registration_tools.hpp"

namespace rmcs::location {

/// 点云采集结果：累积 odom 系点云 + 用于参考时刻对齐的元数据。
///
/// reference_stamp 取首帧与末帧 header.stamp 的中点，代表本次累积点云
/// 在时间维度上的"观测质心"，用于按时间查询 odom->base 作为 SC/GICP 的统一先验锚点。
struct CollectedCloud {
    std::shared_ptr<PointCloud> cloud;
    rclcpp::Time reference_stamp{0, 0, RCL_ROS_TIME};
    std::size_t frame_count = 0;
};

class Collector final {
public:
    Collector();
    ~Collector();

    RMCS_LOCATION_DELETE_COPY(Collector)

    /**
     * @brief 采集点云直到时长到期 或 累积点数达到 min_points（取先到者）
     *
     * 输入话题（如 /cloud_registered_undistort）必须已是 odom 系点云，本函数不再
     * 额外做 frame_id->odom 的 TF 变换。点云 header.frame_id 不是 odom 的帧将被丢弃。
     *
     * duration_sec 仍是上界，避免话题阻塞时一直等下去。
     * min_points <= 0 表示禁用早停，等满 duration（旧行为）。
     *
     * 返回的 CollectedCloud 中的 cloud 可能为空，调用方需自行校验点数。
     * 当至少累积到一帧时，reference_stamp 为首末帧 stamp 的中点。
     */
    auto collect(
        rclcpp::Node& node, const rclcpp::CallbackGroup::SharedPtr& callback_group,
        const std::string& topic_name, const std::string& expected_frame_id, double duration_sec,
        int min_points = 0) const -> CollectedCloud;

private:
    RMCS_LOCATION_DECLARE_PIMPL(Collector)
};

} // namespace rmcs::location
