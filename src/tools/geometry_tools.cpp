#include "tools/geometry_tools.hpp"

#include <cmath>

namespace rmcs::location::tools {

auto normalize_quaternion(float w, float x, float y, float z) -> Eigen::Quaternionf {
    auto quaternion = Eigen::Quaternionf { w, x, y, z };
    if (quaternion.norm() <= 1e-6F)
        return Eigen::Quaternionf::Identity();

    quaternion.normalize();
    return quaternion;
}

auto normalize_pose(const geometry_msgs::msg::Pose& pose) -> geometry_msgs::msg::Pose {
    auto normalized = pose;
    const auto quaternion = normalize_quaternion(
        static_cast<float>(pose.orientation.w),
        static_cast<float>(pose.orientation.x),
        static_cast<float>(pose.orientation.y),
        static_cast<float>(pose.orientation.z));
    normalized.orientation.w = quaternion.w();
    normalized.orientation.x = quaternion.x();
    normalized.orientation.y = quaternion.y();
    normalized.orientation.z = quaternion.z();
    return normalized;
}

auto pose_to_isometry(const geometry_msgs::msg::Pose& pose) -> Eigen::Isometry3f {
    auto transform = Eigen::Isometry3f::Identity();
    transform.translation().x() = static_cast<float>(pose.position.x);
    transform.translation().y() = static_cast<float>(pose.position.y);
    transform.translation().z() = static_cast<float>(pose.position.z);
    transform.linear() =
        normalize_quaternion(
            static_cast<float>(pose.orientation.w),
            static_cast<float>(pose.orientation.x),
            static_cast<float>(pose.orientation.y),
            static_cast<float>(pose.orientation.z))
            .toRotationMatrix();
    return transform;
}

auto transform_to_isometry(const geometry_msgs::msg::Transform& transform_msg) -> Eigen::Isometry3f {
    auto transform = Eigen::Isometry3f::Identity();
    transform.translation().x() = static_cast<float>(transform_msg.translation.x);
    transform.translation().y() = static_cast<float>(transform_msg.translation.y);
    transform.translation().z() = static_cast<float>(transform_msg.translation.z);
    transform.linear() =
        normalize_quaternion(
            static_cast<float>(transform_msg.rotation.w),
            static_cast<float>(transform_msg.rotation.x),
            static_cast<float>(transform_msg.rotation.y),
            static_cast<float>(transform_msg.rotation.z))
            .toRotationMatrix();
    return transform;
}

auto isometry_to_pose(const Eigen::Isometry3f& transform) -> geometry_msgs::msg::Pose {
    auto pose = geometry_msgs::msg::Pose {};
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();

    const auto quaternion = Eigen::Quaternionf { transform.rotation() }.normalized();
    pose.orientation.w = quaternion.w();
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    return pose;
}

auto isometry_to_transform(const Eigen::Isometry3f& transform) -> geometry_msgs::msg::Transform {
    auto transform_msg = geometry_msgs::msg::Transform {};
    transform_msg.translation.x = transform.translation().x();
    transform_msg.translation.y = transform.translation().y();
    transform_msg.translation.z = transform.translation().z();

    const auto quaternion = Eigen::Quaternionf { transform.rotation() }.normalized();
    transform_msg.rotation.w = quaternion.w();
    transform_msg.rotation.x = quaternion.x();
    transform_msg.rotation.y = quaternion.y();
    transform_msg.rotation.z = quaternion.z();
    return transform_msg;
}

} // namespace rmcs::location::tools
