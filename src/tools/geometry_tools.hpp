#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>

namespace rmcs::location::tools {

auto normalize_quaternion(float w, float x, float y, float z) -> Eigen::Quaternionf;

auto normalize_pose(const geometry_msgs::msg::Pose& pose) -> geometry_msgs::msg::Pose;

auto pose_to_isometry(const geometry_msgs::msg::Pose& pose) -> Eigen::Isometry3f;
auto transform_to_isometry(const geometry_msgs::msg::Transform& transform_msg) -> Eigen::Isometry3f;

auto isometry_to_pose(const Eigen::Isometry3f& transform) -> geometry_msgs::msg::Pose;
auto isometry_to_transform(const Eigen::Isometry3f& transform) -> geometry_msgs::msg::Transform;

} // namespace rmcs::location::tools
