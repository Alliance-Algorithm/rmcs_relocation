#include <chrono>
#include <cmath>
#include <thread>

#include <gtest/gtest.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <rmcs_msgs/msg/location_health.hpp>

#include "helpers.hpp"
#include "server/health_monitor.hpp"

using namespace rmcs::location;
using LocationHealthMsg = rmcs_msgs::msg::LocationHealth;
namespace test_helpers = rmcs::location::test;

namespace {

auto default_config() -> HealthRuntimeConfig {
    auto config = HealthRuntimeConfig{};
    config.warn_threshold_m = 0.25;
    config.lost_threshold_m = 0.45;
    config.min_inlier_ratio = 0.30;
    config.warn_dwell_sec = 0.0;
    config.lost_dwell_sec = 0.0;
    config.recover_dwell_sec = 0.0;
    config.recover_margin_m = 0.05;
    config.inlier_distance_m = 1.0;
    config.sample_points = 100;
    return config;
}

auto build_test_kdtree() -> pcl::KdTreeFLANN<Point> {
    auto cloud = test_helpers::create_cube_cloud(10.0F, 0.5F);
    auto kdtree = pcl::KdTreeFLANN<Point>{};
    kdtree.setInputCloud(cloud);
    return kdtree;
}

auto stamp_now() -> rclcpp::Time {
    return rclcpp::Time{1, 0, RCL_ROS_TIME};
}

} // namespace

TEST(HealthMonitorTest, StayHealthyWhenResidualLow) {
    auto monitor = HealthMonitor{default_config()};
    auto kdtree = build_test_kdtree();
    auto map_cloud = test_helpers::create_cube_cloud(10.0F, 0.5F);

    monitor.ingest_cloud_odom(map_cloud);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    const auto health = monitor.evaluate(
        kdtree, true, Eigen::Isometry3f::Identity(), stamp_now());
    EXPECT_EQ(health.state, LocationHealthMsg::STATE_HEALTHY);
    EXPECT_LT(health.residual_median_m, 0.25F);
}

TEST(HealthMonitorTest, TransitionToWarning) {
    auto monitor = HealthMonitor{default_config()};
    auto kdtree = build_test_kdtree();
    auto cloud = test_helpers::create_cube_cloud(10.0F, 0.5F);
    auto shifted = test_helpers::apply_transform(*cloud, -5.0F, -5.0F, 0.0F, 0.0F);

    monitor.ingest_cloud_odom(shifted);
    const auto health = monitor.evaluate(
        kdtree, true, Eigen::Isometry3f::Identity(), stamp_now());
    EXPECT_EQ(health.state, LocationHealthMsg::STATE_WARNING);
}

TEST(HealthMonitorTest, WarningToUnhealthy) {
    auto monitor = HealthMonitor{default_config()};
    auto kdtree = build_test_kdtree();
    auto cloud = test_helpers::create_cube_cloud(10.0F, 0.5F);
    auto far = test_helpers::apply_transform(*cloud, -3.0F, -3.0F, 0.0F, 0.0F);
    auto very_far = test_helpers::apply_transform(*cloud, -10.0F, -10.0F, 0.0F, 0.0F);

    monitor.ingest_cloud_odom(far);
    auto health = monitor.evaluate(
        kdtree, true, Eigen::Isometry3f::Identity(), stamp_now());
    ASSERT_EQ(health.state, LocationHealthMsg::STATE_WARNING);

    monitor.ingest_cloud_odom(very_far);
    health = monitor.evaluate(
        kdtree, true, Eigen::Isometry3f::Identity(), stamp_now());
    EXPECT_EQ(health.state, LocationHealthMsg::STATE_UNHEALTHY);
}

TEST(HealthMonitorTest, WarningRecoverToHealthy) {
    auto monitor = HealthMonitor{default_config()};
    auto kdtree = build_test_kdtree();
    auto cloud = test_helpers::create_cube_cloud(10.0F, 0.5F);
    auto far = test_helpers::apply_transform(*cloud, -3.0F, -3.0F, 0.0F, 0.0F);

    monitor.ingest_cloud_odom(far);
    auto health = monitor.evaluate(
        kdtree, true, Eigen::Isometry3f::Identity(), stamp_now());
    ASSERT_EQ(health.state, LocationHealthMsg::STATE_WARNING);

    monitor.ingest_cloud_odom(cloud);
    health = monitor.evaluate(
        kdtree, true, Eigen::Isometry3f::Identity(), stamp_now());
    EXPECT_EQ(health.state, LocationHealthMsg::STATE_HEALTHY);
}

TEST(HealthMonitorTest, UnhealthyRecoverToWarning) {
    auto monitor = HealthMonitor{default_config()};
    auto kdtree = build_test_kdtree();
    auto cloud = test_helpers::create_cube_cloud(10.0F, 0.5F);
    auto very_far = test_helpers::apply_transform(*cloud, -10.0F, -10.0F, 0.0F, 0.0F);

    monitor.ingest_cloud_odom(very_far);
    monitor.evaluate(kdtree, true, Eigen::Isometry3f::Identity(), stamp_now());
    auto health = monitor.evaluate(
        kdtree, true, Eigen::Isometry3f::Identity(), stamp_now());
    ASSERT_EQ(health.state, LocationHealthMsg::STATE_UNHEALTHY);

    monitor.ingest_cloud_odom(cloud);
    health = monitor.evaluate(
        kdtree, true, Eigen::Isometry3f::Identity(), stamp_now());
    EXPECT_EQ(health.state, LocationHealthMsg::STATE_WARNING);
}

TEST(HealthMonitorTest, EmptyCloudDoesNotChangeState) {
    auto monitor = HealthMonitor{default_config()};
    auto kdtree = build_test_kdtree();

    const auto health = monitor.evaluate(
        kdtree, true, Eigen::Isometry3f::Identity(), stamp_now());
    EXPECT_EQ(health.state, LocationHealthMsg::STATE_HEALTHY);
}

TEST(HealthMonitorTest, KdtreeNotReadyDoesNotChangeState) {
    auto monitor = HealthMonitor{default_config()};
    auto kdtree = build_test_kdtree();
    auto cloud = test_helpers::create_cube_cloud(10.0F, 0.5F);
    monitor.ingest_cloud_odom(cloud);

    const auto health = monitor.evaluate(
        kdtree, false, Eigen::Isometry3f::Identity(), stamp_now());
    EXPECT_EQ(health.state, LocationHealthMsg::STATE_HEALTHY);
}

TEST(HealthMonitorTest, RecoverMarginWorks) {
    auto config = default_config();
    config.warn_threshold_m = 0.15;
    config.recover_margin_m = 0.10;
    config.lost_threshold_m = 1.0;

    auto monitor = HealthMonitor{config};
    auto map_cloud = test_helpers::create_random_cloud(1000, 100.0F);
    auto kdtree = pcl::KdTreeFLANN<Point>{};
    kdtree.setInputCloud(map_cloud);

    auto warning_cloud = test_helpers::apply_transform(*map_cloud, -0.30F, 0.0F, 0.0F, 0.0F);
    monitor.ingest_cloud_odom(warning_cloud);
    auto health = monitor.evaluate(
        kdtree, true, Eigen::Isometry3f::Identity(), stamp_now());
    ASSERT_EQ(health.state, LocationHealthMsg::STATE_WARNING);

    auto between_cloud = test_helpers::apply_transform(*map_cloud, -0.08F, 0.0F, 0.0F, 0.0F);
    monitor.ingest_cloud_odom(between_cloud);
    health = monitor.evaluate(
        kdtree, true, Eigen::Isometry3f::Identity(), stamp_now());
    EXPECT_EQ(health.state, LocationHealthMsg::STATE_WARNING);
}

TEST(HealthMonitorTest, EvaluateReturnsValidMetrics) {
    auto monitor = HealthMonitor{default_config()};
    auto kdtree = build_test_kdtree();
    auto cloud = test_helpers::create_cube_cloud(10.0F, 0.5F);
    monitor.ingest_cloud_odom(cloud);

    const auto health = monitor.evaluate(
        kdtree, true, Eigen::Isometry3f::Identity(), stamp_now());
    EXPECT_TRUE(std::isfinite(health.residual_median_m));
    EXPECT_GE(health.inlier_ratio, 0.0F);
    EXPECT_LE(health.inlier_ratio, 1.0F);
    EXPECT_GT(rclcpp::Time{health.stamp}.nanoseconds(), 0);
}

TEST(HealthMonitorTest, SubsampleLargeCloud) {
    auto config = default_config();
    config.sample_points = 50;

    auto monitor = HealthMonitor{config};
    auto kdtree = build_test_kdtree();
    auto big_cloud = test_helpers::create_random_cloud(1000, 5.0F);
    monitor.ingest_cloud_odom(big_cloud);

    const auto health = monitor.evaluate(
        kdtree, true, Eigen::Isometry3f::Identity(), stamp_now());
    EXPECT_LE(health.inlier_ratio, 1.0F);
    EXPECT_GE(health.inlier_ratio, 0.0F);
}
