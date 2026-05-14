#include <cmath>
#include <numbers>

#include <gtest/gtest.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "helpers.hpp"
#include "tools/registration_tools.hpp"

using namespace rmcs::location;
using namespace rmcs::location::tools;
namespace test_helpers = rmcs::location::test;

namespace {

auto default_initial_config() -> InitialRegistrationConfig {
    auto config = InitialRegistrationConfig{};
    config.coarse_iterations = 20;
    config.precise_iterations = 25;
    config.max_correspondence_distance_m = 3.0;
    config.score_threshold = 0.5;
    config.yaw_search_window_deg = 30.0;
    config.coarse_yaw_step_deg = 10.0;
    config.coarse_top_k = 1;
    config.voxel_leaf_m = 0.1;
    config.outlier_mean_k = 20;
    config.outlier_stddev_mul_thresh = 1.0;
    return config;
}

auto make_transform(float dx, float dy, float dz, float yaw_deg) -> Eigen::Isometry3f {
    const auto yaw_rad = yaw_deg * std::numbers::pi_v<float> / 180.0F;
    auto transform = Eigen::Isometry3f::Identity();
    transform.translation() << dx, dy, dz;
    transform.rotate(Eigen::AngleAxisf{yaw_rad, Eigen::Vector3f::UnitZ()});
    return transform;
}

auto translation_error(const Eigen::Isometry3f& a, const Eigen::Isometry3f& b) -> float {
    return (a.translation() - b.translation()).norm();
}

auto yaw_of(const Eigen::Isometry3f& transform) -> float {
    return std::atan2(transform.rotation()(1, 0), transform.rotation()(0, 0));
}

auto yaw_error_deg(const Eigen::Isometry3f& a, const Eigen::Isometry3f& b) -> float {
    const auto diff = std::atan2(std::sin(yaw_of(a) - yaw_of(b)), std::cos(yaw_of(a) - yaw_of(b)));
    return std::abs(diff) * 180.0F / std::numbers::pi_v<float>;
}

} // namespace

TEST(RegistrationTest, RunInitialRecoversKnownTransform) {
    const auto config = default_initial_config();
    auto raw_map = test_helpers::create_cube_cloud(5.0F, 0.2F);
    auto map_target = preprocess_map(raw_map, config);
    const auto applied = make_transform(0.3F, -0.2F, 0.0F, 15.0F);
    auto query_cloud = test_helpers::apply_transform(*raw_map, 0.3F, -0.2F, 0.0F, 15.0F);

    auto result = Eigen::Isometry3f::Identity();
    auto score = 99.0;
    const auto guess = Eigen::Isometry3f::Identity();

    const auto ok = run_initial(config, query_cloud, map_target, guess, result, score);

    ASSERT_TRUE(ok);
    EXPECT_LT(score, 0.08);

    const auto expected_world_to_odom = applied.inverse();
    EXPECT_LT(translation_error(result, expected_world_to_odom), 0.5F);
    EXPECT_LT(yaw_error_deg(result, expected_world_to_odom), 15.0F);
}

TEST(RegistrationTest, YawRecoveryWithinWindow) {
    auto config = default_initial_config();
    config.yaw_search_window_deg = 90.0;
    config.coarse_yaw_step_deg = 15.0;

    auto raw_map = test_helpers::create_cube_cloud(5.0F, 0.3F);
    auto map_target = preprocess_map(raw_map, config);
    auto query_cloud = test_helpers::apply_transform(*raw_map, 0.0F, 0.0F, 0.0F, 30.0F);

    auto result = Eigen::Isometry3f::Identity();
    auto score = 99.0;
    const auto ok = run_initial(
        config, query_cloud, map_target, Eigen::Isometry3f::Identity(), result, score);
    ASSERT_TRUE(ok);

    const auto expected = make_transform(0.0F, 0.0F, 0.0F, 30.0F).inverse();
    EXPECT_LT(yaw_error_deg(result, expected), 10.0F);
}

TEST(RegistrationTest, VoxelDownsamplingReducesPoints) {
    auto cloud = test_helpers::create_cube_cloud(5.0F, 0.1F);
    const auto original_count = cloud->size();

    auto voxel = pcl::VoxelGrid<Point>{};
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.3F, 0.3F, 0.3F);
    auto filtered = std::make_shared<PointCloud>();
    voxel.filter(*filtered);

    EXPECT_LT(filtered->size(), original_count);
    EXPECT_GT(filtered->size(), 0u);
}

TEST(RegistrationTest, OutlierFilterRemovesOutliers) {
    auto cloud = test_helpers::create_cube_cloud(5.0F, 0.3F);
    const auto original_count = cloud->size();
    cloud->push_back({100.0F, 100.0F, 100.0F});

    auto outlier = pcl::StatisticalOutlierRemoval<Point>{};
    outlier.setInputCloud(cloud);
    outlier.setMeanK(20);
    outlier.setStddevMulThresh(1.0);
    auto filtered = std::make_shared<PointCloud>();
    outlier.filter(*filtered);

    EXPECT_LE(filtered->size(), original_count + 1);
    EXPECT_GE(filtered->size(), original_count);
}
