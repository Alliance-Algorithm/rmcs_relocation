#include <cmath>
#include <cstdint>
#include <numbers>

#include <gtest/gtest.h>

#include "helpers.hpp"
#include "tools/scan_context.hpp"

using namespace rmcs::location;
using namespace rmcs::location::tools;
namespace test_helpers = rmcs::location::test;

namespace {

auto default_config() -> ScanContextConfig {
    return ScanContextConfig{.num_rings = 20, .num_sectors = 60, .max_radius_m = 20.0};
}

/// 构造一个明显非对称的点云（避免立方体的 4 次旋转对称性导致 yaw 多解）
auto create_asymmetric_cloud() -> std::shared_ptr<test_helpers::PointCloud> {
    auto cloud = std::make_shared<test_helpers::PointCloud>();
    // 在不同半径 / 角度 / 高度放置一组非对称特征
    const auto features = std::vector<std::tuple<float, float, float>>{
        { 6.0F, 0.0F, 1.0F },   // 0° 方向，高度 1
        { 0.0F, 4.5F, 2.0F },   // 90° 方向，高度 2
        { -3.0F, 0.0F, 0.5F },  // 180° 方向，高度 0.5
        { 0.0F, -7.0F, 3.0F },  // 270° 方向，高度 3
        { 5.0F, 5.0F, 1.5F },   // 45° 方向，高度 1.5
        { -2.0F, -2.0F, 0.8F }, // 225° 方向，高度 0.8
        { 8.0F, 2.0F, 2.5F },
        { 1.5F, 8.5F, 1.2F },
    };
    for (const auto& [x, y, z] : features) {
        // 每个特征拓宽成小簇，提升 SC 单元命中率
        for (auto dx = -0.3F; dx <= 0.3F; dx += 0.15F)
            for (auto dy = -0.3F; dy <= 0.3F; dy += 0.15F)
                cloud->push_back({ x + dx, y + dy, z });
    }
    return cloud;
}

} // namespace

TEST(ScanContextTest, BuildDescriptorEmptyCloud) {
    const auto config = default_config();
    PointCloud empty;
    const auto desc = build_descriptor(empty, config);
    EXPECT_EQ(desc.num_rings, config.num_rings);
    EXPECT_EQ(desc.num_sectors, config.num_sectors);
    EXPECT_EQ(desc.data.rows(), config.num_rings);
    EXPECT_EQ(desc.data.cols(), config.num_sectors);
    EXPECT_FLOAT_EQ(desc.data.maxCoeff(), 0.0F);
}

TEST(ScanContextTest, BuildDescriptorRadiusClipping) {
    const auto config = default_config();
    PointCloud cloud;
    // 远超 max_radius 的点应被忽略
    cloud.push_back({100.0F, 0.0F, 5.0F});
    cloud.push_back({0.0F, 100.0F, 5.0F});
    // 半径合法的点应入 cell
    cloud.push_back({5.0F, 0.0F, 2.0F});

    const auto desc = build_descriptor(cloud, config);
    EXPECT_FLOAT_EQ(desc.data.maxCoeff(), 2.0F);
}

TEST(ScanContextTest, BuildDescriptorMaxHeightPerCell) {
    const auto config = default_config();
    PointCloud cloud;
    cloud.push_back({5.0F, 0.0F, 1.0F});
    cloud.push_back({5.0F, 0.0F, 3.0F});
    cloud.push_back({5.0F, 0.0F, 2.0F});

    const auto desc = build_descriptor(cloud, config);
    EXPECT_FLOAT_EQ(desc.data.maxCoeff(), 3.0F);
}

TEST(ScanContextTest, YawShiftRecoveryPositive90) {
    const auto config = default_config();
    auto cloud = create_asymmetric_cloud();

    // desc_a：地图描述子（世界系）
    const auto desc_a = build_descriptor(*cloud, config);
    // 模拟机体相对地图 +90° 旋转：世界点在机体系下相对世界系旋转了 -90°
    auto body_view = test_helpers::apply_transform(*cloud, 0.0F, 0.0F, 0.0F, -90.0F);
    const auto desc_b = build_descriptor(*body_view, config);

    const auto sd = best_shifted_distance(desc_b, desc_a);
    const auto recovered_yaw = static_cast<double>(sd.shift) * 360.0 / config.num_sectors;
    EXPECT_NEAR(recovered_yaw, 90.0, 360.0 / config.num_sectors + 1e-3);
    EXPECT_LT(sd.distance, 0.3);
}

TEST(ScanContextTest, YawShiftRecoveryNegative45) {
    const auto config = default_config();
    auto cloud = create_asymmetric_cloud();

    const auto desc_a = build_descriptor(*cloud, config);
    // 模拟机体 -45°：世界点在机体系下旋转 +45°
    auto body_view = test_helpers::apply_transform(*cloud, 0.0F, 0.0F, 0.0F, +45.0F);
    const auto desc_b = build_descriptor(*body_view, config);

    const auto sd = best_shifted_distance(desc_b, desc_a);
    const auto recovered_yaw_deg = static_cast<double>(sd.shift) * 360.0 / config.num_sectors;
    // -45° 等价于 +315°（mod 360）
    const auto wrapped = std::fmod(recovered_yaw_deg + 360.0, 360.0);
    EXPECT_NEAR(wrapped, 315.0, 360.0 / config.num_sectors + 1e-3);
}

TEST(ScanContextTest, IdenticalDescriptorZeroDistanceZeroShift) {
    const auto config = default_config();
    auto cloud = test_helpers::create_cube_cloud(8.0F, 0.5F);
    const auto desc = build_descriptor(*cloud, config);

    const auto sd = best_shifted_distance(desc, desc);
    EXPECT_EQ(sd.shift, 0);
    EXPECT_LT(sd.distance, 1e-6);
}

TEST(ScanContextTest, ComputeMapHashDeterministic) {
    auto cloud = test_helpers::create_cube_cloud(5.0F, 0.5F);
    const auto h1 = compute_map_hash(*cloud);
    const auto h2 = compute_map_hash(*cloud);
    EXPECT_EQ(h1, h2);
    EXPECT_NE(h1, 0u);
}

TEST(ScanContextTest, ComputeMapHashChangesOnPerturbation) {
    auto a = test_helpers::create_cube_cloud(5.0F, 0.5F);
    auto b = test_helpers::create_cube_cloud(5.0F, 0.5F);
    // 改一个点
    if (!b->empty())
        b->points[0].x += 1.0F;
    EXPECT_NE(compute_map_hash(*a), compute_map_hash(*b));
}

TEST(ScanContextTest, ComputeMapHashSinglePoint) {
    PointCloud cloud;
    cloud.push_back({1.5F, -2.25F, 0.75F});
    const auto h = compute_map_hash(cloud);
    EXPECT_NE(h, 0u);
}
