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

/// 测试默认配置：z 区间放宽到 [-10, 10] 以避免单元测试受高度过滤影响。
/// z 过滤本身有专门的用例覆盖。
auto default_config() -> ScanContextConfig {
    return ScanContextConfig{
        .num_rings = 20,
        .num_sectors = 60,
        .max_radius_m = 20.0,
        .z_min_m = -10.0,
        .z_max_m = 10.0,
    };
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
    EXPECT_EQ(desc.channel_count, SC_CHANNEL_COUNT);
    EXPECT_EQ(desc.data.rows(), config.num_rings * SC_CHANNEL_COUNT);
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
    // 只有 1 个有效 cell，且 cell 内单点（z-range == 0），故 ch0/ch1 非零、ch2 = 0。
    const auto rings = desc.num_rings;
    EXPECT_GT(desc.data.topRows(rings).maxCoeff(), 0.0F);  // ch0
    EXPECT_GT(desc.data.block(rings, 0, rings, desc.data.cols()).maxCoeff(), 0.0F);  // ch1
    // 整体非零 cell 数与通道数一致（每个有效通道 1 个非零点）
    auto nonzero_count = 0;
    for (auto r = 0; r < desc.data.rows(); ++r)
        for (auto c = 0; c < desc.data.cols(); ++c)
            if (desc.data(r, c) > 0.0F)
                ++nonzero_count;
    EXPECT_EQ(nonzero_count, 2);
}

TEST(ScanContextTest, BuildDescriptorMaxHeightPerCell) {
    const auto config = default_config();  // z_min=-10, z_max=10 → z_span=20
    PointCloud cloud;
    cloud.push_back({5.0F, 0.0F, 1.0F});
    cloud.push_back({5.0F, 0.0F, 3.0F});  // 该 cell 内 max_z=3, min_z=1
    cloud.push_back({5.0F, 0.0F, 2.0F});

    const auto desc = build_descriptor(cloud, config);
    // ch0 = (3 - (-10)) / 20 = 0.65
    // ch2 = (3 - 1) / 20 = 0.10
    // 通道 0 应该是 cell 内 max_z 的归一化版本，且严格大于通道 2
    const auto rings = desc.num_rings;
    auto ch0_max = desc.data.topRows(rings).maxCoeff();
    auto ch2_max = desc.data.bottomRows(rings).maxCoeff();
    EXPECT_NEAR(ch0_max, (3.0F + 10.0F) / 20.0F, 1e-5F);
    EXPECT_NEAR(ch2_max, (3.0F - 1.0F) / 20.0F, 1e-5F);
    EXPECT_GT(ch0_max, ch2_max);
}

TEST(ScanContextTest, BuildDescriptorZFilter) {
    auto config = default_config();
    config.z_min_m = 0.15;
    config.z_max_m = 2.0;

    PointCloud cloud;
    cloud.push_back({5.0F, 0.0F, 0.05F});  // 低于 z_min，过滤
    cloud.push_back({5.0F, 0.0F, 1.0F});   // 通过
    cloud.push_back({5.0F, 0.0F, 5.0F});   // 高于 z_max，过滤

    const auto desc = build_descriptor(cloud, config);
    const auto rings = desc.num_rings;
    // 只保留 z=1.0 的点，ch0 = (1.0 - 0.15) / 1.85
    const auto expected_ch0 = (1.0F - 0.15F) / 1.85F;
    EXPECT_NEAR(desc.data.topRows(rings).maxCoeff(), expected_ch0, 1e-5F);
    // ch1 = log2(1+1)/log2(65) = 1/6.022 ≈ 0.166
    const auto ch1_max = desc.data.block(rings, 0, rings, desc.data.cols()).maxCoeff();
    EXPECT_NEAR(ch1_max, std::log2(2.0F) / std::log2(65.0F), 1e-5F);
    // ch2 = 0（只有 1 个点，max_z == min_z）
    EXPECT_FLOAT_EQ(desc.data.bottomRows(rings).maxCoeff(), 0.0F);
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
