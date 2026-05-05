#pragma once

#include "tools/registration_tools.hpp"

#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Core>

namespace rmcs::location {

/**
 * @brief ScanContext 描述子配置
 */
struct ScanContextConfig {
    int num_rings = 20;
    int num_sectors = 60;
    double max_radius_m = 20.0;
};

/**
 * @brief 2D 极坐标描述子矩阵 (num_rings × num_sectors)
 *
 * 每个 cell 存放该 (ring, sector) 内点的最大高度 z；空 cell 为 0。
 * row=ring（半径方向，0=最近），col=sector（角度方向 atan2(y,x), [0,2π) 等分，CCW 增长）。
 */
struct ScanContextDescriptor {
    int num_rings = 0;
    int num_sectors = 0;
    Eigen::MatrixXf data;
};

/**
 * @brief 一个 SC 候选种子（来自 top-k 查询或 fallback 生成）
 *
 * - world_position: 该候选在世界系下的位置（来自 map descriptor 关联的 grid 中心）
 * - yaw_deg:        SC 旋转匹配恢复的相对 yaw（query 相对 map 的 +CCW 偏移）
 * - sc_score:       SC 距离（越小越好）
 */
struct ScanContextMatch {
    Eigen::Vector3f world_position = Eigen::Vector3f::Zero();
    double yaw_deg = 0.0;
    double sc_score = std::numeric_limits<double>::infinity();
};

} // namespace rmcs::location

namespace rmcs::location::tools {

/**
 * @brief 由点云构造 ScanContext 描述子
 *
 * 输入点云在描述子的局部坐标系下（即原点 = 描述子参考位置）。
 */
auto build_descriptor(const PointCloud& cloud, const ScanContextConfig& config)
    -> ScanContextDescriptor;

struct ShiftedDistance {
    double distance = std::numeric_limits<double>::infinity();
    int shift = 0;
};

/**
 * @brief 旋转不变 SC 距离：对所有 sector shift 计算列均值 cosine 距离，取最小
 *
 * yaw_deg = shift * 360 / num_sectors 即 query 相对 target 的 +CCW 旋转。
 */
auto best_shifted_distance(
    const ScanContextDescriptor& query, const ScanContextDescriptor& target) -> ShiftedDistance;

/**
 * @brief 计算地图哈希（FNV-1a 32-bit），与 Python generator 严格同构
 *
 * 采样规则（scancontext_plan.md §4.2）：
 *   n = min(10000, N)
 *   if n == 1:  idx = 0
 *   else:       idx_i = floor(i * (N - 1) / (n - 1)),  i ∈ [0, n-1]
 * 每点按 float32 little-endian 顺序写入字节流 (x, y, z)，
 * FNV-1a 32-bit 在该字节流上滚动。
 */
auto compute_map_hash(const PointCloud& map_cloud) -> std::uint32_t;

} // namespace rmcs::location::tools
