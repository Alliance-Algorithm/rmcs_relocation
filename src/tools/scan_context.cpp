/**
 * @file scan_context.cpp
 * @brief ScanContext 描述子构造、旋转不变匹配、地图哈希
 */

#include "tools/scan_context.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numbers>
#include <vector>

namespace rmcs::location::tools {

namespace {

constexpr std::uint32_t FNV_OFFSET_BASIS = 0x811c9dc5u;
constexpr std::uint32_t FNV_PRIME = 0x01000193u;

inline auto fnv_byte(std::uint32_t hash, std::uint8_t byte) -> std::uint32_t {
    return (hash ^ static_cast<std::uint32_t>(byte)) * FNV_PRIME;
}

inline auto fnv_float_le(std::uint32_t hash, float value) -> std::uint32_t {
    std::uint8_t bytes[4];
    static_assert(sizeof(float) == 4, "expect 32-bit float");
    std::memcpy(bytes, &value, 4);
    // RMCS 部署在 x86-64（little-endian）。memcpy 已得 LE 顺序。
    hash = fnv_byte(hash, bytes[0]);
    hash = fnv_byte(hash, bytes[1]);
    hash = fnv_byte(hash, bytes[2]);
    hash = fnv_byte(hash, bytes[3]);
    return hash;
}

constexpr std::size_t HASH_MAX_SAMPLES = 10000;

} // namespace

namespace {

/// log2(1 + 64) ≈ 6.022 — 通道 1 的归一化分母，>=64 个点封顶到 1。
constexpr float DENSITY_LOG_DENOM = 6.022368f;

struct CellAccum {
    float max_z = -std::numeric_limits<float>::infinity();
    float min_z = std::numeric_limits<float>::infinity();
    std::uint32_t count = 0;
};

} // namespace

auto build_descriptor(const PointCloud& cloud, const ScanContextConfig& config)
    -> ScanContextDescriptor {
    auto descriptor = ScanContextDescriptor {};
    descriptor.num_rings = std::max(1, config.num_rings);
    descriptor.num_sectors = std::max(1, config.num_sectors);
    descriptor.channel_count = SC_CHANNEL_COUNT;

    const auto rings = descriptor.num_rings;
    const auto sectors = descriptor.num_sectors;
    const auto channels = descriptor.channel_count;

    descriptor.data = Eigen::MatrixXf::Zero(rings * channels, sectors);

    if (cloud.empty())
        return descriptor;

    const auto max_radius = std::max(1e-3, config.max_radius_m);
    const auto two_pi = 2.0 * std::numbers::pi;
    const auto sector_step = two_pi / static_cast<double>(sectors);
    const auto ring_step_inv = static_cast<double>(rings) / max_radius;

    // z 过滤区间也是 channel 0 / channel 2 的归一化分母。
    const auto z_min = static_cast<float>(config.z_min_m);
    const auto z_max = static_cast<float>(config.z_max_m);
    const auto z_span = std::max(1e-3f, z_max - z_min);

    auto cells = std::vector<CellAccum>(static_cast<std::size_t>(rings) * static_cast<std::size_t>(sectors));

    for (const auto& point : cloud.points) {
        const auto z = static_cast<float>(point.z);
        if (z < z_min || z > z_max)
            continue;

        const auto range = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
        if (range >= max_radius || range < 1e-6)
            continue;

        auto ring = static_cast<int>(range * ring_step_inv);
        if (ring < 0 || ring >= rings)
            continue;

        auto angle = std::atan2(static_cast<double>(point.y), static_cast<double>(point.x));
        if (angle < 0.0)
            angle += two_pi;
        auto sector = static_cast<int>(angle / sector_step);
        if (sector < 0)
            sector = 0;
        if (sector >= sectors)
            sector = sectors - 1;

        auto& cell = cells[static_cast<std::size_t>(ring) * sectors + sector];
        if (z > cell.max_z)
            cell.max_z = z;
        if (z < cell.min_z)
            cell.min_z = z;
        cell.count += 1;
    }

    for (auto r = 0; r < rings; ++r) {
        for (auto s = 0; s < sectors; ++s) {
            const auto& cell = cells[static_cast<std::size_t>(r) * sectors + s];
            if (cell.count == 0)
                continue;

            // ch0: 最高点高度，相对 [z_min, z_max] 归一化
            const auto ch0 = std::clamp((cell.max_z - z_min) / z_span, 0.0f, 1.0f);
            // ch1: log 密度，count>=64 时封顶到 1
            const auto ch1 =
                std::clamp(std::log2(1.0f + static_cast<float>(cell.count)) / DENSITY_LOG_DENOM,
                           0.0f, 1.0f);
            // ch2: cell 内 z 跨度，区分平面（地面/水平面）与立面（柱/箱角）
            const auto ch2 = std::clamp((cell.max_z - cell.min_z) / z_span, 0.0f, 1.0f);

            descriptor.data(r, s) = ch0;
            descriptor.data(rings + r, s) = ch1;
            descriptor.data(2 * rings + r, s) = ch2;
        }
    }

    return descriptor;
}

auto best_shifted_distance(
    const ScanContextDescriptor& query, const ScanContextDescriptor& target) -> ShiftedDistance {
    auto best = ShiftedDistance {};

    if (query.num_rings != target.num_rings || query.num_sectors != target.num_sectors
        || query.channel_count != target.channel_count
        || query.num_sectors <= 0 || query.num_rings <= 0
        || query.data.rows() != target.data.rows() || query.data.cols() != target.data.cols())
        return best;

    const auto num_sectors = query.num_sectors;

    for (auto shift = 0; shift < num_sectors; ++shift) {
        auto sum_distance = 0.0;
        auto valid_columns = 0;

        for (auto column = 0; column < num_sectors; ++column) {
            const auto target_column = (column + shift) % num_sectors;
            const auto query_col = query.data.col(column);
            const auto target_col = target.data.col(target_column);

            const auto query_norm = query_col.norm();
            const auto target_norm = target_col.norm();
            if (query_norm < 1e-6 && target_norm < 1e-6)
                continue;

            // 一方为空但另一方非空：当作最坏（distance = 1）以惩罚结构差异
            if (query_norm < 1e-6 || target_norm < 1e-6) {
                sum_distance += 1.0;
                ++valid_columns;
                continue;
            }

            const auto cosine =
                static_cast<double>(query_col.dot(target_col)) / (query_norm * target_norm);
            sum_distance += std::clamp(1.0 - cosine, 0.0, 2.0);
            ++valid_columns;
        }

        if (valid_columns == 0)
            continue;

        const auto distance = sum_distance / static_cast<double>(valid_columns);
        if (distance < best.distance) {
            best.distance = distance;
            best.shift = shift;
        }
    }

    return best;
}

auto compute_map_hash(const PointCloud& map_cloud) -> std::uint32_t {
    const auto total = map_cloud.points.size();
    if (total == 0)
        return 0;

    const auto sample_count = std::min<std::size_t>(HASH_MAX_SAMPLES, total);

    auto hash = FNV_OFFSET_BASIS;

    if (sample_count == 1) {
        const auto& point = map_cloud.points[0];
        hash = fnv_float_le(hash, point.x);
        hash = fnv_float_le(hash, point.y);
        hash = fnv_float_le(hash, point.z);
        return hash;
    }

    const auto denom = static_cast<double>(sample_count - 1);
    const auto last_index = static_cast<double>(total - 1);
    for (std::size_t i = 0; i < sample_count; ++i) {
        const auto index =
            static_cast<std::size_t>(std::floor(static_cast<double>(i) * last_index / denom));
        const auto& point = map_cloud.points[index];
        hash = fnv_float_le(hash, point.x);
        hash = fnv_float_le(hash, point.y);
        hash = fnv_float_le(hash, point.z);
    }
    return hash;
}

} // namespace rmcs::location::tools
