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

auto build_descriptor(const PointCloud& cloud, const ScanContextConfig& config)
    -> ScanContextDescriptor {
    auto descriptor = ScanContextDescriptor {};
    descriptor.num_rings = std::max(1, config.num_rings);
    descriptor.num_sectors = std::max(1, config.num_sectors);
    descriptor.data = Eigen::MatrixXf::Zero(descriptor.num_rings, descriptor.num_sectors);

    if (cloud.empty())
        return descriptor;

    const auto max_radius = std::max(1e-3, config.max_radius_m);
    const auto two_pi = 2.0 * std::numbers::pi;
    const auto sector_step = two_pi / static_cast<double>(descriptor.num_sectors);
    const auto ring_step_inv =
        static_cast<double>(descriptor.num_rings) / max_radius;

    for (const auto& point : cloud.points) {
        const auto range = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
        if (range >= max_radius || range < 1e-6)
            continue;

        auto ring = static_cast<int>(range * ring_step_inv);
        if (ring < 0 || ring >= descriptor.num_rings)
            continue;

        auto angle =
            std::atan2(static_cast<double>(point.y), static_cast<double>(point.x));
        if (angle < 0.0)
            angle += two_pi;
        auto sector = static_cast<int>(angle / sector_step);
        if (sector < 0)
            sector = 0;
        if (sector >= descriptor.num_sectors)
            sector = descriptor.num_sectors - 1;

        const auto height = point.z;
        if (height > descriptor.data(ring, sector))
            descriptor.data(ring, sector) = height;
    }

    return descriptor;
}

auto best_shifted_distance(
    const ScanContextDescriptor& query, const ScanContextDescriptor& target) -> ShiftedDistance {
    auto best = ShiftedDistance {};

    if (query.num_rings != target.num_rings || query.num_sectors != target.num_sectors
        || query.num_sectors <= 0 || query.num_rings <= 0)
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
