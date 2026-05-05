/**
 * @file map_descriptor_db.cpp
 * @brief 加载 .sc_desc，提供 SC top-k 查询
 */

#include "server/map_descriptor_db.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <string_view>
#include <vector>

namespace rmcs::location {

namespace {

constexpr auto FILE_MAGIC = std::string_view{"SCDS"};
constexpr std::uint32_t FILE_VERSION = 2;

template <typename T>
auto read_le(std::ifstream& file, T& out) -> bool {
    static_assert(std::is_trivially_copyable_v<T>, "POD type required");
    std::array<char, sizeof(T)> buffer{};
    file.read(buffer.data(), sizeof(T));
    if (!file)
        return false;
    // RMCS 部署在 x86-64（little-endian），直接 memcpy
    std::memcpy(&out, buffer.data(), sizeof(T));
    return true;
}

} // namespace

void MapDescriptorDB::clear() {
    entries_.clear();
    config_ = ScanContextConfig{};
    map_hash_ = 0;
}

auto MapDescriptorDB::load(
    const std::filesystem::path& path, const ScanContextConfig& expected_config,
    std::uint32_t expected_map_hash) -> bool {
    clear();

    if (!std::filesystem::exists(path))
        return false;

    auto file = std::ifstream{path, std::ios::binary};
    if (!file)
        return false;

    auto magic = std::array<char, 4>{};
    file.read(magic.data(), 4);
    if (!file || std::string_view{magic.data(), 4} != FILE_MAGIC)
        return false;

    auto version = std::uint32_t{0};
    if (!read_le(file, version) || version != FILE_VERSION)
        return false;

    auto num_desc = std::uint32_t{0};
    auto num_rings = std::uint32_t{0};
    auto num_sectors = std::uint32_t{0};
    auto max_radius = float{0.0F};
    auto map_hash = std::uint32_t{0};

    if (!read_le(file, num_desc) || !read_le(file, num_rings) || !read_le(file, num_sectors)
        || !read_le(file, max_radius) || !read_le(file, map_hash))
        return false;

    const auto expected_rings = static_cast<std::uint32_t>(expected_config.num_rings);
    const auto expected_sectors = static_cast<std::uint32_t>(expected_config.num_sectors);
    const auto expected_radius = static_cast<float>(expected_config.max_radius_m);

    if (num_rings != expected_rings || num_sectors != expected_sectors
        || std::abs(max_radius - expected_radius) > 1e-3F || map_hash != expected_map_hash)
        return false;

    const auto cells = static_cast<std::size_t>(num_rings) * static_cast<std::size_t>(num_sectors);
    auto buffer = std::vector<float>(cells);

    entries_.reserve(num_desc);
    for (auto i = std::uint32_t{0}; i < num_desc; ++i) {
        Entry entry;
        auto xyz = std::array<float, 3>{};
        file.read(reinterpret_cast<char*>(xyz.data()), 12);
        if (!file)
            return false;
        entry.world_position = Eigen::Vector3f{xyz[0], xyz[1], xyz[2]};

        entry.descriptor.num_rings = static_cast<int>(num_rings);
        entry.descriptor.num_sectors = static_cast<int>(num_sectors);
        entry.descriptor.data.resize(num_rings, num_sectors);

        file.read(reinterpret_cast<char*>(buffer.data()), static_cast<std::streamsize>(cells * 4));
        if (!file)
            return false;

        for (std::uint32_t r = 0; r < num_rings; ++r) {
            for (std::uint32_t s = 0; s < num_sectors; ++s) {
                entry.descriptor.data(static_cast<int>(r), static_cast<int>(s)) =
                    buffer[static_cast<std::size_t>(r) * num_sectors + s];
            }
        }

        entries_.push_back(std::move(entry));
    }

    config_ = expected_config;
    map_hash_ = map_hash;
    return true;
}

auto MapDescriptorDB::query(const ScanContextDescriptor& descriptor, std::size_t top_k) const
    -> std::vector<ScanContextMatch> {
    auto matches = std::vector<ScanContextMatch>{};
    if (entries_.empty() || top_k == 0)
        return matches;

    matches.reserve(entries_.size());
    const auto num_sectors = std::max(1, config_.num_sectors);

    for (const auto& entry : entries_) {
        const auto sd = tools::best_shifted_distance(descriptor, entry.descriptor);
        matches.push_back(ScanContextMatch{
            .world_position = entry.world_position,
            .yaw_deg = static_cast<double>(sd.shift) * 360.0 / static_cast<double>(num_sectors),
            .sc_score = sd.distance,
        });
    }

    const auto k = std::min(top_k, matches.size());
    std::partial_sort(
        matches.begin(), matches.begin() + static_cast<std::ptrdiff_t>(k), matches.end(),
        [](const auto& a, const auto& b) { return a.sc_score < b.sc_score; });

    if (matches.size() > k)
        matches.resize(k);
    return matches;
}

} // namespace rmcs::location
