#pragma once

#include "tools/scan_context.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

#include <Eigen/Core>

namespace rmcs::location {

/**
 * @brief 离线生成的地图 SC 描述子数据库
 *
 * 文件格式（v2，SCDS）：
 *   header:
 *     magic        char[4]   = "SCDS"
 *     version      uint32_le = 2
 *     num_desc     uint32_le
 *     num_rings    uint32_le
 *     num_sectors  uint32_le
 *     max_radius   float32_le
 *     map_hash     uint32_le  (FNV-1a, 见 ScanContextConfig docs)
 *   for i in [0, num_desc):
 *     world_xyz    float32_le * 3
 *     desc_data    float32_le * (num_rings * num_sectors), row-major
 */
class MapDescriptorDB {
public:
    struct Entry {
        Eigen::Vector3f world_position = Eigen::Vector3f::Zero();
        ScanContextDescriptor descriptor {};
    };

    /**
     * @brief 加载并校验 .sc_desc 文件
     *
     * 任一校验失败（文件不存在、magic/version 错、config 不匹配、map_hash 不匹配、IO 错）
     * 都返回 false，db 内容被清空。调用方应在 false 时降级到 fallback 路径。
     */
    [[nodiscard]] auto load(
        const std::filesystem::path& path, const ScanContextConfig& expected_config,
        std::uint32_t expected_map_hash) -> bool;

    /**
     * @brief 对查询描述子在全库做 SC 距离匹配，返回 top-k 候选（按 sc_score 升序）
     */
    [[nodiscard]] auto query(const ScanContextDescriptor& descriptor, std::size_t top_k) const
        -> std::vector<ScanContextMatch>;

    [[nodiscard]] auto available() const -> bool { return !entries_.empty(); }
    [[nodiscard]] auto config() const -> const ScanContextConfig& { return config_; }
    [[nodiscard]] auto map_hash() const -> std::uint32_t { return map_hash_; }
    [[nodiscard]] auto size() const -> std::size_t { return entries_.size(); }

    void clear();

private:
    std::vector<Entry> entries_;
    ScanContextConfig config_ {};
    std::uint32_t map_hash_ = 0;
};

} // namespace rmcs::location
