# ScanContext 替代 Yaw 暴力搜索 — 工程化落地方案（定稿）

## 0. 目标与已锁定约束

### 目标
- 将主路径从 yaw 暴力搜索迁移到 ScanContext（SC）候选 + TwoStageGicp。
- 在不改动 `rmcs_msgs/srv/Relocalize.srv` 的前提下完成服务端迁移。
- 保证运行时可用性：SC 不可用时自动降级，不中断重定位服务。

### 已锁定约束（必须满足）
1. **fallback 走统一新接口**：不再依赖“旧版 yaw/tier 代码路径”。
2. **`lost_sc_top_k` 参数键固定为 `lost.lost_sc_top_k`**。
3. **`tier_used` 语义弃用并固定为 `0`**（成功/失败都返回 0，避免残留旧语义）。

### 说明
- 本文档中的代码片段仅表达接口与行为约束，实现时可按工程化需要重构。

---

## 1. 架构变更

### 改前
```text
query_cloud -> preprocess -> yaw candidate sweep -> multi-stage GICP -> validate
```

### 改后（主路径）
```text
query_cloud -> build_descriptor -> MapDescriptorDB::query(top_k)
           -> seed candidates (world_pos, yaw_deg, seed_score)
           -> TwoStageGicp per candidate
           -> ranking select best
           -> validate
```

### 改后（统一 fallback）
```text
if SC unavailable/mismatch/no match:
  build fallback seeds from request.initial_guess_world_base (同一套 seed 接口)
  -> TwoStageGicp per candidate
  -> ranking select best
  -> validate (reference = fallback seed)
```

关键点：
- fallback 不是“调用旧逻辑”，而是“用同一套新接口喂不同 seed 来源”。
- `LOCAL/WIDE tier` 概念从实现中移除；保留服务字段 `tier_used` 仅为兼容，恒置 `0`。

---

## 2. 公共接口与数据结构

### 2.1 新增类型

#### `src/tools/scan_context.hpp`
- `ScanContextConfig`
- `ScanContextDescriptor`
- `ScanContextMatch`
- `build_descriptor(...)`
- `best_shifted_distance(...)`

#### `src/server/map_descriptor_db.hpp`
- `MapDescriptorDB::load(path)`
- `MapDescriptorDB::query(desc, top_k)`
- `MapDescriptorDB::config()`
- `MapDescriptorDB::map_hash()`

#### `src/server/validator.hpp`
- 新增 `InitialReference`：
  - `world_position`
  - `yaw_deg`
  - `source` (`scan_context` / `fallback_guess`，仅用于日志)

### 2.2 调整类型（去 tier / 去旧 yaw 配置）

#### `src/tools/registration_tools.hpp`
- `InitialRegistrationConfig` 保留：
  - `coarse_iterations`
  - `precise_iterations`
  - `max_correspondence_distance_m`
  - `score_threshold`
  - `voxel_leaf_m`
  - `outlier_mean_k`
  - `outlier_stddev_mul_thresh`
- `LostRegistrationConfig` 保留：
  - `submap_radius_m`
  - `coarse_iterations`
  - `precise_iterations`
  - `max_correspondence_distance_m`
  - `score_threshold`
  - `enable_map_consistency_filter`
  - `map_consistency_distance_m`
  - `min_retained_fraction`
  - `lost_sc_top_k`
- 删除：`LostTier`、所有 local/wide 参数、旧 yaw 扫描参数、旧 ranking 权重参数。

### 2.3 统一注册函数签名

`run_initial` 与 `run_lost` 均改为“接收 seed 列表”的统一模式：

```cpp
auto run_initial(
    const InitialRegistrationConfig& initial_cfg,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_world_cloud,
    const std::vector<ScanContextMatch>& seed_candidates,
    const Eigen::Isometry3f& odom_to_base,
    Eigen::Isometry3f& world_to_odom_result,
    double& score,
    std::size_t& used_seed_index) -> bool;

auto run_lost(
    const InitialRegistrationConfig& initial_cfg,
    const LostRegistrationConfig& lost_cfg,
    const std::shared_ptr<PointCloud>& query_odom_cloud,
    const std::shared_ptr<PointCloud>& map_world_cloud,
    const pcl::KdTreeFLANN<Point>& map_kdtree,
    const std::vector<ScanContextMatch>& seed_candidates,
    const Eigen::Isometry3f& odom_to_base,
    Eigen::Isometry3f& world_to_odom_result,
    double& score,
    double& inlier_ratio,
    std::size_t& used_seed_index) -> bool;
```

`seed_candidates` 来源可为：
- SC query 结果（主路径）
- fallback 生成结果（降级路径）

---

## 3. 关键算法约束

### 3.1 SC yaw 符号约定（必须一致）
- `sector` 由 `atan2(y, x)` 映射到 `[0, 2π)`，CCW 增长。
- `best_shifted_distance` 比较 `query[s]` 与 `map[(s + shift) % Ns]`。
- `yaw_deg = shift * 360 / Ns`，用于构造 `W_T_B` 的 Z 轴旋转。
- 测试中固定验证：已知 +90° 场景应恢复 +90°（误差 < 1°）。

### 3.2 统一 ranking 规则
对每个 seed 执行 TwoStageGicp 后，计算：

`ranking = w_gicp * gicp_score + w_seed * normalized_seed_score`

默认：
- `w_gicp = 0.8`
- `w_seed = 0.2`
- 对 fallback seed：`seed_score = 0`
- 对 SC seed：`seed_score = sc_match.score`，按候选集合内 min-max 归一化到 `[0,1]`

说明：
- ranking 权重为实现常量，不暴露回 YAML，避免重新引入大规模调参面。

### 3.3 fallback seed 生成（统一新接口）

#### INITIAL fallback
- 参考位姿：`request.initial_guess_world_base`
- 位置 seed：仅初始位置
- yaw seed 集：`[0, +30, -30, +60, -60, +90, -90]`（deg）

#### LOST fallback
- 参考位姿：`request.initial_guess_world_base`
- 位置 seed：十字 5 点
  - `(0,0)`, `(+1.5,0)`, `(-1.5,0)`, `(0,+1.5)`, `(0,-1.5)`（m）
- yaw seed 集：`[0, +45, -45, +90, -90, +135, -135, 180]`（deg）
- 最终 seed = 位置×yaw 笛卡尔积（固定上限 40 个）

说明：
- 该 fallback 仅用于可用性保障，不追求与旧 LOCAL/WIDE 等价。

---

## 4. 地图-描述子一致性（v2 格式）

### 4.1 `.sc_desc` 文件头
- magic: `"SCDS"`
- version: `2`
- `num_desc`
- `num_rings`
- `num_sectors`
- `max_radius`
- `map_hash`（uint32, FNV-1a）

### 4.2 hash 统一规范（Python/C++ 必须完全同构）
- 点序采样数量：`n = min(10000, N)`
- 索引公式：
  - 若 `n == 1`，取 `idx = 0`
  - 若 `n > 1`，`idx_i = floor(i * (N - 1) / (n - 1))`, `i in [0, n-1]`
- 每个点按 `float32 little-endian` 顺序写入 `(x, y, z)` 字节流。
- 在该字节流上计算 FNV-1a 32-bit。

### 4.3 运行时策略
- `map_hash` 匹配：启用 SC。
- `map_hash` 不匹配 / 文件缺失 / 解析失败：SC 置为 unavailable，自动走 fallback seed 路径。
- 不因为 SC 问题直接返回服务失败。

---

## 5. runtime/validator 行为

### 5.1 `src/server/runtime.cpp`
- 新增成员：
  - `MapDescriptorDB map_descriptor_db_`
  - `ScanContextConfig scan_context_config_`
  - `bool scan_context_available_`
  - `uint32_t map_hash_`
- `load_parameters()`：从 `RuntimeParamsBundle` 读取 `scan_context_config`。
- `load_map()`：计算 `map_hash_`，加载并校验 `.sc_desc`。
- `handle_initial_relocalize()`：
  - 优先 SC seed；失败则 fallback seeds。
  - 调用统一 `run_initial`。
  - 根据 `used_seed_index` 构造 `InitialReference`。
  - `tier_used = 0`。
- `handle_lost_relocalize()`：
  - 优先 SC seeds (`top_k = lost.lost_sc_top_k`)；失败则 fallback seeds。
  - 调用统一 `run_lost`。
  - 仍使用 `request.initial_guess_world_base` 构造 `LostPrior` 做验收。
  - `tier_used = 0`。
- `reset_response()` 将 `tier_used` 默认值改为 `0`（不再使用 `255`）。

### 5.2 `src/server/validator.hpp/.cpp`

#### InitialValidationConfig（泛化 reference）
- `score_threshold`
- `max_translation_from_reference_m`
- `max_yaw_from_reference_deg`
- `field_bounds`

#### evaluate_initial 签名
```cpp
auto evaluate_initial(
    const InitialReference& reference,
    const Eigen::Isometry3f& world_to_base_estimated,
    double score) const -> ValidationResult;
```

说明：
- SC 路径：reference 取 SC 最终采用 seed。
- fallback 路径：reference 取 fallback 最终采用 seed（通常接近 request guess）。

#### LostValidationConfig
- 只保留单值阈值：`score_threshold`, `min_inlier_ratio`, `max_distance_from_prior_m`, `max_yaw_from_prior_deg`。

---

## 6. 参数与配置迁移

### 6.1 `config/location.yaml`
- 删除旧 yaw/tier/local-wide 参数。
- 新增/保留：

```yaml
scan_context:
  num_rings: 20
  num_sectors: 60
  max_radius_m: 20.0

lost:
  lost_sc_top_k: 3
```

### 6.2 `src/tools/param_tools.hpp/.cpp`
- `RuntimeParamsBundle` 新增：
  - `ScanContextConfig scan_context_config {};`
- `load_runtime_params()` 读取：
  - `scan_context.num_rings`
  - `scan_context.num_sectors`
  - `scan_context.max_radius_m`
  - `lost.lost_sc_top_k`
- 保留对旧字段的兼容读取仅用于日志告警（可选）：
  - 若检测到旧 `lost.*local* / *wide* / *yaw*` 参数，打印 deprecated warn。

---

## 7. 构建系统改动

### 7.1 `src/tools/CMakeLists.txt`
- `rmcs-relocation_tools_common` 增加 `scan_context.cpp`。

### 7.2 根 `CMakeLists.txt`
- `rmcs-relocation_server` 增加 `src/server/map_descriptor_db.cpp`。

### 7.3 安装脚本
- 安装 `scripts/generate_map_descriptors.py`。

---

## 8. 测试计划（可执行）

### 8.1 新增单元测试

#### `test_scan_context.cpp`
1. `BuildDescriptorValid`
2. `InvariantToTranslation`
3. `YawShift`
4. `EmptySectors`
5. `RadiusClipping`
6. `YawEndToEnd`
7. `BuildDescriptorDeterministic`
8. `EmptyCloud`

#### `test_map_descriptor_db.cpp`
1. `LoadValid`
2. `LoadWrongMagic`
3. `LoadWrongVersion`
4. `LoadTruncated`
5. `MapHashReadback`
6. `QueryMultipleCandidates`

#### `test_registration.cpp`
1. `RunInitialRecoversKnownTransformWithScSeed`
2. `RunInitialFallbackSeedsWork`
3. `RunLostMultiCandidate`
4. `RunLostFallbackSeedsWork`
5. 保留原有预处理相关测试（voxel/outlier/submap）

#### `test_validator.cpp`
1. `InitialAcceptedWithScReference`
2. `InitialAcceptedWithFallbackReference`
3. `LostRejectedLowInlier`
4. `ConfidenceInRange`

### 8.2 集成验证（手动）
1. `.sc_desc` 正常：走 SC 主路径，日志包含 `scan_context_available=true`。
2. `.sc_desc` 缺失：自动 fallback，服务仍可成功。
3. `.sc_desc` hash mismatch：告警 + fallback，服务仍可成功。
4. `tier_used`：所有响应为 `0`。

---

## 9. 实施步骤与估时

1. `scan_context.hpp/cpp` + 测试：2h
2. `map_descriptor_db.hpp/cpp` + hash 规范落地：2h
3. `registration_tools.hpp/cpp` 统一 seed 接口：3h
4. `validator.hpp/cpp` 引入 `InitialReference`：1h
5. `param_tools.hpp/cpp` 参数迁移：1h
6. `runtime.cpp` 接入 SC + fallback + tier_used 固定：2h
7. `CMakeLists` 与脚本安装：0.5h
8. 新增/更新测试：2h
9. 编译 + 测试 + 回归：1h

总计：约 14.5h（2 个工作日内可完成）

---

## 10. 兼容性与风险

### 兼容性
- `Relocalize.srv` 不改字段。
- `tier_used` 字段保留但弃用，固定返回 `0`。
- `prior_sigma_xy_m/prior_sigma_yaw_deg` 请求字段保留但不再驱动 tier 分支（可在日志中标记 deprecated）。

### 主要风险与对策
1. SC yaw 方向符号错误
- 对策：`YawEndToEnd` 测试 + 小规模实车回放。

2. Python/C++ hash 不一致导致误降级
- 对策：统一索引公式与字节序，增加 hash 对拍测试。

3. fallback 质量低于旧版
- 对策：保留固定多 seed fallback；必要时仅调固定 seed 常量，不回滚旧 tier 体系。

---

## 11. 验收标准

满足以下全部条件即验收通过：
1. `colcon test --packages-select rmcs-relocation` 全通过。
2. SC 可用时，INITIAL/LOST 请求可成功返回并更新 `world->odom`。
3. SC 不可用（缺文件/坏文件/hash mismatch）时，INITIAL/LOST 仍可通过 fallback 完成重定位。
4. 所有响应 `tier_used == 0`。
5. 配置中仅使用 `lost.lost_sc_top_k`，不存在新引入的 local/wide/tier/yaw 搜索参数。
