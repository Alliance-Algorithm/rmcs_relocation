# 三模式重定位重构（INITIAL / LOCAL / WIDE）+ WIDE 用 ScanContext

## 0. Context

### 起因

- 现状：`MODE_INITIAL` + `MODE_LOST`(内部按 sigma 选 `LOCAL/WIDE` tier) + `MODE_MANUAL`（已弃用，路由到 INITIAL）
- 问题：
  - tier 是 sigma 反向触发的隐式分支（`engine.cc::relocalize_local/wide` 借 sigma 偷偷选 tier，`select_lost_tier` 在 server 端再解析），调用方意图被埋在数值里
  - WIDE 当前是 `create_wide_seeds`：以 prior 为中心扔 5 seed，**仍依赖 prior**。开局 INITIAL 也强制要 prior。真正的"全局重定位"能力没有
  - 大量 `*_local / *_wide` 双份配置和分支代码（registration_tools / validator / param_tools / yaml），调参面虚高

### 终态目标

| 模式 | 用途 | 实现 | 是否依赖 prior |
|---|---|---|---|
| INITIAL | 开局调用一次 | 现有实现不动 | 是 |
| LOCAL | Lua 随时调，做局部纠正 | 现有 LOST tier=LOCAL 代码搬出独立 handler | 是（强依赖） |
| WIDE | 全局兜底（lost / kidnap / 正常 LOCAL 失败时升档） | **ScanContext 重写**，描述子查询 top-K 候选 | 否（prior 仅用作可选过滤） |

Lua 侧：
- 开局：`action:relocalize("initial", spawn_x, spawn_y, spawn_yaw, 10.0)`
- 周期/事件触发：`action:try_relocalize_with_fallback("local_", 3.0)` —— LIO/TF 断时自动降到 wide+原点
- 主动全局：`action:try_relocalize_with_fallback("wide", 6.0)`

### 与 `scancontext_plan.md` 的关系

本 plan 复用 `docs/scancontext_plan.md` 的 SC 算法层（§1-§7 的 scan_context、map_descriptor_db、TwoStageGicp 接口、hash 规范、generate 脚本），**但只把 SC 应用到 WIDE handler**。LOCAL 保留现有实现。`scancontext_plan.md` 中"删除 LOCAL/WIDE"的论断在本 plan 不成立 —— LOCAL/WIDE 升格为顶层独立模式而非 LOST 内部 tier。

不在范围：
- collector.cpp/hpp（原"运动补偿改 collector"方案在当前 topic 上是 no-op，不动）
- point_lio per-point deskew（独立条线）
- INITIAL handler 实现细节（保持现状）

---

## 1. 服务接口变更

### `rmcs_msgs/srv/Relocalize.srv`

```diff
 uint8 MODE_INITIAL=0
-uint8 MODE_MANUAL=1
-uint8 MODE_LOST=2
+uint8 MODE_LOCAL=1
+uint8 MODE_WIDE=2
 uint8 mode

 geometry_msgs/Pose initial_guess_world_base
-float32 prior_sigma_xy_m
-float32 prior_sigma_yaw_deg
 string pointcloud_topic
 float32 collect_duration_sec
 ---
 bool ok
 string error_message
 geometry_msgs/Pose world_to_odom
-uint8 tier_used
 float32 score
 float32 confidence
```

`prior_sigma_*` 和 `tier_used` 字段也一并删除 —— 不需要"保留兼容"，因为目前唯一调用方是 `engine.cc`，我们一起改。

### `engine.hh / engine.cc`（rmcs-navigation/src/cxx/util/localization）

```diff
 // engine.hh Config
-double lost_local_sigma_xy_m;
-double lost_local_sigma_yaw_deg;
-double lost_wide_sigma_xy_m;
-double lost_wide_sigma_yaw_deg;
```

```diff
 // engine.cc:265-289
-auto make_lost_request(double x, double y, double yaw, double sigma_xy, double sigma_yaw) {
-    request->mode = Request::MODE_LOST;
-    request->prior_sigma_xy_m = sigma_xy;
-    request->prior_sigma_yaw_deg = sigma_yaw;
-    ...
-}
+auto make_request(uint8_t mode, double x, double y, double yaw) { ... }

 auto relocalize_local(double x, double y, double yaw) -> bool {
-    return send(make_lost_request(x, y, yaw, config.lost_local_sigma_xy_m, ...));
+    return send(make_request(Request::MODE_LOCAL, x, y, yaw));
 }
 auto relocalize_wide(double x, double y, double yaw) -> bool {
-    return send(make_lost_request(x, y, yaw, config.lost_wide_sigma_xy_m, ...));
+    return send(make_request(Request::MODE_WIDE, x, y, yaw));
 }
```

### Lua API

`api.lua` / `action.lua` 的 `relocalize_initial / relocalize_local / relocalize_wide` 三函数签名不变，含义直接对应三 mode。零 Lua 业务代码改动（除新增 helper）。

---

## 2. Server-side handler 拆分

### `runtime.cpp` dispatcher

```cpp
switch (request->mode) {
case Request::MODE_INITIAL: handle_initial_relocalize(*request, *response); break;
case Request::MODE_LOCAL:   handle_local_relocalize(*request, *response);   break;
case Request::MODE_WIDE:    handle_wide_relocalize(*request, *response);    break;
default:
    response->ok = false;
    response->error_message = "unknown mode";
    break;
}
```

### `handle_local_relocalize`（搬家，不动逻辑）

把现 `handle_lost_relocalize` 中 tier=LOCAL 路径整段提出来：
- 用 prior 作为单 seed
- `tools::run_local(...)`（从 `run_lost` 重命名 + 简化签名，去掉 tier/seed-list 参数）
- `LocalValidationConfig`（从 `LostValidationConfig` 简化，单值阈值）

### `handle_wide_relocalize`（重写，按 scancontext_plan.md）

```cpp
auto query_desc = scan_context::build_descriptor(*query_cloud, sc_cfg_);
std::vector<ScanContextMatch> seeds;
if (scan_context_available_) {
    seeds = map_descriptor_db_.query(query_desc, wide_cfg_.sc_top_k);
    RCLCPP_INFO(logger_, "wide: SC returned %zu candidates", seeds.size());
} else {
    RCLCPP_WARN(logger_, "wide: SC unavailable, using fallback seeds");
    seeds = build_wide_fallback_seeds(request.initial_guess_world_base, wide_cfg_);
}
auto result = tools::run_wide(initial_cfg_, wide_cfg_, query_cloud, map_world_cloud_,
                              map_kdtree_, seeds, odom_to_base);
// ranking 用 plan §3.2 的 0.8*gicp_score + 0.2*sc_score
// validator: wide_validation_cfg_，max_distance_from_prior_m 默认放宽
```

WIDE fallback seeds（SC 不可用时）：plan §3.3 的 5 位置 × 8 yaw 笛卡尔积，上限 40 seed。

---

## 3. 配置结构

### `config/location.yaml` 终态

```yaml
odom_frame: "odom"
map_path: "..."

initial:                        # 不动
  collect_duration_sec: 2.0
  min_query_points: 2500
  ...

local:                          # 从 lost.*_local 搬过来，去 _local 后缀
  collect_duration_sec: 0.8
  min_query_points: 1500
  submap_radius_m: 5.0
  coarse_iterations: ...
  refine_iterations: ...
  precise_iterations: ...
  score_threshold: ...
  min_inlier_ratio: ...
  max_distance_from_prior_m: 3.0
  max_yaw_from_prior_deg: 30.0
  enable_map_consistency_filter: true
  map_consistency_distance_m: ...
  min_retained_fraction: ...

wide:                           # 全新
  collect_duration_sec: 1.5
  min_query_points: 2000
  submap_radius_m: 6.0
  sc_top_k: 5
  coarse_iterations: ...
  refine_iterations: ...
  precise_iterations: ...
  score_threshold: ...           # 略宽于 local
  min_inlier_ratio: ...
  max_distance_from_prior_m: 999.0   # 默认不限
  max_yaw_from_prior_deg: 180.0
  fallback_seed_radius_m: 1.5
  fallback_yaw_set_deg: [0, 45, -45, 90, -90, 135, -135, 180]

scan_context:
  num_rings: 20
  num_sectors: 60
  max_radius_m: 20.0

diagnostics:
  log_failure_details: false   # 默认关；开启后 RCLCPP_WARN 出详细字段
```

**删除的 yaml 顶层段**：`lost`（整段消失，键迁移到 `local` / `wide`）。

---

## 4. 失败诊断（改用日志）

不写文件，全部走 ROS log。`location.yaml::diagnostics.log_failure_details` 控制是否启用，默认关。

启用后每次 handler 失败时，紧跟一条 `RCLCPP_WARN`：

```
[relocalize_failed] mode=wide
  request: prior=(x=12.3, y=4.5, yaw=78.2°), pointcloud_topic=/cloud_registered_undistort, collect_duration=1.50s
  collected: duration_actual=1.51s, points=12345
  scan_context: available=true, candidates=3
    [0] world_pos=(11.8, 4.6, 0.1) yaw=80.1° sc_score=0.234
    [1] world_pos=(15.2, 8.0, 0.0) yaw=12.5° sc_score=0.189
    [2] world_pos=(-3.4, 2.1, 0.0) yaw=170.0° sc_score=0.156
  gicp_per_seed:
    [0] fitness=0.045 inliers=0.78 iters=14 converged_pose=(11.79, 4.61, 0.10, yaw=80.0)
    [1] fitness=0.182 inliers=0.31 iters=20
    [2] fitness=0.310 inliers=0.18 iters=20
  validator: decision=rejected reason=low_inlier_ratio (0.78 < threshold 0.85)
  response: ok=false error="all candidates rejected"
```

成本：单条 `std::stringstream` 拼接 + `RCLCPP_WARN_STREAM`，~25 行。无文件 IO，无 json 库依赖，无磁盘清理。

排查对照表：

| 看到什么 | 结论 |
|---|---|
| collected.points 远低于 min_query_points | collector 收得太少（topic 没数据 / duration 太短 / qos 不匹配） |
| scan_context.available=false 且 wide 频繁失败 | 描述子文件缺失或 hash mismatch，看 launch 日志确认加载步骤 |
| sc 候选位置全错 | SC 在该地图判别力差，调 num_rings/sectors 或重训 |
| sc 候选对、gicp fitness 大 | GICP 收敛参数问题，或 query cloud 帧内 smear（fast motion）→ 启动 per-point deskew 子项目 |
| gicp 收敛但 validator 拒 | validator 阈值太严，按字段调 |

---

## 5. 死代码清理清单（来自审计）

按 `enum/类型` → `函数` → `配置键` → `测试` 分类。每行格式：`文件:行 | 项 | 动作`。

### 5.1 类型 / 枚举（DELETE 即整段移除）

| 文件:行 | 项 | 动作 |
|---|---|---|
| `src/tools/registration_tools.hpp:36-39` | `enum class LostTier { LOCAL, WIDE }` | DELETE |
| `src/tools/registration_tools.hpp:42-43` | `LostRegistrationConfig.local_sigma_{xy_m,yaw_deg}` | DELETE |
| `src/tools/registration_tools.hpp:45-46` | `submap_radius_local_m`, `submap_radius_wide_m` | RENAME→单值 `submap_radius_m`（拆到 `LocalRegistrationConfig` 和 `WideRegistrationConfig`） |
| `src/tools/registration_tools.hpp:48-66` | 全部 `*_local / *_wide` 迭代次数 / yaw 窗 / yaw 步 / 阈值字段 | DELETE 或 RENAME→单值 |
| `src/tools/registration_tools.hpp:73-74` | `rank_weight_inlier`, `rank_weight_distance` | DELETE（SC ranking 用 plan §3.2 硬编码权重） |
| `src/tools/registration_tools.hpp:75-76` | `max_distance_from_prior_local/wide_m` | RENAME→单值 |
| `src/tools/registration_tools.hpp:83-84` | `LostPrior.sigma_{xy_m,yaw_deg}` | DELETE |
| `src/tools/registration_tools.hpp:91` | `LostResult.tier_used` | DELETE |
| `src/server/validator.hpp:46-56` | 全部 `*_local / *_wide` 阈值对 | RENAME→单值 |
| `src/server/runtime.hpp` `LostRuntimeConfig` | 整体 | RENAME→拆为 `LocalRuntimeConfig` + `WideRuntimeConfig` |

### 5.2 函数

| 文件:行 | 项 | 动作 |
|---|---|---|
| `src/tools/registration_tools.cpp:76-120` | `StageParams::from_lost()` | DELETE（SC 用统一 `from_config()`） |
| `src/tools/registration_tools.cpp:156-171` | `RankedCandidate::compute_ranking()` (tier-aware) | DELETE |
| `src/tools/registration_tools.cpp:269-293` | `generate_yaw_candidates()` | DELETE（SC 不再 yaw 暴搜） |
| `src/tools/registration_tools.cpp:382-389` | `select_lost_tier()` | DELETE |
| `src/tools/registration_tools.cpp:398-418` | `create_wide_seeds()` | DELETE（被 SC + fallback 替代） |
| `src/tools/registration_tools.cpp:446-466` | `MultiStageGicp::run_coarse()` yaw sweep loop | DELETE |
| `src/tools/registration_tools.cpp:474-489` | `MultiStageGicp::run_refine()` yaw sweep loop | DELETE |
| `src/tools/registration_tools.cpp:550-563` | `run_lost_seed()` | DELETE |
| `src/tools/registration_tools.cpp:666-751` | `run_lost()` | RENAME→`run_local`（简化为单 seed）；同时新增 `run_wide`（接收 seed list） |
| `src/server/validator.cpp:11-13` | `select_by_tier()` | DELETE |
| `src/server/validator.cpp:16-28` | `confidence_lower_better/higher_better` | DELETE |
| `src/server/validator.cpp:112-142` | `evaluate_lost` 内 tier 分支 | EDIT→拆为 `evaluate_local` + `evaluate_wide`，单值阈值 |
| `src/server/runtime.cpp:368-439` | `handle_lost_relocalize` | EDIT→拆为 `handle_local_relocalize` + `handle_wide_relocalize` |
| `src/server/runtime.cpp:470-474` | `case MODE_MANUAL` | DELETE |
| `src/server/runtime.cpp:396-401` | sigma → tier 提取 | DELETE |
| `src/server/runtime.cpp:428` | `tier_used = ...` 赋值 | DELETE |
| `src/tools/param_tools.cpp:153-163` | `load_lost_runtime_config()` | RENAME→拆为 `load_local_runtime_config` + `load_wide_runtime_config` |
| `src/tools/param_tools.cpp:165-222` | `load_lost_registration_config()` | EDIT→去 tier 字段，拆为 local / wide |
| `src/tools/param_tools.cpp:224-257` | `load_lost_validation_config()` | EDIT→单值阈值，拆为 local / wide |

`engine.cc:265-289` 的 `make_lost_request / relocalize_local / relocalize_wide`：EDIT，去 sigma，按 mode 分发（见 §1）。

### 5.3 配置键

| 文件 | 段 | 动作 |
|---|---|---|
| `config/location.yaml` | 整个 `lost:` 段（约第 71-157 行） | DELETE，键迁移到新 `local:` / `wide:` 段 |
| `config/location.yaml` | `lost.local_sigma_xy_m`, `lost.local_sigma_yaw_deg` | DELETE（不再需要 sigma 触发） |
| `config/location.yaml` | `lost.submap_radius_local_m`, `lost.submap_radius_wide_m` | MERGE→单值 |
| `config/location.yaml` | 所有 `lost.*_yaw_window_deg`, `*_yaw_step_deg` | DELETE |
| `config/location.yaml` | `lost.rank_weight_*` | DELETE |
| `config/location.yaml` | 所有 `lost.score_threshold_local/wide`, `min_inlier_ratio_local/wide` 等 | MERGE→单值 |
| `config/location.yaml` | 新增 `scan_context:` 段 | NEW |
| `config/location.yaml` | 新增 `wide.sc_top_k`, `wide.fallback_*` | NEW |
| `config/location.yaml` | 新增 `diagnostics:` 段 | NEW |

### 5.4 测试

| 文件:行 | 项 | 动作 |
|---|---|---|
| `test/test_validator.cpp:110` | `evaluate_lost(..., LostTier::LOCAL)` | EDIT→`evaluate_local(...)` |
| `test/test_validator.cpp:122` | `evaluate_lost(..., LostTier::WIDE)` | EDIT→`evaluate_wide(...)` |
| `test/test_registration.cpp` | 涉及 `from_lost` / `select_lost_tier` 的用例 | DELETE 或 EDIT |
| `test/test_scan_context.cpp` | （新增） | NEW，按 scancontext_plan.md §8.1 |
| `test/test_map_descriptor_db.cpp` | （新增） | NEW |

### 5.5 其他

- `apply_map_consistency_filter`（registration_tools.cpp:335-375）+ 关联配置 `map_consistency_distance_m / min_retained_fraction / enable_map_consistency_filter`：**保留**（功能与 tier 无关，LOCAL/WIDE 都可用）
- `world_to_odom_from_world_to_base`、`extract_submap_radius`、`preprocess_cloud`：**保留**（共用工具）
- `scancontext_plan.md` §0.3 "tier_used 固定为 0" 的约束：**作废**（本 plan 直接删字段，更彻底）

---

## 6. 实施阶段

| 阶段 | 工作 | 估时 |
|---|---|---|
| 1 | srv 改字段、engine.cc 切到三 mode、Lua API 联动校验、编译过 | 0.5d |
| 2 | server 端 dispatcher 三分支，handle_lost 拆 handle_local + handle_wide（wide 暂用 prior 单 seed 占位） | 0.5d |
| 3 | 死代码清理：删 LostTier、select_lost_tier、create_wide_seeds、yaw sweep、tier 双份配置/字段，配置 yaml 改名 | 1d |
| 4 | SC 主路径：scan_context.{hpp,cpp}、map_descriptor_db.{hpp,cpp}、generate_map_descriptors.py、`.sc_desc` 加载 + hash 校验 | 1.5d |
| 5 | handle_wide_relocalize 接 SC：query top-K、跑 TwoStageGicp per seed、ranking、SC 不可用 fallback seeds | 0.5d |
| 6 | Lua helper `try_relocalize_with_fallback`，blackboard NaN 兜底；接到具体 intent | 0.25d |
| 7 | 失败日志诊断（log_failure_details 开关 + 拼字符串） | 0.25d |
| 8 | 单测（scan_context、map_descriptor_db、新 validator/run_local/run_wide） + 实车回归 | 1d |

**合计 ~5 工作日**。

---

## 7. 验证

每阶段独立可验证，避免一次集成调爆。

### 阶段 1（接口）
- 编译过；`grep -r 'MODE_LOST\|MODE_MANUAL\|LostTier\|tier_used\|prior_sigma' src/` 应无残留
- `ros2 service type /relocalize` 看新 enum

### 阶段 2-3（拆分 + 清理）
- `colcon test --packages-select rmcs-relocation` 现有测试调整后全过
- 旧 lost rosbag 回放，LOCAL 模式行为 1:1 匹配 baseline（pose、score、validator decision）
- `wc -l src/tools/registration_tools.cpp` 应明显下降

### 阶段 4-5（SC + WIDE）
- scancontext_plan.md §8.1 全部新单测 PASS
- 实车：`.sc_desc` 缺失 → wide 走 fallback，仍能成功；`.sc_desc` hash mismatch → 同样
- 在已知点位调 wide（不传 prior，传 0,0,0）→ SC 应能恢复正确 pose
- 与旧版 wide 对比：成功率、平均耗时

### 阶段 6（Lua）
- `action:relocalize("initial", spawn, ...)` 在出生点附近能定位
- 测试 endpoint 注 NaN：`blackboard.user.x = 0/0` → `try_relocalize_with_fallback("local_", 3)` 走 wide+原点，日志含 "LIO/TF lost"
- `pkill pointlio_mapping` → 下一次 relocalize 走兜底

### 阶段 7（日志）
- 默认关：失败时只见原 `error_message`
- 开 `log_failure_details: true`：失败时见完整段落，字段齐全
- 不影响成功路径（无成本）

### 阶段 8（实车 fast motion）
- 2 rad/s + 2 m/s 下 wide 重定位成功率 ≥ X%（需现场定 baseline）
- 失败 case 用日志判定瓶颈（按 §4 排查表）

---

## 8. Open Questions

1. **`blackboard.user` 字段名** —— 看 `src/lua/blackboard.lua` 即可确认（可能是 `pose.x` / `position.x`）
2. **谁调用 `try_relocalize_with_fallback`** —— intent/* 下当前没有 relocalize 调用点，需要补到具体节点（cruise-to-kill / 心跳 / 事件触发）
3. **`.sc_desc` 生成时机** —— 强烈建议纳入 mapping pipeline 自动化，避免换地图忘生成静默降级
4. **`maps/` 下哪些地图需要 `.sc_desc`** —— 仅部署中实际使用的几张
5. **engine.hh 的 `lost_local_sigma_*` / `lost_wide_sigma_*` 配置项**：是否还有 navigation 端别处引用？删除前 grep 全仓
6. **现存 LOCAL 实参（`lost_local_sigma_xy_m=5.0` 等）的使用方**：grep 后确认没有调用方依赖才能删
7. **WIDE 阶段 GICP 阈值是否需要不同于 LOCAL** —— 跑实车 baseline 后定

---

## 9. 风险

| 风险 | 影响 | 对策 |
|---|---|---|
| SC yaw 符号约定写反 | wide 永远找错朝向 | scancontext_plan.md §3.1 + `YawEndToEnd` 单测严格守住 |
| Python/C++ map_hash 实现不一致 | 描述子静默被弃用，wide 永远走 fallback（慢且失败率高） | 对拍单测，hash 字节流写法严格按 plan §4.2 |
| SC 在赛场几何对称区域出双峰 | wide 偶发选错点位 | top_k=5 + ranking 把 GICP fitness 占 0.8 权重，可缓解 |
| 删 `prior_sigma_*` 后某 navigation 节点引用 | 编译失败或运行时崩 | Open Q5 grep 确认 |
| 帧内 11° smear 仍在 | fast motion 下 GICP 收敛劣化 | 不在本 plan 范围；用诊断日志确认是否成主要矛盾后启动独立 plan |

---

## 10. 关键文件路径

**改动**：
- `src/rmcs_msgs/srv/Relocalize.srv`
- `src/rmcs-navigation-deps/rmcs-navigation/src/cxx/util/localization/engine.hh`
- `src/rmcs-navigation-deps/rmcs-navigation/src/cxx/util/localization/engine.cc:265-289`
- `src/rmcs-navigation-deps/rmcs-navigation/src/lua/action.lua`（加 helper）
- `src/rmcs-navigation-deps/rmcs-relocation/src/server/runtime.{hpp,cpp}`
- `src/rmcs-navigation-deps/rmcs-relocation/src/server/validator.{hpp,cpp}`
- `src/rmcs-navigation-deps/rmcs-relocation/src/tools/registration_tools.{hpp,cpp}`
- `src/rmcs-navigation-deps/rmcs-relocation/src/tools/param_tools.{hpp,cpp}`
- `src/rmcs-navigation-deps/rmcs-relocation/config/location.yaml`
- `src/rmcs-navigation-deps/rmcs-relocation/test/test_validator.cpp`
- `src/rmcs-navigation-deps/rmcs-relocation/test/test_registration.cpp`

**新增**：
- `src/rmcs-navigation-deps/rmcs-relocation/src/tools/scan_context.{hpp,cpp}`
- `src/rmcs-navigation-deps/rmcs-relocation/src/server/map_descriptor_db.{hpp,cpp}`
- `src/rmcs-navigation-deps/rmcs-relocation/scripts/generate_map_descriptors.py`
- `src/rmcs-navigation-deps/rmcs-relocation/test/test_scan_context.cpp`
- `src/rmcs-navigation-deps/rmcs-relocation/test/test_map_descriptor_db.cpp`

**不动**：
- `src/rmcs-navigation-deps/rmcs-relocation/src/server/collector.{hpp,cpp}`
- `src/rmcs-navigation-deps/point_lio/`（per-point deskew 是独立条线）
- `src/rmcs-navigation-deps/rmcs-navigation/src/lua/util/math.lua`（复用 check_nan）
- `src/rmcs-navigation-deps/rmcs-navigation/src/lua/api.lua`（签名不变）
