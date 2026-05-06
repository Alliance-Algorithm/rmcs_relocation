# SC-local: ScanContext 驱动的运动中重定位

## 背景与目标

哨兵 + RoboMaster 镜像对称场地（RMUC/RMUL），需要在 1-2 m/s 运动中持续 1Hz 触发本地重定位。

旧 MODE_LOCAL 用 `request.initial_guess_world_base` 作 GICP 起点；运动中此 prior 必然偏（0.5-1.5m），子图错位 + GICP 难收敛 → 失败率高。

**新方案**：MODE_LOCAL 改为 SC 选种子，prior 仅作 validator 安全网。运动中工作。

预期单次 ~1.3s，错配兜底 ~2.1s（仍快于降级 wide 的 ~5s）。

> **前置已就绪**（wide SC 主路径上线后实测验证，2026-05-05）：
> - `sc_match_to_seed(match, odom_to_base)` 双参签名 — yaw 必须 = SC yaw (α) + odom_to_base.yaw (β)
> - `grid_centers` 锚原点 — `.sc_desc` 的 grid 中心必为 step 整数倍，(0,0) 一定在 grid 上
> - SC `yaw_window=6 / coarse_step=6` — 实测 wide 走 SC 主路径 10/10 通过，平均 ~50-70ms，dist <0.05m yaw_err <1°
>
> Local 的 SC 主路径**直接复用**这套既有 helper，不重写。下面骨架已对齐这套调用约定。

## 三个关键决策

### 1. top_k=2 + 早停（不是 1 也不是 3）

镜像对称场地下 SC top-1 在中线附近会错配到对方半场。top_k=2 留个备胎，但**早停**：seed 1 通过 validator 立即返回，seed 2 仅当 seed 1 失败时才跑。

| 场景 | 耗时 |
|---|---|
| seed 1 命中（95%+ 场景） | ~1.3s |
| seed 1 错配被拦 → seed 2 命中 | ~2.1s |
| 都不行 → Lua 降级 wide | ~1.3s + wide |

**为什么不 top_k=1**：错配后只能降级 wide（重新 collect 1.5s + 多 seed）≈ 5s；本地 seed 2 直接接上只 ~2.1s。
**为什么不 top_k=3**：对称错配最多 2 个候选位置（己方/对方），第 3 候选意义低。

### 2. prior 拆两个角色

| 角色 | 来源 | 用途 |
|---|---|---|
| GICP seed | SC top-K 匹配 | 给 GICP 当起点（不依赖 prior 准） |
| Validator reference | `request.initial_guess_world_base`（Lua 传入 `blackboard.user`） | 拦截 SC 错配（estimated 跨半场则 distance 检查必拒） |

handle_local_relocalize 内部分别构造两个 `RegistrationPrior` —— seed_prior 给 `tools::run_local`，validator_prior 给 `validator_->evaluate_local`。

`evaluate_local` 签名不动；只是把 user pose 塞进它的 prior 入参。

### 3. yaw_window=6（留 1 sector buffer）

SC yaw 量化误差 = 360° / num_sectors = 6°。`yaw_window_deg: 6` + `coarse_yaw_step_deg: 6` 让 coarse/refine 各跑 3 个 yaw 候选（0/+6/-6），多 ~260ms 换 6° 量化误差的鲁棒性。

## 架构

```
MODE_LOCAL (server)
  │
  ├─ map / SC unavailable ──▶ "SC unavailable" → success=false (Lua 降 wide)
  │
  ├─ collect cloud (用 request.collect_duration_sec, default 0.5s)
  ├─ translate_to_robot_frame (odom_to_base_before)
  ├─ build_descriptor → query_descriptor
  ├─ map_descriptor_db_.query(top_k = local.sc_top_k = 2)
  │
  ├─ matches.empty() ──▶ "SC no match" → success=false
  │
  └─ for each match in matches (按 sc_score 升序):
       │
       ├─ seed_prior.world_to_base = sc_match_to_seed(match)
       ├─ tools::run_local(seed_prior, ...) → registration_result
       │     ├─ 不收敛 ──▶ continue (last_msg = "seed[i] GICP not converged")
       │
       ├─ validator_prior.world_to_base = pose_to_isometry(request.initial_guess_world_base)
       ├─ validator_->evaluate_local(validator_prior, estimated, score, inlier)
       │     ├─ 拒（包括距离检查拦错配） ──▶ continue (last_msg = "seed[i] rejected: ...")
       │
       └─ accepted ──▶ update TF + success=true, message="ok (seed i)"  ← 早停
  │
  └─ 所有 seed 都不过 ──▶ success=false, message=last_msg
```

## 实现差量

### `src/server/runtime.cpp::handle_local_relocalize` —— 完整重写

新增 ~90 行（删旧 ~50 行，含 seed 循环 + 早停 + prior 拆角色）。**复用**已有：
- `translate_to_robot_frame` / `sc_match_to_seed` / `make_prior` —— 现 wide handler 的同名 helper
- `tools::run_local` —— 签名不变
- `validator_->evaluate_local` —— 签名不变（只是 caller 喂的 prior 来源不同）
- `build_validation_failure_message` —— 复用
- `collect_and_validate_points` / `lookup_odom_to_base` / `update_and_publish_world_to_odom` —— 复用

骨架：

```cpp
void handle_local_relocalize(
    const rmcs_relocation::srv::Relocalize::Request& request,
    rmcs_relocation::srv::Relocalize::Response& response) {
    if (!map_available_) { response.message = "map unavailable"; return; }
    if (!scan_context_available_) { response.message = "SC unavailable"; return; }

    auto odom_to_base_before = Eigen::Isometry3f::Identity();
    if (!lookup_odom_to_base(odom_to_base_before)) {
        response.message = "failed to query odom->base"; return;
    }

    const auto query_result = collect_and_validate_points(
        request, local_runtime_config_.pointcloud_topic,
        local_runtime_config_.collect_duration_sec,
        local_runtime_config_.min_accumulated_points);
    if (!query_result.ok) { response.message = query_result.error_message; return; }

    // SC top-K
    const auto query_local =
        translate_to_robot_frame(query_result.cloud, odom_to_base_before);
    const auto query_descriptor =
        tools::build_descriptor(*query_local, scan_context_config_);
    const auto matches = map_descriptor_db_.query(
        query_descriptor, local_registration_config_.sc_top_k);
    if (matches.empty()) { response.message = "SC no match"; return; }

    // user prior 仅作 validator 安全网
    const auto user_world_to_base =
        tools::pose_to_isometry(request.initial_guess_world_base);

    auto last_msg = std::string{"local: all SC seeds failed"};

    for (std::size_t i = 0; i < matches.size(); ++i) {
        const auto sc_seed = sc_match_to_seed(matches[i], odom_to_base_before);
        RCLCPP_INFO(node_.get_logger(),
            "local: try seed[%zu] sc_score=%.4f pos=(%.2f,%.2f) yaw=%.1f",
            i, matches[i].sc_score,
            sc_seed.translation().x(), sc_seed.translation().y(),
            static_cast<double>(matches[i].yaw_deg));

        auto seed_prior = RegistrationPrior{};
        seed_prior.has_prior = true;
        seed_prior.world_to_base = sc_seed;
        seed_prior.odom_to_base = odom_to_base_before;

        auto registration_result = RegistrationResult{};
        if (!tools::run_local(
                initial_registration_config_, local_registration_config_,
                query_result.cloud, map_cloud_, map_kdtree_,
                seed_prior, registration_result)) {
            last_msg = "local seed[" + std::to_string(i) + "] GICP not converged";
            continue;
        }

        auto odom_to_base_after = Eigen::Isometry3f::Identity();
        if (!lookup_odom_to_base(odom_to_base_after)) {
            response.message = "failed to query odom->base after registration"; return;
        }

        const auto world_to_base_estimated =
            registration_result.world_to_odom * odom_to_base_after;

        // validator 用 user prior 作参考（拦截跨半场错配）
        auto validator_prior = RegistrationPrior{};
        validator_prior.has_prior = true;
        validator_prior.world_to_base = user_world_to_base;
        validator_prior.odom_to_base = odom_to_base_before;

        const auto validation = validator_->evaluate_local(
            validator_prior, world_to_base_estimated,
            registration_result.score, registration_result.inlier_ratio);

        if (!validation.accepted) {
            last_msg = build_validation_failure_message(
                "local seed[" + std::to_string(i) + "]", validation, true);
            continue;
        }

        // 早停
        response.estimated_world_base =
            tools::isometry_to_pose(world_to_base_estimated);
        response.world_to_odom =
            tools::isometry_to_transform(registration_result.world_to_odom);
        response.fitness_score = static_cast<float>(registration_result.score);
        response.within_field_bounds = validation.within_bounds;
        response.confidence = validation.confidence;
        update_and_publish_world_to_odom(registration_result.world_to_odom);
        response.success = true;
        response.message = "ok (seed " + std::to_string(i) + ")";
        return;
    }

    response.message = std::move(last_msg);
}
```

### `src/tools/registration_tools.hpp::LocalRegistrationConfig` —— 加 `sc_top_k`

```cpp
struct LocalRegistrationConfig {
    // ... 既有字段不变 ...
    std::size_t sc_top_k = 2;  // 镜像对称场地兜底；早停，平时只跑 1 次
};
```

### `src/tools/param_tools.cpp::load_local_registration_config` —— 读 yaml

```cpp
config.sc_top_k =
    static_cast<std::size_t>(reader.read_positive_int("local.sc_top_k", 2));
```

### `src/tools/registration_tools.cpp` / `src/server/validator.{hpp,cpp}` —— 零改动

`run_local` 接 `RegistrationPrior` 已经支持任意 seed；`evaluate_local` 签名不变（caller 喂哪个 prior 是 caller 的事）。

### `config/location.yaml` —— `local.*` 调参（基于当前线上版本的差量）

```yaml
local:
    pointcloud_topic: "/cloud_registered_undistort"
    collect_duration_sec: 0.5         # ← 旧 0.8。运动中减漂移 (1.5m/s × 0.5s = 0.75m)
    min_accumulated_points: 1000

    submap_radius_m: 5.0              # SC seed 离机器人最多 grid_step·√2/2 ≈ 1.4m，
                                      # 留 ~3.5m 余量给 GICP 收敛
    coarse_iterations: 8              # ← 旧 12
    refine_iterations: 4              # ← 旧 6
    precise_iterations: 8             # ← 旧 12（SC 给好 yaw 后无需深迭代）
    max_correspondence_distance_m: 3.0  # ← 旧 4.0
    coarse_score_threshold: 0.5

    yaw_window_deg: 6.0               # ← 旧 30.0。= 360/num_sectors=6° 量化误差
    coarse_yaw_step_deg: 6.0          # ← 旧 15.0。3 候选 (0, +6, -6)
    refine_yaw_step_deg: 6.0          # ← 旧 12.0

    enable_map_consistency_filter: false
    map_consistency_distance_m: 0.8
    min_retained_fraction: 0.15

    sc_top_k: 2                       # ← 新增

    score_threshold: 0.40             # ← 旧 0.50。运动中 score 略高，放宽
    min_inlier_ratio: 0.08            # ← 旧 0.10
    max_distance_from_prior_m: 6.0    # ← 关键：拦跨半场。RMUC 镜像最近距离 ~7m，
                                      # 留 4m 给运动 prior 漂移；正常 <2m 必过
    max_yaw_from_prior_deg: 90.0      # ← 旧 60.0

    field_bounds: ...                 # 不改
```

> **`max_distance_from_prior_m` 不同场地必看 §"快速调参指南"**：默认 6.0 是 RMUC 几何下的取值，
> RMUL（11×7m 半场）镜像最近 ~5m，需要降到 3.5；其它场地按半场最近镜像距离 × 0.6 计算。

### `src/lua/action.lua` —— 新增 `try_relocalize_local_then_wide`

当前 `action:relocalize(mode, x, y, yaw)` / `action:relocalize_wide(x, y, yaw)` 已是终态签名（无 per-call collect/timeout 覆盖；timeout 由 cxx `Localization::Config::request_timeout_sec` 控）。

```lua
--- LOCAL 失败（SC 不可用 / 无匹配 / 错配被拦 / GICP 不收敛）时降级 WIDE。
--- blackboard.user.x/y/yaw 任一 NaN（LIO 死）则跳过 local 直接 wide+原点。
function action:try_relocalize_local_then_wide()
    local user = blackboard.user
    if util.check_nan(user.x, user.y, user.yaw) then
        self:warn("LIO/TF lost (NaN), skip local, wide+origin")
        return self:relocalize_wide(0.0, 0.0, 0.0)
    end
    local ok = self:relocalize("local_", user.x, user.y, user.yaw)
    if ok then return true end
    self:warn("local failed, fallback wide with user prior")
    return self:relocalize_wide(user.x, user.y, user.yaw)
end
```

> **设计选择**：当前方案 server 端只跑 SC（unavailable / no-match / 全 seed 拒收 → success=false），
> Lua 端做 `local → wide` 级联。如果上层希望 local 失败即停车（不自动降 wide），把这个 helper 改名为
> `try_relocalize_local`，去掉 fallback 即可。

## 时间预算

| 阶段 | 耗时 | 备注 |
|---|---|---|
| collect 0.5s | ~500ms | LiDAR 频率限制 |
| translate + build_descriptor | <5ms | ~5k 点 → 60×20 cell |
| db.query top_k=2 (K=200 grid) | ~10ms | 60 sector shift × 200 desc cosine |
| 单 seed pipeline (3 yaw × coarse 8 + refine 4 + precise 8) | ~770ms | yaw_window=6 |
| evaluate + publish | <5ms | |
| **合计 (seed 1 命中)** | **~1.3s** | 95%+ 场景 |
| seed 1 错配 + seed 2 重试 | ~2.1s | 跨中线时偶发 |

满足实时性目标（运动中 1Hz 触发）。

## 验证清单

| # | 场景 | 预期 |
|---|---|---|
| 1 | SC 可用 + 静止己方半场 | seed[0] sc_score 低，GICP 收敛，validator 通过 → success="ok (seed 0)" |
| 2 | SC 可用 + 1.5m/s 运动 | seed[0] 命中，score 略高仍 <0.40 → success |
| 3 | SC 不可用（descriptor_path=""） | "SC unavailable" → Lua try_relocalize_local_then_wide 切 wide |
| 4 | SC 匹配为空（query 描述子全零） | "SC no match" → 同上 |
| 5 | SC top-1 错配到对方半场 | seed[0] GICP 收敛但 estimated 离 user_prior >15m → validator 距离拒 → seed[1] 重试 |
| 6 | 两个 seed 都不过 | "local: all SC seeds failed" → Lua 降 wide |
| 7 | LIO 死（blackboard.user.x = NaN） | Lua 跳过 local 直接 wide+原点 |
| 8 | 旧 wide 路径不受影响 | wide handler 与 SC-local 无共用代码改动 |

## 改动文件汇总

| 文件 | 操作 | 行数 |
|---|---|---|
| `src/server/runtime.cpp::handle_local_relocalize` | 重写（seed 循环 + 早停 + prior 拆角色） | -50 / +90 |
| `src/tools/registration_tools.hpp::LocalRegistrationConfig` | 加 `sc_top_k` | +1 |
| `src/tools/param_tools.cpp::load_local_registration_config` | 读取 `local.sc_top_k` | +2 |
| `config/location.yaml::local.*` | 调参 | ±15 |
| `src/lua/action.lua::try_relocalize_local_then_wide` | 新增级联 helper | +12 |

总改动 ~120 行。**不新增类型，不新增 tools 函数，不动 validator/wide handler**。

## 上行依赖

1. **wide 模式 SC 路径已实测稳定**（2026-05-05，10/10 通过，平均 ~70ms，dist <0.05m yaw_err <1°）
2. **`sc_match_to_seed` 双参签名已就位**（runtime.cpp）— local 直接调用，无须再修
3. **离线工具 grid 锚原点已就位**（`offline_pcd_align_tool/pcd_align_tool.py` + repo 内 `scripts/generate_map_descriptors.py`）— 实施 local 前必须用新工具重生 `.sc_desc`
4. **`.sc_desc` 已加载并 hash 校验通过**：启动日志确认 `scan_context descriptor loaded: K entries (rings=20, sectors=60, radius=20.00m, hash=0x…)` 后才能上 SC-local
5. **Lua 调用方在 request 里塞入 user pose**：现有 `try_relocalize` 已是这样

## 快速调参指南（local + wide 适配不同场地）

调参顺序：**先量场地 → 算 grid_step / max_radius → 重生 .sc_desc → 调 submap / max_distance_from_prior_m → 验**。
所有数值参数都跟场地几何耦合，照拷别的赛季的 yaml 容易翻车。

### 第 0 步：准备场地参数

| 量 | 怎么得 | 用途 |
|---|---|---|
| **field_xy_size** = (W, H) | PCD 点云 XY 包围盒 | 决定 grid 数量、`field_bounds`、SC 半径 |
| **mirror_min_dist** | 镜像中线两侧最近的非对称点对距离 | 决定 `max_distance_from_prior_m` |
| **walk_max_speed** | 决策层最大平移速度 (m/s) | 决定 `collect_duration_sec` |

RMUC 参考：W=28m H=15m, mirror_min_dist≈7m, walk_max_speed≈2 m/s。
RMUL 参考：W=22m H=14m, mirror_min_dist≈5m, walk_max_speed≈3 m/s。

### 第 1 步：SC 描述子库参数（`scan_context.*` + `wide.sc_top_k`）

| 参数 | 公式 / 经验 | 解释 |
|---|---|---|
| `num_rings` | 16-24 | ring 太多噪声大，太少分辨率不够 |
| `num_sectors` | 60 推荐（=6°/格） | 量化误差直接 = `yaw_window` |
| `max_radius_m` | min(W, H) × 0.7 | 覆盖大部分场地特征但不超出 |
| `grid_step` (离线工具) | min(W, H) / 10 ~ /15 | 太密浪费空间，太稀 SC seed 离机器人 >2m 会让 GICP 难收敛 |
| `wide.sc_top_k` | 3-8 | 全局兜底用，留多个候选 |
| `local.sc_top_k` | 2 | 镜像场地兜底，1 跨半场无救，3+ 边际收益低 |

> 改 `num_sectors` / `grid_step` 后 **必须重生 `.sc_desc`** 并重启 server。`map_hash` 不会变（只看 PCD），但 grid 中心和量化粒度会变。

### 第 2 步：GICP 收敛域（`*.submap_radius_m` + `*.max_correspondence_distance_m`）

`submap_radius_m` 必须满足：
```
submap_radius_m ≥ (grid_step × √2 / 2) + max_seed_displacement_m + safety
                  ↑ SC seed 偏离机器人最远  ↑ 运动漂移         ↑ ~1m
```

| 模式 | submap_radius_m | max_correspondence_distance_m |
|---|---|---|
| local | grid_step·0.7 + 3.5 | submap_radius × 0.6 |
| wide  | submap_radius_m × 1.5 | 2.5 × max_seed_displacement |

经验值：

| 场地 | grid_step | local.submap | wide.submap | local.max_corr | wide.max_corr |
|---|---|---|---|---|---|
| RMUC | 2.0 | 5.0 | 6.0 | 3.0 | 6.0 |
| RMUL | 1.5 | 4.5 | 5.5 | 2.7 | 5.0 |

### 第 3 步：跨半场拦截阈值（`*.max_distance_from_prior_m`）

```
local.max_distance_from_prior_m ≈ mirror_min_dist × 0.6
wide .max_distance_from_prior_m ≈ field_xy_size 对角线 × 0.7   (wide 给 prior 信任度低)
```

逻辑：
- 取值 < 镜像最近距离 → 镜像错配必拒
- 取值 > 运动 prior 漂移（一般 <2m） → 正常情况必过

| 场地 | local 阈值 | wide 阈值 |
|---|---|---|
| RMUC | 6.0 (=7×0.85) | 12.0 |
| RMUL | 3.5 (=5×0.7) | 9.0 |

**这个数错了的现象**：local 一直 fallback 到 wide / wide 接受了对方半场的镜像位姿。

### 第 4 步：运动适配（`collect_duration_sec` + score 阈值）

```
collect_duration_sec ≤ acceptable_drift_m / walk_max_speed
                     # 推荐 0.3-0.5s，> 0.8s 必然漂移过大
```

| 速度 (m/s) | local.collect | local.score_threshold |
|---|---|---|
| 静止-1 | 0.5 | 0.40 |
| 1-2 | 0.4 | 0.45 |
| 2-3 | 0.3 | 0.50 |

`min_inlier_ratio`：实测取最低稳定值的 0.7 倍。Wide 实测 inlier ~0.5-0.6，wide 阈值 0.08 充足；local SC seed 比 wide 准，inlier 更高，0.08 一样够。

### 第 5 步：yaw 容差（`*.yaw_window_deg`）

```
yaw_window_deg = 360 / num_sectors        # SC 量化误差
coarse_yaw_step_deg = yaw_window_deg      # 3 candidates: 0, +w, -w
refine_yaw_step_deg = yaw_window_deg / 2  # 5 candidates around coarse best
```

**唯一例外**：wide 的 fallback 路径（`fallback_config.yaw_window_deg = 180`）需要全角扫描，
那是没有 SC seed 时的兜底，跟 SC 路径完全独立。

### 调参 checklist（按顺序勾）

- [ ] PCD bounding box 量过，对照 yaml `field_bounds`
- [ ] `.sc_desc` 用最新离线工具重生（grid 锚原点）
- [ ] 启动日志看 `scan_context descriptor loaded: K entries` K ≈ (W/grid_step) × (H/grid_step)
- [ ] 静止跑 wide：`SC[0] pos` 应贴 (0,0) 附近 grid 中心，accepted path=scan_context
- [ ] 静止跑 local：`success=true` 且 `dist < 0.1m`、`yaw_err < 2°`
- [ ] 运动 1-2 m/s 跑 local：`score < 0.45` 且 90%+ 通过
- [ ] 跨中线靠近 `mirror_min_dist` 边界跑 local：观察 seed[1] 是否被触发

---

## 已知限制

- **哨兵真跨到对方半场**：错配候选恰好 = 真实位置，validator 通过——但此时 SC 选的就是对的，是 feature 不是 bug
- **己方半场内部小尺度对称**（左右补给区等）：尺度通常 <5m，被 SC ring/sector 分辨率（1m × 6°）模糊掉，错配概率低
- **SC 量化外的 yaw 偏差**（>6°）：靠 yaw_window=6 + GICP iter 内消化（单 iter 可吃 ~10°），余量充足
- **帧内 11° smear**（point_lio 不做 per-point deskew）：对 SC 统计描述子（max-height per cell）影响小，对 GICP 影响大但不致命
- **`ranking_cost` 在多 seed 早停模式下未启用**：当前设计 seed 1 通过即返回，不与 seed 2 横向比较；如果 seed 1 score 边缘但勉强通过，seed 2 可能更好但不会被尝试。这是早停的代价；如要避免，可改成「跑完所有 seed 后选 ranking 最优」（多 ~770ms）
