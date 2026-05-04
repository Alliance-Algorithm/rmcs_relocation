# SC-local: ScanContext 驱动的运动中重定位

## 目标

MODE_LOCAL 改为以 SC 为种子源的快速重定位，**不依赖 prior 准**，运动中可用。

适用场景：RoboMaster 哨兵 + RMUC/RMUL 镜像对称场地，1-2 m/s 运动，要求单次 ~1.3s 完成。

## 关键决策

### 1. SC top_k=2 + 早停（不是 1 也不是 3）

**为什么不是 1**：镜像对称场地下，SC 在中线附近会出红蓝双峰，top-1 概率错配到对方半场。

**为什么不是 3**：哨兵实际只在己方半场+中线附近活动，SC 的真实错配只有 2 个候选（己方/对方），第 3 个匹配距离已经显著拉大，意义不大。

**早停**：seed 1 跑完通过 validator → 立即返回。seed 2 仅当 seed 1 失败（GICP 不收敛 / validator 拒）时才跑。

| 场景 | 总耗时 |
|---|---|
| seed 1 一发命中（成功路径，95%+ 场景） | ~1.3s |
| seed 1 错配被拦，seed 2 命中 | ~2.1s |
| 都不行 → Lua 降级 wide | ~1.3s + wide 时间 |

平时不付 top_k>1 的代价。

### 2. prior 拆两个角色

- **GICP seed**：来自 SC top-K，与 user prior 无关 → 解决「运动中 prior 偏」
- **Validator reference**：来自 `request.initial_guess_world_base`（Lua 传入 blackboard.user）→ 拦截 SC 错配（estimated 跨半场 → 距离检查必拒）

Lua 必须传 user pose 进 request，prior 在请求里扮演「我相信哨兵大致在这里」的角色，而非 GICP 起点。

### 3. yaw_window 留 1 sector buffer

SC yaw 量化误差 = 360° / 60 sectors = 6°。`yaw_window_deg: 6.0` + `coarse_yaw_step_deg: 6.0` 让 coarse/refine 各跑 3 个 yaw 候选（0/+6/-6），多 ~260ms 换 6° 量化误差的鲁棒性。

## 架构

```
MODE_LOCAL
  │
  ├─ map / SC unavailable ──▶ "SC unavailable" → fail (Lua 降 wide)
  │
  ├─ collect cloud (0.5s) → translate_to_robot_frame → build_descriptor
  ├─ db.query(top_k=2) → 0/1/2 个候选
  │
  ├─ 0 候选 ──▶ "SC no match" → fail
  │
  └─ 对每个 SC match (按 sc_score 升序)：
       │
       ├─ sc_match_to_seed → world_to_base (位置+yaw)
       ├─ run_local(seed_as_prior, yaw_window=6) → GICP 收敛
       │
       ├─ run_local 失败 ──▶ 试下一个 seed
       │
       └─ evaluate_local(estimated, validator_reference=user_prior, ...)
            ├─ 通过 → update TF → success（早停，剩余 seed 不跑）
            └─ 不通过 → 试下一个 seed
  │
  └─ 所有 seed 都不过 ──▶ "local rejected after K seeds: ..." → fail
```

## 实现

### 1. `runtime.cpp::handle_local_relocalize` —— 完整重写

```cpp
void handle_local_relocalize(
    const rmcs_msgs::srv::Relocalize::Request& request,
    rmcs_msgs::srv::Relocalize::Response& response) {
    if (!map_available_) {
        response.message = "map unavailable";
        return;
    }
    if (!scan_context_available_) {
        response.message = "SC unavailable";
        return;
    }

    auto odom_to_base_before = Eigen::Isometry3f::Identity();
    if (!lookup_odom_to_base(odom_to_base_before)) {
        response.message = "failed to query odom->base";
        return;
    }

    const auto query_result = collect_and_validate_points(
        request, local_runtime_config_.pointcloud_topic,
        local_runtime_config_.collect_duration_sec,
        local_runtime_config_.min_accumulated_points);
    if (!query_result.ok) {
        response.message = query_result.error_message;
        return;
    }

    // SC 查询 top_k 候选
    const auto query_local =
        translate_to_robot_frame(query_result.cloud, odom_to_base_before);
    const auto query_descriptor =
        tools::build_descriptor(*query_local, scan_context_config_);
    const auto matches = map_descriptor_db_.query(
        query_descriptor, local_registration_config_.sc_top_k);
    if (matches.empty()) {
        response.message = "SC no match";
        return;
    }

    // user prior 仅作 validator 安全网（拦截 SC 错配跨半场）
    const auto user_prior_world_to_base =
        tools::pose_to_isometry(request.initial_guess_world_base);

    auto last_failure_message = std::string{"local: all SC seeds failed"};

    for (std::size_t i = 0; i < matches.size(); ++i) {
        const auto& match = matches[i];
        const auto sc_seed = sc_match_to_seed(match);

        RCLCPP_INFO(node_.get_logger(),
            "local: try SC seed[%zu] sc_score=%.4f pos=(%.2f,%.2f) yaw=%.1f",
            i, match.sc_score,
            sc_seed.translation().x(), sc_seed.translation().y(),
            static_cast<double>(match.yaw_deg));

        auto seed_prior = RegistrationPrior{};
        seed_prior.has_prior = true;
        seed_prior.world_to_base = sc_seed;
        seed_prior.odom_to_base = odom_to_base_before;

        auto registration_result = RegistrationResult{};
        if (!tools::run_local(
                initial_registration_config_, local_registration_config_,
                query_result.cloud, map_cloud_, map_kdtree_,
                seed_prior, registration_result)) {
            last_failure_message =
                "local seed[" + std::to_string(i) + "] GICP did not converge";
            continue;
        }

        auto odom_to_base_after = Eigen::Isometry3f::Identity();
        if (!lookup_odom_to_base(odom_to_base_after)) {
            response.message = "failed to query odom->base after registration";
            return;
        }

        const auto world_to_base_estimated =
            registration_result.world_to_odom * odom_to_base_after;

        // validator 用 user prior 作参考（拦截跨半场错配）
        auto validator_prior = RegistrationPrior{};
        validator_prior.has_prior = true;
        validator_prior.world_to_base = user_prior_world_to_base;
        validator_prior.odom_to_base = odom_to_base_before;

        const auto validation = validator_->evaluate_local(
            validator_prior, world_to_base_estimated,
            registration_result.score, registration_result.inlier_ratio);

        if (!validation.accepted) {
            last_failure_message = build_validation_failure_message(
                "local seed[" + std::to_string(i) + "]", validation, true);
            continue;
        }

        // 早停：第一个通过验收的 seed 直接返回
        response.estimated_world_base =
            tools::isometry_to_pose(world_to_base_estimated);
        response.world_to_odom =
            tools::isometry_to_transform(registration_result.world_to_odom);
        response.fitness_score =
            static_cast<float>(registration_result.score);
        response.within_field_bounds = validation.within_bounds;
        response.confidence = validation.confidence;
        update_and_publish_world_to_odom(registration_result.world_to_odom);
        response.success = true;
        response.message = "ok (seed " + std::to_string(i) + ")";
        return;
    }

    response.message = std::move(last_failure_message);
}
```

### 2. `tools/registration_tools.hpp` —— `LocalRegistrationConfig` 加 `sc_top_k`

```cpp
struct LocalRegistrationConfig {
    // ... 既有字段不变 ...
    std::size_t sc_top_k = 2;     // 镜像对称场地兜底（早停，平时只跑 1 次）
};
```

### 3. `validator.{hpp,cpp}` —— 零改动

`evaluate_local(prior, estimated, score, inlier)` 签名不动。caller 决定塞入的 prior 是 SC seed 还是 user pose。SC-local 把 user pose 塞进去，让 prior 距离检查发挥拦错配作用。

### 4. `tools/registration_tools.cpp` —— 零改动

`run_local` 接受 `RegistrationPrior` 已经支持任意 seed。把 SC seed 填入 `prior.world_to_base` 即可。`yaw_window_deg` 由 yaml 传入。

### 5. `tools/param_tools.cpp` —— 加载 `sc_top_k`

```cpp
config.sc_top_k =
    static_cast<std::size_t>(reader.read_positive_int("local.sc_top_k", 2));
```

### 6. `config/location.yaml` —— `local.*` 调参

```yaml
local:
    pointcloud_topic: "/cloud_registered_undistort"
    collect_duration_sec: 0.5         # 运动中采集，旧 0.8
    min_accumulated_points: 1000

    submap_radius_m: 4.0              # SC 种子比 prior 准，子图缩小
    coarse_iterations: 8              # SC 给了好 yaw，砍迭代
    refine_iterations: 4              # 旧 6
    precise_iterations: 8             # 旧 12
    max_correspondence_distance_m: 3.0
    coarse_score_threshold: 0.5

    yaw_window_deg: 6.0               # ← 1 sector buffer, 吸收 SC yaw 量化误差
    coarse_yaw_step_deg: 6.0          # 3 候选 (0, +6, -6)
    refine_yaw_step_deg: 6.0

    enable_map_consistency_filter: false
    map_consistency_distance_m: 0.8
    min_retained_fraction: 0.15

    sc_top_k: 2                       # ← 新增：镜像对称兜底，早停

    score_threshold: 0.40             # 运动中允许略高
    min_inlier_ratio: 0.08
    max_distance_from_prior_m: 10.0   # ← 关键：拦跨半场错配（己方半场 ~14m，错配 >15m 必拦）
    max_yaw_from_prior_deg: 90.0      # 比 wide 严，比旧 local 宽

    field_bounds:
      min_x: -2.0
      max_x: 7.0
      min_y: -5.5
      max_y: 5.5
      min_z: -2.0
      max_z: 2.0
```

### 7. `lua/action.lua` —— 新增级联 helper

```lua
--- LOCAL 失败（SC 不可用 / 无匹配 / 错配 / GICP 不收敛）时降级到 WIDE。
--- blackboard.user.x/y/yaw 任一 NaN（LIO 死）则跳过 local 直接走 wide+原点。
function action:relocalize_local_then_wide(local_timeout_sec, wide_timeout_sec)
    local user = blackboard.user
    if util.check_nan(user.x, user.y, user.yaw) then
        self:warn("LIO/TF lost (NaN), skip local, wide+origin")
        return self:relocalize("wide", 0.0, 0.0, 0.0, wide_timeout_sec * 2)
    end
    local ok = self:relocalize("local_", user.x, user.y, user.yaw, local_timeout_sec)
    if ok then return true end
    self:warn("local failed, fallback to wide with user prior")
    return self:relocalize("wide", user.x, user.y, user.yaw, wide_timeout_sec)
end
```

调用方式：

```lua
-- 周期/事件触发
action:relocalize_local_then_wide(2.5, 6.0)
```

## 时间预算

| 阶段 | 耗时 | 备注 |
|---|---|---|
| collect 0.5s | ~500ms | 受 LiDAR 频率限制 |
| translate + build_descriptor | <5ms | 5k 点 × 60×20 SC cell |
| db.query top_k=2 (K=200 grid) | ~10ms | 60 sector shift × 200 desc cosine |
| 单 seed pipeline (preprocess + 3 yaw × coarse/refine + precise) | ~770ms | yaw_window=6 |
| evaluate + publish | <5ms | |
| **合计 (seed 1 命中)** | **~1.3s** | 95%+ 场景 |
| seed 1 错配被拦 + seed 2 重试 | ~2.1s | 跨中线时偶发 |

满足实时性目标（运动中 1Hz 触发）。

## 验证清单

| # | 场景 | 预期 |
|---|---|---|
| 1 | SC 可用 + 静止己方半场 | seed[0] sc_score 低，GICP 收敛，validator 通过 → success |
| 2 | SC 可用 + 1.5m/s 运动 | seed[0] 命中，score 略高仍 <0.40 |
| 3 | SC 不可用 (descriptor_path="") | "SC unavailable" → Lua 降 wide |
| 4 | SC 匹配为空 (query 描述子全零) | "SC no match" → Lua 降 wide |
| 5 | SC top-1 错配到对方半场 | seed[0] GICP 收敛但 estimated 离 user_prior >15m → validator 距离检查拒 → seed[1] 重试 |
| 6 | 两个 seed 都错或都不收敛 | "local: all SC seeds failed" → Lua 降 wide |
| 7 | LIO 死 (blackboard.user.x = NaN) | Lua 跳过 local 直接 wide+原点 |
| 8 | wide 路径不受影响 | wide handler 与 SC-local 无共用代码 |

## 改动文件汇总

| 文件 | 操作 | 行数 |
|---|---|---|
| `src/server/runtime.cpp::handle_local_relocalize` | 完整重写（含 seed 循环 + user prior validator） | -50 / +90 |
| `src/tools/registration_tools.hpp::LocalRegistrationConfig` | 加 `sc_top_k` 字段 | +1 |
| `src/tools/param_tools.cpp::load_local_registration_config` | 读取 `local.sc_top_k` | +2 |
| `config/location.yaml::local.*` | 调参（迭代砍半、yaw_window=6、加 sc_top_k 等） | ±10 |
| `src/lua/action.lua::relocalize_local_then_wide` | 新增级联 helper | +10 |

总改动 ~115 行，**不新增类型，不新增 tools 函数**，不动 validator/wide handler。

## 上行依赖

1. **wide 模式 SC 路径已稳定**（Phase 5 已完成，需实车验证 SC 描述子库质量与 map_hash 一致性）
2. **`.sc_desc` 已生成并配置**：`descriptor_path` 指向有效文件，启动日志确认 `scan_context descriptor loaded: K entries`
3. **Lua endpoint 把 blackboard.user.x/y/yaw 传给 relocalize 调用**（现有 endpoint/test.lua 已是这种模式）

## 已知限制

- **哨兵真跨到对方半场**：错配恰好等于真实位置，validator 通过——但此时 SC 选的就是对的，是 feature 不是 bug
- **己方半场内部小尺度对称**（如左右补给区）：尺度通常 <5m，被 SC 描述子的 ring/sector 分辨率（1m × 6°）模糊掉，错配概率低
- **SC 量化外的 yaw 偏差**（>6°）：靠 yaw_window=6 + GICP iter 内部消化，单 iter 可吃 ~10°，余量充足
- **帧内 11° smear**（point_lio 不做 per-point deskew）：对 SC 统计描述子影响小（max-height per cell），对 GICP 影响大但不致命
