# local 导航安全型参考时刻对齐改造

## 目标

`local` 的定位不是全局重定位，而是导航运行中的小幅纠偏器。它必须满足：

```text
不影响导航连续性 > 成功率 > 纠偏幅度
```

因此，`local` 应该只在结果足够确定、修正幅度足够小的时候更新 `world->odom`。任何不确定、低质量或大跳变结果都应拒绝，并保持当前导航 TF 不变。

本计划是一套完整的 local 运动中导航安全纠偏方案：

```text
短快照点云 + snapshot 参考时刻位姿对齐 + 严格验收 + 小幅 TF 修正门控 + 修正速率限制
```

## 术语澄清：不是"运动补偿"

`/cloud_registered_undistort` 由 Point-LIO 输出，**每个点已经被 LIO 去畸变并注册到 odom 系**。
本改造**不做**逐点 deskew、不复现 LIO 的运动补偿，而是修正一个时序错配：

> SC 描述子、SC seed、GICP prior 三者都需要一个 `odom->base` 作为"机体在哪儿"的锚点。
> 旧实现使用采集开始前那一刻的 `odom->base`，与点云累积窗口中段的实际观测位姿不一致。

正确的做法是**把锚点对齐到点云的观测中心时刻**，本文称为 **snapshot 参考时刻对齐**。

## 背景与约束

现场条件：

- `/cloud_registered_undistort` 来自 Point-LIO，`header.frame_id == odom`。
- 已确认 Point-LIO 的 `header.stamp` 是 scan 结束时刻。
- 车辆最高线速度约 `2.0 m/s`，最高角速度约 `120 deg/s`。
- `wide` 仅停车使用。
- `local` 需要运动中使用，但绝不能因为错误重定位导致导航乱跳。

## 非目标

1. 逐点 deskew（LIO 已做）。
2. 每帧点云单独保存后重新投影。
3. 复杂状态机。
4. 多套 fallback。
5. TF 查不到参考时刻时 fallback 到 latest。
6. 让 local 承担大范围重定位。
7. 修改 wide 的停车全局重定位语义。
8. 修改 `initial` 路径。

## 旧 local 时序问题

```text
SC/query 锚点: t0（采集前）
query 点云: [t0, t1]
GICP prior: t0
最终导航时刻: t2
```

当车辆在采集窗口里运动时 `t0` 到 `t1` 的位移/旋转不可忽略：

| 采集窗口 | 位移 @2m/s | 旋转 @120deg/s |
| --- | ---: | ---: |
| 0.5s | 1.0m | 60deg |
| 0.3s | 0.6m | 36deg |
| 0.2s | 0.4m | 24deg |

## 新 local 流程

```text
1. collect short snapshot query cloud
   -> cloud
   -> reference_stamp  = midpoint(first_frame.stamp, last_frame.stamp)
   -> frame_count

2. lookup odom_to_base_ref at reference_stamp (with tf_lookup_timeout_sec)
   - 失败立即结束本次 local，不 fallback 到 latest。

3. query_sc_matches(query_cloud, odom_to_base_ref)

4. for each SC match:
       sc_seed = sc_match_to_seed(match, odom_to_base_ref)

       seed_prior = { world_to_base = sc_seed, odom_to_base = odom_to_base_ref }
       run_local(query_cloud, seed_prior) -> registration_result

       lookup odom_to_base_now (latest)
       world_to_base_estimated = registration_result.world_to_odom * odom_to_base_now

       validator (user_prior, estimated, score, inlier)

       safety gate (导航安全门控):
           - candidate world->odom 相对当前发布的增量 <= max_tf_correction_m / yaw
           - 与上次 accept 间隔 >= min_accept_interval_sec

       if all pass: publish + 更新 last_accept_time
       else:        keep current world_to_odom unchanged
```

## 关键设计决定

### reference_stamp 取首末帧中点

不是最后一帧。原因：

- 累积点云在时间维度上的"观测质心"在窗口中部，midpoint 把整段 cloud 与单点姿态的几何错配降到最小。
- 末帧 stamp 易让人误以为可以"匹配最新时刻"，但 cloud 是窗口内整段的混合。
- 首末中点用 `first + (last - first) * 0.5` 实现，仅占两个 `rclcpp::Time` 成员。

### Collector 同步清理冗余 odom 变换

旧 collector 把每帧点云用 `lookup_frame_to_odom` 变换到 `odom_frame`，但
`/cloud_registered_undistort` 的 `frame_id` 本来就是 `odom`，等价于做一次 identity。
改造后：

- Collector 不再持有 `odom_frame`，不查任何 TF；
- 订阅回调要求 `message->header.frame_id == expected_frame_id`，否则丢弃；
- Runtime 在调用 Collector 时显式传入 `odom_frame_` 作为期望 frame。

### TF 查询带 timeout

按 stamp 查 `odom->base` 在 Point-LIO 正常发布时基本立刻就能拿到，但 cloud 消息
与 odom TF 之间常有几 ms 乱序。带 `tf_lookup_timeout_sec`（默认 50 ms）等待，
减少正常情况下的间歇失败，但**绝不 fallback 到 latest**（那会还原旧时序错配）。

### 导航安全门控（两层）

publish 前必须两层都通过：

1. **单次幅度** —— 候选 `world->odom` 相对当前已发布值的增量：
   - `||Δt|| <= max_tf_correction_m`（默认 0.5 m）
   - `|Δyaw| <= max_tf_correction_yaw_deg`（默认 10°）
   - 必须**显著紧于** validator 的 `max_distance_from_prior_m`，否则 TF gate 形同摆设。
2. **修正速率** —— 与上次成功发布的间隔：
   - `elapsed >= min_accept_interval_sec`（默认 0.5 s）
   - 配合幅度上限即可限制累计漂移速度。0.5 m / 0.5 s = 1.0 m/s 上限，可调更严。

### 显式承认的取舍

> **local 无法修复大漂移。**

一旦 odom 累积漂超过 `max_tf_correction_m`，local 永远拒绝正确答案，必须靠
停车 `wide` 救。这是**特性**：导航连续性优先于成功率。日后调参时不要为了
"提高 local 成功率"反向放宽 gate。

## Collector 接口

```cpp
struct CollectedCloud {
    std::shared_ptr<PointCloud> cloud;
    rclcpp::Time reference_stamp;   // = midpoint(first, last)
    std::size_t frame_count = 0;
};

class Collector {
public:
    auto collect(
        rclcpp::Node& node,
        const rclcpp::CallbackGroup::SharedPtr& callback_group,
        const std::string& topic_name,
        const std::string& expected_frame_id,
        double duration_sec,
        int min_points = 0) const -> CollectedCloud;
};
```

## Runtime 接口（节选）

```cpp
struct CollectResult {
    std::shared_ptr<PointCloud> cloud;
    rclcpp::Time reference_stamp;
    std::size_t frame_count = 0;
    std::string error_message;
    bool ok = false;
};

// 新增重载：按指定时刻查询 odom->base，带 timeout
auto lookup_odom_to_base_at(
    const rclcpp::Time& stamp, double timeout_sec,
    Eigen::Isometry3f& transform, std::string& error_message) const -> bool;

// 新增：candidate world->odom 相对当前发布的增量
struct TfCorrectionDelta { double translation_m; double yaw_deg; };
auto compute_tf_correction_delta(const Eigen::Isometry3f& candidate) const -> TfCorrectionDelta;
auto check_local_safety_gate(const TfCorrectionDelta& delta, std::string& reason) -> bool;
```

`try_local_seed` 的参数语义由 `odom_to_base_before` 改为 `odom_to_base_ref`：

- `sc_match_to_seed(match, odom_to_base_ref)`
- `seed_prior.odom_to_base = odom_to_base_ref`
- 发布前调用 `compute_tf_correction_delta` + `check_local_safety_gate`

## 配置参数

`config/location.yaml` / `config/rmuc.yaml` 的 `local.*` 新增：

```yaml
local:
  # 采集窗口（保持现状或按现场缩短）
  collect_duration_sec: 0.5
  min_accumulated_points: 1000

  # 导航安全门控
  max_tf_correction_m: 0.5
  max_tf_correction_yaw_deg: 10.0
  min_accept_interval_sec: 0.5
  tf_lookup_timeout_sec: 0.05
```

调参原则：

- 现场频繁出现 `tf_correction_*` 拒绝 → 先**确认是漂移过大还是修正过小**，再决定是
  「放宽 gate」还是「触发 wide」。**绝不**直接放宽到与 validator 同量级。
- `min_accept_interval_sec` 是地板，单次幅度是天花板，二者共同决定累计修正速度。
- `tf_lookup_timeout_sec` 设 30~100 ms；过大会拖慢 service，过小会因 ms 级乱序失败。

## 日志锚点

```text
local: snapshot frames=... points=... ref_stamp=... ref_pose=(x,y,yaw)
local: try seed[i] sc_score=... pos=(x,y) yaw=...
local: seed[i] rejected ...                 (validator 拒绝)
local: seed[i] rejected: tf_correction_... (安全门控拒绝)
local: seed[i] rejected: accept_rate ...   (速率限制拒绝)
local: seed[i] accepted ... tf_dx=... tf_dyaw=...
local: ref_stamp tf lookup failed ...      (TF 不可达)
```

## 验证计划

### 1. 编译与单元测试

```bash
colcon build --merge-install --packages-select rmcs_relocation
colcon test  --merge-install --packages-select rmcs_relocation
```

### 2. 静态场景

确保新时序不破坏原有 local：

- 静止 local：reference_stamp TF 必须能查到；score/inlier 与改造前接近；tf 增量极小。
- 低速直线 local：accept 频率应受 `min_accept_interval_sec` 控制。

### 3. 运动场景

按速度分级 `0.5 / 1.0 / 2.0 m/s` 直线 + 旋转。每组观察：

- 是否错误 accepted（应当为 0）；
- 拒绝原因分布：`score` / `inlier` / `tf_correction_*` / `accept_rate`；
- 导航 `world->odom` 是否平滑（无明显跳变）。

## 成功标准

不是 local 成功率最高，而是：

1. local 成功时不会造成导航明显跳变。
2. local 不确定时保持当前 `world->odom` 不变。
3. 高速运动下错误结果不被 accepted。
4. 静止或低速下 local 仍能正常小幅纠偏。
5. wide 停车全局重定位流程不受影响。

## 最终口径

local 是：

```text
导航安全型 snapshot 参考时刻对齐纠偏器
```

不是：

```text
运动中全局重定位器
```

local 只在导航已基本可信时做小幅修正；大偏差、大角度、低置信情况交给停车 wide。
