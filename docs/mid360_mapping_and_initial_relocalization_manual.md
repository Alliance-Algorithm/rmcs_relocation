# MID360 建图与 `rmcs-relocation` 实车测试手册

本文档用于在实车上完成以下四件事：

1. 使用 `MID360 + point_lio` 建图并生成 `world_map.pcd`
2. 验证 `rmcs-relocation` 的初始化重定位（`MODE_INITIAL`）
3. 验证比赛中丢失位置重定位（`MODE_LOST`）
4. 验证 `Supervisor` 的自动触发策略（倒计时触发 INITIAL，其余触发 LOST）

---

## 0. 版本说明（与当前代码一致）

- 当前 `rmcs-relocation` 配准能力已收敛到 `tools/registration_tools.*`，不再使用旧的 `server/registration.*` 与 `tools/pointcloud_tools.*`。
- ROS 外部接口保持不变：
  - 服务：`/rmcs_relocation/relocalize`
  - 健康话题：`/rmcs_relocation/health`
  - 参数入口：`config/location.yaml`
- 本手册按以上现状编写。

---

## 0.5 Foxglove 可视化验证

建议全程开 Foxglove，直观观察 relocalization 前后 `world` 帧的跳变。

### TF 链全景

pointlio 与 `rmcs-relocation` 共同维护以下 TF 树：

```
world ──(rmcs_relocation, 10Hz)──→ odom ──(pointlio, 静态)──→ camera_init ──(pointlio, 动态)──→ aft_mapped ──(pointlio, 静态)──→ base_link
```

- `world → odom`：identity 起始，**relocalization 成功后跳变为配准结果**，是整个可视化最关键的观察点
- `odom → camera_init`：pointlio 启动时发一次静态变换（从 `mid360.yaml` 的 `init_pose` 提取 yaw）
- `camera_init → aft_mapped`：pointlio 动态发布，数值等于 `/aft_mapped_to_init` odometry
- `aft_mapped → base_link`：pointlio 启动时发一次静态变换

### Foxglove 面板配置

**面板 1：3D 视图 — 点云 + 全部 TF 帧**

| 订阅类型 | Topic / 设置 | 用途 |
|----------|-------------|------|
| PointCloud | `/cloud_registered_undistort` | 实时点云（`camera_init` 帧） |
| TF | `/tf` + `/tf_static` | 显示所有坐标系 |
| 设置 | 勾选 "Show TF frames" | 可视化坐标轴 |

测试时观察：调用 `ros2 service call /rmcs_relocation/relocalize ...` 成功后，**3D 视图中 `world` 帧位置会立刻跳变**，点云随之平移到正确位置。

**面板 2：Plot — odometry 轨迹**

| 订阅 | 路径 | 说明 |
|------|------|------|
| `/aft_mapped_to_init` | `.pose.pose.position.x` | odom 位置 X |
| `/aft_mapped_to_init` | `.pose.pose.position.y` | odom 位置 Y |
| `/aft_mapped_to_init` | `.pose.pose.position.z` | odom 位置 Z |

**面板 3：Plot — 健康状态**

| 订阅 | 路径 | 说明 |
|------|------|------|
| `/rmcs_relocation/health` | `.state` | 0=HEALTHY, 1=WARNING, 2=UNHEALTHY |
| `/rmcs_relocation/health` | `.residual_median_m` | 残差中位数（m） |
| `/rmcs_relocation/health` | `.inlier_ratio` | 内点比例（0~1） |

**面板 4：Raw Messages — 查看 service 响应**

| 订阅 | Topic |
|------|-------|
| Raw Message | `/rmcs_relocation/health` |

**面板 5（终端）— 实时 `world→odom` 数值**

```bash
ros2 run tf2_ros tf2_echo world odom
```

---

## 1. 适用范围与前提

### 1.1 适用范围

- 传感器：Livox MID360
- 建图：`point_lio`
- 重定位：`rmcs-relocation`
  - 初始化模式：`MODE_INITIAL`（`mode: 0`）
  - 丢失模式：`MODE_LOST`（`mode: 2`）

### 1.2 前置条件

- MID360 已接线上电，网络可达
- `point_lio/config/mid360.yaml` 外参与实车一致
- 测试场地坐标系定义与比赛坐标系一致
- 工作区已完成编译

建议先执行：

```bash
cd /workspaces/RMCS/rmcs_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
```

如未编译，先执行：

```bash
cd /workspaces/RMCS/rmcs_ws
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select rmcs_msgs rmcs-relocation --merge-install --symlink-install
source install/setup.zsh
```

建议先确认接口版本（包含 LOST 新字段）：

```bash
ros2 interface show rmcs_msgs/srv/Relocalize
ros2 interface show rmcs_msgs/msg/LocationHealth
```

---

## 2. 终端分工

- 终端 A：MID360 驱动
- 终端 B：Point-LIO
- 终端 C：检查与服务调用
- 终端 D：`rmcs-relocation` 服务
- 终端 E：`rmcs_executor`（含 `rmcs_relocation_supervisor`）自动触发联调（需要完整硬件，单雷达测试跳过）
- 终端 F（可选）：Foxglove，按 [0.5 节](#05-foxglove-可视化验证) 配置面板

所有终端都先执行：

```bash
cd /workspaces/RMCS/rmcs_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
```

---

## 3. 阶段一：MID360 建图

### 3.1 启动 MID360 驱动（终端 A）

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 3.2 启动 Point-LIO（终端 B）

```bash
ros2 launch point_lio point_lio.launch.py \
  point_lio_cfg_dir:=/workspaces/RMCS/rmcs_ws/src/rmcs-navigation-deps/point_lio/config/mid360.yaml
```

### 3.3 建图前健康检查（终端 C）

```bash
ros2 topic list | rg livox
ros2 topic hz /cloud_registered_undistort
ros2 run tf2_ros tf2_echo odom base_link
```

通过标准：

- 能看到 `livox/lidar_*` 与 `livox/imu_*`
- `/cloud_registered_undistort` 连续输出
- `odom -> base_link` 可查询

### 3.4 实车建图动作建议

- 起步前静止 5~10 秒
- 低速匀速覆盖目标区域
- 补齐拐角、墙边、关键障碍物附近
- 避免高速急转和剧烈震动
- 结束后静止 3~5 秒

### 3.5 保存地图（终端 C）

```bash
ros2 service call /save_pcd_map std_srvs/srv/Trigger "{}"
```

预期：返回 `success: true`。

### 3.6 放置地图到 `rmcs-relocation` 默认路径（终端 C）

```bash
LATEST_PCD=$(ls -t /tmp/point-lio/*.pcd | head -n 1)
echo "$LATEST_PCD"
cp "$LATEST_PCD" /workspaces/RMCS/rmcs_ws/src/rmcs-navigation-deps/rmcs-relocation/maps/world_map.pcd
ls -lh /workspaces/RMCS/rmcs_ws/src/rmcs-navigation-deps/rmcs-relocation/maps/world_map.pcd
```

如果你希望使用自定义地图路径，请修改 `config/location.yaml` 的 `map_path`，或准备单独配置文件并通过 launch 参数传入。

```bash
ros2 launch rmcs-relocation location.launch.py \
  config:=/absolute/path/to/your_location.yaml
```

---

## 4. 阶段二：启动重定位服务并确认接口

### 4.1 启动 `rmcs-relocation`（终端 D）

```bash
ros2 launch rmcs-relocation location.launch.py
```

### 4.2 基础检查（终端 C）

```bash
ros2 service list | rg rmcs_relocation
ros2 service type /rmcs_relocation/relocalize
ros2 param get /rmcs_relocation map_path
ros2 topic hz /cloud_registered_undistort
ros2 run tf2_ros tf2_echo world odom
ros2 topic echo /rmcs_relocation/health
```

通过标准：

- 存在 `/rmcs_relocation/relocalize`
- 类型是 `rmcs_msgs/srv/Relocalize`
- 点云话题连续输出
- `world -> odom` 可读
- `/rmcs_relocation/health` 有持续发布
- `map_path` 指向存在的 `.pcd`

---

## 5. 阶段三：初始化重定位（MODE_INITIAL）测试

说明：

- 初始化只用于“开局定位链路验证”与“倒计时触发验证”。
- 比赛中位置恢复主流程应使用 `MODE_LOST`。
- `MODE_MANUAL` 在服务端当前按 `MODE_INITIAL` 处理，联调时建议直接使用 `MODE_INITIAL`。

### 5.1 默认侧测试（终端 C）

```bash
time ros2 service call /rmcs_relocation/relocalize rmcs_msgs/srv/Relocalize "{
  mode: 0,
  initial_guess_world_base: {
    position: {x: 0.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  },
  pointcloud_topic: '/cloud_registered_undistort',
  collect_duration_sec: 2.0,
  prior_sigma_xy_m: 0.0,
  prior_sigma_yaw_deg: 0.0
}"
```

### 5.2 对侧测试（终端 C）

```bash
time ros2 service call /rmcs_relocation/relocalize rmcs_msgs/srv/Relocalize "{
  mode: 0,
  initial_guess_world_base: {
    position: {x: 21.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}
  },
  pointcloud_topic: '/cloud_registered_undistort',
  collect_duration_sec: 2.0,
  prior_sigma_xy_m: 0.0,
  prior_sigma_yaw_deg: 0.0
}"
```

### 5.3 检查点

每次调用后执行：

```bash
ros2 run tf2_ros tf2_echo world odom
```

关注：

- 成功时：`success: true`、`message: "ok"`、`within_field_bounds: true`
- 失败时：`world -> odom` 不应被错误写入

---

## 6. 阶段四：丢失位置重定位（MODE_LOST）测试

### 6.1 读取当前 `world->base` 作为先验基准（终端 C）

```bash
ros2 run tf2_ros tf2_echo world base_link
```

记录当前姿态 `x y z yaw`。

### 6.2 LOCAL 档（小漂移）测试

构造一个小偏差先验（示例偏差：平移 0.5m，航向 10°）：

```bash
time ros2 service call /rmcs_relocation/relocalize rmcs_msgs/srv/Relocalize "{
  mode: 2,
  initial_guess_world_base: {
    position: {x: 0.5, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0872, w: 0.9962}
  },
  pointcloud_topic: '/cloud_registered_undistort',
  collect_duration_sec: 2.0,
  prior_sigma_xy_m: 1.0,
  prior_sigma_yaw_deg: 15.0
}"
```

### 6.3 WIDE 档（中等漂移）测试

构造中等偏差先验（示例偏差：平移 2~4m，航向 40°）：

```bash
time ros2 service call /rmcs_relocation/relocalize rmcs_msgs/srv/Relocalize "{
  mode: 2,
  initial_guess_world_base: {
    position: {x: 3.0, y: 0.5, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.3420, w: 0.9397}
  },
  pointcloud_topic: '/cloud_registered_undistort',
  collect_duration_sec: 2.0,
  prior_sigma_xy_m: 4.0,
  prior_sigma_yaw_deg: 60.0
}"
```

### 6.4 错误先验拒绝测试

给明显错误先验（例如偏差 > 8m 或 yaw > 90°）：

```bash
time ros2 service call /rmcs_relocation/relocalize rmcs_msgs/srv/Relocalize "{
  mode: 2,
  initial_guess_world_base: {
    position: {x: 12.0, y: 6.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.9239, w: 0.3827}
  },
  pointcloud_topic: '/cloud_registered_undistort',
  collect_duration_sec: 2.0,
  prior_sigma_xy_m: 1.0,
  prior_sigma_yaw_deg: 15.0
}"
```

### 6.5 LOST 模式检查点

- 成功样例至少满足：
  - `success: true`
  - `message: "ok"`
  - `within_field_bounds: true`
  - `fitness_score` 有限
  - `confidence` 在 `0..1`
  - `tier_used` 合理（`0=LOCAL, 1=WIDE`）
- 失败样例至少满足：
  - `success: false`
  - `message` 包含拒绝原因（`score/inlier/distance/yaw/out_of_bounds`）
  - `world -> odom` 不被污染

---

## 7. 阶段五：Supervisor 自动触发联调

本阶段验证运行策略：

- `MODE_INITIAL` 只在 `COUNTDOWN` 上升沿触发
- 其余自动恢复场景都触发 `MODE_LOST`

> **实车前提**：本节依赖 `rmcs_executor` DAG 中的裁判接口（`/referee/game/stage`、`/referee/id`、`/referee/current_hp`），必须连接裁判系统串口。
> **单雷达测试方案**：使用 `scripts/test_supervisor.py` 替代，3 秒后自动发 INITIAL，监控 `/rmcs_relocation/health` 进入 UNHEALTHY 后自动发 LOST（含指数退避）。

### 7.1 实车方式：启动包含 `rmcs_relocation_supervisor` 的执行器（终端 E）

按你当前流程启动 `rmcs_executor`（例如 `launch-rmcs`）。

同时开监视窗口（终端 C）：

```bash
ros2 topic echo /rmcs_relocation/health
ros2 run tf2_ros tf2_echo world odom
```

### 7.2 单雷达方式：运行模拟 Supervisor（终端 E）

```bash
python3 /workspaces/RMCS/rmcs_ws/src/rmcs-navigation-deps/rmcs-relocation/scripts/test_supervisor.py
```

行为：
- 启动 3 秒后模拟 COUNTDOWN 触发一次 `MODE_INITIAL`
- 监控 `/rmcs_relocation/health`，UNHEALTHY 持续 1 秒后触发 `MODE_LOST`
- 失败时指数退避扩大 sigma：`sigma = base * 2^fail_count`（自动从 LOCAL 切 WIDE）
- 连续失败 3 次后暂停
- 成功时保存位姿作为后续 LOST 先验

### 7.3 COUNTDOWN 触发验证

实车：观察日志出现 `initial relocalization armed on COUNTDOWN edge` → `sent initial relocalization request`

单雷达：test_supervisor.py 启动 3 秒后自动触发，日志输出 `[INITIAL] sending`

### 7.4 健康 UNHEALTHY 触发验证

制造定位质量下降后观察：

- `/rmcs_relocation/health` 进入 `STATE_UNHEALTHY`
- 达到停留时间 + 冷却后，日志出现 `[LOST(WIDE)] sending (sigma_xy=...)`

**触发 UNHEALTHY 方法：**
- 将 mid360 物理移动到地图未覆盖区域
- 或临时降低健康阈值（详见 10.4 节症状速查）

### 7.5 复活触发验证（若条件允许）

当裁判系统上报 `current_hp` 从 `0 -> 正值` 时，观察日志：

- 出现 `sent lost relocalization request (trigger=hp revival, ...)`

---

## 8. 验收指标（实测判定）

### 8.1 必须满足（硬门槛）

1. 初始化测试（默认侧+对侧）总计 10 次，成功率 `>= 80%`
2. LOST 小中漂移场景总计 20 次，成功率 `>= 80%`
3. 错误先验场景 10 次，拒绝率 `>= 80%`
4. 所有失败请求不得写坏 `world->odom`
5. `Supervisor` 满足触发策略：
   - COUNTDOWN 触发 INITIAL
   - 非 COUNTDOWN 自动恢复触发 LOST

### 8.2 建议目标（性能）

1. 成功请求 P90 响应时间 `<= 5.0s`
2. 同点位重复 3 次成功请求，`world->odom` 结果波动：
   - 平移 `<= 0.30m`
   - 航向 `<= 5°`

---

## 9. 测试记录模板

| 日期 | 场地 | 车体 | 模式 | 先验(x,y,z,yaw) | sigma_xy | sigma_yaw | success | fitness | confidence | tier_used | within_bounds | 响应时间(s) | world->odom更新 | 备注 |
|---|---|---|---|---|---:|---:|---|---:|---:|---:|---|---:|---|---|
|  |  |  | INITIAL |  | 0.0 | 0.0 |  |  |  |  |  |  |  |  |
|  |  |  | LOST-LOCAL |  | 1.0 | 15.0 |  |  |  |  |  |  |  |  |
|  |  |  | LOST-WIDE |  | 4.0 | 60.0 |  |  |  |  |  |  |  |  |
|  |  |  | LOST-REJECT |  | 1.0 | 15.0 |  |  |  |  |  |  |  |  |

---

## 10. `location.yaml` 参数调优

### 10.0 调参总则

**调参顺序不可乱**：配准质量参数 → 接受阈值参数。先让 registration 收敛到它能达到的最好 score，再根据这个 score 分布设置阈值。

```
max_correspondence_distance_m → submap_radius_m → yaw_step → precise_iterations → score_threshold/inlier/distance/yaw
  └── 配准质量（影响 score 绝对值） ──┘                                    └── 接受阈值（依据实测 score 设置）──┘
```

**核心认知**：fitness_score 是 GICP 配准后剩余对应点均方距离。单帧 mid360 点云约 4000~8000 点，而地图是移动建图的稠密数据，**单帧 vs 全图的 score 天生高于多帧累积的结果**。实车上的 score 目标值（0.01~0.03）不适用于单雷达静态测试，后者 0.04~0.12 属于正常范围。

### 10.1 `field_bounds` — 场地边界

始终最先调，否则误杀正确结果。

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `min_x` / `max_x` | 地图长边范围 | 包围盒 + 1m 裕量 |
| `min_y` / `max_y` | 地图短边范围 | 包围盒 + 1m 裕量 |
| `min_z` / `max_z` | 高度范围 | `[-0.5, 1.0]`，MID360 地面起伏可放宽到 `[-1.0, 2.0]` |

- 现象 `out_of_bounds`：先大幅放宽到 ±50 确认是否为边界误杀，确认后再逐步收紧到包围盒 + 1m。

### 10.2 `initial.*` — 初始化重定位

算法流程（`registration_tools.cpp:353-465`）：

```
query_cloud → voxel_downsample(0.2m) → outlier_removal → 粗搜(yaw窗,coarse_step) → 精搜(refine_step) → 精确配准(precise_iterations) → validator
                                                                              └── score≤threshold 则早停(early_break)
```

| 参数 | 默认值 | 推荐起点 | 作用 | 调大效果 | 调小效果 |
|------|--------|----------|------|----------|----------|
| `collect_duration_sec` | 2.0 | **2.0~3.0** | 采集时长 | 点更多=稳健 | 更快 |
| `min_accumulated_points` | 2500 | **2500** | 最小点数阈值 | 更宽松 | 更严格 |
| `submap_radius_m` | 25.0 | **6~10** | 子图半径(m) | 覆盖大=搜索广 | 无关点少=score好 |
| `max_correspondence_distance_m` | 5.0 | **1.0~1.5** | GICP 最大对应距离(m) | 允许远距离对应 | 过滤错误匹配=score好 |
| `score_threshold` | 0.01 | **0.03~0.05** | ①粗搜早停阈值 ②验收阈值 | 更容易通过 | 更严格 |
| `coarse_iterations` | 50 | **30~50** | 粗搜 GICP 迭代数 | 收敛好 | 更快 |
| `refine_iterations` | 20 | **15~20** | 精搜 GICP 迭代数 | 同上 | 同上 |
| `precise_iterations` | 500 | **300~500** | 精确配准迭代数 | 收敛好=score降 | 更快 |
| `yaw_search_window_deg` | 30.0 | **18~30** | 粗搜 yaw 窗(°) | 搜索更广 | 更快 |
| `coarse_yaw_step_deg` | 15.0 | **9~15** | 粗搜步长(°) | 更快 | 候选多=质量好 |
| `refine_yaw_step_deg` | 5.0 | **3~5** | 精搜步长(°) | 同上 | 同上 |
| `coarse_top_k` | 2 | **2~3** | 精搜候选数 | 候选多=鲁棒 | 更快 |
| `voxel_leaf_m` | 0.2 | **0.2** | 降采样体素(m) | 点少=快 | 点多=精度 |
| `initial_max_translation_error_m` | 2.0 | **1.0~2.0** | 平移验收阈值(m) | 更宽松 | 更严格 |
| `initial_max_yaw_error_deg` | 30.0 | **15~25** | yaw 验收阈值(°) | 更宽松 | 更严格 |

**症状 → 对策速查：**

| 现象 | 原因 | 调整 |
|------|------|------|
| `insufficient query cloud points` | 采集点数不够 | ① 延长 `collect_duration_sec` 到 3.0s ② 降低 `min_accumulated_points` 到 1800 |
| score 略高于 threshold，配准位置正确 | threshold 太紧 | 放宽 `score_threshold` 到实测最大 score + 20% |
| 初值 yaw 有偏差就失败 | 搜索不够 | ① 扩大 `yaw_search_window_deg` ② 缩小 `coarse_yaw_step_deg` |
| 响应太慢 | 计算量过大 | ① 缩小 `submap_radius_m` ② 增大 `coarse_yaw_step_deg` ③ 降 `collect_duration_sec` |
| 粗配都不过 | 对应距离/迭代不够 | ① 增大 `max_correspondence_distance_m` ② 增 `coarse_iterations` |

### 10.3 `lost.*` — 丢失位置重定位

算法流程（`registration_tools.cpp:467-581`）：

```
sigma判定 LOCAL/WIDE → 提取子图 → 点云预处理 → (可选)map_consistency_filter
→ 单种子(LOCAL)或多种子(WIDE, 4方向偏移) → run_lost_seed(粗搜→精搜→精确配准→ranking) → 选最优 → validator
```

**WIDE 模式多种子机制**：在 `prior.world_to_base` 的 X/Y 方向 ±offset（offset = `2 * sigma_xy`，钳位在 0.5 ~ submap_radius_m）生成 4 个额外种子，每个种子独立跑完整配准链，最终按 `ranking_cost = score + 0.5*(1-inlier) + 0.3*(distance/prior_distance_max)` 挑选最优候选。

#### 配准质量参数（先调这些）

| 参数 | 默认值 | 推荐起点 | 作用 |
|------|--------|----------|------|
| `max_correspondence_distance_m` | 2.5 | **1.5~2.5** | GICP 最大对应距离。**减小=score 直接下降**，关键参数 |
| `submap_radius_local_m` | 10.0 | **6~10** | LOCAL 子图半径 |
| `submap_radius_wide_m` | 18.0 | **12~18** | WIDE 子图半径 |
| `coarse_iterations_local` | 30 | **30~40** | LOCAL 粗搜迭代 |
| `coarse_iterations_wide` | 40 | **40~50** | WIDE 粗搜迭代 |
| `refine_iterations_local` | 20 | **20~25** | LOCAL 精搜迭代 |
| `refine_iterations_wide` | 20 | **25~30** | WIDE 精搜迭代 |
| `precise_iterations_local` | 200 | **250~350** | LOCAL 精确迭代。增大 = score 改善 |
| `precise_iterations_wide` | 300 | **400~500** | WIDE 精确迭代。同上 |
| `max_candidate_count` | 3 | **3** | 精搜候选数。WIDE 的每个种子产生一个候选，总数可能 >3 |
| `local_yaw_window_deg` | 20 | **20~30** | LOCAL yaw 搜索窗 |
| `wide_yaw_window_deg` | 60 | **45~60** | WIDE yaw 搜索窗 |
| `local_coarse_yaw_step_deg` | 10 | **8~12** | LOCAL 粗步长。缩小 = 候选质量好 |
| `wide_coarse_yaw_step_deg` | 15 | **10~15** | WIDE 粗步长。同上 |
| `refine_yaw_step_deg` | 5 | **5** | 精搜步长 |
| `enable_map_consistency_filter` | true | **true** | 滤除不在子图中的动态点 |
| `map_consistency_distance_m` | 0.5 | **0.5~0.8** | 一致性距离阈值 |

#### tier 判定参数

| 参数 | 默认值 | 推荐值 | 作用 |
|------|--------|--------|------|
| `local_sigma_xy_m` | 1.0 | **1.0~1.5** | sigma_xy ≤ 此值 → LOCAL，否则 WIDE |
| `local_sigma_yaw_deg` | 15.0 | **15~20** | sigma_yaw ≤ 此值 → LOCAL，否则 WIDE |

#### 接受阈值参数（依据配准质量定值）

| 参数 | 默认值 | 建议依据实测设置 |
|------|--------|-----------------|
| `score_threshold_local` | 0.015 | LOCAL 模式实测 score 的最大值 × 1.3 |
| `score_threshold_wide` | 0.03 | WIDE 模式实测 score 的最大值 × 1.3 |
| `min_inlier_ratio_local` | 0.35 | LOCAL 实测最小值 × 0.8 |
| `min_inlier_ratio_wide` | 0.25 | WIDE 实测最小值 × 0.8 |
| `max_distance_from_prior_local_m` | 1.5 | 与 `local_sigma_xy_m` 配套 |
| `max_distance_from_prior_wide_m` | 5.0 | 与 WIDE sigma 最大缩放配套 |
| `max_yaw_from_prior_local_deg` | 20 | 与 `local_sigma_yaw_deg` 配套 |
| `max_yaw_from_prior_wide_deg` | 60 | 与 WIDE sigma 最大缩放配套 |
| `rank_weight_inlier` | 0.5 | ranking 中 inlier 权重 |
| `rank_weight_distance` | 0.3 | ranking 中 prior 距离权重 |

#### 症状 → 对策速查

| 现象 | 原因 | 调整 |
|------|------|------|
| 配准经常 fails（无候选存活） | 粗配链破坏 | ① 增大 `max_correspondence_distance_m` ② 增 `coarse_iterations_*` ③ 扩大 `submap_radius_*` |
| score 偏高（0.08+）但配准成功 | 对应质量差 | ① **缩小** `max_correspondence_distance_m` ② 增 `precise_iterations_*` ③ 缩小 `submap_radius_*` |
| `rejected: score` | threshold 太紧 | 先查配准质量能否改善（上述），不能则放宽 `score_threshold_*` |
| `rejected: inlier` | 离群点多 | ① 检查动态干扰 ② 开启 `map_consistency_filter` ③ 降低 `min_inlier_ratio_*` |
| `rejected: distance` 或 `yaw` | 先验与结果偏差大 | ① 增大 Supervisor `lost_sigma_*_base` 让系统走 WIDE ② 放宽对应阈值 |
| WIDE 耗时过大 | 搜索空间太大 | ① 保持 `max_candidate_count=3` ② 缩小 `submap_radius_wide_m` ③ 增大 `wide_coarse_yaw_step_deg` |

### 10.4 `health.*` — 健康监控

状态机（`health_monitor.cpp:102-167`）：

```
HEALTHY ──(residual > warn_thresh, dwell warn_dwell_sec)──→ WARNING
WARNING ──(residual > lost_thresh 或 inlier < min_inlier, dwell lost_dwell_sec)──→ UNHEALTHY
UNHEALTHY ──(residual < recover_thresh 且 inlier ≥ min_inlier, dwell recover_dwell_sec)──→ WARNING
WARNING ──(同上)──→ HEALTHY
```

| 参数 | 默认值 | 推荐值 | 作用 |
|------|--------|--------|------|
| `rate_hz` | 5.0 | **3~5** | 健康评估频率 |
| `sample_points` | 500 | **300~500** | 每次评估采样点数 |
| `warn_threshold_m` | 0.25 | **0.25~0.35** | 进入 WARNING 的残差阈值(m) |
| `lost_threshold_m` | 0.45 | **0.40~0.55** | 进入 UNHEALTHY 的残差阈值(m) |
| `min_inlier_ratio` | 0.30 | **0.25~0.30** | 内点比例下限（WARNING→UNHEALTHY 条件之一） |
| `warn_dwell_sec` | 0.6 | **0.6~1.0** | WARNING 停留时间 |
| `lost_dwell_sec` | 1.0 | **1.0~2.0** | UNHEALTHY 停留时间 |
| `recover_margin_m` | 0.05 | **0.05~0.10** | 恢复阈值 = `warn_threshold - recover_margin` |
| `recover_dwell_sec` | 2.0 | **2.0** | 恢复停留时间（防抖动） |
| `inlier_distance_m` | 0.5 | **0.5~0.6** | 内点判定距离阈值 |

**症状速查：**

| 现象 | 调整 |
|------|------|
| 正常行驶频繁 UNHEALTHY | 增大 `lost_threshold_m` + `lost_dwell_sec` |
| 明显漂移不触发 UNHEALTHY | 减小 `lost_threshold_m` ~0.35 |
| 状态来回抖动 | 增大 `recover_dwell_sec` + `recover_margin_m` |

### 10.5 Supervisor 参数

Supervisor 内置于 `rmcs_executor`，通过 YAML 配置（参考 `rmcs_bringup/config/sentry.yaml:70-109`）。在单雷达测试场景中，使用 `scripts/test_supervisor.py` 替代。

| 参数 | 默认值 | 推荐值 | 作用 |
|------|--------|--------|------|
| `health_unhealthy_dwell_sec` | 0.8 | **0.8~1.0** | UNHEALTHY 需持续多久才触发 |
| `lost_cooldown_sec` | 3.0 | **3.0~5.0** | 两次 LOST 请求最小间隔 |
| `lost_max_consecutive_failures` | 5 | **3~5** | 连续失败多少次后暂停 |
| `lost_sigma_xy_base_m` | 1.0 | **1.0~1.5** | LOST 先验 sigma_xy 基值 |
| `lost_sigma_yaw_base_deg` | 20.0 | **15~25** | LOST 先验 sigma_yaw 基值 |
| `retry_interval_sec` | 2.0 | **2.0~3.0** | INITIAL 重试间隔 |
| `request_timeout_sec` | 8.0 | **10~15** | 单次请求超时 |
| `max_retry_count` | 3 | **3** | INITIAL 最大重试次数 |

**指数退避**：LOST 每次失败后 `sigma = base * 2^(failure_count)`，钳位到 sigma_xy≤12.0, sigma_yaw≤180.0。失败 3 次即可从 LOCAL 切换到 WIDE。

### 10.6 快速调参速查表

| 你想… | 改什么 |
|--------|--------|
| 降低 score（让配准更精确） | ↓ `max_correspondence_distance_m`，↑ `precise_iterations`，↓ `submap_radius` |
| 让配准更鲁棒（别经常失败） | ↑ `max_correspondence_distance_m`，↑ `coarse_iterations`，↑ `submap_radius` |
| 让接受更容易 | ↑ `score_threshold_*`，↓ `min_inlier_ratio_*` |
| 让接受更严格 | ↓ `score_threshold_*`，↑ `min_inlier_ratio_*` |
| 加速响应 | ↓ `submap_radius`，↑ `*_yaw_step_deg`，↓ `collect_duration_sec` |
| UNHEALTHY 更敏感 | ↓ `lost_threshold_m`，↓ `lost_dwell_sec` |
| UNHEALTHY 更迟钝 | ↑ `lost_threshold_m`，↑ `lost_dwell_sec` |


