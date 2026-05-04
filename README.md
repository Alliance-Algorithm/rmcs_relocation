# rmcs-relocation

RMCS 重定位模块，提供基于点云配准的位姿重定位服务。

- `rmcs-relocation_server`：重定位服务节点 `/rmcs_relocation/relocalize`
- 三模式：`MODE_INITIAL` / `MODE_LOCAL` / `MODE_WIDE`，由 `rmcs-navigation` 的 Lua 决策主动调用
- 配准核心：`small_gicp`（仓内 vendored）+ ScanContext（可选，用于 wide 全局兜底）

## 三模式概览

| 模式 | 数值 | 用途 | 实现 | 依赖 prior |
| --- | --- | --- | --- | --- |
| `MODE_INITIAL` | 0 | 开局调用一次 | yaw 扫描 + 多阶段 GICP | 是 |
| `MODE_LOCAL` | 1 | 局部纠正，热路径 | 单 seed @ prior + 多阶段 GICP | 是（强） |
| `MODE_WIDE` | 2 | 全局兜底 | ScanContext top-K → seed；SC 不可用降级 5×8 = 40 fallback seed | 否（SC 路径） |

## 环境要求

| 项目 | 推荐/要求 | 说明 |
| --- | --- | --- |
| 操作系统 | Ubuntu 24.04 LTS | 与 ROS 2 Jazzy 官方支持版本一致 |
| ROS 2 | Jazzy | 依赖 `rclcpp/tf2/tf2_ros/launch_ros` |
| 构建系统 | CMake >= 3.16 | 顶层 `CMakeLists.txt` 要求 |
| 点云库 | PCL（common/io/filters/registration/kdtree） | 配准、滤波、KDTree 依赖 |
| 线性代数 | Eigen3 | 位姿与矩阵计算 |
| 并行库 | OpenMP | `small_gicp` 检测到时启用并行 |
| ROS 接口包 | `rmcs_msgs` | 同一 workspace 源码编译 |
| 离线 SC 生成（可选） | Python 3 + numpy（可选 open3d） | `scripts/generate_map_descriptors.py` 用 |

## 依赖安装（APT）

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git pkg-config \
  python3-colcon-common-extensions \
  libeigen3-dev libpcl-dev libomp-dev \
  ros-jazzy-rclcpp \
  ros-jazzy-tf2 ros-jazzy-tf2-ros \
  ros-jazzy-geometry-msgs ros-jazzy-sensor-msgs \
  ros-jazzy-pcl-conversions \
  ros-jazzy-launch ros-jazzy-launch-ros
```

可选（启用 ScanContext 时离线生成描述子用）：

```bash
pip3 install numpy open3d   # 二进制 PCD 需要 open3d；ASCII PCD 仅 numpy
```

## 服务接口（`rmcs_msgs/srv/Relocalize.srv`）

```
uint8 MODE_INITIAL=0
uint8 MODE_LOCAL=1
uint8 MODE_WIDE=2

uint8 mode
geometry_msgs/Pose initial_guess_world_base
string pointcloud_topic
float32 collect_duration_sec
---
bool success
string message
geometry_msgs/Pose estimated_world_base
geometry_msgs/Transform world_to_odom
float32 fitness_score
bool within_field_bounds
float32 confidence
```

## Lua 调用方式（`rmcs-navigation/src/lua/action.lua`）

```lua
-- 阻塞等待结果，返回 (ok: bool, status: table | nil)
action:relocalize("initial", x, y, yaw, timeout_sec)
action:relocalize("local_", x, y, yaw, timeout_sec)
action:relocalize("wide",   x, y, yaw, timeout_sec)

-- LIO/TF 兜底：blackboard.user.x/y/yaw 任一为 NaN 则降级到 wide+原点（timeout × 2）
action:try_relocalize_with_fallback("local_", timeout_sec)

-- 非阻塞状态查询
local st = action:relocalize_status()
-- st = { state, message, fitness_score, confidence,
--        estimated_x, estimated_y, estimated_z,
--        estimated_qx, estimated_qy, estimated_qz, estimated_qw }
```

## bringup 配置

`rmcs_bringup/config/{navigation,sentry}.yaml`：

```yaml
rmcs_navigation:
  ros__parameters:
    endpoint: "main"
    localization:
      service_name: "/rmcs_relocation/relocalize"
      pointcloud_topic: "/cloud_registered_undistort"
      collect_duration_sec: 0.0     # 0 = 用 server 端各 mode 的默认值
      request_timeout_sec: 15.0
```

## 构建

```bash
cd /workspaces/RMCS/rmcs_ws
source /opt/ros/jazzy/setup.bash
build-rmcs
source install/setup.bash
```

## 启动

```bash
ros2 launch rmcs-relocation location.launch.py
```

常用检查：

```bash
ros2 service list | rg rmcs_relocation
ros2 service type /rmcs_relocation/relocalize     # → rmcs_msgs/srv/Relocalize
ros2 run tf2_ros tf2_echo world odom
```

## ScanContext 启用

WIDE 默认走 fallback（5 位置 × 8 yaw = 40 seed）。要让 wide 用真正的全局识别，需要离线生成描述子库：

### 1) 生成 `.sc_desc`

```bash
python3 src/rmcs-navigation-deps/rmcs-relocation/scripts/generate_map_descriptors.py \
    --map /tmp/point-lio/1.pcd \
    --output /tmp/point-lio/1.sc_desc \
    --num-rings 20 --num-sectors 60 --max-radius 20.0 \
    --grid-step 2.0 --min-points-per-grid 200
```

输出会打印 `map_hash = 0x...`，运行时 server 会对地图重算 hash 并校验匹配。

### 2) 修改 `config/location.yaml`

```yaml
descriptor_path: "/tmp/point-lio/1.sc_desc"
scan_context:
    num_rings: 20
    num_sectors: 60
    max_radius_m: 20.0    # 必须与 generator 完全一致，否则启动时 hash mismatch
```

### 3) 启动确认日志

| 日志 | 含义 |
| --- | --- |
| `scan_context descriptor loaded: K entries (...)` | SC 启用，wide 走 SC 主路径 |
| `descriptor_path is empty, scan_context disabled` | yaml 默认配置，wide 走 fallback |
| `scan_context descriptor load failed (...)` | 文件缺失/坏文件/版本不匹配/hash 不匹配 → wide 自动 fallback |
| `wide: SC returned K candidates` | 单次 wide 调用走了 SC 主路径 |
| `wide: SC query empty, falling back to multi-seed` | 单次 SC 匹配失败，本次降级 |

## 调试

`config/location.yaml`：

```yaml
diagnostics:
    log_failure_details: true   # 每次失败 dispatcher 多打一行：mode + prior + score + conf + msg
```

## 各模块功能

| 模块 | 主要职责 |
| --- | --- |
| `src/server/runtime.*` | 服务主流程、参数加载、地图加载、三 mode dispatcher、TF 发布、SC bootstrap |
| `src/server/collector.*` | 点云采集与坐标系转换（输入 topic，输出 odom 点云） |
| `src/server/validator.*` | 三 mode 验收：边界、score、inlier、prior 距离 / yaw |
| `src/server/map_descriptor_db.*` | `.sc_desc` 加载 + map_hash 校验 + top-K 查询 |
| `src/tools/registration_tools.*` | `run_initial / run_local / run_wide` 配准核心 + `build_wide_fallback_seeds` |
| `src/tools/scan_context.*` | SC 描述子构造 + 旋转不变匹配 + map hash（FNV-1a，与 Python 同构） |
| `src/tools/param_tools.*` | yaml 参数加载 |
| `scripts/generate_map_descriptors.py` | 离线生成 `.sc_desc`（与 C++ 字节序、采样规则、哈希严格同构） |

## 配置参数

详见 `config/location.yaml`。顶层结构：

```yaml
rmcs_relocation:
  ros__parameters:
    map_path: ...
    descriptor_path: ""               # 留空则关 SC

    scan_context:                     # SC 描述子参数（与 generator 必须一致）
      num_rings, num_sectors, max_radius_m

    diagnostics:                      # 调试日志
      log_failure_details: false

```

## 移植/集成 checklist

- `rmcs_msgs`：`srv/Relocalize.srv`（已有），`CMakeLists.txt` 通过 `rosidl_generate_interfaces` 生成接口
- `rmcs-navigation/package.xml`：依赖 `rmcs_msgs` 与 `rmcs-relocation`
- `rmcs_bringup/config/{navigation,sentry}.yaml`：`rmcs_navigation.ros__parameters.localization.*`
- 部署目录：`maps/*.pcd`（必需）+ 可选的 `maps/*.sc_desc`

## 已知限制

- 帧内 ~11° smear @ 2 rad/s：源于 point_lio 的 `IMU_Processing` 未做 per-point deskew + `pointBodyToWorld` 用单一 scan-end pose；本仓不涉及，需 fork upstream 解决
- ScanContext 在几何对称环境（长走廊、空旷场地）会出双峰候选，依赖 `sc_top_k > 1` + GICP fitness ranking 收敛到正确解
