# rmcs_relocation

RMCS 重定位模块，提供基于点云配准的位姿重定位服务。

- `rmcs_relocation_server`：重定位服务节点 `/rmcs_relocation/relocalize`
- 三模式：`MODE_INITIAL` / `MODE_LOCAL` / `MODE_WIDE`，由 `rmcs-navigation` 的 Lua 决策主动调用
- 配准核心：`small_gicp`（仓内 vendored）+ ScanContext（可选，用于 wide 全局兜底）

## 三模式概览

| 模式 | 数值 | 用途 | 实现 | 依赖 prior |
| --- | --- | --- | --- | --- |
| `MODE_INITIAL` | 0 | 开局调用一次 | yaw 扫描 + 多阶段 GICP | 是 |
| `MODE_LOCAL` | 1 | 局部纠正，热路径 | ScanContext top-K seed（默认 2）+ 多阶段 GICP + 早停 | prior 仅用于验收约束 |
| `MODE_WIDE` | 2 | 全局兜底 | ScanContext top-K → seed；SC 不可用降级 5 位置 × 8 yaw fallback seed | 否（SC 路径） |

## 环境要求

| 项目 | 推荐/要求 | 说明 |
| --- | --- | --- |
| 操作系统 | Ubuntu 24.04 LTS | 与 ROS 2 Jazzy 官方支持版本一致 |
| ROS 2 | Jazzy | 依赖 `rclcpp/tf2/tf2_ros/launch_ros` |
| 构建系统 | CMake >= 3.16 | 顶层 `CMakeLists.txt` 要求 |
| 点云库 | PCL（common/io/filters/registration/kdtree） | 配准、滤波、KDTree 依赖 |
| 线性代数 | Eigen3 | 位姿与矩阵计算 |
| 并行库 | OpenMP | `small_gicp` 检测到时启用并行 |
| ROS 接口包 | 内置 `srv/Relocalize.srv` | 包内定义，无需外部依赖 |

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


## Lua 调用方式（`rmcs-navigation/src/lua/action.lua`）

```lua
-- 阻塞等待结果，返回 (ok: bool, status: table | nil)
action:relocalize_initial(0, 0, 0,20) --20为超时时间，超时返回false，下面同理
action:relocalize_local(20) 
action:relocalize_wide(20)


-- 非阻塞状态查询
local st = action:relocalize_status()
-- st = { state, message, fitness_score, confidence,
--        estimated_x, estimated_y, estimated_z,
--        estimated_qx, estimated_qy, estimated_qz, estimated_qw }
```

## 启动

```bash
ros2 launch rmcs_relocation location.launch.py
```

常用检查：

```bash
ros2 service list | rg rmcs_relocation
ros2 service type /rmcs_relocation/relocalize     # → rmcs_relocation/srv/Relocalize
ros2 run tf2_ros tf2_echo world odom
```

## ScanContext 启用

WIDE 默认走 fallback（5 位置 × 8 yaw）。要让 wide 用真正的全局识别，需要离线生成描述子库：

### 1) 生成 `.sc_desc` --用离线工具生成


输出会打印 `map_hash = 0x...`，运行时 server 会对地图重算 hash 并校验匹配。

### 2) 修改 `config/location.yaml`

```yaml
descriptor_path: "/tmp/point-lio/1.sc_desc"
scan_context:
    num_rings: 20
    num_sectors: 60
    max_radius_m: 20.0    # 必须与 generator 完全一致，否则启动时 hash mismatch
```
### 3) 从 minpc 拉回 pcd 地图

```bash
scp remote:/tmp/point-lio/1.pcd src/rmcs-navigation-deps/rmcs_relocation/maps/1.pcd
```