# rmcs-relocation

`rmcs-relocation` 提供两部分能力：

- `rmcs-relocation_server`：重定位服务节点，提供 `/rmcs_relocation/relocalize`。
- 重定位触发策略由 `rmcs-navigation` 的 Lua 决策主动调用。

当前实现基于 `small_gicp`（仓内 vendored）完成点云配准，并通过验证器做结果验收。

## 环境要求

| 项目 | 推荐/要求 | 说明 |
| --- | --- | --- |
| 操作系统 | Ubuntu 24.04 LTS（推荐） | 与 ROS 2 Jazzy 官方支持版本一致。 |
| ROS 2 | Jazzy | 代码中依赖 `rclcpp/tf2/tf2_ros/launch_ros`。 |
| 构建系统 | CMake >= 3.16 | 顶层 `CMakeLists.txt` 要求。 |
| 点云库 | PCL（common/io/filters/registration/kdtree） | 核心配准、滤波、KDTree 依赖。 |
| 线性代数 | Eigen3 | 位姿与矩阵计算依赖。 |
| 并行库 | OpenMP（推荐） | `small_gicp` 检测到 OpenMP 时会启用并行路径。 |
| ROS 接口包 | `rmcs_msgs` | 不是 apt 包，需在同一工作区源码编译。 |
| 地图文件 | `.pcd`（默认 `maps/world_map.pcd`） | 运行时由 `map_path` 参数加载。 |
| 开发方式 | Docker devcontainer（推荐） | RMCS 项目推荐镜像：`qzhhhi/rmcs-develop`。 |

## 依赖安装（APT）

以下为 `rmcs-relocation` 相关核心依赖的 apt 安装示例（Ubuntu 24.04 + ROS 2 Jazzy）：

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

说明：

- `rmcs_msgs` 需在 RMCS 工作区源码中一起编译，不通过 apt 安装。
- `third_party/small_gicp` 已随包内置，不需要单独 apt 安装。

## 构建与运行

### 1) 与 RMCS 集成（当前方案）

当前接入方式为：

- `rmcs-relocation` 仅提供服务 `/rmcs_relocation/relocalize`
- `rmcs-navigation` 在 Lua 决策中主动调用重定位

在 `rmcs_bringup/config/navigation.yaml`（以及需要导航的机器人配置，如 `sentry.yaml`）中，配置 `rmcs_navigation.ros__parameters.localization`：

```yaml
rmcs_navigation:
  ros__parameters:
    endpoint: "main"  # 或对应 endpoint
    localization:
      service_name: "/rmcs_relocation/relocalize"
      pointcloud_topic: "/cloud_registered_undistort"
      collect_duration_sec: 0.0
      request_timeout_sec: 15.0
      lost_local_sigma_xy_m: 1.0
      lost_local_sigma_yaw_deg: 20.0
      lost_wide_sigma_xy_m: 8.0
      lost_wide_sigma_yaw_deg: 120.0
```

Lua 侧通过 `action` 直接调用：

```lua
action:relocalize_initial(x, y, yaw)
action:relocalize_local(x, y, yaw)
action:relocalize_wide(x, y, yaw)
action:relocalize_status()

-- action.lua 内已封装:
action:do_initial() -- 固定 (0,0,0)
action:do_local()   -- 使用 blackboard.user.{x,y,yaw}
action:do_wide()    -- 使用 blackboard.user.{x,y,yaw}
```

#### 参数生效优先级（重要）

- `pointcloud_topic` 与 `collect_duration_sec`：请求值优先，服务端默认值（`location.yaml`）仅在请求为空/<=0 时回退。
- 这意味着：
  - `collect_duration_sec > 0`：`initial` 和 `lost` 都按 navigation 发出的同一值执行。
  - `collect_duration_sec <= 0`：按模式回退到 `location.yaml`，`initial` 用 `initial.collect_duration_sec`，`lost` 用 `lost.collect_duration_sec`。

#### sigma 含义（LOST 模式）

- `lost_local_sigma_xy_m` / `lost_local_sigma_yaw_deg` 与 `lost_wide_sigma_xy_m` / `lost_wide_sigma_yaw_deg`，是 navigation 侧传给服务端的 `prior_sigma`（先验不确定性）。
- `sigma` 数值越小，表示你越相信初始位姿先验；越大表示不确定性更高、搜索范围更宽。
- 服务端 tier 判定规则：
  - `prior_sigma_xy_m <= location.yaml: lost.local_sigma_xy_m`
  - 且 `prior_sigma_yaw_deg <= location.yaml: lost.local_sigma_yaw_deg`
  - 则走 `LOCAL`，否则走 `WIDE`。
- 若请求中的 `prior_sigma <= 0`，服务端会回退为 `local_sigma + 1.0`，从而触发 `WIDE`，不是“按 local 阈值自动选 local”。

#### rmcs-navigation 与非 rmcs-relocation 迁移清单

1. C++ Localization 引擎改造（服务客户端）
- 文件：`rmcs-navigation/src/cxx/util/localization/engine.hh`、`rmcs-navigation/src/cxx/util/localization/engine.cc`
- 从旧的本地 NDT stub 改为 `rmcs_msgs::srv::Relocalize` 客户端。
- 对外接口：
  - `relocalize_initial(x, y, yaw)`
  - `relocalize_local(x, y, yaw)`（使用 `lost_local_sigma_*`）
  - `relocalize_wide(x, y, yaw)`（使用 `lost_wide_sigma_*`）
  - `relocalize_status()`（返回最近一次请求状态）
- 线程安全：
  - 使用 `last_status.state` 作为单一状态源，避免并发重入；
  - 互斥锁保护 request 状态；
  - 请求超时（`request_timeout_sec`）自动转 `FAILED`；
  - 使用回调更新最后结果；
  - 析构时清理 pending request。

2. Component 注入 Lua API
- 文件：`rmcs-navigation/src/cxx/component.cc`
- 在 `Navigation` 构造阶段读取 `localization.*` 参数并创建 `Localization`。
- 在 `make_api_injection()` 注入：
  - `relocalize_initial`
  - `relocalize_local`
  - `relocalize_wide`
  - `relocalize_status`

3. Lua API 与 Action 透传
- 文件：`rmcs-navigation/src/lua/api.lua`、`rmcs-navigation/src/lua/action.lua`
- `action` 层新增方法：
  - `action:relocalize_initial(...)`
  - `action:relocalize_local(...)`
  - `action:relocalize_wide(...)`
  - `action:relocalize_status()`
  - `action:wait_relocalize_done(timeout_sec)`
  - `action:do_initial()` / `action:do_local()` / `action:do_wide()`

4. 状态语义（供 Lua 决策使用）
- `relocalize_*` 返回值：`boolean`，表示“是否成功发起请求”。
- `relocalize_status()` 返回表字段：
  - `state`: `0=IDLE`, `1=IN_FLIGHT`, `2=SUCCEEDED`, `3=FAILED`
  - `message`: 服务返回或错误信息
  - `fitness_score`, `confidence`, `tier_used`

Lua 端典型用法（先发起，再轮询状态）：

```lua
local started = action:relocalize_local(x, y, yaw)
if started then
  local st = action:relocalize_status()
  if st.state == 1 then
    -- in flight
  elseif st.state == 2 then
    -- succeeded, st.tier_used / st.fitness_score 可用于后续决策
  elseif st.state == 3 then
    -- failed, st.message 可用于日志与重试策略
  end
end
```

5. 非 relocation 包的接口改动（移植时别漏）
- `rmcs_msgs`：
  - 新增 `srv/Relocalize.srv`
  - `CMakeLists.txt` 通过 `rosidl_generate_interfaces(... "srv/*.srv")` 生成服务接口
  - `package.xml` 保持 `rosidl_default_generators`/`rosidl_default_runtime`
  - 已删除 `msg/LocationHealth.msg`（health 通道不再用于重定位触发）
- `rmcs-navigation/package.xml`：
  - 依赖 `rmcs_msgs` 与 `rmcs-relocation`
- `rmcs_bringup/config/navigation.yaml`、`rmcs_bringup/config/sentry.yaml`：
  - 新增 `rmcs_navigation.ros__parameters.localization.*` 配置段


### 2) 构建

```bash
cd /workspaces/RMCS/rmcs_ws
source /opt/ros/jazzy/setup.bash
build-rmcs
source install/setup.bash
```

### 3) 启动重定位服务

```bash
ros2 launch rmcs-relocation location.launch.py
```


### 4) 常用检查

```bash
ros2 service list | rg rmcs_relocation
ros2 service type /rmcs_relocation/relocalize
ros2 run tf2_ros tf2_echo world odom
```

## 各模块功能

| 模块 | 主要职责 |
| --- | --- |
| `src/server/runtime.*` | 服务主流程：参数加载、地图加载、INITIAL/LOST 处理、TF 发布。 |
| `src/server/collector.*` | 点云采集与坐标系转换（输入 topic，输出 odom 点云）。 |
| `src/tools/registration_tools.*` | 配准核心：点云预处理、子图提取、INITIAL/LOST 配准策略。 |
| `src/server/validator.*` | 重定位结果验收：边界、score、inlier、先验距离、yaw 等。 |

## YAML 参数说明（详见`config/location.yaml`）

参数根节点：

```yaml
rmcs_relocation:
  ros__parameters:
    ...
```

## ToDo Lists
- [x] 测试initial，lost，supervisor基础功能
- [x] initial mode静态调参，目标5-6s内完成。
- [x] lost mode静态调参，目标2-3s内完成。
- [ ] initial，local实车测试调优
- [ ] wide调参
