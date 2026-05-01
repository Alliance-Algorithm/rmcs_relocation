# rmcs-relocation

`rmcs-relocation` 提供两部分能力：

- `rmcs-relocation_server`：重定位服务节点，提供 `/rmcs_relocation/relocalize` 与 `/rmcs_relocation/health`。
- `rmcs-relocation_component`：`rmcs_executor` 插件组件 `rmcs::location::Supervisor`，负责自动触发 INITIAL/LOST 重定位。

当前实现基于 `small_gicp`（仓内 vendored）完成点云配准，并通过验证器做结果验收。

## 环境要求

| 项目 | 推荐/要求 | 说明 |
| --- | --- | --- |
| 操作系统 | Ubuntu 24.04 LTS（推荐） | 与 ROS 2 Jazzy 官方支持版本一致。 |
| ROS 2 | Jazzy | 代码中依赖 `rclcpp/tf2/tf2_ros/launch_ros`。 |
| 构建系统 | CMake >= 3.16 | 顶层 `CMakeLists.txt` 要求。 |
| C++ 标准 | C++23 | `rmcs-relocation_component` 与 `rmcs-relocation_server` 均设置 `cxx_std_23`。 |
| 编译器 | GCC 或 Clang（支持 C++23） | 建议 GCC 13+。 |
| 点云库 | PCL（common/io/filters/registration/kdtree） | 核心配准、滤波、KDTree 依赖。 |
| 线性代数 | Eigen3 | 位姿与矩阵计算依赖。 |
| 并行库 | OpenMP（推荐） | `small_gicp` 检测到 OpenMP 时会启用并行路径。 |
| ROS 接口包 | `rmcs_msgs`、`rmcs_executor` | 两者不是 apt 包，需在同一工作区源码编译。 |
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
  ros-jazzy-pluginlib ros-jazzy-pcl-conversions \
  ros-jazzy-launch ros-jazzy-launch-ros
```

说明：

- `rmcs_msgs`、`rmcs_executor` 需在 RMCS 工作区源码中一起编译，不通过 apt 安装。
- `third_party/small_gicp` 已随包内置，不需要单独 apt 安装。

## 构建与运行

### 1) 与RMCS集成
1. (在rmcs-navigation的package.xml中添加依赖)
```xml
  <exec_depend>rmcs-relocation</exec_depend>  
```
2. 在sentry.yaml中添加Supervisor组件配置
```yaml
rmcs_relocation_supervisor:
  ros__parameters:
    service_name: "/rmcs_relocation/relocalize"
    world_frame: "world"
    odom_frame: "odom"
    base_frame: "base_link"
    initial:
      pointcloud_topic: "/cloud_registered_undistort"
      collect_duration_sec: 2.0
    lost:
      pointcloud_topic: "/cloud_registered_undistort"
      collect_duration_sec: 2.0
    retry_interval_sec: 2.0
    request_timeout_sec: 15.0
    max_retry_count: 3
    health_unhealthy_dwell_sec: 0.8
    lost_cooldown_sec: 3.0
    lost_max_consecutive_failures: 5
    lost_sigma_xy_base_m: 1.0
    lost_sigma_yaw_base_deg: 20.0
    default_initial_guess:
      translation:
        x: 0.0
        y: 0.0
        z: 0.0
      orientation:
        w: 1.0
        x: 0.0
        y: 0.0
        z: 0.0
    opposite_initial_guess:
      translation:
        x: 21.0
        y: 0.0
        z: 0.0
      orientation:
        w: 0.0
        x: 0.0
        y: 0.0
        z: 1.0

```
3. rmcs_msgs新增服务消息
- 添加rmcs_msgs/service/Relocalize.srv
```
  uint8 MODE_INITIAL=0
  uint8 MODE_MANUAL=1
  uint8 MODE_LOST=2

  uint8 mode
  geometry_msgs/Pose initial_guess_world_base
  string pointcloud_topic
  float32 collect_duration_sec
  float32 prior_sigma_xy_m
  float32 prior_sigma_yaw_deg
  ---
  bool success
  string message
  geometry_msgs/Pose estimated_world_base
  geometry_msgs/Transform world_to_odom
  float32 fitness_score
  bool within_field_bounds
  float32 confidence
  uint8 tier_used

```
- 添加rmcs_msgs/msg/LocationHealth.msg
```
  uint8 STATE_HEALTHY=0
  uint8 STATE_WARNING=1
  uint8 STATE_UNHEALTHY=2

  uint8 state
  float32 residual_median_m
  float32 inlier_ratio
  builtin_interfaces/Time stamp

```
- CmakeLists.txt和package.xml配置
```cmake
  cmake_minimum_required(VERSION 3.12)
  project(rmcs_msgs)

  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-O2 -Wall -Wextra -Wpedantic)
  endif()

  find_package(ament_cmake_auto REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  ament_auto_find_build_dependencies()

  file(
    GLOB RMCS_MSGS_MSGS
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
    "msg/*.msg"
  )
  file(
    GLOB RMCS_MSGS_SRVS
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
    "srv/*.srv"
  )

  rosidl_generate_interfaces(
    ${PROJECT_NAME}
    ${RMCS_MSGS_MSGS}
    ${RMCS_MSGS_SRVS}
    DEPENDENCIES
      builtin_interfaces
      std_msgs
      geometry_msgs
  )

  include_directories(${PROJECT_SOURCE_DIR}/include)

  ament_export_dependencies(rosidl_default_runtime)

  ament_auto_package()
```

```xml
  <?xml version="1.0"?>
  <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
  <package format="3">
    <name>rmcs_msgs</name>
    <version>0.0.0</version>
    <description>TODO: Package description</description>
    <maintainer email="zihanqin2048@gmail.com">Qzh</maintainer>
    <license>TODO: License declaration</license>

    <buildtool_depend>ament_cmake</buildtool_depend>
    <buildtool_depend>rosidl_default_generators</buildtool_depend>
    
    <member_of_group>rosidl_interface_packages</member_of_group>
    
    <depend>rclcpp</depend>
    <depend>std_msgs</depend>
    <depend>geometry_msgs</depend>
    <depend>builtin_interfaces</depend>
    <exec_depend>rosidl_default_runtime</exec_depend>

    <export>
      <build_type>ament_cmake</build_type>
    </export>
  </package>
```


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
ros2 topic echo /rmcs_relocation/health
ros2 run tf2_ros tf2_echo world odom
```

## 各模块功能

| 模块 | 主要职责 |
| --- | --- |
| `src/server/runtime.*` | 服务主流程：参数加载、地图加载、INITIAL/LOST 处理、TF 发布、健康监测发布。 |
| `src/server/collector.*` | 点云采集与坐标系转换（输入 topic，输出 odom 点云）。 |
| `src/tools/registration_tools.*` | 配准核心：点云预处理、子图提取、INITIAL/LOST 配准策略。 |
| `src/server/validator.*` | 重定位结果验收：边界、score、inlier、先验距离、yaw 等。 |
| `src/server/health_monitor.*` | 健康度评估状态机：HEALTHY/WARNING/UNHEALTHY。 |
| `src/tools/param_tools.*` | 统一参数读取，组装 runtime/registration/validation/health 配置。 |
| `src/component/supervisor.*` | `rmcs_executor` 插件，自动触发重定位服务调用。 |
| `src/tools/geometry_tools.*` | Pose/Transform 与 Eigen Isometry 互转。 |

## YAML 参数说明（详见`config/location.yaml`）

参数根节点：

```yaml
rmcs_relocation:
  ros__parameters:
    ...
```

## Supervisor 组件参数（由上层组件 YAML 注入）

`Supervisor` 是插件组件，不读取 `config/location.yaml`，其参数写在机器人组件配置 sentry.yaml 中。

## 常见问题

- 启动后 `map unavailable`  
  - 检查 `map_path` 是否存在且可读，确认 `.pcd` 文件路径正确。
- 服务存在但持续 `initial/lost registration failed`  
  - 检查点云话题是否有数据、`odom->base_link` TF 是否可查、`min_accumulated_points` 是否过大。
- `lost registration rejected:*`  
  - 代表配准算出候选，但被验收器拒绝；需要联调 `score/inlier/field_bounds/先验阈值` 参数。

## ToDo Lists
- [x] 测试initial，lost，supervisor基础功能
- [x] initial mode静态调参，目标5-6s内完成。
- [x] lost mode静态调参，目标2-3s内完成。
- [ ] initial，local实车测试调优
- [ ] health,supervisor实车联调
- [ ] wide调参
- [ ] 实车全程测试跑supervisor自动监控health，调度重定位