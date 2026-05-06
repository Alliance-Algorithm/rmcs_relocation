# Relocalize.srv 迁移方案

> 将 `Relocalize.srv` 从 `rmcs_msgs` 移到 `rmcs_relocation`，`rmcs_msgs` 恢复为纯 header-only 包。

## 影响面分析

`rmcs_msgs::srv::Relocalize` 有两个消费者：

| 包 | 角色 | 核心文件 |
|---|------|---------|
| `rmcs_relocation` | **服务端**（`create_service`） | `src/server/runtime.cpp` |
| `rmcs-navigation` | **客户端**（`create_client`） | `src/cxx/util/localization/engine.cc` |

`rmcs-navigation` 同时还用 `rmcs_msgs` 的其他 header-only 类型（`rmcs_msgs::to_string()` on game_stage/switch），所以不能完全断开 `rmcs_msgs`。

## Part A — `rmcs_relocation`（服务端 + 接口定义）

### A1. 新建 `srv/Relocalize.srv`

```sh
mkdir -p srv
```

内容与 `rmcs_msgs/srv/Relocalize.srv` 相同：

```srv
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

### A2. `package.xml` — 新增 3 行

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
...
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

同时删除 `<depend>rmcs_msgs</depend>`。

### A3. `CMakeLists.txt` — 3 处改动

1. `find_package(rmcs_msgs REQUIRED)` → `find_package(rosidl_default_generators REQUIRED)`
2. 在 `add_subdirectory(src/tools)` 之前插入 srv 生成：
   ```cmake
   rosidl_generate_interfaces(
     ${PROJECT_NAME}
     "srv/Relocalize.srv"
     DEPENDENCIES geometry_msgs
   )
   ```
3. `ament_target_dependencies(rmcs_relocation_server ...)` 中：
   - 删除 `rmcs_msgs`
   - 替换为 `rosidl_target_interfaces(rmcs_relocation_server ${PROJECT_NAME} "rosidl_typesupport_cpp")`

### A4. `src/tools/CMakeLists.txt` — 1 处改动

`ament_target_dependencies(rmcs_relocation_tools_common ...)` 中删除 `rmcs_msgs`。

### A5. `src/server/runtime.cpp` — 批量替换

- `#include <rmcs_msgs/srv/relocalize.hpp>` → `#include <rmcs_relocation/srv/relocalize.hpp>`
- `rmcs_msgs::srv::Relocalize` → `rmcs_relocation::srv::Relocalize`（30+ 处）

## Part B — `rmcs-navigation`（客户端）

### B1. `package.xml`

`<exec_depend>rmcs_relocation</exec_depend>` → `<depend>rmcs_relocation</depend>`

### B2. `CMakeLists.txt`

新增：
```cmake
find_package(rmcs_relocation REQUIRED)
```
并添加：
```cmake
${rmcs_relocation_INCLUDE_DIRS}
${rmcs_relocation_LIBRARIES}
```

### B3. `src/cxx/util/localization/engine.cc`

- `#include <rmcs_msgs/srv/relocalize.hpp>` → `#include <rmcs_relocation/srv/relocalize.hpp>`
- `using Relocalize = rmcs_msgs::srv::Relocalize` → `rmcs_relocation::srv::Relocalize`

## Part C — `rmcs_msgs`（恢复原版）

恢复到纯 header-only 包状态：

### C1. 删除文件

```sh
rm -rf srv/
```

### C2. `CMakeLists.txt` 恢复

改用 `ament_cmake_auto`，去掉 `rosidl_generate_interfaces` 和 srv 相关处理，回归 `include_directories + ament_auto_package` 模式。

### C3. `package.xml` 恢复

删除 `rosidl_default_generators`、`rosidl_default_runtime`，回到只有 `ament_cmake` buildtool 的原版。

## 注意事项

- 包名 `rmcs_relocation`（带连字符），`rosidl_generate_interfaces` 生成的 C++ 命名空间自动转为 `rmcs_relocation`（下划线）
- `rmcs_relocation` 已有 `<depend>geometry_msgs</depend>`，满足 srv 字段类型依赖
- `rmcs_msgs` 的 6 个 `.msg` 文件保留在磁盘，但不再通过 CMake 生成（原版即为 header-only）
