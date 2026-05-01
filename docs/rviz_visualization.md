# RVIZ 可视化重定位方案

## 目标

在 RVIZ 中直观观察初始重定位结果（地图 + 估算位姿 + TF 链路），零主代码侵入。

## 设计原则

- 不修改 `src/` 下任何 C++ 代码（`runtime.cpp` 等不动）
- 通过独立 Python 节点补足缺失的 topic
- 所有新增文件在 `scripts/`、`launch/`、`config/` 下

## 新增文件

| 文件 | 作用 |
|---|---|
| `scripts/pcd_publisher.py` | 读取 PCD 地图文件，发布为 `transient_local` 点云 |
| `scripts/tf_to_pose.py` | 订阅 TF，将 `world→base_link` 转为 PoseStamped 发布 |
| `launch/rviz.launch.py` | 启动上述两个节点 + 可选 rviz2 |
| `config/rviz.rviz` | 预置 RVIZ 布局 |

## 数据流

```
pcd_publisher    ──PointCloud2──▶  /rmcs_relocation/map          (一次，地图)
tf_to_pose       ──PoseStamped──▶  /rmcs_relocation/estimated_pose (10Hz，位姿)
rmcs_relocation  ──TF──────────▶  world→odom→base_link          (10Hz，链路)
```

## RVIZ 配置

| Display | Topic | 说明 |
|---|---|---|
| PointCloud2 | `/rmcs_relocation/map` | 静态地图，灰色显示 |
| PoseStamped | `/rmcs_relocation/estimated_pose` | 绿色坐标轴，机器人在世界系位姿 |
| TF | (内置) | 显示 `world→odom→base_link` 箭头链 |

首次重定位成功后，绿色轴从地图对应点跳出，TF 链路 `world→odom` 跳变到正确变换。

## 使用方式

```bash
# 正常启动 relocation server 后
ros2 launch rmcs-relocation rviz.launch.py

# 附带打开 rviz
ros2 launch rmcs-relocation rviz.launch.py use_rviz:=true
```

## LOST 错误先验拒绝验证

收紧 `location.yaml` 中 LOCAL 的先验距离验收阈值：

```yaml
# 从 3.0 收紧到 1.5
max_distance_from_prior_local_m: 1.5
```

测试：故意给偏 2m 的先验，应触发 validator 拒绝。

```bash
ros2 service call /rmcs_relocation/relocalize rmcs_msgs/srv/Relocalize "{
  mode: 2,
  initial_guess_world_base: {
    position: {x: 2.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  },
  pointcloud_topic: '/cloud_registered_undistort',
  collect_duration_sec: 0.7,
  prior_sigma_xy_m: 0.5,
  prior_sigma_yaw_deg: 10.0
}"
```

预期响应：
```
success=False
message='lost registration rejected: distance'
```
