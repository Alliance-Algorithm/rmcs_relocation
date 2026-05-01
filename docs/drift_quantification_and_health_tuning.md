# point_lio 漂移量化与 Health 调参手册

## 目标

不上 re-localization，纯跑 point_lio odometry，录制 health 数据。用采集到的残差/内点比统计量，反推 `location.yaml` 中的 health 阈值。

---

## 一、采集步骤

### 1. 起点定标（只做一次 INITIAL）

```bash
ros2 service call /rmcs_relocation/relocalize rmcs_msgs/srv/Relocalize "{
  mode: 0,
  initial_guess_world_base: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
  pointcloud_topic: '/cloud_registered_undistort',
  collect_duration_sec: 2.0,
  prior_sigma_xy_m: 0.0, prior_sigma_yaw_deg: 0.0
}"
```

成功（`success=True`）后 `world→odom` 已建立且 **不再更新**。之后 point_lio 漂移将直接体现在 health 指标上。

### 2. 录制 rosbag

```bash
ros2 bag record -o pointlio_drift_$(date +%Y%m%d_%H%M) \
  /tf \
  /tf_static \
  /rmcs_relocation/health \
  /cloud_registered_undistort
```

### 3. 跑测试路径

- **建议闭环**（起点=终点），总长 100~200m
- 含直行、转弯（正常比赛动作）
- **不需要做异常机动**（翻倒、撞墙等极端情况另行测试）
- 记录跑完用时和大致路径形状

### 4. 停止录制

Ctrl+C 停止 rosbag。bag 文件在 `pointlio_drift_*/` 目录下。

---

## 二、Foxglove 查看

### 面板配置

| 面板类型 | Topic | 数据 | 看什么 |
|---|---|---|---|
| Plot | `/rmcs_relocation/health.residual` | 残差中位数(m) | 全程残差曲线，关注趋势和尖峰 |
| Plot | `/rmcs_relocation/health.inlier_ratio` | 内点比例(0~1) | 下降趋势，最低点 |
| Plot | `/rmcs_relocation/health.state` | 状态码(0/1/2) | 当前阈值下何时切 WARNING/UNHEALTHY |
| 3D | `/cloud_registered_undistort` | 点云(world 系) | 点云是否逐渐偏离地图 |
| 3D | Map 点云（话题 `/rmcs_relocation/map`） | 静态背景 | 参考基准 |

**播放 bag 时**：Foxglove 左下角 Data Source → "Open Local File" → 选 `.mcap` 文件。

### 关键观测

1. **残差起步值**：起点附近的中位数（应 <0.1m）
2. **残差终点值**：闭环回到起点时的值（反映回路漂移）
3. **残差最大值**：全程最大尖峰
4. **内点比最低值**：全程最低点位
5. **残差随时间走势**：线性增长还是局部跳动

---

## 三、从数据定阈值

### 取数方法

在 Foxglove Plot 面板上框选关键区间，读数。或导出 CSV 后用 Python/pandas 算：

```python
import pandas as pd
# 从 bag 提取后
df = pd.read_csv("health.csv")
median_residual = df["residual"].median()
max_residual = df["residual"].max()
min_inlier = df["inlier_ratio"].min()
median_inlier = df["inlier_ratio"].median()
```

### 参数公式

| 参数 (location.yaml) | 公式 | 说明 |
|---|---|---|
| `warn_threshold_m` | 全程残差中位数 × **2.0** | 正常行驶 2 倍裕度；有急转/坡道时用 2.5 |
| `lost_threshold_m` | 全程残差中位数 × **3.5** | 明显漂移才触发；若漂移本身小则用绝对 0.5m 兜底 |
| `min_inlier_ratio` | 全程内点比中位数 × **0.6** | 低于正常 60% 视为异常 |
| `recover_margin_m` | 0.1~0.3 | 低于 warn 多少才能恢复，防状态抖动 |
| `warn_dwell_sec` | 急转弯时 WARNING 最长持续时间 × 1.5 | 防短时干扰误触发 |
| `lost_dwell_sec` | 同上，取值更大 | 进入 UNHEALTHY 前的确认时长 |
| `recover_dwell_sec` | 2.0~3.0 | 恢复确认时长，防 HEALTHY↔WARNING 来回跳 |

### 示例

假设某次实车 200m 闭环测试：
- 残差中位数 = 0.15m，最大值 = 0.52m
- 内点比中位数 = 0.45，最小值 = 0.22
- 急转时 WARNING 最长持续 1.8s

**定值**：
```
warn_threshold_m:   0.30   (0.15 × 2.0)
lost_threshold_m:   0.52   (0.15 × 3.5 = 0.525, 取 0.52)
min_inlier_ratio:   0.27   (0.45 × 0.6)
recover_margin_m:   0.15
warn_dwell_sec:     2.7    (1.8 × 1.5)
lost_dwell_sec:     3.0
recover_dwell_sec:  2.5
```

---

## 四、校验

调完参数后，重新跑一次同路径，Foxglove 观察：

- **正常行驶**：health state 全程 `HEALTHY=0`
- **急转弯/坡道**：短暂 `WARNING=1` 后自行恢复
- **明显误漂**：切到 `UNHEALTHY=2`，Supervisor 应在 `lost_dwell_sec + unhealthy_dwell` 后触发 LOST

如果频繁误切 UNHEALTHY → 增大 `lost_threshold_m` 或加长 `lost_dwell_sec`。
如果明显漂移迟迟不切 → 减小 `lost_threshold_m`。
