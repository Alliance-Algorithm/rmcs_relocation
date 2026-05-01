#!/usr/bin/env python3
"""模拟 Supervisor 自动触发逻辑，无需裁判系统

对标 rmcs::location::Supervisor 的行为：
1. 模拟 COUNTDOWN 触发 MODE_INITIAL
2. 监控 /rmcs_relocation/health，UNHEALTHY 触发 MODE_LOST
3. 指数退避扩大 sigma（LOCAL→WIDE 自动切换）
4. 保存上一次成功位姿作为 LOST 先验
"""

import rclpy
from rclpy.node import Node
from rmcs_msgs.srv import Relocalize
from rmcs_msgs.msg import LocationHealth
from geometry_msgs.msg import Pose, Point, Quaternion
import time


class TestSupervisor(Node):
    def __init__(self):
        super().__init__("test_supervisor")
        self.cli = self.create_client(Relocalize, "/rmcs_relocation/relocalize")
        self.sub = self.create_subscription(
            LocationHealth, "/rmcs_relocation/health", self.health_cb, 10
        )

        self.unhealthy_since = None
        self.last_lost_time = 0.0
        self.start_time = time.time()

        # ---- 对标 Supervisor config/location.yaml 参数 ----
        self.lost_cooldown = 3.0          # lost_cooldown_sec
        self.unhealthy_dwell = 0.8        # health_unhealthy_dwell_sec
        self.lost_max_consecutive = 5     # lost_max_consecutive_failures
        self.sigma_xy_base = 1.0          # lost_sigma_xy_base_m
        self.sigma_yaw_base = 20.0        # lost_sigma_yaw_base_deg
        self.sigma_xy_max = 12.0
        self.sigma_yaw_max = 180.0

        self.initial_triggered = False
        self.lost_count = 0

        # 保存上次成功位姿作为 LOST 先验 (对标 Supervisor::last_known_world_base_)
        self.last_known_world_base: Pose | None = None

        self.get_logger().info(
            f"Test Supervisor started (lost_cooldown={self.lost_cooldown}s, "
            f"unhealthy_dwell={self.unhealthy_dwell}s, max_consecutive={self.lost_max_consecutive})"
        )
        self.get_logger().info("Simulating COUNTDOWN INITIAL in 3s...")
        self.timer = self.create_timer(0.2, self.loop)

    def health_cb(self, msg):
        now = time.time()
        if msg.state == LocationHealth.STATE_UNHEALTHY:
            if self.unhealthy_since is None:
                self.unhealthy_since = now
                self.get_logger().warn("health → UNHEALTHY detected")
        else:
            if self.unhealthy_since is not None:
                self.get_logger().info(f"health → {msg.state} (recovered)")
            self.unhealthy_since = None

    def loop(self):
        now = time.time()

        # 模拟 COUNTDOWN 触发初始重定位（3 秒后发一次）
        if not self.initial_triggered and (now - self.start_time) > 3.0:
            self.initial_triggered = True
            self.send_request(
                mode=0,
                guess=Pose(position=Point(x=0.0, y=0.0, z=0.0),
                           orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
                duration=2.0,
                sigma_xy=0.0,
                sigma_yaw=0.0,
                label="INITIAL")

        # 健康状态触发 LOST 重定位
        if self.unhealthy_since is not None:
            unhealthy_elapsed = now - self.unhealthy_since
            cooldown_ok = (now - self.last_lost_time) >= self.lost_cooldown
            if unhealthy_elapsed >= self.unhealthy_dwell and cooldown_ok:
                if self.lost_count < self.lost_max_consecutive:
                    scale = 2 ** min(self.lost_count, 8)
                    sigma_xy = min(self.sigma_xy_max, self.sigma_xy_base * scale)
                    sigma_yaw = min(self.sigma_yaw_max, self.sigma_yaw_base * scale)

                    # 使用上次成功位姿作为先验，否则用 identity
                    if self.last_known_world_base is not None:
                        guess = self.last_known_world_base
                    else:
                        guess = Pose(position=Point(x=0.0, y=0.0, z=0.0),
                                     orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

                    # 对标 location.yaml local_sigma_xy_m / local_sigma_yaw_deg
                    tier = "LOCAL" if sigma_xy <= 5.0 and sigma_yaw <= 60.0 else "WIDE"
                    self.send_request(
                        mode=2, guess=guess, duration=2.0,
                        sigma_xy=sigma_xy, sigma_yaw=sigma_yaw,
                        label=f"LOST({tier})")

    def send_request(self, mode, guess, duration, sigma_xy, sigma_yaw, label):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"[{label}] service not ready")
            return
        req = Relocalize.Request()
        req.mode = mode
        req.initial_guess_world_base = guess
        req.pointcloud_topic = "/cloud_registered_undistort"
        req.collect_duration_sec = duration
        req.prior_sigma_xy_m = sigma_xy
        req.prior_sigma_yaw_deg = sigma_yaw

        self.get_logger().info(
            f"[{label}] sending (sigma_xy={sigma_xy:.1f}, sigma_yaw={sigma_yaw:.1f})")
        self.last_lost_time = time.time()

        def done(future):
            resp = future.result()
            if resp is None:
                self.get_logger().error(f"[{label}] no response")
                self.lost_count = min(self.lost_count + 1, self.lost_max_consecutive)
                return
            if resp.success:
                self.get_logger().info(
                    f"[{label}] SUCCESS score={resp.fitness_score:.4f}"
                    f"{f' tier={resp.tier_used}' if mode == 2 else ''}")
                self.lost_count = 0
                # 保存成功位姿作为后续 LOST 先验
                self.last_known_world_base = resp.estimated_world_base
            else:
                self.get_logger().warn(f"[{label}] REJECTED: {resp.message}")
                self.lost_count = min(self.lost_count + 1, self.lost_max_consecutive)
                if self.lost_count >= self.lost_max_consecutive:
                    self.get_logger().error(
                        f"[{label}] max consecutive failures reached, paused")

        self.cli.call_async(req).add_done_callback(done)


def main():
    rclpy.init()
    node = TestSupervisor()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
