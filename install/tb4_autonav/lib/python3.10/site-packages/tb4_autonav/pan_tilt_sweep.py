#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from pan_tilt_msgs.msg import PanTiltCmdDeg
from tb4_autonav_interfaces.msg import YoloTargetBias
from tb4_autonav_interfaces.msg import PickPlaceEvent


class PanTiltSweepController(Node):
    """
    云台控制节点（受 PickPlaceEvent.status 控制）：

    status % 2 == 0 -> Nav 开 -> 云台正常工作（SWEEP / TRACK）
    status % 2 == 1 -> Nav 关 -> 云台 INHIBIT（不发布任何控制指令）

    模式：
      - SWEEP  : 巡航扫描
      - TRACK  : 视觉目标追踪
      - INHIBIT: Nav 关闭，完全不控制云台（不 publish）
    """

    def __init__(self):
        super().__init__("pan_tilt_sweep_controller")

        # ---------------- 参数 ----------------
        self.declare_parameter("yaw_min_deg", -5.0)
        self.declare_parameter("yaw_max_deg", 5.0)
        self.declare_parameter("pitch_min_deg", 18.0)
        self.declare_parameter("pitch_max_deg", 25.0)

        self.declare_parameter("yaw_speed_deg", 8.0)
        self.declare_parameter("pitch_speed_deg", 13.0)
        self.declare_parameter("soft_zone_deg", 0.0)

        self.declare_parameter("track_yaw_limit_deg", 10.0)
        self.declare_parameter("track_pitch_limit_deg", 10.0)
        self.declare_parameter("track_k_yaw", 30.0)
        self.declare_parameter("track_k_pitch", 30.0)
        self.declare_parameter("track_deadband", 0.05)
        self.declare_parameter("track_timeout", 5.0)

        # ---------------- 读取参数 ----------------
        self.yaw_min = float(self.get_parameter("yaw_min_deg").value)
        self.yaw_max = float(self.get_parameter("yaw_max_deg").value)
        self.pitch_min = float(self.get_parameter("pitch_min_deg").value)
        self.pitch_max = float(self.get_parameter("pitch_max_deg").value)

        self.yaw_speed = abs(float(self.get_parameter("yaw_speed_deg").value))
        self.pitch_speed = abs(float(self.get_parameter("pitch_speed_deg").value))
        self.soft_zone = float(self.get_parameter("soft_zone_deg").value)

        self.track_yaw_limit = float(self.get_parameter("track_yaw_limit_deg").value)
        self.track_pitch_limit = float(self.get_parameter("track_pitch_limit_deg").value)
        self.track_k_yaw = float(self.get_parameter("track_k_yaw").value)
        self.track_k_pitch = float(self.get_parameter("track_k_pitch").value)
        self.track_deadband = float(self.get_parameter("track_deadband").value)
        self.track_timeout = float(self.get_parameter("track_timeout").value)

        # 保存 SWEEP 原始范围
        self.base_yaw_min = self.yaw_min
        self.base_yaw_max = self.yaw_max
        self.base_pitch_min = self.pitch_min
        self.base_pitch_max = self.pitch_max

        # ---------------- Publisher ----------------
        self.cmd_pub = self.create_publisher(
            PanTiltCmdDeg,
            "/pan_tilt_cmd_deg",
            10,
        )

        # ---------------- 状态 ----------------
        self.yaw_current = 0.0
        self.pitch_current = 0.0
        self.yaw_dir = 1.0
        self.pitch_dir = 1.0

        self.speed_field = min(max(self.yaw_speed, self.pitch_speed, 1.0), 30.0)

        # 模式管理
        self.mode = "SWEEP"          # SWEEP / TRACK / INHIBIT
        self.prev_mode = "SWEEP"
        self.inhibit_active = False

        # TRACK 相关
        self.target_u = 0.0
        self.target_v = 0.0
        self.last_bias_time = None

        self.last_time = self.get_clock().now()

        # ---------------- Subscriptions ----------------
        self.yolo_sub = self.create_subscription(
            YoloTargetBias,
            "/yolo_target_bias",
            self.yolo_callback,
            10,
        )

        self.pick_place_sub = self.create_subscription(
            PickPlaceEvent,
            "/PickPlaceEvent",
            self.pick_place_callback,
            10,
        )

        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(
            "PanTiltSweepController started.\n"
            "  Nav OFF (status%2==1) -> INHIBIT (no publish)\n"
            "  Nav ON  (status%2==0) -> normal SWEEP/TRACK"
        )

    # ---------------- PickPlaceEvent 回调 ----------------
    def pick_place_callback(self, msg: PickPlaceEvent):
        status = int(msg.status)
        nav_on = (status % 2 == 0)

        if (not nav_on) and (not self.inhibit_active):
            self.inhibit_active = True
            self.prev_mode = self.mode
            self.mode = "INHIBIT"
            self.last_time = self.get_clock().now()
            self.get_logger().info(f"[MODE] INHIBIT (Nav OFF), status={status}")
            return

        if nav_on and self.inhibit_active:
            self.inhibit_active = False
            self.mode = self.prev_mode if self.prev_mode in ("SWEEP", "TRACK") else "SWEEP"
            self.last_bias_time = None
            self.last_time = self.get_clock().now()
            self.get_logger().info(f"[MODE] Resume {self.mode} (Nav ON), status={status}")

    # ---------------- YOLO 回调 ----------------
    def yolo_callback(self, msg: YoloTargetBias):
        if self.mode == "INHIBIT":
            return

        now = self.get_clock().now()

        if msg.has_target:
            self.target_u = float(msg.u_norm)
            self.target_v = float(msg.v_norm)
            self.last_bias_time = now

            if self.mode != "TRACK":
                self.enter_track_mode()
        else:
            if self.mode == "TRACK":
                self.exit_track_mode("has_target=False")

    # ---------------- 模式切换 ----------------
    def enter_track_mode(self):
        self.mode = "TRACK"
        self.yaw_min = -self.track_yaw_limit
        self.yaw_max = self.track_yaw_limit
        self.pitch_min = -self.track_pitch_limit
        self.pitch_max = self.track_pitch_limit

    def exit_track_mode(self, reason=""):
        self.mode = "SWEEP"
        self.yaw_min = self.base_yaw_min
        self.yaw_max = self.base_yaw_max
        self.pitch_min = self.base_pitch_min
        self.pitch_max = self.base_pitch_max

        self.yaw_current = max(min(self.yaw_current, self.yaw_max), self.yaw_min)
        self.pitch_current = max(min(self.pitch_current, self.pitch_max), self.pitch_min)

    # ---------------- 主循环 ----------------
    def control_loop(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time.nanoseconds) * 1e-9
        if dt <= 0:
            return
        self.last_time = now

        # 🚫 Nav 关：完全不发布云台控制
        if self.mode == "INHIBIT":
            return

        if self.mode == "TRACK" and self.last_bias_time is not None:
            if (now - self.last_bias_time).nanoseconds * 1e-9 > self.track_timeout:
                self.exit_track_mode("timeout")

        if self.mode == "SWEEP":
            self.update_sweep(dt)
        else:
            self.update_track(dt)

        cmd = PanTiltCmdDeg()
        cmd.speed = int(self.speed_field)
        cmd.yaw = self.yaw_current
        cmd.pitch = self.pitch_current
        self.cmd_pub.publish(cmd)

    # ---------------- SWEEP ----------------
    def update_sweep(self, dt):
        self.yaw_current, self.yaw_dir = self.update_axis(
            self.yaw_current, self.yaw_dir, self.yaw_min, self.yaw_max, self.yaw_speed, dt
        )
        self.pitch_current, self.pitch_dir = self.update_axis(
            self.pitch_current, self.pitch_dir, self.pitch_min, self.pitch_max, self.pitch_speed, dt
        )

    def update_axis(self, cur, direction, mn, mx, speed, dt):
        if mx <= mn or speed <= 0:
            return cur, direction

        d = (mx - cur) if direction > 0 else (cur - mn)
        d = max(d, 0.0)

        factor = 1.0
        if self.soft_zone > 0 and d <= self.soft_zone:
            factor = 0.1 + 0.9 * (d / self.soft_zone)

        cur += direction * speed * factor * dt

        if cur >= mx:
            cur = mx
            direction = -1.0
        elif cur <= mn:
            cur = mn
            direction = 1.0

        return cur, direction

    # ---------------- TRACK ----------------
    def update_track(self, dt):
        u = 0.0 if abs(self.target_u) < self.track_deadband else self.target_u
        v = 0.0 if abs(self.target_v) < self.track_deadband else self.target_v

        self.yaw_current += -self.track_k_yaw * u * dt
        self.pitch_current += self.track_k_pitch * v * dt

        self.yaw_current = max(-self.track_yaw_limit, min(self.yaw_current, self.track_yaw_limit))
        self.pitch_current = max(-self.track_pitch_limit, min(self.pitch_current, self.track_pitch_limit))


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltSweepController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
