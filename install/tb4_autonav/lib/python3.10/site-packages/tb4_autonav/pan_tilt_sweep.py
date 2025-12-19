#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from pan_tilt_msgs.msg import PanTiltCmdDeg
from tb4_autonav_interfaces.msg import YoloTargetBias
from tb4_autonav_interfaces.msg import PickPlaceEvent


class PanTiltSweepController(Node):
    """
    云台控制节点（最终稳定语义版）：

    PickPlaceEvent.status 语义：
      - status == 1 : INHIBIT（唯一进入条件）
      - status != 1 : 正常 SWEEP / TRACK 循环（包括 status == 2）

    设计原则：
      - INHIBIT 是硬中断态
      - 退出 INHIBIT 后不强制 TRACK
      - TRACK 只由 YOLO has_target 决定
    """

    def __init__(self):
        super().__init__("pan_tilt_sweep_controller")

        # ---------------- 参数 ----------------
        self.declare_parameter("yaw_min_deg", -3.0)
        self.declare_parameter("yaw_max_deg", 3.0)
        self.declare_parameter("pitch_min_deg", 18.0)
        self.declare_parameter("pitch_max_deg", 25.0)

        self.declare_parameter("yaw_speed_deg", 4.0)
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

        self.base_yaw_min = self.yaw_min
        self.base_yaw_max = self.yaw_max
        self.base_pitch_min = self.pitch_min
        self.base_pitch_max = self.pitch_max

        self.yaw_speed = abs(float(self.get_parameter("yaw_speed_deg").value))
        self.pitch_speed = abs(float(self.get_parameter("pitch_speed_deg").value))
        self.soft_zone = float(self.get_parameter("soft_zone_deg").value)

        self.track_yaw_limit = float(self.get_parameter("track_yaw_limit_deg").value)
        self.track_pitch_limit = float(self.get_parameter("track_pitch_limit_deg").value)
        self.track_k_yaw = float(self.get_parameter("track_k_yaw").value)
        self.track_k_pitch = float(self.get_parameter("track_k_pitch").value)
        self.track_deadband = float(self.get_parameter("track_deadband").value)
        self.track_timeout = float(self.get_parameter("track_timeout").value)

        # ---------------- Publisher ----------------
        self.cmd_pub = self.create_publisher(
            PanTiltCmdDeg,
            "/pan_tilt_cmd_deg",
            10,
        )

        # ---------------- 状态 ----------------
        self.mode = "SWEEP"          # SWEEP / TRACK / INHIBIT
        self.inhibit_active = False

        self.yaw_current = 0.0
        self.pitch_current = 0.0
        self.yaw_dir = 1.0
        self.pitch_dir = 1.0

        self.target_u = 0.0
        self.target_v = 0.0
        self.last_bias_time = None

        self.speed_field = min(max(self.yaw_speed, self.pitch_speed, 1.0), 30.0)
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
            "/PickPlaceEvent_N2P",
            self.pick_place_callback,
            10,
        )

        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(
            "PanTiltSweepController started.\n"
            "  status == 1  -> INHIBIT\n"
            "  status != 1  -> SWEEP / TRACK"
        )

    # ------------------------------------------------
    # PickPlaceEvent 回调（核心修改点）
    # ------------------------------------------------
    def pick_place_callback(self, msg: PickPlaceEvent):
        status = int(msg.status)

        # ========= 进入 INHIBIT（唯一条件） =========
        if status == 1:
            if not self.inhibit_active:
                self.inhibit_active = True
                self.mode = "INHIBIT"
                self.last_time = self.get_clock().now()
                self.get_logger().info("[MODE] INHIBIT (status == 1)")
            return

        # ========= 退出 INHIBIT（status != 1） =========
        if self.inhibit_active and status != 1:
            self.inhibit_active = False
            self.mode = "SWEEP"   # 回到确定态，TRACK 由 YOLO 再触发
            self.last_bias_time = None
            self.last_time = self.get_clock().now()
            self.get_logger().info(
                f"[MODE] Exit INHIBIT (status={status}) → resume SWEEP/TRACK"
            )

    # ------------------------------------------------
    # YOLO 回调（唯一进入 TRACK 的入口）
    # ------------------------------------------------
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

    # ------------------------------------------------
    # 主控制循环
    # ------------------------------------------------
    def control_loop(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time.nanoseconds) * 1e-9
        if dt <= 0:
            return
        self.last_time = now

        # INHIBIT：不发布任何控制
        if self.mode == "INHIBIT":
            return

        # TRACK 超时
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

    # ------------------------------------------------
    # SWEEP
    # ------------------------------------------------
    def update_sweep(self, dt):
        self.yaw_current, self.yaw_dir = self.update_axis(
            self.yaw_current, self.yaw_dir,
            self.yaw_min, self.yaw_max,
            self.yaw_speed, dt
        )
        self.pitch_current, self.pitch_dir = self.update_axis(
            self.pitch_current, self.pitch_dir,
            self.pitch_min, self.pitch_max,
            self.pitch_speed, dt
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

    # ------------------------------------------------
    # TRACK
    # ------------------------------------------------
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

        if reason:
            self.get_logger().info(f"[MODE] TRACK → SWEEP ({reason})")


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
