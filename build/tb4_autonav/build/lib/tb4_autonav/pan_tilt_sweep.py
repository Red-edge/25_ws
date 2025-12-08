#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node

from pan_tilt_msgs.msg import PanTiltCmdDeg


class PanTiltSweepController(Node):
    """
    让云台在 yaw / pitch 方向上分别做匀速往复运动：

      yaw(t): 在 [yaw_min_deg, yaw_max_deg] 之间来回扫
      pitch(t): 在 [pitch_min_deg, pitch_max_deg] 之间来回扫

    所有角度以正中心为 0 度，0 不变原点。
    扫动速度可分别设置（deg/s）。
    """

    def __init__(self):
        super().__init__("pan_tilt_sweep_controller")

        # ---------------- 参数声明 ----------------
        # 角度区间（度）
        self.declare_parameter("yaw_min_deg", -20.0)
        self.declare_parameter("yaw_max_deg", 20.0)
        self.declare_parameter("pitch_min_deg", 10.0)
        self.declare_parameter("pitch_max_deg", 30.0)

        # 角速度（度/秒）
        self.declare_parameter("yaw_speed_deg", 16.0)    # 最大 30 deg/s
        self.declare_parameter("pitch_speed_deg", 23.0)

        # 读取参数
        self.yaw_min = float(self.get_parameter("yaw_min_deg").value)
        self.yaw_max = float(self.get_parameter("yaw_max_deg").value)
        self.pitch_min = float(self.get_parameter("pitch_min_deg").value)
        self.pitch_max = float(self.get_parameter("pitch_max_deg").value)

        self.yaw_speed = abs(float(self.get_parameter("yaw_speed_deg").value))
        self.pitch_speed = abs(float(self.get_parameter("pitch_speed_deg").value))

        # 做个防呆：保证 min <= max
        if self.yaw_min > self.yaw_max:
            self.yaw_min, self.yaw_max = self.yaw_max, self.yaw_min
        if self.pitch_min > self.pitch_max:
            self.pitch_min, self.pitch_max = self.pitch_max, self.pitch_min

        # 发布命令（角度制）
        self.cmd_pub = self.create_publisher(
            PanTiltCmdDeg,
            "/pan_tilt_cmd_deg",
            10,
        )

        # 状态：当前角度 & 方向（+1 / -1）
        # 起点都从区间中点开始
        self.yaw_current = 0.0
        self.pitch_current = 0.0
        self.yaw_dir = 1.0      # +1 向 yaw_max 增加，-1 向 yaw_min 减小
        self.pitch_dir = 1.0    # 同理

        # 固定速度字段（协议里的 speed，单位 deg/s）
        # 这里简单取 yaw/pitch 中较大的一个，你也可以改成独立逻辑
        self.speed_field = max(self.yaw_speed, self.pitch_speed, 1.0)

        # 时间
        self.last_time = self.get_clock().now()

        # 控制循环
        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

        self.get_logger().info(
            "PanTiltSweepController started.\n"
            f"  yaw   in [{self.yaw_min:.1f}, {self.yaw_max:.1f}] deg @ {self.yaw_speed:.1f} deg/s\n"
            f"  pitch in [{self.pitch_min:.1f}, {self.pitch_max:.1f}] deg @ {self.pitch_speed:.1f} deg/s"
        )

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time.nanoseconds) * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now

        # ---------------- yaw 匀速往复 ----------------
        if self.yaw_speed > 0.0 and self.yaw_min != self.yaw_max:
            self.yaw_current += self.yaw_dir * self.yaw_speed * dt

            # 到达上界：钳住并反向
            if self.yaw_current > self.yaw_max:
                self.yaw_current = self.yaw_max
                self.yaw_dir = -1.0

            # 到达下界：钳住并反向
            if self.yaw_current < self.yaw_min:
                self.yaw_current = self.yaw_min
                self.yaw_dir = 1.0

        # ---------------- pitch 匀速往复 ----------------
        if self.pitch_speed > 0.0 and self.pitch_min != self.pitch_max:
            self.pitch_current += self.pitch_dir * self.pitch_speed * dt

            if self.pitch_current > self.pitch_max:
                self.pitch_current = self.pitch_max
                self.pitch_dir = -1.0

            if self.pitch_current < self.pitch_min:
                self.pitch_current = self.pitch_min
                self.pitch_dir = 1.0

        # ---------------- 发送命令 ----------------
        cmd = PanTiltCmdDeg()
        cmd.speed = int(self.speed_field)
        cmd.yaw = float(self.yaw_current)
        cmd.pitch = float(self.pitch_current)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltSweepController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("PanTiltSweepController interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
