#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node

from pan_tilt_msgs.msg import PanTiltCmdDeg  # 你已经确认过这个类型


class PanTiltCircleController(Node):
    """
    让云台绕 (yaw=0, pitch=0) 做圆周运动：
      yaw(t)   =  R * cos(ω t)
      pitch(t) =  R * sin(ω t)

    R = 45°，即 π/4 rad。
    """

    def __init__(self):
        super().__init__("pan_tilt_circle_controller")

        # 发布云台命令（弧度模式）
        self.cmd_pub = self.create_publisher(
            PanTiltCmdDeg,
            "/pan_tilt_cmd_deg",
            10,
        )

        # 圆的参数
        self.radius = 45  # 圆半径 45°
        self.omega = 1                   # 角速度 rad/s，一圈大约 2π/0.4 ≈ 15.7 s

        # 起始时间，用 ROS 时钟
        self.start_time = self.get_clock().now()

        # 固定一个速度（协议里的 speed 字段）
        self.speed = 31.0

        # 控制循环 timer
        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

        self.get_logger().info(
            "PanTiltCircleController started. "
            "Drawing circle around yaw=0, pitch=0 with radius 45 degrees."
        )

    def control_loop(self):
        # t: 从启动到现在的时间（秒）
        now = self.get_clock().now()
        t = (now.nanoseconds - self.start_time.nanoseconds) * 1e-9

        # 圆周轨迹
        yaw_cmd = self.radius * math.cos(self.omega * t)
        pitch_cmd = self.radius * math.sin(self.omega * t)

        # 构造命令消息
        cmd = PanTiltCmdDeg()
        cmd.speed = int(self.speed)
        cmd.yaw = float(yaw_cmd)
        cmd.pitch = float(pitch_cmd)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltCircleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("PanTiltCircleController interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
