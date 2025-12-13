#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from pan_tilt_msgs.msg import PanTiltCmdDeg


class TiltTest(Node):
    """
    纯 Tilt（pitch）测试节点：
      - yaw 固定为 0
      - pitch 在 [0, +15, -15, 0] 之间循环
      - 每 2 秒切换一次
    """

    def __init__(self):
        super().__init__("tilt_test")

        # 发布到真实存在的话题 & 消息类型
        self.pub = self.create_publisher(
            PanTiltCmdDeg,
            "/pan_tilt_cmd_deg",
            10,
        )

        self.cmd = PanTiltCmdDeg()
        self.cmd.yaw = 0.0
        self.cmd.speed = 10  # deg/s

        self.state = 0

        self.timer = self.create_timer(2.0, self.timer_cb)

        self.get_logger().info("Tilt test node started.")

    def timer_cb(self):
        if self.state == 0:
            self.cmd.pitch = 0.0
            self.get_logger().info("Pitch = 0 deg (center)")

        elif self.state == 1:
            self.cmd.pitch = 15.0
            self.get_logger().info("Pitch = +15 deg")

        elif self.state == 2:
            self.cmd.pitch = -15.0
            self.get_logger().info("Pitch = -15 deg")

        elif self.state == 3:
            self.cmd.pitch = 0.0
            self.get_logger().info("Pitch = 0 deg (back to center)")
            self.state = -1  # reset

        self.pub.publish(self.cmd)
        self.state += 1


def main(args=None):
    rclpy.init(args=args)
    node = TiltTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
