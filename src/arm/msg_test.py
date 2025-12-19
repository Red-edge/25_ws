#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node

from tb4_autonav_interfaces.msg import PickPlaceEvent


class PickPlaceEventTester(Node):
    """
    用来测试 PickPlace 节点的事件机制：
      - 周期性发布 status（默认 1）
      - 同时订阅 /PickPlaceEvent，把收到的 status 打印出来
    """

    def __init__(self):
        super().__init__("pickplace_event_tester")

        self.topic = "/PickPlaceEvent"
        self.pub = self.create_publisher(PickPlaceEvent, self.topic, 10)
        self.sub = self.create_subscription(PickPlaceEvent, self.topic, self.cb, 10)

        # 参数：发布频率 & 初始状态
        self.declare_parameter("rate_hz", 0.1)
        self.declare_parameter("status", 3)

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.status = int(self.get_parameter("status").value)

        period = 1.0 / max(1e-6, self.rate_hz)
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info(
            f"PickPlaceEventTester started. Publishing to {self.topic} at {self.rate_hz} Hz, status={self.status}.\n"
            f"Tips:\n"
            f"  ros2 param set /pickplace_event_tester status 1|3|0\n"
            f"  ros2 param set /pickplace_event_tester rate_hz 5.0\n"
        )

        self.last_rx = None

    def tick(self):
        msg = PickPlaceEvent()
        msg.status = self.status
        self.pub.publish(msg)
        self.get_logger().info(f"[TX] status={msg.status}")

    def cb(self, msg: PickPlaceEvent):
        # 过滤掉重复刷屏：只有变化时打印（你想每条都打印就删掉 if）
        if self.last_rx != int(msg.status):
            self.last_rx = int(msg.status)
            self.get_logger().info(f"[RX] status={int(msg.status)}")


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceEventTester()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
