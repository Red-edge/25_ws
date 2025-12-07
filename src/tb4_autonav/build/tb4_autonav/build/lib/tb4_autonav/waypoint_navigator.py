#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit


class WaypointNavigator(Node):
    """
    逻辑：
      - 在代码中写死一组导航目标点（map 坐标系）
      - 顺序发送 navigate_to_pose 目标
      - 只有当前目标成功到达后才发送下一个
      - 同时订阅 /amcl_pose，监控 AMCL 置信度（用协方差计算 std）
      - 如果 std 连续 4 次 > 1.3 -> 进入“慢速档”（速度降低 50%）
      - 如果 std 连续 4 次 < 0.8 -> 退出慢速档（恢复正常速度）
      - 通过发布 /speed_limit（nav2_msgs/SpeedLimit）实现限速
    """

    def __init__(self):
        super().__init__("waypoint_navigator")

        # =============== 1) 参数 / 状态 ===============
        # 这里写死一组 waypoints: (x, y, yaw) in map frame
        # 你可以根据自己的地图修改这些值
        self.waypoints: List[Tuple[float, float, float]] = [
            (4.7, 4.1, 0.0),
            (7.2, 6.2, math.pi / 2),
            (10.0, 4.5, math.pi),
            (9.4, 6.6, -math.pi),
        ]
        self.current_index: int = 0

        # AMCL 置信度相关
        self.std_high_threshold = 1.3
        self.std_low_threshold = 0.8
        self.required_consecutive = 4  # 连续几次满足条件才触发模式切换

        self.high_count = 0
        self.low_count = 0
        self.slow_mode = False  # False=正常速度, True=慢速档

        # 导航目标状态
        self.navigation_active = False

        # =============== 2) ActionClient: NavigateToPose ===============
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
        )

        # =============== 3) SpeedLimit publisher ===============
        # 注：要求 Nav2 的 controller_server 中配置了 speed_limit_topic=/speed_limit
        self.speed_limit_pub = self.create_publisher(
            SpeedLimit,
            "speed_limit",
            10,
        )

        # 上电时先发一次“正常速度”的 speed_limit
        self.publish_speed_limit(percentage=1.0)

        # =============== 4) 订阅 AMCL pose ===============
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            qos,
        )

        # =============== 5) 用定时器稍后启动整个任务 ===============
        # 等 3 秒，让 nav2 / localization 完全起来
        self.create_timer(3.0, self.start_if_ready)

        self.get_logger().info("WaypointNavigator node started. Waiting to start mission...")

    # ------------------------------------------------------------------
    # 导航任务启动
    # ------------------------------------------------------------------
    def start_if_ready(self):
        """
        定时器回调：检查 navigate_to_pose action server 是否已经 ready，
        如果 ready 且还没开始任务，就启动第一个目标。
        """
        if self.navigation_active:
            # 已经在执行任务了，不再重复
            return

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("NavigateToPose action server not available yet...")
            return

        self.get_logger().info("NavigateToPose action server available. Starting mission.")
        self.navigation_active = True
        self.send_next_goal()

    # ------------------------------------------------------------------
    # 顺序发送导航目标
    # ------------------------------------------------------------------
    def send_next_goal(self):
        """发送下一个导航目标点（如果还有的话）"""
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("All waypoints completed. Mission finished.")
            # 任务结束时恢复正常速度
            self.set_slow_mode(False, reason="mission_finished")
            self.navigation_active = False
            return

        x, y, yaw = self.waypoints[self.current_index]
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # yaw -> 四元数
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Sending goal #{self.current_index + 1}/{len(self.waypoints)}: "
            f"x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server.")
            # 可以根据需要重试，这里简单跳过
            self.current_index += 1
            self.send_next_goal()
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        status = result.status
        if status == 4:  # STATUS_SUCCEEDED (参见 action_msgs/msg/GoalStatus)
            self.get_logger().info(f"Goal #{self.current_index + 1} reached successfully.")
        else:
            self.get_logger().warn(
                f"Goal #{self.current_index + 1} finished with status: {status}"
            )
        self.current_index += 1
        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        # 如果需要，可以在这里打印距离终点的距离等
        feedback = feedback_msg.feedback
        # self.get_logger().debug(f"Feedback: {feedback.distance_remaining:.2f} m")
        pass

    # ------------------------------------------------------------------
    # AMCL 置信度监控 + 自适应限速
    # ------------------------------------------------------------------
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance  # 长度 36 的数组

        # 提取 x、y 的方差（cov[0], cov[7]）
        var_x = cov[0]
        var_y = cov[7]

        # 做个简单合成：取 max(std_x, std_y)
        std_x = math.sqrt(var_x) if var_x > 0.0 else 0.0
        std_y = math.sqrt(var_y) if var_y > 0.0 else 0.0
        std_xy = max(std_x, std_y)

        # self.get_logger().debug(f"AMCL std_x={std_x:.3f}, std_y={std_y:.3f}, std={std_xy:.3f}")

        # 根据阈值更新计数器和模式
        if std_xy > self.std_high_threshold:
            self.high_count += 1
            self.low_count = 0
        elif std_xy < self.std_low_threshold:
            self.low_count += 1
            self.high_count = 0
        else:
            # 中间区域，计数清零，防止频繁抖动
            self.high_count = 0
            self.low_count = 0

        # 检查是否需要切换到慢速档
        if (not self.slow_mode) and (self.high_count >= self.required_consecutive):
            self.set_slow_mode(True, reason=f"std_xy={std_xy:.3f} high for {self.high_count} samples")
            self.high_count = 0

        # 检查是否可以退出慢速档
        if self.slow_mode and (self.low_count >= self.required_consecutive):
            self.set_slow_mode(False, reason=f"std_xy={std_xy:.3f} low for {self.low_count} samples")
            self.low_count = 0

    def set_slow_mode(self, enable: bool, reason: str = ""):
        """切换慢速 / 正常模式，并发布 speed_limit"""
        if self.slow_mode == enable:
            return  # 没变化

        self.slow_mode = enable
        if enable:
            self.get_logger().warn(f"Enter SLOW mode (50% speed). Reason: {reason}")
            self.publish_speed_limit(percentage=0.5)
        else:
            self.get_logger().info(f"Back to NORMAL mode (100% speed). Reason: {reason}")
            self.publish_speed_limit(percentage=1.0)

    def publish_speed_limit(self, percentage: float):
        """
        发布速度限制：
          - percentage=True 表示 speed_limit 是原速度的比例（0.5 即 50%）
          - 要求 Nav2 的 speed_filter / controller_server 正确配置了这个话题
        """
        msg = SpeedLimit()
        msg.speed_limit = float(percentage * 100.0)  # nav2 的实现里通常用百分比
        msg.percentage = True
        # 可以设置一个持续时间，也可以留空（由 Nav2 解释），这里简单不设
        self.speed_limit_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("WaypointNavigator interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
