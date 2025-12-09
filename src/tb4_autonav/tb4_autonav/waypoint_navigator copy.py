#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import subprocess
import signal
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from std_msgs.msg import String   # === NEW ===


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__("waypoint_navigator")

        # =============== Waypoints ===============
        self.waypoints: List[Tuple[float, float, float]] = [
            (2.250,-19.900,-0.002),
            (4.534, -22.262,0.001),
            (6.382,-19.826,-0.002),
            (4.599,- 18.400,-0.001),
            (2.250,-19.900,-0.002),
        ]
        self.current_index: int = 0
        self.paused = False              # === NEW === 红灯暂停导航
        
        # ======== AMCL 置信度 ========
        self.std_high_threshold = 1.3
        self.std_low_threshold = 0.8
        self.required_consecutive = 4
        self.high_count = 0
        self.low_count = 0
        self.slow_mode = False

        self.navigation_active = False

        # =============== NavigateToPose Action Client ===============
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._latest_goal_handle = None     # === NEW === 用于 cancel

        # =============== SpeedLimit publisher ===============
        self.speed_limit_pub = self.create_publisher(SpeedLimit, "speed_limit", 10)
        self.publish_speed_limit(percentage=1.0)

        # =============== Traffic signal subscriber ===============
        self.create_subscription(String, '/traffic_signal', self.signal_callback, 10)  # === NEW ===

        # =============== AMCL subscriber ===============
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose", self.amcl_callback, qos,
        )

        # =============== Detector 控制 ===============
        self.detector_process = None

        # 延迟启动 mission
        self.create_timer(3.0, self.start_if_ready)

        self.get_logger().info("WaypointNavigator node started. Waiting to start mission...")

    # ==================================================================
    # 交通灯信号处理（核心修改）
    # ==================================================================
    def signal_callback(self, msg: String):
        """收到交通信号信息：red / green / stop"""
        signal = msg.data

        # -------------------------
        # 处理 STOP SIGN
        # -------------------------
        if signal == "stop":
            self.get_logger().warn("🛑 STOP SIGN detected → stopping for 3 seconds")

            # 暂停导航
            self.paused = True

            # 取消当前导航
            if self._latest_goal_handle is not None:
                try:
                    self._latest_goal_handle.cancel_goal_async()
                except Exception:
                    pass

            # 定时器：3秒后恢复
            self.create_timer(5.0, self._resume_after_stop)
            self.stop_detector()
            return

        # -------------------------
        # 处理红灯
        # -------------------------
        if signal == "red":
            self.get_logger().warn("🚦 RED detected → stopping navigation")
            self.paused = True

            if self._latest_goal_handle is not None:
                try:
                    self._latest_goal_handle.cancel_goal_async()
                except Exception:
                    pass
            return

        # -------------------------
        # 处理绿灯
        # -------------------------
        if signal == "green":
            self.get_logger().info("🚦 GREEN detected → resuming navigation")
            if self.paused:
                self.paused = False
                self.resend_current_goal()
            return
    # === NEW === 重新发送当前目标（绿灯恢复时调用）
    def resend_current_goal(self):
        if self.current_index < len(self.waypoints):
            self.get_logger().info(f"Resend goal #{self.current_index}")
            self.send_next_goal()
    
    def _resume_after_stop(self):
        """STOP SIGN 停车后恢复导航"""
        if self.paused:
            self.get_logger().info("🛑 Stop sign pause finished → resuming navigation")
            self.paused = False
            self.resend_current_goal()

    # ==================================================================
    # 导航任务启动
    # ==================================================================
    def start_if_ready(self):
        if self.navigation_active:
            return

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("NavigateToPose action server not available yet...")
            return

        self.navigation_active = True
        self.send_next_goal()

    # ==================================================================
    # 发送导航目标（同时控制 detector）
    # ==================================================================
    def send_next_goal(self):

        if self.paused:
            self.get_logger().info("Paused by traffic signal. Not sending new goal.")
            return

        # 开关 Detector（你的原逻辑保留）
        if self.current_index == 0:
            self.start_detector()
        elif self.current_index == 2:
            self.start_detector()
        elif self.current_index == 3:
            self.stop_detector()

        if self.current_index >= len(self.waypoints):
            self.get_logger().info("All waypoints completed.")
            self.stop_detector()
            self.set_slow_mode(False)
            self.navigation_active = False
            return

        x, y, yaw = self.waypoints[self.current_index]
        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw/2.0)
        goal.pose.pose.orientation.w = math.cos(yaw/2.0)

        future = self.nav_to_pose_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

        self.get_logger().info(f"Sending goal #{self.current_index}: x={x}, y={y}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self._latest_goal_handle = goal_handle  # === NEW === 存下句柄，以便 cancel

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self.current_index += 1
            self.send_next_goal()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        if self.paused:
            return
        self.current_index += 1
        self.send_next_goal()

    # ==================================================================
    # AMCL & speed limit（不改动）
    # ==================================================================
    def amcl_callback(self, msg):
        cov = msg.pose.covariance
        var_x = cov[0]
        var_y = cov[7]
        std_x = math.sqrt(var_x) if var_x > 0 else 0
        std_y = math.sqrt(var_y) if var_y > 0 else 0
        std_xy = max(std_x, std_y)

        if std_xy > self.std_high_threshold:
            self.high_count += 1
            self.low_count = 0
        elif std_xy < self.std_low_threshold:
            self.low_count += 1
            self.high_count = 0
        else:
            self.high_count = 0
            self.low_count = 0

        if not self.slow_mode and self.high_count >= self.required_consecutive:
            self.set_slow_mode(True)
            self.high_count = 0

        if self.slow_mode and self.low_count >= self.required_consecutive:
            self.set_slow_mode(False)
            self.low_count = 0

    def set_slow_mode(self, enable):
        if self.slow_mode == enable:
            return
        self.slow_mode = enable
        if enable:
            self.get_logger().warn("Enter SLOW mode")
            self.publish_speed_limit(0.5)
        else:
            self.get_logger().info("Back to NORMAL")
            self.publish_speed_limit(1.0)

    def publish_speed_limit(self, percentage):
        msg = SpeedLimit()
        msg.speed_limit = percentage * 100.0
        msg.percentage = True
        self.speed_limit_pub.publish(msg)

    # ==================================================================
    # Detector 管理（保留你的原逻辑）
    # ==================================================================
    def start_detector(self):
        if self.detector_process:
            return
        self.detector_process = subprocess.Popen(
            ["ros2", "run", "tb4_autonav", "traffic_detector"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def stop_detector(self):
        if self.detector_process:
            self.detector_process.send_signal(signal.SIGINT)
            self.detector_process = None


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()