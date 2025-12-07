#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit

from tb4_autonav_interfaces.msg import TrafficEvent


class WaypointNavigator(Node):
    """
    逻辑概览：

      【启动阶段】
        1. 等待系统自检通过：
           - nav2（navigate_to_pose action server 就绪）
           - locomotion（收到 /odom）
           - vision（收到 TrafficEvent 且 is_ready==True）
        2. 自检全部 OK 后，开始任务执行（此处可以插入重定位预热）

      【任务执行阶段】
        - 按顺序发送预设导航目标点
        - 前一目标成功到达后再发下一个
        - 订阅 /amcl_pose，基于 std 动态调节速度（慢速档/正常档）
        - 订阅 /traffic_event，在每个 goal 内：
            * 若首次检测到 RED/STOP_SIGN 且 distance<=1.0m，则触发一次中断：
              - STOP_SIGN：停 3 秒后恢复（若这时无 RED）
              - RED      ：停住，直到检测到 GREEN 才恢复
    """

    def __init__(self):
        super().__init__("waypoint_navigator")

        # ========== 1) 导航目标点 ==========
        self.waypoints: List[Tuple[float, float, float]] = [
            (1.28, -14.38, -0.67),
            (0.83, -18.44, -1.0),
            (-2.7, -17.8, 0.75),
            (-1.97, -15.53, 0.06),
            (1.28, -14.38, -0.67),
        ]
        self.current_index: int = 0

        self.navigation_active = False   # 是否已经进入“任务执行阶段”

        # ========== 2) 系统自检标志 ==========
        self.nav2_ready = False
        self.loco_ready = False
        self.vision_ready = False
        self.system_ready = False

        self.last_odom_time = 0.0

        # ========== 3) AMCL 置信度相关 ==========
        self.std_high_threshold = 1.3
        self.std_low_threshold = 0.8
        self.required_consecutive = 4

        self.high_count = 0
        self.low_count = 0
        self.slow_mode = False  # False=正常速度，True=慢速档（50%）

        # ========== 4) 交通事件相关 ==========
        self.current_event_type = "NONE"
        self.current_event_distance = float("inf")

        self.event_already_handled = False  # 每个 goal 只处理一次交通事件
        self.stop_sign_active = False
        self.stop_sign_end_time = 0.0
        self.red_waiting = False

        # ========== 5) ActionClient: NavigateToPose ==========
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
        )

        # ========== 6) SpeedLimit publisher ==========
        self.speed_limit_pub = self.create_publisher(
            SpeedLimit,
            "speed_limit",
            10,
        )
        # 默认设为正常速度
        self.publish_speed_limit(percentage=1.0)

        # ========== 7) 订阅 AMCL pose ==========
        qos_amcl = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            qos_amcl,
        )

        # ========== 8) 订阅 Odom（locomotion 自检用） ==========
        qos_odom = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos_odom,
        )

        # ========== 9) 订阅 TrafficEvent（vision 状态 + 交通事件） ==========
        self.traffic_sub = self.create_subscription(
            TrafficEvent,
            "/traffic_event",
            self.traffic_event_callback,
            10,
        )

        # ========== 10) 定时器 ==========
        # 系统自检定时器
        self.system_check_timer = self.create_timer(1.0, self.system_check_callback)
        # 交通事件 & 中断逻辑守护
        self.guard_timer = self.create_timer(0.1, self.navigation_guard_timer)

        self.get_logger().info("WaypointNavigator started. Waiting for system check to pass...")

    # ------------------------------------------------------------------
    # 系统自检：nav2 / locomotion / vision
    # ------------------------------------------------------------------
    def system_check_callback(self):
        if self.system_ready:
            return

        # 1) nav2：检查 navigate_to_pose action server 是否可用
        if not self.nav2_ready:
            if self.nav_to_pose_client.wait_for_server(timeout_sec=0.1):
                self.nav2_ready = True
                self.get_logger().info("[SYSTEM CHECK] nav2 OK (navigate_to_pose available).")
            else:
                self.get_logger().warn("[SYSTEM CHECK] nav2 NOT ready yet.")

        # 2) locomotion：收到 /odom 即认为底盘链路正常
        if not self.loco_ready:
            if self.last_odom_time > 0.0 and (time.time() - self.last_odom_time) < 2.0:
                self.loco_ready = True
                self.get_logger().info("[SYSTEM CHECK] locomotion OK (/odom received).")
            else:
                self.get_logger().warn("[SYSTEM CHECK] locomotion NOT ready yet (/odom missing).")

        # 3) vision：收到 TrafficEvent 且 is_ready == True
        if not self.vision_ready:
            if self.current_event_type is not None:
                # 在 traffic_event_callback 里会更新 vision_ready
                if self.vision_ready:
                    self.get_logger().info("[SYSTEM CHECK] vision OK (TrafficDetector ready).")
                else:
                    self.get_logger().warn("[SYSTEM CHECK] vision NOT ready yet.")
            else:
                self.get_logger().warn("[SYSTEM CHECK] no TrafficEvent received yet.")

        if self.nav2_ready and self.loco_ready and self.vision_ready:
            self.system_ready = True
            self.system_check_timer.cancel()
            self.get_logger().info("[SYSTEM CHECK] All subsystems READY. Start mission.")
            # 这里可以插入“重定位预热”逻辑；目前直接开始任务
            self.start_mission()

    def start_mission(self):
        if self.navigation_active:
            return
        self.navigation_active = True
        self.send_next_goal()

    # ------------------------------------------------------------------
    # 导航目标发送 & 回调
    # ------------------------------------------------------------------
    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("All waypoints completed. Mission finished.")
            self.set_slow_mode(False, reason="mission_finished")
            self.navigation_active = False
            return

        # 新 goal 开始时，重置交通事件标记
        self.event_already_handled = False
        self.stop_sign_active = False
        self.red_waiting = False

        x, y, yaw = self.waypoints[self.current_index]
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

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
            self.current_index += 1
            self.send_next_goal()
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        status = result.status
        if status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info(f"Goal #{self.current_index + 1} reached successfully.")
        else:
            self.get_logger().warn(
                f"Goal #{self.current_index + 1} finished with status: {status}"
            )
        self.current_index += 1
        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        # 可选：打印导航剩余距离等信息
        pass

    # ------------------------------------------------------------------
    # AMCL / Odom / TrafficEvent 回调
    # ------------------------------------------------------------------
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance
        var_x = cov[0]
        var_y = cov[7]

        std_x = math.sqrt(var_x) if var_x > 0.0 else 0.0
        std_y = math.sqrt(var_y) if var_y > 0.0 else 0.0
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

        if (not self.slow_mode) and (self.high_count >= self.required_consecutive):
            self.set_slow_mode(True, reason=f"std_xy={std_xy:.3f} high")
            self.high_count = 0

        if self.slow_mode and (self.low_count >= self.required_consecutive):
            self.set_slow_mode(False, reason=f"std_xy={std_xy:.3f} low")
            self.low_count = 0

    def odom_callback(self, msg: Odometry):
        self.last_odom_time = time.time()

    def traffic_event_callback(self, msg: TrafficEvent):
        # vision 自检：只要收到 is_ready=True，即可认为 vision_ready
        if msg.is_ready:
            if not self.vision_ready:
                self.get_logger().info("WaypointNavigator: vision reported ready.")
            self.vision_ready = True

        self.current_event_type = msg.type
        self.current_event_distance = msg.distance

    # ------------------------------------------------------------------
    # 自适应限速
    # ------------------------------------------------------------------
    def set_slow_mode(self, enable: bool, reason: str = ""):
        if self.slow_mode == enable:
            return
        self.slow_mode = enable
        if enable:
            self.get_logger().warn(f"Enter SLOW mode (50% speed). Reason: {reason}")
            self.publish_speed_limit(percentage=0.5)
        else:
            self.get_logger().info(f"Back to NORMAL mode (100% speed). Reason: {reason}")
            self.publish_speed_limit(percentage=1.0)

    def publish_speed_limit(self, percentage: float):
        msg = SpeedLimit()
        msg.speed_limit = float(percentage * 100.0)  # 0–100%
        msg.percentage = True
        self.speed_limit_pub.publish(msg)

    # ------------------------------------------------------------------
    # 交通事件守护：处理 RED/STOP_SIGN 中断 + 恢复
    # ------------------------------------------------------------------
    def navigation_guard_timer(self):
        if not self.navigation_active or not self.system_ready:
            return

        now = time.time()

        # ========== 1) 本 goal 第一次触发事件 ==========
        if not self.event_already_handled:
            ev = self.current_event_type
            dist = self.current_event_distance

            if ev in ("RED", "STOP_SIGN") and 0.0 < dist <= 1.0:
                self.event_already_handled = True

                if ev == "STOP_SIGN":
                    self.handle_stop_sign_start(now)
                elif ev == "RED":
                    self.handle_red_light_start()

        # ========== 2) 停止牌超时恢复 ==========
        if self.stop_sign_active:
            if now >= self.stop_sign_end_time:
                if self.current_event_type == "RED":
                    # 转入红灯等待状态
                    if not self.red_waiting:
                        self.handle_red_light_start()
                else:
                    self.get_logger().info("Stop sign hold finished → resume navigation.")
                    self.stop_sign_active = False
                    self.publish_speed_limit(percentage=1.0)

        # ========== 3) 红灯 → 绿灯 恢复 ==========
        if self.red_waiting and self.current_event_type == "GREEN":
            self.get_logger().info("GREEN detected → resume navigation from RED.")
            self.red_waiting = False
            self.publish_speed_limit(percentage=1.0)

    def handle_stop_sign_start(self, now: float):
        self.get_logger().info("STOP SIGN within 1m → full stop for 3 seconds.")
        self.stop_sign_active = True
        self.stop_sign_end_time = now + 3.0
        self.publish_speed_limit(percentage=0.0)

    def handle_red_light_start(self):
        self.get_logger().info("RED light within 1m → stop and wait for GREEN.")
        self.red_waiting = True
        self.publish_speed_limit(percentage=0.0)


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
