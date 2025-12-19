#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from sensor_msgs.msg import LaserScan

from tb4_autonav_interfaces.msg import TrafficEvent
from tb4_autonav_interfaces.msg import PickPlaceEvent
import numpy as np


class WaypointNavigator(Node):
    """
    逻辑概览：

      【启动阶段】
        1. 等待系统自检通过：
           - nav2（navigate_to_pose action server 就绪）
           - locomotion（收到 /odom）
           - vision（收到 TrafficEvent 且 is_ready==True）
        2. 自检全部 OK 后，进入“重定位预热阶段”：
           - 利用 /scan + /odom 在前方安全直线上仅“前进 + 原地 180° 转向 + 前进回原点”往复运动
           - 启动时限速为 30%（nav_speed_factor = 0.3）
           - 监控 /amcl_pose 的 std_xy，连续 N 次 < 阈值视为重定位完成
        3. 预热结束后：
           - 停车
           - 恢复 100% nav_speed_factor
           - 正式开始导航任务

      【任务执行阶段】
        - 按顺序发送预设导航目标点
        - 前一目标成功到达后再发下一个
        - 订阅 /amcl_pose，基于 std 动态调节速度（慢速档/正常档）
        - 订阅 /traffic_event（来自 YOLO 检测）处理 STOP_SIGN / RED
        - PickPlace 双向通信：
            到达 PICK 点 -> 发布 status=1，暂停发送下一导航点
            收到 status=2（5Hz重复）-> 去重，仅第一次处理：index+1，继续导航
            到达 PLACE 点 -> 发布 status=3，暂停发送下一导航点
            收到 status=4（5Hz重复）-> 去重，仅第一次处理：index+1，继续导航
    """

    def __init__(self):
        super().__init__("waypoint_navigator")

        # ========== 1) 导航目标点 ==========
        self.waypoints: List[Tuple[float, float, float]] = [
            (2.040, -19.800, -0.7),     # starting point
            (4.534, -22.262,  0.00),     # first
            (6.382, -19.826, np.pi/4),     # second (pick)
            (4.599, -18.400, 2.33),     # third (place)
            (2.040, -19.800, -0.7),     # back to start
        ]
        self.current_index: int = 0
        self.navigation_active = False

        # 哪些 waypoint 是操作点
        self.PICK_INDEX = 2
        self.PLACE_INDEX = 3

        # ========== 1.2) PickPlace 双向通信设置 ==========
        self.waiting_for_pick_place = False  # 正在等待 Pick/Place 完成时，禁止 send_next_goal
        self._last_pickplace_status = None   # 用于去重（PickPlace 5Hz 重复发）

        self.operation_pub = self.create_publisher(
            PickPlaceEvent,
            "/PickPlaceEvent_N2P",
            10
        )
        self.pick_status_sub = self.create_subscription(
            PickPlaceEvent,
            "/PickPlaceEvent_P2N",
            self.pick_status_callback,
            10
        )

        # ========== 2) 系统自检标志 ==========
        self.nav2_ready = False
        self.loco_ready = False
        self.vision_ready = False
        self.system_ready = False
        self.last_odom_time = 0.0

        # ========== 3) AMCL 置信度相关（导航阶段用） ==========
        self.std_high_threshold = 1.3
        self.std_low_threshold = 0.8
        self.required_consecutive = 4
        self.high_count = 0
        self.low_count = 0
        self.slow_mode = False
        self.nav_speed_factor = 1.0
        self.event_speed_factor = 0.8

        # ========== 4) 重定位预热相关 ==========
        self.prelocalization_active = False
        self.ready_to_navigate = False
        self.init_good_count = 0
        self.init_std_threshold = 1.0
        self.init_required_consecutive = 5

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.preloc_linear_speed = 0.05
        self.preloc_angular_speed = 0.4
        self.preloc_distance_limit = None

        self.preloc_safety_margin = 0.2
        self.preloc_min_distance = 0.1
        self.preloc_max_distance = 1.0

        self.preloc_state = "FORWARD"
        self.turn_start_yaw = None
        self.turn_tolerance = math.radians(10.0)

        self.last_scan: LaserScan | None = None
        self.last_odom_x: float | None = None
        self.last_odom_y: float | None = None
        self.last_yaw: float | None = None
        self.preloc_leg_start_x: float | None = None
        self.preloc_leg_start_y: float | None = None

        # ========== 5) 交通事件相关 ==========
        self.current_event_type = "NONE"
        self.current_event_distance = float("inf")
        
        self.stop_sign_state = "IDLE"
        self.stop_sign_hold_end_time = 0.0
        self.red_light_hold_end_time = 0.0
        self.red_light_active = False

        self.stop_event_active = False
        self.stop_event_hold_end_time = 0.0

        # ========== 6) ActionClient: NavigateToPose ==========
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ========== 7) SpeedLimit publisher ==========
        self.speed_limit_pub = self.create_publisher(SpeedLimit, "speed_limit", 10)
        self.update_speed_limit()

        # ========== 8) 订阅 AMCL pose ==========
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            qos_best_effort,
        )

        # ========== 9) 订阅 Odom ==========
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos_best_effort,
        )

        # ========== 10) 订阅 LiDAR 扫描 ==========
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_best_effort,
        )

        # ========== 11) 订阅 TrafficEvent ==========
        self.traffic_sub = self.create_subscription(
            TrafficEvent,
            "/traffic_event",
            self.traffic_event_callback,
            10,
        )

        # ========== 12) 定时器 ==========
        self.system_check_timer = self.create_timer(1.0, self.system_check_callback)
        self.guard_timer = self.create_timer(0.1, self.navigation_guard_timer)
        self.preloc_timer = self.create_timer(0.1, self.prelocalization_motion)

        self.get_logger().info("WaypointNavigator started. Waiting for system check to pass...")

    # ------------------------------------------------------------------
    # 工具函数：从四元数取 yaw / 角度归一化
    # ------------------------------------------------------------------
    @staticmethod
    def _quat_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    # ------------------------------------------------------------------
    # 系统自检：nav2 / locomotion / vision
    # ------------------------------------------------------------------
    def system_check_callback(self):
        if self.system_ready:
            return

        # 1) nav2
        if not self.nav2_ready:
            if self.nav_to_pose_client.wait_for_server(timeout_sec=0.1):
                self.nav2_ready = True
                self.get_logger().info("[SYSTEM CHECK] nav2 OK (navigate_to_pose available).")
            else:
                self.get_logger().warn("[SYSTEM CHECK] nav2 NOT ready yet.")

        # 2) locomotion
        if not self.loco_ready:
            if self.last_odom_time > 0.0 and (time.time() - self.last_odom_time) < 2.0:
                self.loco_ready = True
                self.get_logger().info("[SYSTEM CHECK] locomotion OK (/odom received).")
            else:
                self.get_logger().warn("[SYSTEM CHECK] locomotion NOT ready yet (/odom missing).")

        # 3) vision：由 traffic_event_callback 设置 self.vision_ready
        if not self.vision_ready:
            self.get_logger().warn("[SYSTEM CHECK] vision NOT ready yet (TrafficEvent is_ready not True).")
        else:
            self.get_logger().info("[SYSTEM CHECK] vision OK (TrafficDetector ready).")

        if self.nav2_ready and self.loco_ready and self.vision_ready:
            self.system_ready = True
            self.system_check_timer.cancel()
            self.get_logger().info("[SYSTEM CHECK] All subsystems READY. Start prelocalization.")
            self.start_prelocalization()

    def start_prelocalization(self):
        if self.prelocalization_active or self.navigation_active:
            return

        self.prelocalization_active = True
        self.ready_to_navigate = False
        self.init_good_count = 0
        self.preloc_distance_limit = None
        self.preloc_state = "FORWARD"
        self.turn_start_yaw = None
        self.preloc_leg_start_x = None
        self.preloc_leg_start_y = None

        self.nav_speed_factor = 0.3
        self.event_speed_factor = 0.8
        self.update_speed_limit()

        self.get_logger().info(
            "Prelocalization phase started: forward + 180° turns, waiting for AMCL std to stabilize..."
        )

    def start_mission(self):
        if self.navigation_active:
            return
        self.navigation_active = True
        self.send_next_goal()

    # ------------------------------------------------------------------
    # LiDAR & Odom 回调
    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg
        if self.prelocalization_active and self.preloc_distance_limit is None:
            self.update_preloc_distance_from_scan()

    def odom_callback(self, msg: Odometry):
        self.last_odom_time = time.time()

        self.last_odom_x = msg.pose.pose.position.x
        self.last_odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.last_yaw = self._quat_to_yaw(q)

        if self.preloc_leg_start_x is None or self.preloc_leg_start_y is None:
            self.preloc_leg_start_x = self.last_odom_x
            self.preloc_leg_start_y = self.last_odom_y

    # ------------------------------------------------------------------
    # 预热阶段：用雷达计算安全直线距离
    # ------------------------------------------------------------------
    def _compute_min_forward_range(self) -> float | None:
        if self.last_scan is None:
            return None

        scan = self.last_scan
        if scan.angle_increment == 0.0:
            return None

        half_angle = math.radians(45.0)
        angle_min = scan.angle_min
        inc = scan.angle_increment

        start_angle = -half_angle
        end_angle = half_angle

        idx_min = int(max(0, math.floor((start_angle - angle_min) / inc)))
        idx_max = int(min(len(scan.ranges) - 1, math.ceil((end_angle - angle_min) / inc)))

        if idx_min > idx_max:
            return None

        valid_ranges = []
        for r in scan.ranges[idx_min:idx_max + 1]:
            if scan.range_min < r < scan.range_max:
                valid_ranges.append(r)

        if not valid_ranges:
            return None

        return min(valid_ranges)

    def update_preloc_distance_from_scan(self):
        min_forward = self._compute_min_forward_range()
        if min_forward is None:
            return

        if min_forward <= self.preloc_safety_margin + 0.05:
            d = self.preloc_min_distance
        else:
            d = min_forward - self.preloc_safety_margin
            d = max(d, self.preloc_min_distance)
            d = min(d, self.preloc_max_distance)

        self.preloc_distance_limit = d
        self.get_logger().info(
            f"[Preloc] Straight-line distance set to {d:.2f} m (min forward LiDAR={min_forward:.2f} m)"
        )

    # ------------------------------------------------------------------
    # 预热阶段运动：FORWARD / TURN
    # ------------------------------------------------------------------
    def prelocalization_motion(self):
        if not self.prelocalization_active or not self.system_ready:
            return

        if self.preloc_distance_limit is None:
            if self.last_scan is not None:
                self.update_preloc_distance_from_scan()
            if self.preloc_distance_limit is None:
                return

        if self.last_odom_x is None or self.last_odom_y is None or self.last_yaw is None:
            return

        if self.preloc_leg_start_x is None or self.preloc_leg_start_y is None:
            self.preloc_leg_start_x = self.last_odom_x
            self.preloc_leg_start_y = self.last_odom_y

        if self.preloc_state == "FORWARD":
            dx = self.last_odom_x - self.preloc_leg_start_x
            dy = self.last_odom_y - self.preloc_leg_start_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist >= self.preloc_distance_limit:
                self.get_logger().info(f"[Preloc] Forward leg reached {dist:.2f} m, start turning 180°.")
                self.stop_robot_motion()
                self.preloc_state = "TURN"
                self.turn_start_yaw = self.last_yaw
                return

            if self.last_scan is not None:
                min_forward = self._compute_min_forward_range()
                if min_forward is not None and min_forward <= self.preloc_safety_margin:
                    self.get_logger().warn(
                        f"[Preloc] Obstacle at {min_forward:.2f} m ahead, early start turning 180°."
                    )
                    self.stop_robot_motion()
                    self.preloc_state = "TURN"
                    self.turn_start_yaw = self.last_yaw
                    return

            twist = Twist()
            twist.linear.x = self.preloc_linear_speed
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

        elif self.preloc_state == "TURN":
            if self.turn_start_yaw is None:
                self.turn_start_yaw = self.last_yaw

            delta = self._normalize_angle(self.last_yaw - self.turn_start_yaw)

            if abs(delta) >= math.pi - self.turn_tolerance:
                self.get_logger().info(f"[Preloc] Turn 180° done (delta={delta:.2f} rad). Start next leg.")
                self.stop_robot_motion()
                self.preloc_state = "FORWARD"
                self.turn_start_yaw = None
                self.preloc_leg_start_x = self.last_odom_x
                self.preloc_leg_start_y = self.last_odom_y
                return

            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = self.preloc_angular_speed
            self.cmd_vel_pub.publish(twist)

    def stop_robot_motion(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    # ------------------------------------------------------------------
    # 导航目标发送 & 回调
    # ------------------------------------------------------------------
    def send_next_goal(self):
        # ✅ 正在等 pick/place 完成时，不允许发下一个导航点
        if self.waiting_for_pick_place:
            self.get_logger().info("Waiting for pick/place to finish. Not sending next goal yet.")
            return

        if self.current_index >= len(self.waypoints):
            self.get_logger().info("All waypoints completed. Mission finished.")
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

        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

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
            self.get_logger().error("Goal rejected by server. Skip to next.")
            self.current_index += 1
            self.send_next_goal()
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        status = result.status
        completed_index = self.current_index

        if status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info(f"Goal #{completed_index + 1} reached successfully.")

            # ✅ 到达操作点就暂停，并等待 PickPlace 回传完成
            if completed_index == self.PICK_INDEX and (not self.waiting_for_pick_place):
                self.get_logger().info("Arrived at PICK location. Sending 'start pick' command.")
                self.send_pick_command()
                self.waiting_for_pick_place = True
                return

            if completed_index == self.PLACE_INDEX and (not self.waiting_for_pick_place):
                self.get_logger().info("Arrived at PLACE location. Sending 'start place' command.")
                self.send_place_command()
                self.waiting_for_pick_place = True
                return
        else:
            self.get_logger().warn(
                f"Goal #{completed_index + 1} failed or canceled (status={status}). Proceeding to next..."
            )

        # ✅ 非操作点（或无需暂停）才推进
        self.current_index += 1
        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        pass

    # ------------------------------------------------------------------
    # AMCL / TrafficEvent 回调
    # ------------------------------------------------------------------
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance
        var_x = cov[0]
        var_y = cov[7]

        std_x = math.sqrt(var_x) if var_x > 0.0 else 0.0
        std_y = math.sqrt(var_y) if var_y > 0.0 else 0.0
        std_xy = max(std_x, std_y)

        # ======== 预热重定位 ========
        if self.prelocalization_active:
            if std_xy < self.init_std_threshold:
                self.init_good_count += 1
            else:
                self.init_good_count = 0

            if self.init_good_count >= self.init_required_consecutive:
                self.prelocalization_active = False
                self.ready_to_navigate = True
                self.init_good_count = 0

                self.stop_robot_motion()
                self.nav_speed_factor = 1.0
                self.slow_mode = False
                self.update_speed_limit()

                self.get_logger().info(
                    f"Prelocalization finished (std_xy < {self.init_std_threshold} "
                    f"for {self.init_required_consecutive} samples)."
                )

                if self.system_ready and not self.navigation_active:
                    self.get_logger().info("Starting mission after prelocalization.")
                    self.start_mission()
            return

        # ======== 正常导航时的慢速档逻辑 ========
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

    def traffic_event_callback(self, msg: TrafficEvent):
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
            self.nav_speed_factor = 0.5
            self.get_logger().warn(f"Enter SLOW mode (50% speed). Reason: {reason}")
        else:
            self.nav_speed_factor = 1.0
            self.get_logger().info(f"Back to NORMAL mode (100% speed). Reason: {reason}")
        self.update_speed_limit()

    def update_speed_limit(self):
        final_factor = self.nav_speed_factor * self.event_speed_factor
        final_factor = max(0.0, min(1.0, final_factor))

        msg = SpeedLimit()
        msg.speed_limit = float(final_factor * 100.0)
        msg.percentage = True

        self.get_logger().info(
            f"Speed limit updated: nav_factor={self.nav_speed_factor:.2f}, "
            f"event_factor={self.event_speed_factor:.2f}, final_factor={final_factor:.2f}, "
            f"published limit={msg.speed_limit:.1f}%"
        )
        self.speed_limit_pub.publish(msg)

    # ------------------------------------------------------------------
    # 交通事件守护：处理 STOP_SIGN / RED
    # ------------------------------------------------------------------
    def navigation_guard_timer(self):
        if not self.navigation_active or not self.system_ready:
            return

        now = time.time()
        ev = self.current_event_type
        dist = self.current_event_distance

        # ================= STOP SIGN 逻辑 =================
        # if self.stop_sign_state == "IDLE":
        #     if ev == "STOP_SIGN" and 0.0 < dist < 10.0:
        #         self.stop_sign_state = "APPROACHING"
        #         if not self.red_light_active and self.event_speed_factor > 0.5:
        #             self.event_speed_factor = 0.5
        #             self.get_logger().info(f"[STOP_SIGN] Detected at {dist:.2f} m → slow down to 0.5x")
        #             self.update_speed_limit()

        # if self.stop_sign_state == "APPROACHING":
        #     if ev == "STOP_SIGN" and 0.0 < dist <= 1.0:
        #         self.stop_sign_state = "HOLDING"
        #         self.stop_sign_hold_end_time = now + 3.0
        #         self.event_speed_factor = 0.00
        #         self.get_logger().info(f"[STOP_SIGN] Reached close ({dist:.2f} m) → HOLD 3s")
        #         self.update_speed_limit()

        # if self.stop_sign_state == "HOLDING":
        #     if ev == "STOP_SIGN" and 0.0 < dist <= 1.0:
        #         self.stop_sign_hold_end_time = now + 3.0  # 刷新计时
        #     if now >= self.stop_sign_hold_end_time:
        #         self.stop_sign_state = "DONE"
        #         if not self.red_light_active:
        #             self.event_speed_factor = 0.8
        #             self.get_logger().info("[STOP_SIGN] Timer expired → RESUME (0.8x)")
        #             self.update_speed_limit()
        #     else:
        #         self.event_speed_factor = 0.00
        #         self.update_speed_limit()

        # # ================= RED / GREEN 逻辑 =================
        # if ev == "RED" and 0.0 < dist <= 1.2:
        #     if not self.red_light_active:
        #         self.red_light_active = True
        #         self.red_light_hold_end_time = now + 3.0
        #         self.event_speed_factor = 0.01
        #         self.get_logger().info(f"[RED] Detected at {dist:.2f} m → STOP (buffer)")
        #         self.update_speed_limit()

        # if self.red_light_active:
        #     if ev == "RED" and 0.0 < dist <= 1.2:
        #         self.red_light_hold_end_time = now + 3.0
        #     if now >= self.red_light_hold_end_time:
        #         self.red_light_active = False
        #         self.event_speed_factor = 0.8
        #         self.get_logger().info("[RED] Buffer expired (GREEN/NONE) → RESUME")
        #         self.update_speed_limit()
        #     else:
        #         self.event_speed_factor = 0.01
        #         self.update_speed_limit()

        # ================= STOP EVENT 逻辑 (STOP_SIGN / RED 统一) =================
        # 检测到 STOP_SIGN 或 RED 且距离 <=1.2 m → 停车 3s 缓冲
        if (ev == "STOP_SIGN" or ev == "RED") and 0.0 < dist <= 1.2:
            if not self.stop_event_active:
                self.stop_event_active = True
                self.stop_event_hold_end_time = now + 3.0  # 初始化3s计时（防抖）
                self.event_speed_factor = 0.01
                event_name = "STOP_SIGN" if ev == "STOP_SIGN" else "RED"
                self.get_logger().info(
                    f"[{event_name}] Detected at {dist:.2f} m → STOP and wait (3s buffer)"
                )
                self.update_speed_limit()

        # 事件解除条件：缓冲时间结束后变为 GREEN/NONE 或距离无效才解除
        if self.stop_event_active:
            if (ev == "STOP_SIGN" or ev == "RED") and 0.0 < dist <= 1.2:
                # 目标还在，重置3s计时
                self.stop_event_hold_end_time = now + 3.0
                event_name = "STOP_SIGN" if ev == "STOP_SIGN" else "RED"
                self.get_logger().debug(f"[{event_name}] Target still close ({dist:.2f} m), refresh 3s buffer")
            else:
                # 目标丢失或变化，继续计时
                self.get_logger().debug(f"[STOP_EVENT] Target missed ({dist:.2f} m), buffer remaining: {self.stop_event_hold_end_time - now:.2f}s")
            
            # 检查计时是否结束 AND 事件变化
            if now >= self.stop_event_hold_end_time and (ev not in ["STOP_SIGN", "RED"] or dist < 0.0):
                self.stop_event_active = False
                self.event_speed_factor = 0.8
                self.get_logger().info("[STOP_EVENT] Buffer expired (GREEN/NONE) → RESUME")
                self.update_speed_limit()
            else:
                # 计时未结束，保持停车
                self.event_speed_factor = 0.01
                self.update_speed_limit()

    # ------------------------------------------------------------------
    # Pick / Place 命令发送
    # ------------------------------------------------------------------
    def send_pick_command(self):
        msg = PickPlaceEvent()
        msg.status = 1  # 到达 pick 点，启动 Pick
        self.operation_pub.publish(msg)

    def send_place_command(self):
        msg = PickPlaceEvent()
        msg.status = 3  # 到达 place 点，启动 Place
        self.operation_pub.publish(msg)

    # ------------------------------------------------------------------
    # PickPlace 状态回传（注意：对方 5Hz 重复发，需要去重）
    # ------------------------------------------------------------------
    def pick_status_callback(self, msg: PickPlaceEvent):
        # ✅ 去重：同一个 status 重复发（5Hz）只处理第一次
        if msg.status == self._last_pickplace_status:
            return
        self._last_pickplace_status = msg.status

        if msg.status == 2:
            # PICK completed
            if self.waiting_for_pick_place and self.current_index == self.PICK_INDEX:
                self.get_logger().info("Pick completed. Advancing to next waypoint.")
                self.waiting_for_pick_place = False
                self.current_index += 1
                self.send_next_goal()

        elif msg.status == 4:
            # PLACE completed
            if self.waiting_for_pick_place and self.current_index == self.PLACE_INDEX:
                self.get_logger().info("Place completed. Advancing to next waypoint.")
                self.waiting_for_pick_place = False
                self.current_index += 1
                self.send_next_goal()


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
