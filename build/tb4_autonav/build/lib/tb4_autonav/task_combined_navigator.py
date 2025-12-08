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
          * nav_speed_factor = 1.0（正常）或 0.5（慢速）
        - 订阅 /traffic_event（来自 YOLO 检测）：
          * STOP_SIGN：
              - 第一次检测到 → 进入 APPROACHING
              - 当距离 <= 1.0 m 时 → 停车 3s（event_speed_factor = 0.0）
              - 3s 后恢复（event_speed_factor = 1.0），状态标记为 DONE（整个任务只触发一次）
          * RED：
              - 当 RED 且距离在 (0.5, 1.5] m 时 → 停车（event_speed_factor = 0.0）
              - 直到事件变为 GREEN 或 NONE → 恢复（event_speed_factor = 1.0）
        - 最终下发速度限制：
              speed_limit = nav_speed_factor * event_speed_factor （百分比 0–1）
    """

    def __init__(self):
        super().__init__("waypoint_navigator")

        # ========== 1) 导航目标点 ==========
        self.waypoints: List[Tuple[float, float, float]] = [
            (-2.19, -15.47, -1.38),
            (-2.319, -16.316, -1.418),
            (-1.996, -18.327, 0.015),
            (0.349, -17.850, 0.116),
            (0.985, -16.312, 1.713),
            (-0.781, -16.476, -3.112),
            (-2.19, -15.47, -1.38)
        ]
        self.current_index: int = 0

        self.navigation_active = False   # 是否已经进入“任务执行阶段”

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
        self.slow_mode = False  # False=正常速度，True=慢速档（50%）

        # nav_speed_factor: 来自 AMCL / 预热的速度因子（0~1）
        self.nav_speed_factor = 1.0

        # event_speed_factor: 来自 STOP_SIGN / RED 等事件（0~1）
        self.event_speed_factor = 1.0

        # ========== 4) 重定位预热相关 ==========
        self.prelocalization_active = False   # 是否在预热阶段
        self.ready_to_navigate = False        # 预热是否完成
        self.init_good_count = 0
        self.init_std_threshold = 1.0         # std_xy 阈值
        self.init_required_consecutive = 5    # 连续 5 次小于阈值

        # 预热运动参数
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.preloc_linear_speed = 0.05             # 前进线速度（m/s）
        self.preloc_angular_speed = 0.4             # 转向角速度（rad/s）
        self.preloc_distance_limit = None           # 本次预热直线的最大位移（米）

        self.preloc_safety_margin = 0.2             # 与障碍物预留的安全距离（米）
        self.preloc_min_distance = 0.1              # 最少也挪 0.1m
        self.preloc_max_distance = 1.0              # 最多 1m

        # 预热状态机：FORWARD（直线前进） / TURN（原地旋转 180°）
        self.preloc_state = "FORWARD"
        self.turn_start_yaw = None                  # 开始转向时的 yaw
        self.turn_tolerance = math.radians(10.0)    # 允许误差 ±10°

        # LiDAR / Odom 状态
        self.last_scan: LaserScan | None = None
        self.last_odom_x: float | None = None
        self.last_odom_y: float | None = None
        self.last_yaw: float | None = None
        self.preloc_leg_start_x: float | None = None
        self.preloc_leg_start_y: float | None = None

        # ========== 5) 交通事件相关 ==========
        self.current_event_type = "NONE"
        self.current_event_distance = float("inf")

        # STOP SIGN 状态机："IDLE" / "APPROACHING" / "HOLDING" / "DONE"
        self.stop_sign_state = "IDLE"
        self.stop_sign_hold_end_time = 0.0

        # 红灯状态
        self.red_light_active = False

        # ========== 6) ActionClient: NavigateToPose ==========
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
        )

        # ========== 7) SpeedLimit publisher ==========
        self.speed_limit_pub = self.create_publisher(
            SpeedLimit,
            "speed_limit",
            10,
        )
        # 初始化一次速度限制
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

        # ========== 9) 订阅 Odom（locomotion 自检 + 预热用） ==========
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos_best_effort,
        )

        # ========== 10) 订阅 LiDAR 扫描（预热用） ==========
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_best_effort,
        )

        # ========== 11) 订阅 TrafficEvent（vision 状态 + 交通事件） ==========
        self.traffic_sub = self.create_subscription(
            TrafficEvent,
            "/traffic_event",
            self.traffic_event_callback,
            10,
        )

        # ========== 12) 定时器 ==========
        # 系统自检定时器
        self.system_check_timer = self.create_timer(1.0, self.system_check_callback)
        # 交通事件 & 中断逻辑守护
        self.guard_timer = self.create_timer(0.1, self.navigation_guard_timer)
        # 预热运动定时器（0.1s 一次）
        self.preloc_timer = self.create_timer(0.1, self.prelocalization_motion)

        self.get_logger().info("WaypointNavigator started. Waiting for system check to pass...")

    # ------------------------------------------------------------------
    # 工具函数：从四元数取 yaw / 角度归一化
    # ------------------------------------------------------------------
    @staticmethod
    def _quat_to_yaw(q) -> float:
        """从 geometry_msgs/Quaternion 计算 yaw (弧度)"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(a: float) -> float:
        """把角度归一化到 [-pi, pi]"""
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
                if self.vision_ready:
                    self.get_logger().info("[SYSTEM CHECK] vision OK (TrafficDetector ready).")
                else:
                    self.get_logger().warn("[SYSTEM CHECK] vision NOT ready yet.")
            else:
                self.get_logger().warn("[SYSTEM CHECK] no TrafficEvent received yet.")

        if self.nav2_ready and self.loco_ready and self.vision_ready:
            self.system_ready = True
            self.system_check_timer.cancel()
            self.get_logger().info("[SYSTEM CHECK] All subsystems READY. Start prelocalization.")
            # 所有子系统 OK 后，启动预热阶段
            self.start_prelocalization()

    def start_prelocalization(self):
        """进入重定位预热阶段：限制速度为 30%，开始直线往复运动"""
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

        # 预热阶段：nav_speed_factor=0.3，event_speed_factor保持1.0
        self.nav_speed_factor = 0.3
        self.event_speed_factor = 1.0
        self.update_speed_limit()

        self.get_logger().info(
            "Prelocalization phase started: straight-line forward motion "
            "with 180° turns, waiting for AMCL std to stabilize..."
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
        # 只有在预热阶段并且还没计算出直线距离时，才用这帧 scan 来估算
        if self.prelocalization_active and self.preloc_distance_limit is None:
            self.update_preloc_distance_from_scan()

    def odom_callback(self, msg: Odometry):
        # locomotion 自检使用的时间戳
        self.last_odom_time = time.time()

        # 预热 / 其他用途使用的位置和朝向
        self.last_odom_x = msg.pose.pose.position.x
        self.last_odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.last_yaw = self._quat_to_yaw(q)

        # 初始化当前半程的起点
        if self.preloc_leg_start_x is None or self.preloc_leg_start_y is None:
            self.preloc_leg_start_x = self.last_odom_x
            self.preloc_leg_start_y = self.last_odom_y

    # ------------------------------------------------------------------
    # 预热阶段：用雷达计算安全直线距离
    # ------------------------------------------------------------------
    def _compute_min_forward_range(self) -> float | None:
        """
        从当前 last_scan 中，取正前方 ±45° 的范围，找出其中最小的合法距离。
        返回 None 表示当前 scan 无法得到有效距离。
        """
        if self.last_scan is None:
            return None

        scan = self.last_scan
        if scan.angle_increment == 0.0:
            return None

        half_angle = math.radians(45.0)  # 正前方 ±45°
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
        """根据当前 scan 估算一条前方安全直线运动距离（0.1m ~ 1.0m）"""
        min_forward = self._compute_min_forward_range()
        if min_forward is None:
            return

        # 在最近障碍物前预留 safety_margin
        if min_forward <= self.preloc_safety_margin + 0.05:
            # 前面很近就有障碍，退而求其次，预设一个很小的位移
            d = self.preloc_min_distance
        else:
            d = min_forward - self.preloc_safety_margin
            d = max(d, self.preloc_min_distance)
            d = min(d, self.preloc_max_distance)

        self.preloc_distance_limit = d
        self.get_logger().info(
            f"[Preloc] Straight-line distance set to {d:.2f} m "
            f"(min forward LiDAR range={min_forward:.2f} m)"
        )

    # ------------------------------------------------------------------
    # 预热阶段：仅“前进 + 原地 180° 转向 + 前进回原点”
    # ------------------------------------------------------------------
    def prelocalization_motion(self):
        """
        在预热阶段：
          FORWARD：沿当前朝向前进，走到 preloc_distance_limit
          TURN   ：原地旋转 180°，然后切回 FORWARD

        整个过程始终 linear.x > 0，不允许后退。
        """
        if not self.prelocalization_active or not self.system_ready:
            return

        # 必须先有 LiDAR 距离估计
        if self.preloc_distance_limit is None:
            if self.last_scan is not None:
                self.update_preloc_distance_from_scan()
            if self.preloc_distance_limit is None:
                # 还无法估算安全距离，先不动
                return

        # 必须要有 odom 和 yaw
        if (
            self.last_odom_x is None
            or self.last_odom_y is None
            or self.last_yaw is None
        ):
            return

        if self.preloc_leg_start_x is None or self.preloc_leg_start_y is None:
            self.preloc_leg_start_x = self.last_odom_x
            self.preloc_leg_start_y = self.last_odom_y

        # ========== 状态 1：直线前进 ==========
        if self.preloc_state == "FORWARD":
            # 计算当前这一“腿”已经走了多少
            dx = self.last_odom_x - self.preloc_leg_start_x
            dy = self.last_odom_y - self.preloc_leg_start_y
            dist = math.sqrt(dx * dx + dy * dy)

            # 如果走够了预设距离 -> 停车 + 切到 TURN 状态
            if dist >= self.preloc_distance_limit:
                self.get_logger().info(
                    f"[Preloc] Forward leg reached {dist:.2f} m, start turning 180°."
                )
                self.stop_robot_motion()
                self.preloc_state = "TURN"
                self.turn_start_yaw = self.last_yaw
                return

            # 安全检查：前方障碍太近 -> 提前转向
            if self.last_scan is not None:
                min_forward = self._compute_min_forward_range()
                if (
                    min_forward is not None
                    and min_forward <= self.preloc_safety_margin
                ):
                    self.get_logger().warn(
                        f"[Preloc] Obstacle at {min_forward:.2f} m ahead, "
                        "early start turning 180°."
                    )
                    self.stop_robot_motion()
                    self.preloc_state = "TURN"
                    self.turn_start_yaw = self.last_yaw
                    return

            # 正常直线前进（只允许正向）
            twist = Twist()
            twist.linear.x = self.preloc_linear_speed   # 始终 > 0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

        # ========== 状态 2：原地转 180° ==========
        elif self.preloc_state == "TURN":
            if self.turn_start_yaw is None:
                # 第一次进来时记录起始 yaw
                self.turn_start_yaw = self.last_yaw

            # 计算已经转过的角度（归一化到 [-pi, pi]）
            delta = self._normalize_angle(self.last_yaw - self.turn_start_yaw)

            if abs(delta) >= math.pi - self.turn_tolerance:
                # 已经差不多转了 180°，停止旋转，开始下一段直线
                self.get_logger().info(
                    f"[Preloc] Turn 180° done (delta={delta:.2f} rad). "
                    "Start next forward leg."
                )
                self.stop_robot_motion()
                self.preloc_state = "FORWARD"
                self.turn_start_yaw = None

                # 新的一腿从当前位置重新计距离
                self.preloc_leg_start_x = self.last_odom_x
                self.preloc_leg_start_y = self.last_odom_y
                return

            # 继续原地旋转
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = self.preloc_angular_speed  # 固定一个方向转
            self.cmd_vel_pub.publish(twist)

    def stop_robot_motion(self):
        """发布一个零速度，确保机器人停住"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    # ------------------------------------------------------------------
    # 导航目标发送 & 回调
    # ------------------------------------------------------------------
    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("All waypoints completed. Mission finished.")
            self.set_slow_mode(False, reason="mission_finished")
            self.navigation_active = False
            return

        # 每个 goal 开始，只重置“与 goal 相关”的逻辑；
        # STOP_SIGN / RED 交通规则由全局状态机控制，不在这里清零。
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
    # AMCL / TrafficEvent 回调
    # ------------------------------------------------------------------
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance
        var_x = cov[0]
        var_y = cov[7]

        std_x = math.sqrt(var_x) if var_x > 0.0 else 0.0
        std_y = math.sqrt(var_y) if var_y > 0.0 else 0.0
        std_xy = max(std_x, std_y)

        # ======== 先处理“预热重定位”逻辑 ========
        if self.prelocalization_active:
            if std_xy < self.init_std_threshold:
                self.init_good_count += 1
            else:
                self.init_good_count = 0

            if self.init_good_count >= self.init_required_consecutive:
                # 通过预热阶段
                self.prelocalization_active = False
                self.ready_to_navigate = True
                self.init_good_count = 0

                # 停止预热运动，恢复正常速度限制
                self.stop_robot_motion()
                self.nav_speed_factor = 1.0
                self.slow_mode = False
                self.update_speed_limit()

                self.get_logger().info(
                    f"Prelocalization finished (std_xy < {self.init_std_threshold} "
                    f"for {self.init_required_consecutive} samples)."
                )

                # 预热完成且系统已就绪，启动任务
                if self.system_ready and not self.navigation_active:
                    self.get_logger().info("Starting mission after prelocalization.")
                    self.start_mission()

            # 预热阶段不再执行慢速档逻辑，直接返回
            return

        # ======== 预热完成后，执行原来的慢速档逻辑 ========
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
        """AMCL 慢速档：只修改 nav_speed_factor"""
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
        """根据 nav_speed_factor * event_speed_factor 合成最终 SpeedLimit"""
        final_factor = self.nav_speed_factor * self.event_speed_factor
        final_factor = max(0.0, min(1.0, final_factor))

        msg = SpeedLimit()
        msg.speed_limit = float(final_factor * 100.0)  # 0–100%
        msg.percentage = True
        self.speed_limit_pub.publish(msg)

    # ------------------------------------------------------------------
    # 交通事件守护：处理 RED/STOP_SIGN 规则 + 恢复
    # ------------------------------------------------------------------
    def navigation_guard_timer(self):
        # 只有在导航任务进行中、系统 ready 时才处理交通规则
        if not self.navigation_active or not self.system_ready:
            return

        now = time.time()
        ev = self.current_event_type
        dist = self.current_event_distance

        # ================= STOP SIGN 逻辑 =================
        if self.stop_sign_state == "IDLE":
            # 第一次看到 STOP_SIGN，进入 APPROACHING 状态
            if ev == "STOP_SIGN" and 0.0 < dist < 10.0:
                self.stop_sign_state = "APPROACHING"
                self.get_logger().info(
                    f"[STOP_SIGN] Detected at {dist:.2f} m → APPROACHING"
                )

        if self.stop_sign_state == "APPROACHING":
            # 当距离 <= 1.0m 时停车 3 秒
            if ev == "STOP_SIGN" and 0.0 < dist <= 1.0:
                self.stop_sign_state = "HOLDING"
                self.stop_sign_hold_end_time = now + 3.0
                self.event_speed_factor = 0.0
                self.get_logger().info(
                    f"[STOP_SIGN] Reached ~1m ({dist:.2f} m) → HOLD 3s"
                )
                self.update_speed_limit()

        if self.stop_sign_state == "HOLDING":
            # 计时结束，恢复前进
            if now >= self.stop_sign_hold_end_time:
                self.stop_sign_state = "DONE"
                # 若此时没有红灯生效，则恢复 event_speed_factor
                if not self.red_light_active:
                    self.event_speed_factor = 1.0
                    self.get_logger().info("[STOP_SIGN] Hold finished → RESUME")
                    self.update_speed_limit()

        # DONE 状态下不再响应后续 STOP_SIGN（整个任务只触发一次）

        # ================= RED / GREEN 逻辑 =================
        # RED 且距离在 (0.5, 1.5] m 区间 → 停车
        if ev == "RED" and 0.5 < dist <= 1.5:
            if not self.red_light_active:
                self.red_light_active = True
                self.event_speed_factor = 0.0
                self.get_logger().info(
                    f"[RED] Detected at {dist:.2f} m → STOP and wait"
                )
                self.update_speed_limit()

        # 红灯解除条件：事件变为 GREEN 或 NONE（或距离无效）
        if self.red_light_active:
            if ev in ("GREEN", "NONE") or dist < 0.0:
                self.red_light_active = False
                # 若此时 STOP_SIGN 不在 HOLDING 状态，才真正恢复 event_speed_factor
                if self.stop_sign_state != "HOLDING":
                    self.event_speed_factor = 1.0
                    self.get_logger().info("[RED] Cleared (GREEN/NONE) → RESUME")
                    self.update_speed_limit()


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
