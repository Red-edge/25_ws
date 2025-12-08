#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class WaypointNavigator(Node):
    """
    逻辑：
      - 启动后先进入“重定位预热阶段”：
          * 根据 /scan 计算正前方一条安全直线距离（最多 1m）
          * 在这条直线上前后往复运动，速度限制为 30%
          * 监控 AMCL std，连续 5 次 < 1.0 认为定位稳定
          * 然后停止预热运动，恢复 100% 速度，开始导航
      - 导航阶段：
          * 顺序依次导航到写死的 waypoints（map 坐标系）
          * 订阅 /amcl_pose，自适应调整 speed_limit（慢速/正常）
    """

    def __init__(self):
        super().__init__("waypoint_navigator")

        # =============== 1) 参数 / 状态 ===============
        # 这里写死一组 waypoints: (x, y, yaw) in map frame
        self.waypoints: List[Tuple[float, float, float]] = [
            (1.28, -14.38, -0.67),
            (0.83, -18.44, -1.0),
            (-2.7, -17.8, 0.75),
            (-1.97, -15.53, 0.06),
            (1.28, -14.38, -0.67),
        ]
        self.current_index: int = 0

        # AMCL 置信度相关（导航阶段用）
        self.std_high_threshold = 1.3
        self.std_low_threshold = 0.8
        self.required_consecutive = 4  # 连续几次满足条件才触发模式切换

        self.high_count = 0
        self.low_count = 0
        self.slow_mode = False  # False=正常速度, True=慢速档

        # 导航目标状态
        self.navigation_active = False

        # ======== 重定位预热相关状态 ========
        self.prelocalization_active = True          # 是否在“预热重定位”阶段
        self.ready_to_navigate = False              # 是否可以开始导航
        self.init_good_count = 0                    # std < 1.0 的连续次数
        self.init_std_threshold = 1.0               # 预热阶段判定“好”的阈值
        self.init_required_consecutive = 5          # 连续 5 次“好”才通过

        # 预热直线运动相关
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.preloc_linear_speed = 0.05             # 线速度（m/s），很慢
        self.preloc_distance_limit = None           # 本次预热直线的最大位移（米）

        # 预热直线运动相关
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # 预热阶段只允许“前进”，不再用正负方向，而是通过朝向 + 状态机控制往复
        self.preloc_linear_speed = 0.05             # 前进线速度（m/s）
        self.preloc_angular_speed = 0.4             # 转向角速度（rad/s）
        self.preloc_distance_limit = None           # 本次预热直线的最大位移（米）

        # 预热状态机：FORWARD（直线前进） / TURN（原地旋转 180°）
        self.preloc_state = "FORWARD"
        self.turn_start_yaw = None                  # 开始转向时的 yaw
        self.turn_tolerance = math.radians(10.0)    # 允许误差 ±10°

        # 安全距离等参数
        self.preloc_safety_margin = 0.2             # 与障碍物预留的安全距离（米）
        self.preloc_min_distance = 0.1              # 最少也挪 0.1m
        self.preloc_max_distance = 1.0              # 最多 1m

        # LiDAR / Odom 状态
        self.last_scan: LaserScan | None = None
        self.last_odom_x: float | None = None
        self.last_odom_y: float | None = None
        self.last_yaw: float | None = None          # 通过 odom 获取的当前 yaw
        self.preloc_leg_start_x: float | None = None
        self.preloc_leg_start_y: float | None = None

        # =============== 2) ActionClient: NavigateToPose ===============
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
        )

        # =============== 3) SpeedLimit publisher ===============
        self.speed_limit_pub = self.create_publisher(
            SpeedLimit,
            "speed_limit",
            10,
        )

        # 启动时先进入“预热模式”：速度限制到 30%
        self.publish_speed_limit(percentage=0.3)

        # =============== 4) 订阅 AMCL pose ===============
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

        # 订阅 LiDAR 扫描
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_best_effort,
        )

        # 订阅 odom，用于计算行驶距离
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos_best_effort,
        )

        # =============== 5) 定时器：预热运动 & 启动导航检查 ===============
        # 预热阶段前后往复运动（0.1s 一次，比较平滑）
        self.preloc_timer = self.create_timer(0.1, self.prelocalization_motion)

        # 每 0.5s 检查是否可以开始导航（nav2 ready + 预热通过）
        self.start_timer = self.create_timer(0.5, self.start_if_ready)

        self.get_logger().info(
            "WaypointNavigator node started. Prelocalization phase (safe straight-line forward-backward motion)..."
        )

    # ------------------------------------------------------------------
    # 工具函数：从四元数取 yaw / 角度归一化
    # ------------------------------------------------------------------
    @staticmethod
    def _quat_to_yaw(q) -> float:
        """从 geometry_msgs/Quaternion 计算 yaw (弧度)"""
        # ROS 默认四元数: x, y, z, w
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
    # LiDAR & Odom 回调
    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg
        # 只有在预热阶段并且还没计算出直线距离时，才用这帧 scan 来估算
        if self.prelocalization_active and self.preloc_distance_limit is None:
            self.update_preloc_distance_from_scan()

    def odom_callback(self, msg: Odometry):
        self.last_odom_x = msg.pose.pose.position.x
        self.last_odom_y = msg.pose.pose.position.y

        # 当前 yaw
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
        从当前 last_scan 中，取正前方 ±15° 的范围，找出其中最小的合法距离。
        返回 None 表示当前 scan 无法得到有效距离。
        """
        if self.last_scan is None:
            return None

        scan = self.last_scan
        if scan.angle_increment == 0.0:
            return None

        half_angle = math.radians(45.0)  # 正前方 ±15°
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
    # 一、预热阶段：在安全直线上前后往复
    # ------------------------------------------------------------------
    def prelocalization_motion(self):
        """
        在预热阶段：
          FORWARD：沿当前朝向前进，走到 preloc_distance_limit
          TURN   ：原地旋转 180°，然后切回 FORWARD

        整个过程始终 linear.x > 0，不允许后退。
        """
        if not self.prelocalization_active:
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
    # 二、导航任务启动
    # ------------------------------------------------------------------
    def start_if_ready(self):
        """
        定时器回调：只有在
          1) 预热阶段通过（ready_to_navigate=True）
          2) navigate_to_pose action server 已经 ready
        时才启动第一个导航目标。
        """
        if self.navigation_active:
            return

        if not self.ready_to_navigate:
            # 还在重定位预热阶段，等待 AMCL std 稳定
            return

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("NavigateToPose action server not available yet...")
            return

        self.get_logger().info(
            "Prelocalization OK and NavigateToPose server ready. Starting mission."
        )
        self.navigation_active = True
        self.send_next_goal()

    # ------------------------------------------------------------------
    # 三、顺序发送导航目标
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
        if status == 4:  # STATUS_SUCCEEDED
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
    # 四、AMCL 置信度监控
    # ------------------------------------------------------------------
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance  # 长度 36 的数组

        # 提取 x、y 的方差（cov[0], cov[7]）
        var_x = cov[0]
        var_y = cov[7]

        std_x = math.sqrt(var_x) if var_x > 0.0 else 0.0
        std_y = math.sqrt(var_y) if var_y > 0.0 else 0.0
        std_xy = max(std_x, std_y)

        # ======== 先处理“预热重定位”逻辑 ========
        if self.prelocalization_active:
            # 认为 std_xy < 1.0 是“定位稳定”
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
                self.publish_speed_limit(percentage=1.0)

                self.get_logger().info(
                    f"Prelocalization finished (std_xy < {self.init_std_threshold} "
                    f"for {self.init_required_consecutive} samples)."
                )

            # 预热阶段不再执行慢速档逻辑，直接返回
            return

        # ======== 预热完成后，执行原来的慢速档逻辑 ========
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

    # ------------------------------------------------------------------
    # 五、速度限制控制
    # ------------------------------------------------------------------
    def set_slow_mode(self, enable: bool, reason: str = ""):
        """切换慢速 / 正常模式，并发布 speed_limit"""
        # 预热阶段不走这里，只有预热结束后才会用到
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
        # nav2 在 percentage=True 时通常用 0..100 表示百分比
        msg.speed_limit = float(percentage * 100.0)
        msg.percentage = True
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