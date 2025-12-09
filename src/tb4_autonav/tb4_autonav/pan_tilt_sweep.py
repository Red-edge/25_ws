#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node

from pan_tilt_msgs.msg import PanTiltCmdDeg
from tb4_autonav_interfaces.msg import YoloTargetBias


class PanTiltSweepController(Node):
    """
    云台控制节点：

    - 模式 1：巡航扫动（SWEEP）
        yaw 在 [yaw_min_deg, yaw_max_deg] 之间匀速往返
        pitch 在 [pitch_min_deg, pitch_max_deg] 之间匀速往返
        两侧 soft_zone_deg 内速度线性下降到 10%，实现平滑减速再反向

    - 模式 2：目标追踪（TRACK）
        订阅 /yolo_target_bias:
          has_target=True 时:
            * 切换到 TRACK 模式
            * 根据 u_norm, v_norm 调整 yaw/pitch，使目标尽量保持在画面中心
            * yaw 限制扩大到 [-track_yaw_limit_deg, +track_yaw_limit_deg]
            * pitch 限制扩大到 [-track_pitch_limit_deg, +track_pitch_limit_deg]
          has_target=False 或超过 track_timeout 秒未收到目标:
            * 回到 SWEEP 模式，恢复原始角度限制
    """

    def __init__(self):
        super().__init__("pan_tilt_sweep_controller")

        # --------------- 参数声明：巡航模式 ---------------

        # 巡航角度区间（度）
        self.declare_parameter("yaw_min_deg", -10.0)
        self.declare_parameter("yaw_max_deg", 10.0)
        self.declare_parameter("pitch_min_deg", 20.0)
        self.declare_parameter("pitch_max_deg", 26.0)

        # 巡航角速度（度/秒）
        self.declare_parameter("yaw_speed_deg", 15.0)    # 最大 30 deg/s
        self.declare_parameter("pitch_speed_deg", 13.0)

        # 角度软区间：在距离边界 soft_zone_deg 内线性减速
        self.declare_parameter("soft_zone_deg", 0.0)

        # --------------- 参数声明：追踪模式 ---------------

        # 追踪模式下的角度限制（度）
        self.declare_parameter("track_yaw_limit_deg", 40.0)
        self.declare_parameter("track_pitch_limit_deg", 30.0)

        # 追踪模式下的简单 P 控制增益（deg / (norm * s)）
        self.declare_parameter("track_k_yaw", 50.0)
        self.declare_parameter("track_k_pitch", 50.0)

        # 归一化偏差死区（小于这个就不再动）
        self.declare_parameter("track_deadband", 0.05)

        # 若超过该时间没收到新的 has_target=True，则认为目标丢失（秒）
        self.declare_parameter("track_timeout", 5.0)

        # --------------- 读取参数 ---------------

        self.yaw_min = float(self.get_parameter("yaw_min_deg").value)
        self.yaw_max = float(self.get_parameter("yaw_max_deg").value)
        self.pitch_min = float(self.get_parameter("pitch_min_deg").value)
        self.pitch_max = float(self.get_parameter("pitch_max_deg").value)

        self.yaw_speed = abs(float(self.get_parameter("yaw_speed_deg").value))
        self.pitch_speed = abs(float(self.get_parameter("pitch_speed_deg").value))

        self.soft_zone = float(self.get_parameter("soft_zone_deg").value)

        self.track_yaw_limit = float(self.get_parameter("track_yaw_limit_deg").value)
        self.track_pitch_limit = float(self.get_parameter("track_pitch_limit_deg").value)
        self.track_k_yaw = float(self.get_parameter("track_k_yaw").value)
        self.track_k_pitch = float(self.get_parameter("track_k_pitch").value)
        self.track_deadband = float(self.get_parameter("track_deadband").value)
        self.track_timeout = float(self.get_parameter("track_timeout").value)

        # 防呆：保证 min <= max
        if self.yaw_min > self.yaw_max:
            self.yaw_min, self.yaw_max = self.yaw_max, self.yaw_min
        if self.pitch_min > self.pitch_max:
            self.pitch_min, self.pitch_max = self.pitch_max, self.pitch_min

        # 保存巡航模式下的原始角度范围，方便之后恢复
        self.base_yaw_min = self.yaw_min
        self.base_yaw_max = self.yaw_max
        self.base_pitch_min = self.pitch_min
        self.base_pitch_max = self.pitch_max

        # --------------- 发布命令 ---------------

        self.cmd_pub = self.create_publisher(
            PanTiltCmdDeg,
            "/pan_tilt_cmd_deg",
            10,
        )

        # 状态：当前角度 & 方向（+1 / -1）
        self.yaw_current = 0.0
        self.pitch_current = 0.0
        self.yaw_dir = 1.0      # +1 向 yaw_max 增加，-1 向 yaw_min 减小
        self.pitch_dir = 1.0    # 同理

        # 固定 speed 字段（协议里的速度，单位 deg/s）
        self.speed_field = max(self.yaw_speed, self.pitch_speed, 1.0)
        if self.speed_field > 30.0:
            self.speed_field = 30.0

        # --------------- 模式管理 ---------------

        # "SWEEP" 或 "TRACK"
        self.mode = "SWEEP"

        # 追踪目标的偏差
        self.target_u = 0.0
        self.target_v = 0.0
        self.last_bias_time = None  # rclpy.time.Time

        # 时间
        self.last_time = self.get_clock().now()

        # 订阅 YOLO 偏差信息
        self.yolo_sub = self.create_subscription(
            YoloTargetBias,
            "/yolo_target_bias",
            self.yolo_callback,
            10,
        )

        # 控制循环
        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

        self.get_logger().info(
            "PanTiltSweepController started.\n"
            f"  [SWEEP] yaw in [{self.base_yaw_min:.1f}, {self.base_yaw_max:.1f}] deg @ {self.yaw_speed:.1f} deg/s\n"
            f"  [SWEEP] pitch in [{self.base_pitch_min:.1f}, {self.base_pitch_max:.1f}] deg @ {self.pitch_speed:.1f} deg/s\n"
            f"  [TRACK] yaw limit ±{self.track_yaw_limit:.1f} deg, "
            f"pitch limit ±{self.track_pitch_limit:.1f} deg"
        )

    # -------------------- YOLO 偏差信息回调 --------------------
    def yolo_callback(self, msg: YoloTargetBias):
        """
        /yolo_target_bias 回调：
          - has_target=True 时进入/保持 TRACK 模式，并更新目标偏差
          - has_target=False 时立即退回 SWEEP 模式
        """
        now = self.get_clock().now()

        if msg.has_target:
            # 更新目标偏差
            self.target_u = float(msg.u_norm)
            self.target_v = float(msg.v_norm)
            self.last_bias_time = now

            # 进入 TRACK 模式
            if self.mode != "TRACK":
                self.enter_track_mode(reason=f"has_target=True, type={msg.type}")
        else:
            # YOLO 显式告诉我们“没有目标” -> 回到 SWEEP
            if self.mode == "TRACK":
                self.exit_track_mode(reason="has_target=False")

    # -------------------- 模式切换 --------------------
    def enter_track_mode(self, reason: str = ""):
        self.mode = "TRACK"
        # 扩大角度限制
        self.yaw_min = -self.track_yaw_limit
        self.yaw_max = +self.track_yaw_limit
        self.pitch_min = -self.track_pitch_limit
        self.pitch_max = +self.track_pitch_limit

        self.get_logger().info(
            f"[MODE] Enter TRACK mode. {reason} "
            f"(yaw ∈ [{self.yaw_min:.1f},{self.yaw_max:.1f}], "
            f"pitch ∈ [{self.pitch_min:.1f},{self.pitch_max:.1f}])"
        )

    def exit_track_mode(self, reason: str = ""):
        self.mode = "SWEEP"
        # 恢复原始巡航角度限制
        self.yaw_min = self.base_yaw_min
        self.yaw_max = self.base_yaw_max
        self.pitch_min = self.base_pitch_min
        self.pitch_max = self.base_pitch_max

        # 为了回到巡航模式时平滑一些，可以把当前角度钳在巡航范围内
        self.yaw_current = max(min(self.yaw_current, self.yaw_max), self.yaw_min)
        self.pitch_current = max(min(self.pitch_current, self.pitch_max), self.pitch_min)

        self.get_logger().info(
            f"[MODE] Back to SWEEP mode. {reason} "
            f"(yaw ∈ [{self.yaw_min:.1f},{self.yaw_max:.1f}], "
            f"pitch ∈ [{self.pitch_min:.1f},{self.pitch_max:.1f}])"
        )

    # -------------------- 主控制循环 --------------------
    def control_loop(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time.nanoseconds) * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now

        # TRACK 模式下检查超时
        if self.mode == "TRACK" and self.last_bias_time is not None:
            dt_since = (now.nanoseconds - self.last_bias_time.nanoseconds) * 1e-9
            if dt_since > self.track_timeout:
                # 超过 track_timeout 秒没新目标，认为丢失 -> 回到 SWEEP
                self.exit_track_mode(reason=f"target timeout {dt_since:.2f}s")

        # ---------------- 模式 1：SWEEP ----------------
        if self.mode == "SWEEP":
            self._update_sweep(dt)

        # ---------------- 模式 2：TRACK ----------------
        else:  # self.mode == "TRACK"
            self._update_track(dt)

        # 发送命令
        cmd = PanTiltCmdDeg()
        cmd.speed = int(self.speed_field)
        cmd.yaw = float(self.yaw_current)
        cmd.pitch = float(self.pitch_current)

        self.cmd_pub.publish(cmd)

    # -------------------- 巡航模式更新 --------------------
    def _update_sweep(self, dt: float):
        """
        在 yaw / pitch 上做匀速往返，角度两端 soft_zone_deg 内减速。
        """

        # yaw 更新
        if self.yaw_speed > 0.0 and self.yaw_min != self.yaw_max:
            self.yaw_current, self.yaw_dir = self._update_sweep_angle(
                current=self.yaw_current,
                direction=self.yaw_dir,
                min_angle=self.yaw_min,
                max_angle=self.yaw_max,
                base_speed=self.yaw_speed,
                dt=dt,
            )

        # pitch 更新
        if self.pitch_speed > 0.0 and self.pitch_min != self.pitch_max:
            self.pitch_current, self.pitch_dir = self._update_sweep_angle(
                current=self.pitch_current,
                direction=self.pitch_dir,
                min_angle=self.pitch_min,
                max_angle=self.pitch_max,
                base_speed=self.pitch_speed,
                dt=dt,
            )

    def _update_sweep_angle(
        self,
        current: float,
        direction: float,
        min_angle: float,
        max_angle: float,
        base_speed: float,
        dt: float,
    ):
        """
        单轴（yaw 或 pitch）的匀速往返更新，带软区间减速。

        direction: +1 往 max_angle 方向，-1 往 min_angle 方向
        """
        if max_angle <= min_angle or base_speed <= 0.0:
            return current, direction

        # 计算到“当前运动方向所在边界”的距离
        if direction > 0:
            d = max_angle - current
        else:
            d = current - min_angle

        d = max(0.0, d)

        if self.soft_zone > 0.0:
            # 软区间内速度从 10% ~ 100% 线性变化
            if d <= self.soft_zone:
                factor = 0.1 + 0.9 * (d / self.soft_zone)
            else:
                factor = 1.0
        else:
            factor = 1.0

        v = base_speed * factor * direction  # deg/s
        new_angle = current + v * dt

        # 处理超出边界 -> 钳住 + 反向
        if new_angle > max_angle:
            new_angle = max_angle
            direction = -1.0
        elif new_angle < min_angle:
            new_angle = min_angle
            direction = 1.0

        return new_angle, direction

    # -------------------- 追踪模式更新 --------------------
    def _update_track(self, dt: float):
        """
        使用简单 P 控制，将目标偏差 (u_norm,v_norm) 转换为 yaw/pitch 的角度变化。
          - u_norm > 0: 目标在画面右侧 -> yaw 增大（向右转）
          - v_norm > 0: 目标在画面下侧 -> pitch 减小（向下转）
        """

        u = float(self.target_u)
        v = float(self.target_v)

        # 死区处理，避免小抖动不断微调
        if abs(u) < self.track_deadband:
            u = 0.0
        if abs(v) < self.track_deadband:
            v = 0.0

        # yaw 调整
        if u != 0.0:
            delta_yaw = -self.track_k_yaw * u * dt  # deg
            self.yaw_current += delta_yaw

        # pitch 调整（这里约定 v>0 表示目标在下方 -> pitch 减小）
        if v != 0.0:
            delta_pitch = self.track_k_pitch * v * dt  # deg
            self.pitch_current += delta_pitch

        # 限制在追踪范围内
        self.yaw_current = max(-self.track_yaw_limit, min(self.yaw_current, self.track_yaw_limit))
        self.pitch_current = max(-self.track_pitch_limit, min(self.pitch_current, self.track_pitch_limit))


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
