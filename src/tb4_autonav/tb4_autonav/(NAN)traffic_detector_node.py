#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from tb4_autonav_interfaces.msg import TrafficEvent


class TrafficDetector(Node):
    """
    独立视觉检测节点：
      - 订阅 /camera/camera/color/image_raw
      - 检测红灯 / 绿灯（圆形）/ Stop sign（正八边形）
      - 使用已知几何尺寸 + 相机焦距估算距离
      - 发布 /traffic_event: TrafficEvent
          is_ready: 节点是否已成功处理过图像
          type    : "RED" / "GREEN" / "STOP_SIGN" / "NONE"
          distance: 单位 m，未知则 -1.0

    特性：
      - 所有目标在画面下半区域检测
      - 非 NONE 信号：连续 stable_duration_threshold 秒才算识别成功
      - 当前帧未检测到任何信号 -> 立即重置为 NONE
      - 每种信号有最大识别距离，用于过滤远端噪声
      - Stop sign 在整个运行过程中只触发一次
    """

    def __init__(self):
        super().__init__("traffic_detector")

        self.bridge = CvBridge()
        self.debug_view = False
        # self.saved_once = False
        self.ready = False

        # === 事件稳定性参数 ===
        self.stable_duration_threshold = 1.0  # 非 NONE 连续多长时间才确认（秒）
        self.candidate_event_type = "NONE"
        self.candidate_event_distance = -1.0
        self.candidate_start_time = None

        # 当前对外发布的稳定事件
        self.stable_event_type = "NONE"
        self.stable_event_distance = -1.0

        # Stop sign 只触发一次
        self.stop_sign_triggered = False

        # === 目标真实尺寸（单位：m） ===
        # 红/绿灯：圆形，直径 0.03m
        # Stop sign：正八边形，高度 0.12m
        self.real_size_m = {
            "RED": 0.03,        # 圆形直径
            "GREEN": 0.03,      # 圆形直径
            "STOP_SIGN": 0.12,  # 正八边形高度
        }

        # D435i 彩色相机像素焦距（典型值），建议后续从 /camera_info 中读取 K[0] 替换
        self.focal_length_px = 610.0

        # 每种信号的最大有效识别距离（单位 m）
        self.max_detect_dist = {
            "RED": 8.0,
            "GREEN": 8.0,
            "STOP_SIGN": 12.0,
        }

        # 形态学操作核，去噪 / 填洞
        self.morph_kernel = np.ones((3, 3), np.uint8)

        # 发布 traffic_event
        self.event_pub = self.create_publisher(
            TrafficEvent,
            "/traffic_event",
            10,
        )

        # 订阅相机图像
        qos_cam = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.img_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback,
            qos_cam,
        )

        self.get_logger().info("TrafficDetector started, listening to /camera/camera/color/image_raw")

    # ------------------------------------------------------------------
    # 图像回调：检测 + 事件稳定化 + 发布
    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        try:
            bgr_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")
            return

        if not self.ready:
            self.ready = True
            self.get_logger().info("TrafficDetector: first image received, detector ready.")

        # 原始检测（未做 1s 去抖）
        raw_type, raw_distance = self.detect_traffic(bgr_img)

        # Stop sign 只触发一次：触发过之后，再检测到 STOP_SIGN 一律当 NONE
        if self.stop_sign_triggered and raw_type == "STOP_SIGN":
            raw_type = "NONE"
            raw_distance = -1.0

        now = self.get_clock().now().nanoseconds / 1e9
        self._update_stable_event(raw_type, raw_distance, now)

        # 发布“稳定后的事件”
        ev_msg = TrafficEvent()
        ev_msg.stamp = self.get_clock().now().to_msg()
        ev_msg.is_ready = self.ready
        ev_msg.type = self.stable_event_type
        ev_msg.distance = float(self.stable_event_distance)

        self.event_pub.publish(ev_msg)

        if self.debug_view:
            debug_img = bgr_img.copy()
            cv2.putText(
                debug_img,
                f"stable: {self.stable_event_type} dist: {self.stable_event_distance:.2f}m",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("traffic_detector", debug_img)
            cv2.waitKey(1)
        #     key = cv2.waitKey(1)
        # if key == ord('s') and not self.saved_once:
        #     cv2.imwrite("traffic_frame.png", bgr_img)
        #     self.saved_once = True
        #     self.get_logger().info("Saved frame to traffic_frame.png")

    # ------------------------------------------------------------------
    # 稳定事件更新逻辑：
    #   - 当前帧是 NONE：立刻重置为 NONE；
    #   - 当前帧是非 NONE：需要连续 stable_duration_threshold 秒才确认。
    # ------------------------------------------------------------------
    def _update_stable_event(self, raw_type: str, raw_distance: float, now: float):
        # 1. 当前帧没读到任何有效信息 -> 立即回到 NONE
        if raw_type == "NONE":
            self.candidate_event_type = "NONE"
            self.candidate_event_distance = -1.0
            self.candidate_start_time = None

            if self.stable_event_type != "NONE":
                self.stable_event_type = "NONE"
                self.stable_event_distance = -1.0
                self.get_logger().info("Stable event reset to NONE (no detection).")
            return

        # 2. 当前帧检测到非 NONE：做 1s 稳定检测
        if raw_type != self.candidate_event_type:
            self.candidate_event_type = raw_type
            self.candidate_event_distance = raw_distance
            self.candidate_start_time = now
            return

        if self.candidate_start_time is None:
            self.candidate_start_time = now

        duration = now - self.candidate_start_time

        if duration >= self.stable_duration_threshold and self.stable_event_type != self.candidate_event_type:
            self.stable_event_type = self.candidate_event_type
            self.stable_event_distance = self.candidate_event_distance

            if self.stable_event_type == "STOP_SIGN":
                self.stop_sign_triggered = True
                self.get_logger().info("STOP_SIGN confirmed (only once for this run).")

            self.get_logger().info(
                f"Stable event updated to {self.stable_event_type}, "
                f"distance = {self.stable_event_distance:.2f} m"
            )

    # ------------------------------------------------------------------
    # 检测：返回 (event_type, distance_m) —— 原始检测结果
    # ------------------------------------------------------------------
    def detect_traffic(self, bgr_img):
        """
        检测逻辑：
          - 下半部分中间区域：检测红灯 / 绿灯（圆形）
          - 下半部分：检测 Stop sign（大红色近似正八边形）
        距离估计：
          - 红/绿灯：用圆的直径像素 size_px
          - Stop sign：用外接矩形高度像素 size_px
          - 距离 ≈ real_size_m[type] * f / size_px
          - 再根据 max_detect_dist 过滤远处噪声
        """
        h, w, _ = bgr_img.shape
        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)

        # ------------ 红色掩膜（两段） ------------
        # H: 0~179, 提高 S/V 下限以减少背景干扰
        red_lower1 = np.array([0, 110, 70])
        red_upper1 = np.array([18, 255, 255])
        red_lower2 = np.array([220, 110, 70])
        red_upper2 = np.array([255, 255, 255])
        mask_red1 = cv2.inRange(hsv, red_lower1, red_upper1)
        mask_red2 = cv2.inRange(hsv, red_lower2, red_upper2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # ------------ 绿色掩膜 ------------
        green_lower = np.array([35, 80, 80])
        green_upper = np.array([85, 255, 255])
        mask_green = cv2.inRange(hsv, green_lower, green_upper)

        # 形态学操作：去噪 + 填洞
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, self.morph_kernel, iterations=1)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, self.morph_kernel, iterations=1)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, self.morph_kernel, iterations=1)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, self.morph_kernel, iterations=1)

        # ---------------- 信号灯区域：画面下半部分中间区域（圆形检测） ----------------
        light_top = int(h * 0.5)
        light_bottom = h
        col_start = int(w * 0.25)
        col_end = int(w * 0.75)

        roi_red = mask_red[light_top:light_bottom, col_start:col_end]
        roi_green = mask_green[light_top:light_bottom, col_start:col_end]

        red_found, red_area, red_radius_px = self._detect_circle_blob(
            roi_red,
            min_area=50,
            max_area=4000,
            min_circularity=0.6,
        )
        green_found, green_area, green_radius_px = self._detect_circle_blob(
            roi_green,
            min_area=50,
            max_area=4000,
            min_circularity=0.6,
        )

        # 红灯距离：使用圆直径像素
        red_dist = -1.0
        if red_found and red_radius_px > 0:
            red_diam_px = 2.0 * red_radius_px
            red_dist = self._estimate_distance_from_size(red_diam_px, "RED")
            if red_dist < 0 or red_dist > self.max_detect_dist["RED"]:
                red_found = False
                red_dist = -1.0

        # 绿灯距离：使用圆直径像素
        green_dist = -1.0
        if green_found and green_radius_px > 0:
            green_diam_px = 2.0 * green_radius_px
            green_dist = self._estimate_distance_from_size(green_diam_px, "GREEN")
            if green_dist < 0 or green_dist > self.max_detect_dist["GREEN"]:
                green_found = False
                green_dist = -1.0

        # ---------------- Stop sign 区域：下半部分全宽 ----------------
        stop_roi_top = int(h * 0.5)
        roi_stop = mask_red[stop_roi_top:h, :]
        stop_found, stop_area, stop_bbox_h = self._detect_stop_sign(
            roi_stop,
            min_area=300,  # 比圆灯大一些
        )

        stop_dist = -1.0
        if stop_found and stop_bbox_h > 0:
            stop_dist = self._estimate_distance_from_size(stop_bbox_h, "STOP_SIGN")
            if stop_dist < 0 or stop_dist > self.max_detect_dist["STOP_SIGN"]:
                stop_found = False
                stop_dist = -1.0

        # 优先级：STOP_SIGN > RED > GREEN > NONE
        if stop_found:
            return "STOP_SIGN", stop_dist
        if red_found:
            return "RED", red_dist
        if green_found:
            return "GREEN", green_dist

        return "NONE", -1.0

    # ------------------------------------------------------------------
    # 圆形 blob 检测（用于红/绿灯）
    # ------------------------------------------------------------------
    def _detect_circle_blob(
        self,
        mask,
        min_area=50,
        max_area=4000,
        min_circularity=0.6,
    ):
        """
        在 mask 中查找接近圆形的轮廓：
          - area 在 [min_area, max_area]
          - circularity = 4πA / P² >= min_circularity
        返回 (是否有, 最大面积, 对应外接圆半径像素)
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = 0.0
        best_radius = 0.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area or area > max_area:
                continue

            peri = cv2.arcLength(cnt, True)
            if peri <= 0:
                continue

            circularity = 4.0 * math.pi * area / (peri * peri)
            if circularity < min_circularity:
                continue

            (x, y), radius = cv2.minEnclosingCircle(cnt)
            if radius <= 1.0:
                continue

            if area > best_area:
                best_area = area
                best_radius = radius

        return (best_area > 0.0), best_area, best_radius

    # ------------------------------------------------------------------
    # Stop sign 检测：大红色近似正八边形
    # ------------------------------------------------------------------
    def _detect_stop_sign(self, mask, min_area=300):
        """
        在红色 mask 中寻找大的近似正多边形作为 Stop sign。
        条件：
          - 面积 > min_area
          - 近似多边形顶点数在 [6, 10]（接近正八边形）
          - 外接矩形宽高比接近 1
        返回 (是否有, 最大面积, 外接矩形高度像素)
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = 0.0
        best_h = 0.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue

            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            vertices = len(approx)
            if not (6 <= vertices <= 10):
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            if w <= 0 or h <= 0:
                continue

            aspect = w / float(h)
            if not (0.7 <= aspect <= 1.3):
                continue

            if area > best_area:
                best_area = area
                best_h = float(h)

        return (best_area > 0.0), best_area, best_h

    # ------------------------------------------------------------------
    # 根据“像素尺寸”估计距离（统一公式）
    # ------------------------------------------------------------------
    def _estimate_distance_from_size(self, size_px: float, type_name: str) -> float:
        """
        size_px:
          - 对红/绿灯：圆的直径像素
          - 对 STOP_SIGN：外接矩形高度像素

        distance ≈ real_size_m[type_name] * focal_length_px / size_px
        """
        if size_px <= 1.0:
            return -1.0

        H_real = self.real_size_m.get(type_name, 0.0)
        f = float(self.focal_length_px)
        if H_real <= 0.0 or f <= 0.0:
            return -1.0

        dist = (H_real * f) / float(size_px)
        dist = float(max(0.05, min(100.0, dist)))  # 简单裁剪
        return dist


def main(args=None):
    rclpy.init(args=args)
    node = TrafficDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("TrafficDetector interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    main()
