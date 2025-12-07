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
      - 订阅 /camera/color/image_raw
      - 检测红灯 / 绿灯 / Stop sign
      - 粗略估算距离
      - 发布 /traffic_event: TrafficEvent
          is_ready: 节点是否已成功处理过图像
          type    : "RED" / "GREEN" / "STOP_SIGN" / "NONE"
          distance: 单位 m，未知则 -1.0
    """

    def __init__(self):
        super().__init__("traffic_detector")

        self.bridge = CvBridge()
        self.debug_view = False  # 想看调试画面可以改 True
        self.ready = False       # 是否已成功处理过至少一帧图像

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
            "/camera/color/image_raw",  # 若相机话题不同，请在这里改
            self.image_callback,
            qos_cam,
        )

        self.get_logger().info("TrafficDetector started, listening to /camera/color/image_raw")

    # ------------------------------------------------------------------
    # 图像回调：检测 + 发布事件
    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        try:
            bgr_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")
            return

        # 至少成功处理一帧图像后认为 ready
        if not self.ready:
            self.ready = True
            self.get_logger().info("TrafficDetector: first image received, detector ready.")

        ev_type, distance = self.detect_traffic(bgr_img)

        ev_msg = TrafficEvent()
        ev_msg.stamp = self.get_clock().now().to_msg()
        ev_msg.is_ready = self.ready
        ev_msg.type = ev_type
        ev_msg.distance = float(distance)

        self.event_pub.publish(ev_msg)

        if self.debug_view:
            cv2.imshow("traffic_detector", bgr_img)
            cv2.waitKey(1)

    # ------------------------------------------------------------------
    # 检测：返回 (event_type, distance_m)
    # ------------------------------------------------------------------
    def detect_traffic(self, bgr_img):
        """
        简单版检测：
          - 上方中间区域：小红块 → 红灯；小绿块 → 绿灯
          - 下方区域   ：大红多边形 → Stop sign
        距离估计：distance ≈ k / sqrt(area)，仅作粗略估计。
        """
        h, w, _ = bgr_img.shape
        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)

        # ------------ 红色掩膜（两段） ------------
        red_lower1 = np.array([0, 80, 80])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([160, 80, 80])
        red_upper2 = np.array([179, 255, 255])
        mask_red1 = cv2.inRange(hsv, red_lower1, red_upper1)
        mask_red2 = cv2.inRange(hsv, red_lower2, red_upper2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # ------------ 绿色掩膜 ------------
        green_lower = np.array([40, 80, 80])
        green_upper = np.array([85, 255, 255])
        mask_green = cv2.inRange(hsv, green_lower, green_upper)

        # 全区域：检测信号灯
        # upper_h = int(h / 3)
        upper_h = h
        col_start = int(w * 0.25)
        col_end = int(w * 0.75)

        roi_red = mask_red[0:upper_h, col_start:col_end]
        roi_green = mask_green[0:upper_h, col_start:col_end]

        red_found, red_area = self._detect_blob(roi_red, min_area=80, max_area=2000)
        green_found, green_area = self._detect_blob(roi_green, min_area=80, max_area=2000)

        # 全区域：检测 Stop sign
        # roi_stop = mask_red[int(h * 0.3):h, :]
        roi_stop = mask_red[0:h, :]
        stop_found, stop_area = self._detect_stop_sign(roi_stop)

        # 优先级：STOP_SIGN > RED > GREEN > NONE
        if stop_found:
            dist = self._estimate_distance(stop_area)
            return "STOP_SIGN", dist

        if red_found:
            dist = self._estimate_distance(red_area)
            return "RED", dist

        if green_found:
            dist = self._estimate_distance(green_area)
            return "GREEN", dist

        return "NONE", -1.0

    def _detect_blob(self, mask, min_area=80, max_area=2000):
        """在 mask 中查找面积在 [min_area, max_area] 的轮廓，返回(是否有, 最大面积)"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = 0.0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if min_area <= area <= max_area:
                best_area = max(best_area, area)
        return (best_area > 0.0), best_area

    def _detect_stop_sign(self, mask, min_area=500):
        """
        在红色 mask 中寻找大的多边形作为 Stop sign。
        条件：面积 > min_area，顶点数在 [6, 10]。
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = 0.0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            vertices = len(approx)
            if 6 <= vertices <= 10:
                best_area = max(best_area, area)
        return (best_area > 0.0), best_area

    def _estimate_distance(self, area, k=30.0):
        """
        粗略距离估计：distance ≈ k / sqrt(area)
        area 越大，目标越近。k 需要根据实测调整。
        """
        if area <= 0.0:
            return -1.0
        dist = k / math.sqrt(area)
        dist = max(0.1, min(5.0, dist))  # 限制在 [0.1, 5] m 范围
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
