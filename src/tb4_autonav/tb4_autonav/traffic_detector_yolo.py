#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional, Tuple

import cv2
import numpy as np
from ultralytics import YOLO  # pip install ultralytics

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from tb4_autonav_interfaces.msg import TrafficEvent, YoloTargetBias


class TrafficDetectorYolo(Node):
    """
    YOLO 视觉检测节点：

      - 订阅: /camera/camera/color/image_raw
      - YOLO 检测: stop sign / traffic light
      - 对 traffic light 再做颜色分析 -> RED / GREEN
      - 估算距离
      - 发布:
          1) /traffic_event      : TrafficEvent
          2) /yolo_target_bias   : YoloTargetBias
      - 目标丢失逻辑:
          - 一旦识别到目标，连续帧都发 has_target=True
          - 当连续 5 帧都没有目标时，发送一次 has_target=False
      - 可配置：
          - yolo_model        : 模型权重路径
          - yolo_img_size     : YOLO 推理使用的图片尺寸（imgsz）
          - yolo_infer_rate   : YOLO 推理频率（Hz）
    """

    def __init__(self):
        super().__init__("traffic_detector_yolo")

        self.bridge = CvBridge()
        self.ready = False

        # ---------------- YOLO 模型 & 参数 ----------------

        # 1) 模型路径
        model_path = self.declare_parameter(
            "yolo_model",
            "yolov8n.pt",  # 你可以改成自己训练好的模型路径
        ).get_parameter_value().string_value

        # 2) YOLO 输入图片大小 (imgsz)
        #    - 传给 ultralytics YOLO 的 imgsz 参数
        #    - 不会改变我们对像素坐标的使用（ultralytics 会把 box 映射回原图尺寸）
        self.yolo_img_size = self.declare_parameter(
            "yolo_img_size",
            320  # 默认 640
        ).get_parameter_value().integer_value

        # 3) YOLO 推理频率 (Hz)，例如 5Hz -> 每 0.2s 做一次推理
        yolo_infer_rate = self.declare_parameter(
            "yolo_infer_rate",
            15.0  # 默认 5 Hz
        ).get_parameter_value().double_value

        # 防止除 0，给一个最小频率
        if yolo_infer_rate <= 0.0:
            yolo_infer_rate = 1.0
        self.yolo_infer_period = 1.0 / yolo_infer_rate
        self.last_infer_time = 0.0  # 上一次 YOLO 推理时间（秒）

        self.get_logger().info(
            f"Loading YOLO model: {model_path}, imgsz={self.yolo_img_size}, "
            f"infer_rate={yolo_infer_rate:.2f} Hz (period={self.yolo_infer_period:.3f}s)"
        )
        self.model = YOLO(model_path)

        # YOLO 的类别名 -> 我们内部逻辑类别映射
        self.cls_map = {
            "traffic light": "LIGHT",
            "stop sign": "STOP_SIGN",
        }

        # ---------------- 距离估计参数 ----------------
        self.real_size_m = {
            "RED": 0.03,        # 圆形直径
            "GREEN": 0.03,      # 圆形直径
            "STOP_SIGN": 0.12,  # 正八边形高度
        }

        self.focal_length_px = 610.0

        self.max_detect_dist = {
            "RED": 8.0,
            "GREEN": 8.0,
            "STOP_SIGN": 12.0,
        }

        # ---------------- 发布者 ----------------

        # 1) 给导航用 - 逻辑事件
        self.event_pub = self.create_publisher(
            TrafficEvent,
            "/traffic_event",
            10,
        )

        # 2) 给云台用 - 目标中心偏差 + 距离
        self.bias_pub = self.create_publisher(
            YoloTargetBias,
            "/yolo_target_bias",
            10,
        )

        # ---------------- YOLO 目标丢失计数 ----------------
        self.miss_count = 0
        self.last_has_target = False

        self.current_event_type = "NONE"
        self.current_event_distance = -1.0

        # ---------------- 订阅相机图像 ----------------
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

        self.get_logger().info(
            "TrafficDetectorYolo started, listening to /camera/camera/color/image_raw"
        )

    # ------------------------------------------------------------------
    # 主图像回调
    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        # 先做图像转换
        try:
            bgr_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")
            return

        if not self.ready:
            self.ready = True
            self.get_logger().info("TrafficDetectorYolo: first image received, detector ready.")

        # ------------- 在这里做 YOLO 频率节流 -------------
        now = self.get_clock().now().nanoseconds * 1e-9  # 秒
        if (now - self.last_infer_time) < self.yolo_infer_period:
            # 还没到下一次推理时间，直接返回，不跑 YOLO
            return
        self.last_infer_time = now

        h, w, _ = bgr_img.shape

        # 运行 YOLO（指定 imgsz）
        try:
            results = self.model(bgr_img, imgsz=self.yolo_img_size, verbose=False)[0]
        except Exception as e:
            self.get_logger().error(f"YOLO inference error: {e}")
            return

        # 从 YOLO 结果中选择一个最优目标（最近 / 最大置信度）
        found, event_type, distance_m, cx, cy = self._select_best_target(results, bgr_img)

        # ========== 1) 发布给云台：YoloTargetBias ==========
        if found:
            self._publish_yolo_bias(event_type, cx, cy, distance_m, w, h)
            self.miss_count = 0
            self.last_has_target = True
        else:
            if self.last_has_target:
                self.miss_count += 1
                if self.miss_count >= 10:
                    # 连续 5 帧没目标 -> 正式 announce has_target=False
                    bias_msg = YoloTargetBias()
                    bias_msg.header.stamp = self.get_clock().now().to_msg()
                    bias_msg.has_target = False
                    bias_msg.type = "NONE"
                    bias_msg.distance_m = -1.0
                    bias_msg.u_norm = 0.0
                    bias_msg.v_norm = 0.0
                    self.bias_pub.publish(bias_msg)

                    self.last_has_target = False
                    self.miss_count = 0
            # 本来就没有目标且 miss_count 已经清零时，这一帧可以不发偏差信息

        # ========== 2) 发布给导航：TrafficEvent ==========
        ev_msg = TrafficEvent()
        ev_msg.stamp = self.get_clock().now().to_msg()
        ev_msg.is_ready = self.ready

        if found:
            ev_msg.type = event_type  # "RED" / "GREEN" / "STOP_SIGN"
            ev_msg.distance = float(distance_m)
            self.current_event_type = event_type
            self.current_event_distance = float(distance_m)
        else:
            ev_msg.type = "NONE"
            ev_msg.distance = -1.0
            self.current_event_type = "NONE"
            self.current_event_distance = -1.0

        self.event_pub.publish(ev_msg)

    # ------------------------------------------------------------------
    # YOLO 目标选择 + 类型判定 + 距离估计
    # ------------------------------------------------------------------
    def _select_best_target(
        self, results, bgr_img
    ) -> Tuple[bool, str, float, float, float]:
        """
        根据 YOLO 结果选出一个最优目标。

        返回:
          found: bool
          event_type: "RED" / "GREEN" / "STOP_SIGN" / "NONE"
          distance_m: float
          cx, cy: 目标中心像素坐标
        """
        h, w, _ = bgr_img.shape
        if results.boxes is None or len(results.boxes) == 0:
            return False, "NONE", -1.0, 0.0, 0.0

        names = results.names  # dict: class_id -> class_name

        best_score = 0.0
        best = None  # (event_type, distance, cx, cy)

        for box in results.boxes:
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            cls_name = names.get(cls_id, "")

            if cls_name not in self.cls_map:
                continue

            logical_cls = self.cls_map[cls_name]  # "LIGHT" or "STOP_SIGN"

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx = 0.5 * (x1 + x2)
            cy = 0.5 * (y1 + y2)
            bbox_h = max(1.0, (y2 - y1))
            bbox_w = max(1.0, (x2 - x1))

            # 1) 先估计距离
            if logical_cls == "STOP_SIGN":
                event_type = "STOP_SIGN"
                distance_m = self._estimate_distance_from_size(
                    size_px=bbox_h, type_name=event_type
                )
            else:
                # traffic light -> 需要根据颜色再区分 RED / GREEN
                event_type, distance_m = self._classify_traffic_light(
                    bgr_img, x1, y1, x2, y2
                )

            if event_type == "NONE" or distance_m <= 0.0:
                continue

            # 距离超出最大范围也丢弃
            max_d = self.max_detect_dist.get(event_type, 100.0)
            if distance_m > max_d:
                continue

            # 简单打分：conf / distance，越近 & 越高置信度越好
            score = conf / (distance_m + 1e-3)

            if score > best_score:
                best_score = score
                best = (event_type, distance_m, cx, cy)

        if best is None:
            return False, "NONE", -1.0, 0.0, 0.0

        return True, best[0], best[1], best[2], best[3]

    # ------------------------------------------------------------------
    # 交通灯颜色识别：RED / GREEN
    # ------------------------------------------------------------------
    def _classify_traffic_light(
        self, bgr_img, x1, y1, x2, y2
    ) -> Tuple[str, float]:
        """
        在 YOLO 检出的 traffic light bbox 内，用 HSV 判断红/绿。

        返回:
          ("RED" / "GREEN" / "NONE", distance_m)
        """
        h, w, _ = bgr_img.shape
        x1 = int(max(0, min(w - 1, x1)))
        x2 = int(max(0, min(w - 1, x2)))
        y1 = int(max(0, min(h - 1, y1)))
        y2 = int(max(0, min(h - 1, y2)))

        if x2 <= x1 or y2 <= y1:
            return "NONE", -1.0

        roi = bgr_img[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 红色（分两段）
        red_lower1 = np.array([0, 100, 80])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([160, 100, 80])
        red_upper2 = np.array([179, 255, 255])

        mask_red1 = cv2.inRange(hsv, red_lower1, red_upper1)
        mask_red2 = cv2.inRange(hsv, red_lower2, red_upper2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # 绿色
        green_lower = np.array([35, 80, 80])
        green_upper = np.array([85, 255, 255])
        mask_green = cv2.inRange(hsv, green_lower, green_upper)

        red_count = int(np.count_nonzero(mask_red))
        green_count = int(np.count_nonzero(mask_green))

        if red_count <= 0 and green_count <= 0:
            return "NONE", -1.0

        if red_count >= green_count:
            event_type = "RED"
        else:
            event_type = "GREEN"

        # 用高度估计距离
        bbox_h = float(y2 - y1)
        distance_m = self._estimate_distance_from_size(
            size_px=bbox_h, type_name=event_type
        )

        return event_type, distance_m

    # ------------------------------------------------------------------
    # 距离估计：统一公式
    # ------------------------------------------------------------------
    def _estimate_distance_from_size(self, size_px: float, type_name: str) -> float:
        """
        size_px:
          - 对红/绿灯：用 bbox 高度像素
          - 对 STOP_SIGN：用 bbox 高度像素

        distance ≈ real_size_m[type_name] * focal_length_px / size_px
        """
        if size_px <= 1.0:
            return -1.0

        H_real = self.real_size_m.get(type_name, 0.0)
        f = float(self.focal_length_px)
        if H_real <= 0.0 or f <= 0.0:
            return -1.0

        dist = (H_real * f) / float(size_px)
        dist = float(max(0.05, min(100.0, dist)))
        return dist

    # ------------------------------------------------------------------
    # 发布给云台的偏差信息
    # ------------------------------------------------------------------
    def _publish_yolo_bias(
        self,
        event_type: str,
        cx: float,
        cy: float,
        distance_m: float,
        img_w: int,
        img_h: int,
    ):
        """
        event_type : "RED" / "GREEN" / "STOP_SIGN"
        cx, cy     : 像素坐标
        distance_m : 距离
        img_w/h    : 图像宽高
        """
        msg = YoloTargetBias()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.has_target = True
        msg.type = event_type
        msg.distance_m = float(distance_m)

        # 像素坐标 -> 归一化偏差
        u = (cx - img_w / 2.0) / (img_w / 2.0)   # [-1, 1]
        v = (cy - img_h / 2.0) / (img_h / 2.0)   # [-1, 1]
        msg.u_norm = float(u)
        msg.v_norm = float(v)

        self.bias_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficDetectorYolo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("TrafficDetectorYolo interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    main()
