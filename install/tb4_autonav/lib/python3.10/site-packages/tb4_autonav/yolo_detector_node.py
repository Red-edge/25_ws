#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO


class YoloDetectorNode(Node):
    """
    简单版 YOLO 目标检测节点：

      - 订阅 /camera/color/image_raw
      - 使用 YOLOv8n 做推理
      - 在图像上画框 + 类别，并发布到 /yolo/image_annotated
      - 控制推理频率避免 CPU 被打爆
    """

    def __init__(self):
        super().__init__("yolo_detector_node")

        # 参数：相机话题 & 模型路径 & 推理间隔
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("inference_interval", 0.05)  # 秒，0.2s ~= 5Hz

        image_topic = self.get_parameter("image_topic").value
        model_path = self.get_parameter("model_path").value
        self.inference_interval = float(self.get_parameter("inference_interval").value)

        self.bridge = CvBridge()

        # 订阅相机图像（best effort / queue=1 更稳，可以之后再加 QoS）
        self.sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10,
        )

        # 发布带框图像
        self.pub_annotated = self.create_publisher(
            Image,
            "/yolo/image_annotated",
            10,
        )

        # 可以后面再加一个 /yolo/detections 的结构化结果，这里先从可视化开始

        # 加载 YOLO 模型
        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        self.model = YOLO(model_path)
        self.get_logger().info("YOLO model loaded.")

        # 控制推理频率
        self.last_infer_time = 0.0

    def image_callback(self, msg: Image):
        # 控制推理频率：例如每 0.2 秒推一次
        now = time.time()
        if now - self.last_infer_time < self.inference_interval:
            return
        self.last_infer_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # 可选：降低分辨率以提速
        h, w = frame.shape[:2]
        frame_resized = cv2.resize(frame, (w // 2, h // 2))
        frame_resized = frame

        # YOLO 推理（不需要梯度）
        results = self.model(
            frame_resized,
            verbose=False,
        )

        # 取第一张图片的结果
        annotated = frame_resized.copy()
        if len(results) > 0:
            r = results[0]
            boxes = r.boxes

            for box in boxes:
                # box.xyxy, box.cls, box.conf
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                cls_id = int(box.cls[0].cpu().numpy())
                conf = float(box.conf[0].cpu().numpy())

                x1, y1, x2, y2 = xyxy.tolist()

                # 类别名
                cls_name = self.model.names.get(cls_id, str(cls_id))

                # 画矩形框
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # 标注文字
                label = f"{cls_name} {conf:.2f}"
                cv2.putText(
                    annotated,
                    label,
                    (x1, max(y1 - 5, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

        # 发布带框图像（保持原编码为 bgr8）
        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        img_msg.header = msg.header  # 保留原来的时间戳与 frame_id
        self.pub_annotated.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("YoloDetectorNode interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
