#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class TrafficDetector(Node):
    def __init__(self):
        super().__init__('traffic_detector')

        self.bridge = CvBridge()

        # 加载 YOLOv8 模型（预训练）
        self.model = YOLO("yolov8n.pt")

        # 订阅你的机器人彩色图像话题
        self.subscription = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",   # <-- 已根据你的机器人修改
            self.image_callback,
            10
        )

        # 发布交通状态
        self.pub = self.create_publisher(String, "/traffic_signal", 10)

        self.get_logger().info("YOLO Traffic Detector Started. Subscribing to /camera/camera/color/image_raw")

    def image_callback(self, msg):
        # 自动推断编码类型
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except:
            frame = self.bridge.imgmsg_to_cv2(msg)

        # YOLO 推理
        results = self.model(frame, verbose=False)[0]

        detected_state = "none"

        # 遍历检测结果
        for box in results.boxes:
            cls = int(box.cls[0])
            label = self.model.names[cls]

            # Stop Sign 类别一般为 "stop sign"
            if label.lower() in ["stop sign", "stop"]:
                detected_state = "stop"
                continue

            # YOLO traffic light → 判断颜色
            if label.lower() == "traffic light":
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                roi = frame[y1:y2, x1:x2]

                if roi.size == 0:
                    continue

                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

                # ------ 仅取 ROI 中心区域，避免背景影响 ------
                h, w = roi.shape[:2]
                cx, cy = w // 2, h // 2
                r = min(w, h) // 4        # 中心 1/4 半径

                mask_center = np.zeros((h, w), dtype=np.uint8)
                cv2.circle(mask_center, (cx, cy), r, 255, -1)

                hsv_center = cv2.bitwise_and(hsv, hsv, mask=mask_center)

                # 红灯范围
                red1 = cv2.inRange(hsv_center, (0, 120, 70), (10, 255, 255))
                red2 = cv2.inRange(hsv_center, (170, 120, 70), (180, 255, 255))
                red_mask = red1 + red2

                # 绿灯范围
                green_mask = cv2.inRange(hsv_center, (30, 60, 60), (110, 255, 255))

                if np.sum(red_mask) > 5000:
                    detected_state = "red"

                elif np.sum(green_mask) > 2000:
                    detected_state = "green"

        # 发布检测结果
        msg_pub = String()
        msg_pub.data = detected_state
        self.pub.publish(msg_pub)

        # Debug log（可关闭）
        if detected_state != "none":
            self.get_logger().info(f"Detected: {detected_state}")


def main(args=None):
    rclpy.init(args=args)
    node = TrafficDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()#!/usr/bin/env python3
