#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # 新增：分类状态消息
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloDetectorNode(Node):
    """
    简单版 YOLO 目标检测节点：
      - 订阅 /camera/camera/color/image_raw
      - 使用 YOLOv8n 做推理
      - 在图像上画框 + 类别 + 实时颜色分类/Stop Sign + 距离判断，并发布到 /yolo/image_annotated
      - 实时发布分类状态到 /yolo/traffic_state (e.g., "Stop"/"Red"/"Green"/"Other")，仅近距离时
      - 控制推理频率避免 CPU 被打爆
    """
    def __init__(self):
        super().__init__("yolo_detector_node")
        # 参数：相机话题 & 模型路径 & 推理间隔
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("model_path", "best_1.pt")
        self.declare_parameter("inference_interval", 0.05)  # 秒，~20Hz
        # 新增：最小 box 面积比例阈值 (近距离判断)
        self.declare_parameter("min_box_area_ratio", 0.01)  # 5% 图像面积，约 2m 内
        self.declare_parameter("min_box_area_ratio_stop", 0.05)  # 5% 图像面积，约 2m 内
        image_topic = self.get_parameter("image_topic").value
        model_path = self.get_parameter("model_path").value
        self.inference_interval = float(self.get_parameter("inference_interval").value)
        self.min_box_area_ratio = float(self.get_parameter("min_box_area_ratio").value)
        self.min_box_area_ratio_stop = float(self.get_parameter("min_box_area_ratio_stop").value)
        self.bridge = CvBridge()
        # 订阅相机图像
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
        # 新增：发布实时分类状态 (最高置信的交通信号，近距离时)
        self.pub_traffic_state = self.create_publisher(
            String,
            "/yolo/traffic_state",
            10,
        )
        # 加载 YOLO 模型
        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        self.model = YOLO(model_path)
        self.get_logger().info("YOLO model loaded.")
        # 控制推理频率
        self.last_infer_time = 0.0

    def classify_color(self, roi):
        """实时三分类: Red/Green/Other (基于 HSV 像素比例)"""
        if roi.size == 0:
            return "Other"
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
       
        # 红灯 (双范围)
        red_mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        red_mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        red_mask = red_mask1 + red_mask2
       
        # 绿灯
        green_mask = cv2.inRange(hsv, np.array([30, 60, 60]), np.array([110, 255, 255]))
       
        total = roi.shape[0] * roi.shape[1]
        red_ratio = np.sum(red_mask) / total
        green_ratio = np.sum(green_mask) / total
        threshold = 30  # 最小比例阈值 (可调，基于你的光照)
        # print(f"ROI ratios - Red: {red_ratio:.3f}, Green: {green_ratio:.3f}, Threshold: {threshold}")
       
        if red_ratio > threshold:
            return "Red"
        elif green_ratio > threshold:
            return "Green"
        else:
            return "Other"

    def image_callback(self, msg: Image):
        # 控制推理频率
        now = time.time()
        if now - self.last_infer_time < self.inference_interval:
            return
        self.last_infer_time = now
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return
        # 可选：降低分辨率以提速 (当前用原图)
        h, w = frame.shape[:2]
        frame_resized = cv2.resize(frame, (w // 2, h // 2))
        frame_resized = frame  # 切换到原图
        # YOLO 推理
        results = self.model(
            frame_resized,
            verbose=False,
        )
        # 取第一张图片的结果
        annotated = frame_resized.copy()
        latest_state = "Other"  # 默认状态 (最高置信的交通信号，近距离)
        max_conf = 0.0
       
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
               
                # 新增：计算 box 面积比例 (距离代理)
                box_width = x2 - x1
                box_height = y2 - y1
                box_area = box_width * box_height
                img_area = w * h  # 图像总面积
                box_area_ratio = box_area / img_area  # 归一化比例
               
                # 新增：优先处理 Stop Sign (近距离时)
                if cls_name == "Stop sign":  # 替换为你的实际 Stop Sign 类名
                    if box_area_ratio > self.min_box_area_ratio_stop:  # ← Stop Sign 距离检查
                        print(f"stop area ratio: {box_area_ratio}")
                        traffic_state = "Stop"  # 固定状态
                        
                        # 更新最新状态 (最高置信)
                        max_conf = 45
                        if conf > max_conf:
                            latest_state = traffic_state
                            max_conf = conf
                        
                        # 画矩形框 (红色框突出 Stop)
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        # 标注文字 (带状态 + 距离提示)
                        label = f"{cls_name} {conf:.2f} ({traffic_state}) [Near]"
                        cv2.putText(
                            annotated,
                            label,
                            (x1, max(y1 - 5, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 0, 255),  # 红色字体
                            1,
                            cv2.LINE_AA,
                        )
                        self.get_logger().info(f"Detected: {cls_name} - State: {traffic_state} (conf: {conf:.2f}, area_ratio: {box_area_ratio:.3f})")
                    else:
                        # 远距离：画淡框，不更新状态
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), (128, 128, 128), 1)
                        label = f"{cls_name} {conf:.2f} [Far]"
                        cv2.putText(annotated, label, (x1, max(y1 - 5, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
                
                # 原逻辑：处理红绿灯 (近距离时)
                elif cls_name == "Dim traffic light":  # 替换为你的实际类名
                    if box_area_ratio > self.min_box_area_ratio:  # ← 红绿灯距离检查
                        # 实时抠取 ROI
                        roi = annotated[y1:y2, x1:x2]
                        # 实时分类
                        color_state = self.classify_color(roi)
                       
                        # 更新最新状态 (最高置信)
                        if conf > max_conf:
                            latest_state = color_state
                            max_conf = conf
                       
                        # 画矩形框
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        # 标注文字 (带分类 + 距离提示)
                        label = f"{cls_name} {conf:.2f} ({color_state}) [Near]"
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
                        self.get_logger().info(f"Detected: {cls_name} - Color: {color_state} (conf: {conf:.2f}, area_ratio: {box_area_ratio:.3f})")
                    else:
                        # 远距离：画淡框，不更新状态
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), (128, 128, 128), 1)
                        label = f"{cls_name} {conf:.2f} [Far]"
                        cv2.putText(annotated, label, (x1, max(y1 - 5, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
               
                else:
                    # 非交通信号：原标签 (绿色框)
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
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
       
        # 发布带框图像
        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        img_msg.header = msg.header
        self.pub_annotated.publish(img_msg)
       
        # 发布最新分类状态 (仅近距离有信号时)
        state_msg = String()
        state_msg.data = latest_state
        self.pub_traffic_state.publish(state_msg)

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