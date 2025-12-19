# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import time

# import cv2
# import numpy as np
# import rclpy
# from rclpy.node import Node

# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# from ultralytics import YOLO


# class YoloDetectorNode(Node):
#     """
#     简单版 YOLO 目标检测节点：

#       - 订阅 /camera/color/image_raw
#       - 使用 YOLOv8n 做推理
#       - 在图像上画框 + 类别，并发布到 /yolo/image_annotated
#       - 控制推理频率避免 CPU 被打爆
#     """

#     def __init__(self):
#         super().__init__("yolo_detector_node")

#         # 参数：相机话题 & 模型路径 & 推理间隔
#         self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
#         self.declare_parameter("model_path", "best_1.pt")
#         self.declare_parameter("inference_interval", 0.05)  # 秒，0.2s ~= 5Hz

#         image_topic = self.get_parameter("image_topic").value
#         model_path = self.get_parameter("model_path").value
#         self.inference_interval = float(self.get_parameter("inference_interval").value)

#         self.bridge = CvBridge()

#         # 订阅相机图像（best effort / queue=1 更稳，可以之后再加 QoS）
#         self.sub = self.create_subscription(
#             Image,
#             image_topic,
#             self.image_callback,
#             10,
#         )

#         # 发布带框图像
#         self.pub_annotated = self.create_publisher(
#             Image,
#             "/yolo/image_annotated",
#             10,
#         )

#         # 可以后面再加一个 /yolo/detections 的结构化结果，这里先从可视化开始

#         # 加载 YOLO 模型
#         self.get_logger().info(f"Loading YOLO model from: {model_path}")
#         self.model = YOLO(model_path)
#         self.get_logger().info("YOLO model loaded.")

#         # 控制推理频率
#         self.last_infer_time = 0.0

#     def image_callback(self, msg: Image):
#         # 控制推理频率：例如每 0.2 秒推一次
#         now = time.time()
#         if now - self.last_infer_time < self.inference_interval:
#             return
#         self.last_infer_time = now

#         try:
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
#         except Exception as e:
#             self.get_logger().error(f"cv_bridge error: {e}")
#             return

#         # 可选：降低分辨率以提速
#         h, w = frame.shape[:2]
#         frame_resized = cv2.resize(frame, (w // 2, h // 2))
#         frame_resized = frame

#         # YOLO 推理（不需要梯度）
#         results = self.model(
#             frame_resized,
#             verbose=False,
#         )

#         # 取第一张图片的结果
#         annotated = frame_resized.copy()
#         if len(results) > 0:
#             r = results[0]
#             boxes = r.boxes

#             for box in boxes:
#                 # box.xyxy, box.cls, box.conf
#                 xyxy = box.xyxy[0].cpu().numpy().astype(int)
#                 cls_id = int(box.cls[0].cpu().numpy())
#                 conf = float(box.conf[0].cpu().numpy())

#                 x1, y1, x2, y2 = xyxy.tolist()

#                 # 类别名
#                 cls_name = self.model.names.get(cls_id, str(cls_id))

#                 # 画矩形框
#                 cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

#                 # 标注文字
#                 label = f"{cls_name} {conf:.2f}"
#                 cv2.putText(
#                     annotated,
#                     label,
#                     (x1, max(y1 - 5, 0)),
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     0.5,
#                     (0, 255, 0),
#                     1,
#                     cv2.LINE_AA,
#                 )

#         # 发布带框图像（保持原编码为 bgr8）
#         img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
#         img_msg.header = msg.header  # 保留原来的时间戳与 frame_id
#         self.pub_annotated.publish(img_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = YoloDetectorNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("YoloDetectorNode interrupted by user.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header # Header for YoloTargetBias
from cv_bridge import CvBridge
from ultralytics import YOLO
# 新增：导入自定义消息
from tb4_autonav_interfaces.msg import YoloTargetBias

class YoloDetectorNode(Node):
    """
    简单版 YOLO 目标检测节点：
      - 订阅 /camera/camera/color/image_raw
      - 使用 YOLOv8n 做推理
      - 在图像上画框 + 类别 + 实时颜色分类/Stop Sign，并发布到 /yolo/image_annotated
      - 发布 YoloTargetBias 到 /traffic_event：包含 has_target, type, distance_m (基于面积比例估算), u_norm/v_norm (中心偏差)
        * is_ready 逻辑：模型加载后，每帧 has_target=True (即使 NONE，也表示 vision ready)
        * 所有检测到的交通信号：type="RED"/"GREEN"/"STOP_SIGN"/"NONE" (最高置信)
        * distance_m：粗略估算 d = 2.0 * sqrt(0.01 / area_ratio) [假设 0.01 ratio @ 2m]
        * u_norm/v_norm：基于选中 box 中心 (图像中心为 0, [-1,1])
      - 控制推理频率避免 CPU 被打爆
      - 去除远近距离判断：所有交通目标均参与状态更新，远近统一在 task combine 处理
    """
    def __init__(self):
        super().__init__("yolo_detector_node")
        # 参数：相机话题 & 模型路径 & 推理间隔
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("model_path", "best_1.pt")
        self.declare_parameter("inference_interval", 0.05) # 秒，~20Hz
        # 移除：最小 box 面积比例阈值 (远近判断移到 task combine)
        # 新增：距离估算参考 (ref_ratio=0.01 @ ref_d=2.0m)
        self.declare_parameter("ref_area_ratio", 0.01)
        self.declare_parameter("ref_distance_m", 2.0)
        image_topic = self.get_parameter("image_topic").value
        model_path = self.get_parameter("model_path").value
        self.inference_interval = float(self.get_parameter("inference_interval").value)
        self.ref_area_ratio = float(self.get_parameter("ref_area_ratio").value)
        self.ref_distance_m = float(self.get_parameter("ref_distance_m").value)
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
        # 修改：发布 YoloTargetBias 到 /traffic_event (取代旧 String 状态)
        self.pub_traffic_event = self.create_publisher(
            YoloTargetBias,
            "/traffic_event",
            10,
        )
        # 加载 YOLO 模型
        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        self.model = YOLO(model_path)
        self.model_loaded = True  # 模型加载成功后置 True (用于 vision ready 信号)
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
        threshold = 30 # 最小比例阈值 (可调，基于你的光照)
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
        frame_resized = frame # 切换到原图
        # YOLO 推理
        results = self.model(
            frame_resized,
            verbose=False,
        )
        # 取第一张图片的结果
        annotated = frame_resized.copy()
        latest_state = "NONE" # 默认状态 (最高置信的交通信号)
        max_conf = 0.0
        # 新增：选中目标的 box 信息 (用于 distance/u_norm/v_norm)
        selected_area_ratio = 0.0
        selected_center_x = w / 2.0
        selected_center_y = h / 2.0
      
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
                img_area = w * h # 图像总面积 (原图)
                box_area_ratio = box_area / img_area # 归一化比例
              
                # 计算中心 (用于 u/v_norm)
                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0
              
                # 新增：优先处理 Stop Sign (去除远近判断，直接处理)
                traffic_state = "NONE"
                if cls_name == "Stop sign": # 替换为你的实际 Stop Sign 类名
                    traffic_state = "STOP_SIGN" # 固定状态
                    print(f"stop area ratio: {box_area_ratio}")
                   
                    # 更新最新状态 (最高置信)
                    if conf > max_conf:
                        latest_state = traffic_state
                        max_conf = conf
                        selected_area_ratio = box_area_ratio
                        selected_center_x = center_x
                        selected_center_y = center_y
                   
                    # 画矩形框 (红色框突出 Stop)
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    # 标注文字 (带状态，去除 [Near]/[Far])
                    label = f"{cls_name} {conf:.2f} ({traffic_state})"
                    cv2.putText(
                        annotated,
                        label,
                        (x1, max(y1 - 5, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255), # 红色字体
                        1,
                        cv2.LINE_AA,
                    )
                    self.get_logger().info(f"Detected: {cls_name} - State: {traffic_state} (conf: {conf:.2f}, area_ratio: {box_area_ratio:.3f})")
               
                # 原逻辑：处理红绿灯 (去除远近判断，直接处理)
                elif cls_name == "Dim traffic light": # 替换为你的实际类名
                    # 实时抠取 ROI
                    roi = annotated[y1:y2, x1:x2]
                    # 实时分类
                    color_state = self.classify_color(roi)
                    traffic_state = color_state if color_state != "Other" else "NONE"
                  
                    # 更新最新状态 (最高置信)
                    if conf > max_conf:
                        latest_state = traffic_state
                        max_conf = conf
                        selected_area_ratio = box_area_ratio
                        selected_center_x = center_x
                        selected_center_y = center_y
                  
                    # 画矩形框
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    # 标注文字 (带分类，去除 [Near]/[Far])
                    label = f"{cls_name} {conf:.2f} ({traffic_state})"
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
                    self.get_logger().info(f"Detected: {cls_name} - Color: {traffic_state} (conf: {conf:.2f}, area_ratio: {box_area_ratio:.3f})")
              
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
      
        # 新增：构建并发布 YoloTargetBias 到 /traffic_event
        event_msg = YoloTargetBias()
        event_msg.header = msg.header  # 继承图像时间戳/帧ID
        if not self.model_loaded:
            event_msg.has_target = False
            event_msg.type = "NONE"
            event_msg.distance_m = -1.0
            event_msg.u_norm = 0.0
            event_msg.v_norm = 0.0
        else:
            # 模型加载后，每帧 has_target=True (表示 vision ready，即使无目标)
            event_msg.has_target = True
            event_msg.type = latest_state
            if selected_area_ratio > 0.0 and max_conf > 0.5:  # 置信阈值，避免低conf噪声
                # 距离估算：d = ref_d * sqrt(ref_ratio / current_ratio)
                event_msg.distance_m = self.ref_distance_m * math.sqrt(self.ref_area_ratio / selected_area_ratio)
                # 归一化偏差：[-1,1]，图像中心=0
                event_msg.u_norm = ((selected_center_x / w) - 0.5) * 2.0  # 左负右正
                event_msg.v_norm = ((selected_center_y / h) - 0.5) * 2.0  # 上负下正
            else:
                event_msg.distance_m = -1.0
                event_msg.u_norm = 0.0
                event_msg.v_norm = 0.0
        self.pub_traffic_event.publish(event_msg)
        # 可选：日志当前事件
        if event_msg.has_target and event_msg.type != "NONE":
            self.get_logger().info(f"Traffic Event: {event_msg.type} @ {event_msg.distance_m:.2f}m (u={event_msg.u_norm:.2f}, v={event_msg.v_norm:.2f})")

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
