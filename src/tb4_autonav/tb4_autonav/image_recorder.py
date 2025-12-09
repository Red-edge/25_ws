#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from pan_tilt_msgs.msg import PanTiltCmdDeg  # 按你的实际 msg 包路径修改


class ImageRecorder(Node):
    def __init__(self):
        super().__init__("image_recorder")

        self.bridge = CvBridge()

        # ================== 图像保存相关参数 ==================
        self.declare_parameter("save_dir", "/home/tony/25_ws/traffic/raw_images3")
        self.declare_parameter("save_every_n_frames", 10)

        self.save_dir = self.get_parameter("save_dir").get_parameter_value().string_value
        self.save_every_n = int(self.get_parameter("save_every_n_frames").value)

        os.makedirs(self.save_dir, exist_ok=True)
        self.frame_count = 0

        # ================== 云台姿态相关参数 ==================
        # 以正中心 (0,0) 为基准，默认：yaw=0，pitch=-25 度（向下看）
        self.declare_parameter("pan_tilt_yaw_deg", 0.0)
        self.declare_parameter("pan_tilt_pitch_deg", 20.0)
        self.declare_parameter("pan_tilt_speed_deg", 20.0)  # 速度上限 30deg/s

        self.pan_tilt_yaw_deg = float(self.get_parameter("pan_tilt_yaw_deg").value)
        self.pan_tilt_pitch_deg = float(self.get_parameter("pan_tilt_pitch_deg").value)
        self.pan_tilt_speed_deg = float(self.get_parameter("pan_tilt_speed_deg").value)

        # ================== 订阅相机图像 ==================
        self.sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback,
            10,
        )

        # ================== 发布云台命令 ==================
        self.pan_tilt_pub = self.create_publisher(
            PanTiltCmdDeg,
            "/pan_tilt_cmd_deg",
            10,
        )

        # 定时保持云台在目标姿态（0.5s 发一次）
        self.pan_tilt_timer = self.create_timer(0.5, self.pan_tilt_hold_loop)

        self.get_logger().info(
            f"ImageRecorder started.\n"
            f"  Save dir          : {self.save_dir}\n"
            f"  Save every N frames: {self.save_every_n}\n"
            f"  Pan-tilt target   : yaw={self.pan_tilt_yaw_deg:.1f}°, "
            f"pitch={self.pan_tilt_pitch_deg:.1f}° @ {self.pan_tilt_speed_deg:.1f} deg/s"
        )

    # ================== 保持云台俯视 ==================
    def pan_tilt_hold_loop(self):
        """
        定期发送一个云台角度命令：
          - yaw 固定在 pan_tilt_yaw_deg
          - pitch 固定在 pan_tilt_pitch_deg（默认 -25°，向下）
        """
        cmd = PanTiltCmdDeg()
        cmd.speed = int(self.pan_tilt_speed_deg)
        cmd.yaw = float(self.pan_tilt_yaw_deg)
        cmd.pitch = float(self.pan_tilt_pitch_deg)
        self.pan_tilt_pub.publish(cmd)

    # ================== 图像回调：保存图片 ==================
    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.save_every_n != 0:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = os.path.join(self.save_dir, f"{timestamp}.jpg")
        ok = cv2.imwrite(filename, bgr)
        if ok:
            self.get_logger().info(f"Saved {filename}")
        else:
            self.get_logger().warn(f"Failed to save image to {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ImageRecorder interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
