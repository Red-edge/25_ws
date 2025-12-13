#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from datetime import datetime
from typing import Optional

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

        # ================== 保存参数 ==================
        self.declare_parameter("save_dir", "/home/tony/25_ws/traffic/raw_images3")
        self.declare_parameter("window_name", "D435i Recorder (R=save, Q/ESC=quit)")
        self.declare_parameter("jpeg_quality", 95)

        self.save_dir = self.get_parameter("save_dir").get_parameter_value().string_value
        self.window_name = self.get_parameter("window_name").get_parameter_value().string_value
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        os.makedirs(self.save_dir, exist_ok=True)

        # ================== 云台姿态参数 ==================
        # 以正中心 (0,0) 为基准：yaw=0, pitch= -25（向下）
        self.declare_parameter("pan_tilt_yaw_deg", 0.0)
        self.declare_parameter("pan_tilt_pitch_deg", 20.0)
        self.declare_parameter("pan_tilt_speed_deg", 20.0)  # <= 30 deg/s

        self.pan_tilt_yaw_deg = float(self.get_parameter("pan_tilt_yaw_deg").value)
        self.pan_tilt_pitch_deg = float(self.get_parameter("pan_tilt_pitch_deg").value)
        self.pan_tilt_speed_deg = float(self.get_parameter("pan_tilt_speed_deg").value)

        # ================== ROS I/O ==================
        self.sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback,
            10,
        )

        self.pan_tilt_pub = self.create_publisher(
            PanTiltCmdDeg,
            "/pan_tilt_cmd_deg",
            10,
        )

        # ================== 内部状态 ==================
        self.latest_bgr: Optional[object] = None
        self.latest_stamp_str: str = ""
        self.have_frame = False
        self.paused = False
        self._shutdown_requested = False

        # 定时保持云台姿态
        self.pan_tilt_timer = self.create_timer(0.5, self.pan_tilt_hold_loop)

        # UI 刷新与键盘监听（建议 20~30Hz）
        self.ui_timer = self.create_timer(1.0 / 30.0, self.ui_loop)

        # OpenCV 窗口初始化
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.get_logger().info(
            f"ImageRecorder started.\n"
            f"  Save dir        : {self.save_dir}\n"
            f"  Window          : {self.window_name}\n"
            f"  Controls        : R=save, Q/ESC=quit, Space=pause\n"
            f"  Pan-tilt target : yaw={self.pan_tilt_yaw_deg:.1f}°, "
            f"pitch={self.pan_tilt_pitch_deg:.1f}° @ {self.pan_tilt_speed_deg:.1f} deg/s"
        )

    # ================== 云台保持姿态 ==================
    def pan_tilt_hold_loop(self):
        cmd = PanTiltCmdDeg()
        cmd.speed = int(self.pan_tilt_speed_deg)
        cmd.yaw = float(self.pan_tilt_yaw_deg)
        cmd.pitch = float(self.pan_tilt_pitch_deg)
        self.pan_tilt_pub.publish(cmd)

    # ================== 图像回调：只缓存最新帧 ==================
    def image_callback(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        self.latest_bgr = bgr
        self.have_frame = True

        # 用 ROS stamp 也行；这里用 msg.header.stamp + 本地时间都可
        self.latest_stamp_str = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

    # ================== UI：显示 + 读键盘 ==================
    def ui_loop(self):
        if self._shutdown_requested:
            return

        if not self.have_frame or self.latest_bgr is None:
            # 没帧就别刷
            key = cv2.waitKey(1) & 0xFF
            self._handle_key(key)
            return

        if not self.paused:
            show = self.latest_bgr.copy()
            cv2.putText(
                show,
                "R=save  Q/ESC=quit  Space=pause",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(self.window_name, show)

        key = cv2.waitKey(1) & 0xFF
        self._handle_key(key)

    def _handle_key(self, key: int):
        # q / ESC: 退出
        if key in (ord("q"), ord("Q"), 27):
            self.get_logger().info("Quit key pressed. Shutting down...")
            self._shutdown_requested = True
            self._safe_close()
            # 让 rclpy 退出 spin
            rclpy.shutdown()
            return

        # space: 暂停/继续刷新窗口
        if key == ord(" "):
            self.paused = not self.paused
            self.get_logger().info(f"Paused={self.paused}")
            return

        # r: 保存一张
        if key in (ord("r"), ord("R")):
            self.save_one_frame()

    def save_one_frame(self):
        if not self.have_frame or self.latest_bgr is None:
            self.get_logger().warn("No frame available yet.")
            return

        filename = os.path.join(self.save_dir, f"{self.latest_stamp_str}.jpg")
        params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)]
        ok = cv2.imwrite(filename, self.latest_bgr, params)

        if ok:
            self.get_logger().info(f"Saved {filename}")
        else:
            self.get_logger().warn(f"Failed to save image to {filename}")

    def _safe_close(self):
        try:
            cv2.destroyWindow(self.window_name)
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ImageRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ImageRecorder interrupted.")
    finally:
        node._safe_close()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
