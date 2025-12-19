#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from datetime import datetime
from typing import Optional, List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from pan_tilt_msgs.msg import PanTiltCmdDeg


class VideoRecorder(Node):
    def __init__(self):
        super().__init__("video_recorder")

        self.bridge = CvBridge()

        # ================== 保存参数 ==================
        self.declare_parameter("save_dir", "/home/tony/25_ws/traffic/videos")
        self.declare_parameter("window_name", "D435i Video Recorder (S=start, X=stop+save, Q=quit)")
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("codec", "mp4v")

        self.save_dir = self.get_parameter("save_dir").get_parameter_value().string_value
        self.window_name = self.get_parameter("window_name").get_parameter_value().string_value
        self.fps = float(self.get_parameter("fps").value)
        self.codec = self.get_parameter("codec").get_parameter_value().string_value

        os.makedirs(self.save_dir, exist_ok=True)

        # ================== 云台姿态参数 ==================
        self.declare_parameter("pan_tilt_yaw_deg", 0.0)
        self.declare_parameter("pan_tilt_pitch_deg", 20.0)
        self.declare_parameter("pan_tilt_speed_deg", 20.0)

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
        self.have_frame = False
        self._shutdown_requested = False

        # Recording state
        self.is_recording = False
        self.recorded_frames: List = []  # 存储录制的帧

        # Timers
        self.pan_tilt_timer = self.create_timer(0.5, self.pan_tilt_hold_loop)
        self.ui_timer = self.create_timer(1.0 / 30.0, self.ui_loop)

        # Try to create OpenCV window (safe for headless)
        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        except Exception as e:
            self.get_logger().warn(f"OpenCV GUI not available (headless mode): {e}")

        self.get_logger().info(
            f"VideoRecorder (auto-save) started.\n"
            f"  Save dir        : {self.save_dir}\n"
            f"  FPS             : {self.fps}\n"
            f"  Controls        : S=start, X=stop+save, Q/ESC=quit\n"
            f"  Pan-tilt target : yaw={self.pan_tilt_yaw_deg:.1f}°, pitch={self.pan_tilt_pitch_deg:.1f}°"
        )

    def pan_tilt_hold_loop(self):
        cmd = PanTiltCmdDeg()
        cmd.speed = int(self.pan_tilt_speed_deg)
        cmd.yaw = float(self.pan_tilt_yaw_deg)
        cmd.pitch = float(self.pan_tilt_pitch_deg)
        self.pan_tilt_pub.publish(cmd)

    def image_callback(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        self.latest_bgr = bgr
        self.have_frame = True

        if self.is_recording:
            self.recorded_frames.append(bgr.copy())

    def ui_loop(self):
        if self._shutdown_requested:
            return

        if not self.have_frame or self.latest_bgr is None:
            self._poll_key()
            return

        show = self.latest_bgr.copy()
        status = "RECORDING..." if self.is_recording else "IDLE"
        color = (0, 0, 255) if self.is_recording else (0, 255, 0)
        cv2.putText(
            show,
            f"Status: {status} | S=start | X=stop+save | Q=quit",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
            cv2.LINE_AA,
        )
        if self.is_recording:
            cv2.putText(
                show,
                f"Frames: {len(self.recorded_frames)}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 165, 255),
                1,
                cv2.LINE_AA,
            )

        try:
            cv2.imshow(self.window_name, show)
        except Exception:
            pass  # headless, ignore

        self._poll_key()

    def _poll_key(self):
        try:
            key = cv2.waitKey(1) & 0xFF
        except Exception:
            return

        if key in (ord("q"), ord("Q"), 27):
            self.get_logger().info("Quit requested.")
            self._shutdown_requested = True
            self._safe_close()
            rclpy.shutdown()
        elif key in (ord("s"), ord("S")):
            if not self.is_recording:
                self._start_recording()
        elif key in (ord("x"), ord("X")):
            if self.is_recording:
                self._stop_and_save()

    def _start_recording(self):
        self.recorded_frames.clear()
        self.is_recording = True
        self.get_logger().info("Started recording.")

    def _stop_and_save(self):
        self.is_recording = False
        if not self.recorded_frames:
            self.get_logger().warn("No frames recorded. Nothing to save.")
            return

        # 自动生成时间戳文件名，类似 image_recorder
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{timestamp}.mp4"
        full_path = os.path.join(self.save_dir, filename)

        # 获取帧尺寸
        h, w = self.recorded_frames[0].shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*self.codec)
        out = cv2.VideoWriter(full_path, fourcc, self.fps, (w, h))

        if not out.isOpened():
            self.get_logger().error(f"Failed to open VideoWriter for {full_path}")
            return

        for frame in self.recorded_frames:
            out.write(frame)
        out.release()

        self.get_logger().info(f"Saved video: {full_path}")
        self.recorded_frames.clear()

    def _safe_close(self):
        if self.is_recording:
            self.get_logger().info("Stopping active recording...")
            self._stop_and_save()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node._safe_close()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()