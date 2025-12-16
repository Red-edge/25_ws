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

from pan_tilt_msgs.msg import PanTiltCmdDeg


class VideoRecorder(Node):
    def __init__(self):
        super().__init__("video_recorder")

        self.bridge = CvBridge()

        # ================== 保存参数 ==================
        self.declare_parameter("save_dir", "/home/tony/25_ws/traffic/videos")
        self.declare_parameter("window_name", "D435i Video Recorder (S=start/stop, Q/ESC=quit)")
        # 视频帧率 有点问题 original 15
        self.declare_parameter("fps", 30.0)  # 视频帧率
        self.declare_parameter("codec", "mp4v")  # mp4v for .mp4

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
        self.paused = False
        self._shutdown_requested = False

        # Video recording state
        self.is_recording = False
        self.video_writer = None
        self.current_filename = ""

        # Timers
        self.pan_tilt_timer = self.create_timer(0.5, self.pan_tilt_hold_loop)
        self.ui_timer = self.create_timer(1.0 / 30.0, self.ui_loop)

        # OpenCV window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.get_logger().info(
            f"VideoRecorder started.\n"
            f"  Save dir        : {self.save_dir}\n"
            f"  Window          : {self.window_name}\n"
            f"  Controls        : S=start/stop record, Q/ESC=quit, Space=pause\n"
            f"  Pan-tilt target : yaw={self.pan_tilt_yaw_deg:.1f}°, "
            f"pitch={self.pan_tilt_pitch_deg:.1f}° @ {self.pan_tilt_speed_deg:.1f} deg/s"
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

        # If recording, write frame
        if self.is_recording and self.video_writer is not None:
            self.video_writer.write(bgr)

    def ui_loop(self):
        if self._shutdown_requested:
            return

        if not self.have_frame or self.latest_bgr is None:
            key = cv2.waitKey(1) & 0xFF
            self._handle_key(key)
            return

        show = self.latest_bgr.copy()
        status = "RECORDING" if self.is_recording else "IDLE"
        color = (0, 0, 255) if self.is_recording else (0, 255, 0)
        cv2.putText(
            show,
            f"Status: {status} | S=start/stop | Q/ESC=quit | Space=pause",
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
                f"File: {os.path.basename(self.current_filename)}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )

        if not self.paused:
            cv2.imshow(self.window_name, show)

        key = cv2.waitKey(1) & 0xFF
        self._handle_key(key)

    def _handle_key(self, key: int):
        if key in (ord("q"), ord("Q"), 27):  # q or ESC
            self._stop_recording_if_needed()
            self.get_logger().info("Quit key pressed. Shutting down...")
            self._shutdown_requested = True
            self._safe_close()
            rclpy.shutdown()
            return

        if key == ord(" "):  # space: pause UI
            self.paused = not self.paused
            self.get_logger().info(f"UI Paused={self.paused}")
            return

        if key in (ord("s"), ord("S")):  # s: start/stop recording
            if self.is_recording:
                self._stop_recording()
            else:
                self._start_recording()

    def _start_recording(self):
        if not self.have_frame or self.latest_bgr is None:
            self.get_logger().warn("No frame available. Cannot start recording.")
            return

        # Prompt user for filename in terminal
        print("\n" + "="*50)
        user_input = input("Enter video filename (without path, e.g., 'test1.mp4' or 'my_video'): ").strip()
        print("="*50 + "\n")

        if not user_input:
            self.get_logger().warn("Filename empty. Recording canceled.")
            return

        # Auto-add .mp4 if missing
        if not user_input.lower().endswith(".mp4"):
            user_input += ".mp4"

        full_path = os.path.join(self.save_dir, user_input)

        # Check if file exists
        if os.path.exists(full_path):
            overwrite = input(f"File {full_path} exists. Overwrite? (y/N): ").strip().lower()
            if overwrite != 'y':
                self.get_logger().info("Recording canceled.")
                return

        # Initialize VideoWriter
        height, width = self.latest_bgr.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*self.codec)
        self.video_writer = cv2.VideoWriter(full_path, fourcc, self.fps, (width, height))

        if not self.video_writer.isOpened():
            self.get_logger().error(f"Failed to open VideoWriter for {full_path}")
            self.video_writer = None
            return

        self.is_recording = True
        self.current_filename = full_path
        self.get_logger().info(f"Started recording: {full_path}")

    def _stop_recording(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
        self.is_recording = False
        self.get_logger().info(f"Stopped recording. Saved to: {self.current_filename}")

    def _stop_recording_if_needed(self):
        if self.is_recording:
            self._stop_recording()

    def _safe_close(self):
        self._stop_recording_if_needed()
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
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("VideoRecorder interrupted.")
    finally:
        node._safe_close()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()