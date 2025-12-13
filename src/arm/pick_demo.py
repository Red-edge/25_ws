#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from define import *

import numpy as np
import time


class ArucoPickDemo(Node):
    def __init__(self):
        super().__init__("aruco_pick_demo")

        # ---------------- TF & ArUco ----------------
        self.marker2camera_Matrix = np.eye(4)
        self.camera2base_Matrix = np.eye(4)
        self.marker2base_Matrix = np.eye(4)

        self.rot_matrix1 = np.array([
            [0, 0, 1, 0.02],
            [0, 1, 0, 0.032],
            [-1, 0, 0, 0],
            [0, 0, 0, 1],
        ])

        self.rot_matrix2 = np.array([
            [0, 0, 1, 0],
            [-1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1],
        ])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_sub = self.create_subscription(
            PoseArray, "/aruco_poses", self.aruco_cb, 10
        )

        # ---------------- Arm ----------------
        self.fb_sub = self.create_subscription(
            JointState, "/joint_states", self.js_cb, 10
        )

        self.cmd_pub = self.create_publisher(
            JointSingleCommand, "/px100/commands/joint_single", 10
        )
        self.group_pub = self.create_publisher(
            JointGroupCommand, "/px100/commands/joint_group", 10
        )

        self.pantil_pub = self.create_publisher(
            PanTiltCmdDeg, "/pan_tilt_cmd_deg", 10
        )

        self.arm_cmd = JointSingleCommand()
        self.arm_group_cmd = JointGroupCommand()
        self.pt_cmd = PanTiltCmdDeg()

        self.joint_pos = []

        # ---------------- Robot model ----------------
        self.robot_des: mrd.ModernRoboticsDescription = getattr(mrd, "px100")

        self.initial_guesses = [[0.0, 0.0, 0.0, 0.0]]

        # ---------------- FSM ----------------
        self.state = "INIT"
        self.timer = self.create_timer(0.1, self.main_loop)

        self.get_logger().info("Aruco Pick Demo started.")

    # ======================================================
    # Callbacks
    # ======================================================
    def js_cb(self, msg):
        if len(msg.name) == 7:
            self.joint_pos = list(msg.position)

    def aruco_cb(self, msg):
        if len(msg.poses) == 0:
            return

        # camera -> base
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                "px100/base_link", "camera_link", now
            )
            t = trans.transform.translation
            q = trans.transform.rotation
            pos = np.array([t.x, t.y, t.z])
            quat = np.array([q.x, q.y, q.z, q.w])
            self.camera2base_Matrix = self.quat2matrix(quat, pos)
        except:
            return

        # marker -> camera
        pose = msg.poses[0]
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        quat = np.array(
            [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w]
        )
        self.marker2camera_Matrix = self.quat2matrix(quat, pos)

        # marker -> base（完全沿用你的写法）
        self.marker2base_Matrix = (
            self.camera2base_Matrix
            @ self.rot_matrix2
            @ self.marker2camera_Matrix
            @ self.rot_matrix1
        )

    # ======================================================
    # Utils
    # ======================================================
    def quat2matrix(self, quat, pos):
        r = R.from_quat(quat).as_matrix()
        T = np.eye(4)
        T[:3, :3] = r
        T[:3, 3] = pos
        return T

    def set_group_pos(self, joints):
        self.arm_group_cmd.name = "arm"
        self.arm_group_cmd.cmd = joints
        self.group_pub.publish(self.arm_group_cmd)

    def release(self):
        self.arm_cmd.name = "gripper"
        self.arm_cmd.cmd = 1.5
        self.cmd_pub.publish(self.arm_cmd)

    def grasp(self):
        self.arm_cmd.name = "gripper"
        self.arm_cmd.cmd = -0.6
        self.cmd_pub.publish(self.arm_cmd)

    # ======================================================
    # FSM
    # ======================================================
    def main_loop(self):
        if len(self.joint_pos) != 7:
            return

        match self.state:

            # ---------------- STEP 1 ----------------
            case "INIT":
                self.get_logger().info("STEP 1: Look down 20 deg")
                self.pt_cmd.pitch = 20.0
                self.pt_cmd.yaw = 0.0
                self.pt_cmd.speed = 10
                self.pantil_pub.publish(self.pt_cmd)

                self.release()
                self.state = "WAIT_ARUCO"
                time.sleep(1.0)

            # ---------------- STEP 2 ----------------
            case "WAIT_ARUCO":
                if np.allclose(self.marker2base_Matrix, np.eye(4)):
                    return

                self.get_logger().info("Aruco detected.")
                self.state = "ARM_ALIGN"

            case "ARM_ALIGN":
                angle = np.arctan2(
                    self.marker2base_Matrix[1, 3],
                    self.marker2base_Matrix[0, 3],
                )
                self.set_group_pos([angle, -0.3, 0.8, -1.3])
                self.state = "ARM_REACH"
                time.sleep(1.0)

            case "ARM_REACH":
                guess = [
                    np.arctan2(
                        self.marker2base_Matrix[1, 3],
                        self.marker2base_Matrix[0, 3],
                    ),
                    -0.1,
                    0.0,
                    0.0,
                ]

                thetas, success = mr.IKinSpace(
                    self.robot_des.Slist,
                    self.robot_des.M,
                    self.marker2base_Matrix,
                    guess,
                    5.0,
                    0.02,
                )

                if success:
                    self.set_group_pos(thetas[:4])
                    self.state = "GRASP"
                    time.sleep(1.0)

            case "GRASP":
                self.get_logger().info("Grasping...")
                self.grasp()
                time.sleep(1.0)
                self.state = "DONE"

            case "DONE":
                self.get_logger().info("Pick demo done.")
                self.timer.destroy()
