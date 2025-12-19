#!/usr/bin/env python3
# coding: utf-8

from define import *

import time
import numpy as np
import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# ✅ 改成你实际的消息所在包名
# 例如：from px100_msgs.msg import PickPlaceEvent
from your_pkg_name.msg import PickPlaceEvent


class PickPlace(Node):
    def __init__(self):
        super().__init__("Pickup")

        # =========================
        # Event comm (PickPlaceEvent.msg: int32 status)
        # status:
        # 0 idle
        # 1 run pickup_cb until WAIT, then node set status->2
        # 2 stay in WAIT until external sets status->3
        # 3 go PLACE then QUIT
        # 4 done, stay in QUIT and do nothing
        # =========================
        self.status = 0
        self.wait_reported = False
        self.quit_reported = False

        self.event_pub = self.create_publisher(PickPlaceEvent, "/pick_place/event", 10)
        self.event_sub = self.create_subscription(PickPlaceEvent, "/pick_place/event", self.event_cb, 10)

        # =========================
        # Original init
        # =========================
        self.marker2camera_Matrix = np.eye(4)
        self.camera2base_Matrix = np.eye(4)
        self.marker2base_Matrix = np.eye(4)
        self.have_marker = False

        self.camera_matrix = np.array([
            [909.6, 0.0, 650.7],
            [0.0, 909.8, 349.6],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros(5)

        # Marker 边长（真实尺寸，单位 m）
        self.marker_size = 0.0355

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.move_cmd = Twist()

        self.rot_matrix1 = np.array([[0, 0, 1, 0.00],
                                     [0, 1, 0, 0.00],
                                     [-1, 0, 0, 0],
                                     [0, 0, 0, 1]])

        self.rot_matrix2 = np.array([[0, 0, 1, 0],
                                     [-1, 0, 0, 0],
                                     [0, -1, 0, 0],
                                     [0, 0, 0, 1]])

        self.aruco_update = True

        self.fb_sub = self.create_subscription(JointState, "/joint_states", self.js_cb, 10)

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_cb,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(JointSingleCommand, "/px100/commands/joint_single", 10)
        self.group_pub = self.create_publisher(JointGroupCommand, "/px100/commands/joint_group", 10)

        # ✅ timer 仍然保留，但由 status gate 控制：status=0/4 时 pickup_cb 什么也不做
        self.pub_timer = self.create_timer(0.1, self.pickup_cb)

        self.pantil_pub = self.create_publisher(PanTiltCmdDeg, "/pan_tilt_cmd_deg", 10)

        self.ar_pos = None
        self.ar_quat = None
        self.pantil_deg_cmd = PanTiltCmdDeg()
        self.arm_command = JointSingleCommand()
        self.arm_group_command = JointGroupCommand()

        self.cnt = 0
        self.thred = 0.1
        self.joint_pos = []
        self.moving_time = 2.0
        self.num_joints = 4
        self.joint_lower_limits = [-1.5, -0.4, -1.1, -1.4]
        self.joint_upper_limits = [1.5, 0.9, 0.8, 1.8]
        self.initial_guesses = [[0.0] * self.num_joints] * 3
        self.initial_guesses[1][0] = np.deg2rad(-30)
        self.initial_guesses[2][0] = np.deg2rad(30)
        self.robot_des: mrd.ModernRoboticsDescription = getattr(mrd, 'px100')

        self.machine_state = "INIT"

        self.gripper_pressure: float = 0.5
        self.gripper_pressure_lower_limit: int = 150
        self.gripper_pressure_upper_limit: int = 350
        self.gripper_value = self.gripper_pressure_lower_limit + (
            self.gripper_pressure * (self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit)
        )

        # ===== SEARCH params/state =====
        self.search_yaw = 0.0
        self.search_pitch = 30.0
        self.search_yaw_dir = 1.0
        self.search_pitch_dir = 1.0
        self.search_yaw_min = -60.0
        self.search_yaw_max = 60.0
        self.search_pitch_min = 18.0
        self.search_pitch_max = 40.0
        self.search_yaw_step = 6.0
        self.search_pitch_step = 3.0
        self.marker_lost_cnt = 0

        # ===== AIM params =====
        self.aim_y_tol = 0.02
        self.aim_y_target = -0.03
        self.aim_x_target = 0.28
        self.aim_x_tol = 0.02
        self.aim_k_ang = 2.0
        self.aim_k_lin = 0.8
        self.aim_max_w = 0.8
        self.aim_max_v = 0.25

        self.marker_offset = np.array([-0.05, -0.04, 0.01])

    # =========================
    # Event callback
    # =========================
    def event_cb(self, msg: PickPlaceEvent):
        # status==4 后锁死：永远不再响应
        if self.status == 4:
            return

        new_status = int(msg.status)

        # 只响应 0/1/3（2/4 是本节点自己发的“回执”）
        if new_status == 0:
            self.status = 0
            return

        if new_status == 1:
            # 启动：重置流程
            self.status = 1
            self.wait_reported = False
            self.quit_reported = False
            self.machine_state = "INIT"
            self.aruco_update = True
            self.have_marker = False
            self.marker2base_Matrix = np.eye(4)
            self.marker_lost_cnt = 0
            self.get_logger().info("[EVENT] status=1 -> start running")
            return

        if new_status == 3:
            self.status = 3
            self.get_logger().info("[EVENT] status=3 -> allow PLACE")
            return

    # =========================
    # Vision callback
    # =========================
    def image_cb(self, msg):
        if not self.aruco_update:
            return

        now = rclpy.time.Time()
        try:
            trans = self.tf_buffer.lookup_transform("px100/base_link", "camera_link", now)
            position = trans.transform.translation
            orientation = trans.transform.rotation
            position = np.array([position.x, position.y, position.z])
            orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
            self.camera2base_Matrix = self.quat2matrix(orientation, position)
        except:
            pass

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is None or len(ids) == 0:
            self.have_marker = False
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )

        rvec = rvecs[0]
        tvec = tvecs[0]
        rmat, _ = cv2.Rodrigues(rvec)
        pos = tvec.flatten()

        T = np.eye(4)
        T[:3, :3] = rmat
        T[:3, 3] = pos

        self.marker2camera_Matrix = T
        self.have_marker = True

        self.marker2base_Matrix = np.matmul(np.matmul(self.camera2base_Matrix, self.rot_matrix2), self.marker2camera_Matrix)
        self.marker2base_Matrix = np.matmul(self.marker2base_Matrix, self.rot_matrix1)

    def quat2matrix(self, quat, pos):
        position = pos[:, np.newaxis]
        share_vector = np.array([0, 0, 0, 1], dtype=float)[np.newaxis, :]
        r = R.from_quat(quat)
        rotation_matrix = r.as_matrix()
        m34 = np.concatenate((rotation_matrix, position), axis=1)
        matrix = np.concatenate((m34, share_vector), axis=0)
        return matrix

    def js_cb(self, msg):
        if len(msg.name) == 7:
            self.joint_pos.clear()
            for i in range(7):
                self.joint_pos.append(msg.position[i])

    # =========================
    # Main state machine tick
    # =========================
    def pickup_cb(self):
        # ✅ status=0 或 status=4：节点什么也不做
        if self.status in (0, 4):
            return

        # ✅ status=2：只允许停在 WAIT，不推进其他状态
        if self.status == 2 and self.machine_state != "WAIT":
            return

        if len(self.joint_pos) != 7:
            return

        print(self.machine_state)
        match self.machine_state:
            case "INIT":
                self.pantil_deg_cmd.pitch = 30.0
                self.pantil_deg_cmd.yaw = 0.0
                self.pantil_deg_cmd.speed = 10
                self.pantil_pub.publish(self.pantil_deg_cmd)

                self.release()
                self.aruco_update = True

                if self.go_sleep_pos() == True and self.release():
                    if np.array_equal(self.marker2base_Matrix, np.eye(4)):
                        self.machine_state = "SEARCH"
                    else:
                        self.machine_state = "AIM"

            case "SEARCH":
                self.aruco_update = True

                self.move_cmd.linear.x = 0.0
                self.move_cmd.linear.y = 0.0
                self.move_cmd.angular.z = 0.0
                self.vel_pub.publish(self.move_cmd)

                if getattr(self, "have_marker", False) and (not np.array_equal(self.marker2base_Matrix, np.eye(4))):
                    self.marker_lost_cnt = 0
                    self.machine_state = "AIM"
                    return

                self.search_yaw += self.search_yaw_dir * self.search_yaw_step
                hit_end = False

                if self.search_yaw > self.search_yaw_max:
                    self.search_yaw = self.search_yaw_max
                    self.search_yaw_dir = -1.0
                    hit_end = True
                elif self.search_yaw < self.search_yaw_min:
                    self.search_yaw = self.search_yaw_min
                    self.search_yaw_dir = 1.0
                    hit_end = True

                if hit_end:
                    self.search_pitch += self.search_pitch_dir * self.search_pitch_step
                    if self.search_pitch > self.search_pitch_max:
                        self.search_pitch = self.search_pitch_max
                        self.search_pitch_dir = -1.0
                    elif self.search_pitch < self.search_pitch_min:
                        self.search_pitch = self.search_pitch_min
                        self.search_pitch_dir = 1.0

                self.pantil_deg_cmd.pitch = float(self.search_pitch)
                self.pantil_deg_cmd.yaw = float(self.search_yaw)
                self.pantil_deg_cmd.speed = 12
                self.pantil_pub.publish(self.pantil_deg_cmd)

            case "AIM":
                self.aruco_update = True

                if (not getattr(self, "have_marker", False)) or np.array_equal(self.marker2base_Matrix, np.eye(4)):
                    self.marker_lost_cnt += 1
                    self.move_cmd.linear.x = 0.0
                    self.move_cmd.linear.y = 0.0
                    self.move_cmd.angular.z = 0.0
                    self.vel_pub.publish(self.move_cmd)
                    if self.marker_lost_cnt >= 8:
                        self.machine_state = "SEARCH"
                    return
                else:
                    self.marker_lost_cnt = 0

                x = float(self.marker2base_Matrix[0, 3])
                y = float(self.marker2base_Matrix[1, 3])

                angle = float(np.arctan2(y - self.aim_y_target, max(1e-6, x)))
                if abs(y - self.aim_y_target) > self.aim_y_tol:
                    w = self.aim_k_ang * angle
                    w = float(np.clip(w, -self.aim_max_w, self.aim_max_w))
                    self.move_cmd.linear.x = 0.0
                    self.move_cmd.linear.y = 0.0
                    self.move_cmd.angular.z = w
                    self.vel_pub.publish(self.move_cmd)
                    return

                dx = x - self.aim_x_target
                if abs(dx) > self.aim_x_tol:
                    v = self.aim_k_lin * dx
                    v = float(np.clip(v, -self.aim_max_v, self.aim_max_v))
                    self.move_cmd.linear.x = v
                    self.move_cmd.linear.y = 0.0
                    self.move_cmd.angular.z = 0.0
                    self.vel_pub.publish(self.move_cmd)
                    return

                self.move_cmd.linear.x = 0.0
                self.move_cmd.linear.y = 0.0
                self.move_cmd.angular.z = 0.0
                self.vel_pub.publish(self.move_cmd)
                self.machine_state = "NEXT0"
                return

            case "NEXT0":
                if self.set_group_pos([-1.0, 0.0, 0.8, -1.3]) == True:
                    self.machine_state = "NEXT1"
                    self.marker2base_Matrix[:3, 3] += self.marker_offset
                    self.aruco_update = False

            case "NEXT1":
                angle = np.arctan2(self.marker2base_Matrix[1, 3], self.marker2base_Matrix[0, 3])
                if self.set_group_pos([angle, -0.3, 0.9, -1.3]) == True:
                    self.machine_state = "NEXT2"

            case "NEXT2":
                angle = np.arctan2(self.marker2base_Matrix[1, 3], self.marker2base_Matrix[0, 3])
                guess2 = [angle, -0.1, 0.0, 0.0]
                mlist, _ = self.matrix_control(self.marker2base_Matrix, custom_guess=guess2)

                self.set_group_pos([mlist[0], mlist[1], mlist[2], mlist[3]])
                time.sleep(3.0)

                self.move_cmd.linear.x = 0.2
                self.move_cmd.linear.y = 0.0
                self.vel_pub.publish(self.move_cmd)
                time.sleep(2.0)

                self.move_cmd.linear.x = 0.0
                self.move_cmd.linear.y = 0.0
                self.vel_pub.publish(self.move_cmd)
                time.sleep(1.0)

                self.grasp(0.57)
                self.machine_state = "NEXT3"
                time.sleep(1.0)

            case "NEXT3":
                self.set_group_pos([0.0, -0.2, 0.0, -1.2])
                time.sleep(3.0)
                self.machine_state = "NEXT4"

            case "NEXT4":
                if self.go_sleep_pos() == True:
                    # ✅ 进入 WAIT：准备上报 status=2（只发一次）
                    self.wait_reported = False
                    self.machine_state = "WAIT"

            case "WAIT":
                # ✅ 第一次进入 WAIT：发布 status=2，并把内部 status 置为 2
                if not self.wait_reported:
                    ev = PickPlaceEvent()
                    ev.status = 2
                    self.event_pub.publish(ev)
                    self.status = 2
                    self.wait_reported = True
                    print("[WAIT] published status=2, waiting for external status=3")

                # ✅ 保守停车
                self.move_cmd.linear.x = 0.0
                self.move_cmd.linear.y = 0.0
                self.move_cmd.angular.z = 0.0
                self.vel_pub.publish(self.move_cmd)

                # ✅ WAIT 直到外部把 status 设为 3
                if self.status != 3:
                    return

                # ✅ 收到 3：进入 PLACE
                self.machine_state = "PLACE"
                return

            case "PLACE":
                print("PLACE control start!")
                self.go_home_pos()
                time.sleep(3.0)
                self.release()
                print("PLACE control done!")
                self.machine_state = "QUIT"
                return

            case "QUIT":
                # ✅ 发布 status=4（只发一次），然后永久停在 QUIT，节点啥也不做
                if not self.quit_reported:
                    ev = PickPlaceEvent()
                    ev.status = 4
                    self.event_pub.publish(ev)
                    self.status = 4
                    self.quit_reported = True
                    print("[QUIT] published status=4, staying in QUIT")
                return

    # =========================
    # Arm / gripper utils (原样保留)
    # =========================
    def set_single_pos(self, name, pos, blocking=True):
        self.arm_command.name = name
        self.arm_command.cmd = pos
        self.cmd_pub.publish(self.arm_command)

        thred = self.thred
        if blocking:
            check_pos = None
            cal_name = None
            if len(self.joint_pos) == 7:
                match name:
                    case "waist":
                        check_pos = self.joint_pos[0]; cal_name = 'joint'
                    case "shoulder":
                        check_pos = self.joint_pos[1]; cal_name = 'joint'
                    case "elbow":
                        check_pos = self.joint_pos[2]; cal_name = 'joint'
                    case "wrist_angle":
                        check_pos = self.joint_pos[3]; cal_name = 'joint'
                    case "gripper":
                        check_pos = self.joint_pos[4]; cal_name = 'gripper'
                    case _:
                        print('unvalid name input!')

                match cal_name:
                    case "joint":
                        dis = np.abs(pos - check_pos)
                        if dis < thred:
                            return True
                        else:
                            return False
                    case "gripper":
                        return True

    def set_group_pos(self, pos_list, blocking=True):
        if len(pos_list) != self.num_joints:
            print('unexpect length of list!')
            return False

        self.arm_group_command.name = "arm"
        self.arm_group_command.cmd = pos_list
        w = 0.22
        self.arm_group_command.cmd[0] += w
        self.group_pub.publish(self.arm_group_command)

        thred = self.thred
        if blocking and len(self.joint_pos) == 7:
            check_pos = self.joint_pos
            if (np.abs(pos_list[0] + w - check_pos[0]) < thred and
                np.abs(pos_list[1] - check_pos[1]) < thred and
                np.abs(pos_list[2] - check_pos[2]) < thred and
                np.abs(pos_list[3] - check_pos[3]) < thred):
                return True
            else:
                return False
        return False

    def joint_to_pose(self, joint_state):
        return mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, joint_state)

    def go_home_pos(self):
        return self.set_group_pos([0.0, 0.0, 0.0, 0.0])

    def go_sleep_pos(self):
        return self.set_group_pos([-1.6, -0.35, 0.7, 1.0])

    def matrix_control(self, T_sd, custom_guess=None, execute=True):
        p_d = T_sd[0:3, 3]

        joint_lower = np.array([-1.5, -0.4, -1.1, -1.4])
        joint_upper = np.array([1.5, 0.9, 0.8, -0.8])

        pos_tol = 0.02

        steps = np.array([0.25, 0.2, 0.2, 0.25])
        grids = [np.arange(joint_lower[i], joint_upper[i], steps[i]) for i in range(4)]

        mesh = np.meshgrid(*grids, indexing="ij")
        theta_candidates = np.stack([m.reshape(-1) for m in mesh], axis=1)

        best_err = np.inf
        best_theta = None

        for k in range(theta_candidates.shape[0]):
            theta = theta_candidates[k]
            Tsb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, theta)
            p = Tsb[0:3, 3]
            err = np.linalg.norm(p - p_d)
            if err < best_err:
                best_err = err
                best_theta = theta.copy()

        if best_theta is None:
            return self.initial_guesses[0], False

        refine_range = 0.15
        refine_step = 0.03

        deltas = np.arange(-refine_range, refine_range, refine_step)
        mesh = np.meshgrid(deltas, deltas, deltas, deltas, indexing="ij")
        delta_candidates = np.stack([m.reshape(-1) for m in mesh], axis=1)

        best_theta_f = best_theta.copy()
        best_err_f = best_err

        for d in delta_candidates:
            theta = best_theta + d
            theta = np.clip(theta, joint_lower, joint_upper)
            Tsb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, theta)
            p = Tsb[0:3, 3]
            err = np.linalg.norm(p - p_d)
            if err < best_err_f:
                best_err_f = err
                best_theta_f = theta.copy()

        success = best_err_f < pos_tol
        return best_theta_f, success

    def gripper_controller(self, effort, delay: float):
        name = 'gripper'
        effort = float(effort)
        if len(self.joint_pos) == 7:
            gripper_state = self.set_single_pos(name, effort)
            time.sleep(delay)
            return gripper_state

    def set_pressure(self, pressure: float) -> None:
        self.gripper_value = self.gripper_pressure_lower_limit + pressure * (
            self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit
        )

    def release(self, delay: float = 1.0):
        return self.gripper_controller(1.5, delay)

    def grasp(self, pressure: float = 0.5, delay: float = 1.0):
        return self.gripper_controller(pressure, delay)

    def _wrap_theta_list(self, theta_list: list[np.ndarray]) -> list[np.ndarray]:
        REV = 2 * np.pi
        theta_list = (theta_list + np.pi) % REV - np.pi
        for x in range(len(theta_list)):
            if round(theta_list[x], 3) < round(self.joint_lower_limits[x], 3):
                theta_list[x] += REV
            elif round(theta_list[x], 3) > round(self.joint_upper_limits[x], 3):
                theta_list[x] -= REV
        return theta_list


def main(args=None):
    rclpy.init(args=args)
    node = PickPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
