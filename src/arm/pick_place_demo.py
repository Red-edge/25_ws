from define import *

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2



class PickPlace(Node):
    def __init__(self):
        super().__init__("Pickup")
        self.marker2camera_Matrix = np.eye(4)
        self.camera2base_Matrix = np.eye(4)
        self.marker2base_Matrix = np.eye(4)

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
        # self.marker_sub = self.create_subscription(PoseArray,"/aruco_poses",self.ar_cb, 10)

        # self.marker_sub = self.create_subscription(PoseArray,"/aruco_poses",self.ar_cb, 10)

        # self.bridge = CvBridge()
        # self.image_sub = self.create_subscription(
        #     Image,
        #     "/camera/color/image_raw",   # 确认一下和 ArucoTest 一致
        #     self.image_cb,
        #     10
        # )

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
        self.pub_timer = self.create_timer(0.1, self.pickup_cb)
        self.pantil_pub = self.create_publisher(PanTiltCmdDeg,"/pan_tilt_cmd_deg",10)

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
        self.gripper_value = self.gripper_pressure_lower_limit + (self.gripper_pressure*(self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit))

        # ===== SEARCH params/state =====
        self.search_yaw = 0.0
        self.search_pitch = 30.0
        self.search_yaw_dir = 1.0     # +1 or -1
        self.search_pitch_dir = 1.0   # +1 or -1
        self.search_yaw_min = -60.0
        self.search_yaw_max = 60.0
        self.search_pitch_min = 18.0
        self.search_pitch_max = 40.0
        self.search_yaw_step = 6.0    # deg per tick (0.1s)
        self.search_pitch_step = 3.0  # deg per sweep end
        self.marker_lost_cnt = 0

        # ===== AIM params =====
        self.aim_y_tol = 0.02         # m, y 接近 0
        self.aim_y_target = -0.03
        self.aim_x_target = 0.28      # m, x 接近 0.25
        self.aim_x_tol = 0.02         # m
        self.aim_k_ang = 2.0          # P gain for yaw control (use angle)
        self.aim_k_lin = 0.8          # P gain for x control
        self.aim_max_w = 0.8          # rad/s
        self.aim_max_v = 0.25         # m/s

        self.marker_offset = np.array([-0.05, -0.04, 0.01])

        pass


    def image_cb(self, msg):
        if not self.aruco_update:
            return
        now = rclpy.time.Time()

        try:
            trans = self.tf_buffer.lookup_transform("px100/base_link","camera_link",now)
            position = trans.transform.translation
            orientation = trans.transform.rotation
            position = np.array([position.x, position.y, position.z])
            orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
            self.camera2base_Matrix = self.quat2matrix(orientation, position)
            # self.aruco_update = False
            print("aruco updated")
        except :
            print("pass")
            pass

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 这里可以用一个固定字典，也可以像 ArucoTest 用列表循环
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            aruco_dict,
            parameters=parameters
        )

        if ids is None or len(ids) == 0:
            self.have_marker = False
            print('no marker detected!')
            return

        # 这里示例只用第一个 marker，如有需要可根据 id 选择
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_size,
            self.camera_matrix,
            self.dist_coeffs,
        )

        rvec = rvecs[0]
        tvec = tvecs[0]

        # Rotation matrix
        rmat, _ = cv2.Rodrigues(rvec)
        pos = tvec.flatten()

        # 4x4 T_camera_marker
        T = np.eye(4)
        T[:3, :3] = rmat
        T[:3, 3] = pos

        # 保存为成员变量，供状态机使用
        self.marker2camera_Matrix = T
        self.have_marker = True

        self.marker2base_Matrix = np.matmul(np.matmul(self.camera2base_Matrix, self.rot_matrix2), self.marker2camera_Matrix)
        self.marker2base_Matrix = np.matmul(self.marker2base_Matrix, self.rot_matrix1)
        print('marker2base_Matrix: ', self.marker2base_Matrix)

        # 如果你想看图，也可以画出来（可选）
        # cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        # cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
        #                   rvec, tvec, self.marker_size * 0.5)
        # cv2.imshow("Pickup Aruco", frame)
        # cv2.waitKey(1)


    
    
    def quat2matrix(self,quat,pos):
        position = pos[:,np.newaxis]
        share_vector = np.array([0,0,0,1],dtype = float)[np.newaxis,:]
        r = R.from_quat(quat)
        rotation_matrix = r.as_matrix()
        m34 = np.concatenate((rotation_matrix, position),axis = 1)
        matrix = np.concatenate((m34,share_vector),axis = 0)

        return matrix


    def js_cb(self, msg):
        # print('joint state callback')
        if len(msg.name) == 7:
            self.joint_pos.clear()
            for i in range(7):
                self.joint_pos.append(msg.position[i])
        # print(self.joint_pos)


    # camera2marker matrix
    def ar_cb(self, msg):
        # print('ar_cb')
        if not self.aruco_update:
            return
        # 更新相机与base的转换
        now = rclpy.time.Time()
        try:
            trans = self.tf_buffer.lookup_transform("px100/base_link","camera_link",now)
            position = trans.transform.translation
            orientation = trans.transform.rotation
            position = np.array([position.x, position.y, position.z])
            orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
            self.camera2base_Matrix = self.quat2matrix(orientation, position)
            # self.aruco_update = False
            print("aruco updated")
        except :
            print("pass")
            pass

        # 这里处理接收到的坐标信息
        pose = msg.poses[0]

        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        # 从 Pose 消息中获取方向信息
        orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        matrix = self.quat2matrix(orientation, position)

        self.marker2camera_Matrix = matrix

        self.marker2base_Matrix = np.matmul(np.matmul(self.camera2base_Matrix, self.rot_matrix2), self.marker2camera_Matrix)
        self.marker2base_Matrix = np.matmul(self.marker2base_Matrix, self.rot_matrix1)

        return None


    def pickup_cb(self):
        # print('pickup_cb')
        # print(self.joint_pos)
        if len(self.joint_pos) == 7:
            # # 获取当前时间
            # current_time = datetime.datetime.now()
            # global start_time
            # duration = current_time - start_time
            # print(current_time, duration)
            # # 超时自动跳出
            # if duration > datetime.timedelta(seconds=30.0):
            #     print('Time out! Forced exit!')
            #     print(1/0)

            print(self.machine_state)
            match self.machine_state:
                case "INIT":
                    self.pantil_deg_cmd.pitch = 30.0  # 调整云台俯仰角
                    self.pantil_deg_cmd.yaw = 0.0
                    self.pantil_deg_cmd.speed = 10
                    self.pantil_pub.publish(self.pantil_deg_cmd)
                    self.release()
                    self.aruco_update = True
                    if self.go_sleep_pos() == True and self.release():
                        print('Go sleep pos done!')
                        self.aruco_update = True

                        # print('base2marker_Matrix: ', self.base2marker_Matrix)
                        if np.array_equal(self.marker2base_Matrix, np.eye(4)):
                            print('no marker detected!')
                            self.machine_state = "SEARCH"
                            # time.sleep(3.0)
                        else:
                            print('marker detected!')
                            print('base2marker_Matrix: ', self.marker2base_Matrix)
                            self.machine_state = "AIM"
                            #self.machine_state = "NEXT4"
                            # time.sleep(3.0)
                case "SEARCH":
                    # 进入 SEARCH 时：保证持续更新 aruco
                    self.aruco_update = True

                    # 底盘先停住，避免扫云台时乱跑
                    self.move_cmd.linear.x = 0.0
                    self.move_cmd.linear.y = 0.0
                    self.move_cmd.angular.z = 0.0
                    self.vel_pub.publish(self.move_cmd)

                    # 如果已经看到 marker（你 image_cb 里会 set have_marker=True）
                    if getattr(self, "have_marker", False) and (not np.array_equal(self.marker2base_Matrix, np.eye(4))):
                        print("[SEARCH] marker found -> AIM")
                        self.marker_lost_cnt = 0
                        self.machine_state = "AIM"
                        return

                    # 否则：扫云台（yaw 来回扫，扫到边界就调整 pitch，再反向）
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

                    print(f"[SEARCH] scanning yaw={self.search_yaw:.1f}, pitch={self.search_pitch:.1f}")


                case "AIM":
                    # AIM 时持续更新 marker 位姿（否则你 y/x 不会变化）
                    self.aruco_update = True

                    # 如果突然看不到 marker：回 SEARCH
                    if (not getattr(self, "have_marker", False)) or np.array_equal(self.marker2base_Matrix, np.eye(4)):
                        self.marker_lost_cnt += 1
                        print(f"[AIM] marker lost cnt={self.marker_lost_cnt} -> SEARCH if too long")
                        self.move_cmd.linear.x = 0.0
                        self.move_cmd.linear.y = 0.0
                        self.move_cmd.angular.z = 0.0
                        self.vel_pub.publish(self.move_cmd)
                        if self.marker_lost_cnt >= 8:   # 0.8s 左右
                            self.machine_state = "SEARCH"
                        return
                    else:
                        self.marker_lost_cnt = 0

                    x = float(self.marker2base_Matrix[0, 3])
                    y = float(self.marker2base_Matrix[1, 3])

                    # ---- Step 1: 先转向让 y -> 0（用 angle 更稳）----
                    angle = float(np.arctan2(y-self.aim_y_target, max(1e-6, x)))  # 防止 x=0
                    if abs(y-self.aim_y_target) > self.aim_y_tol:
                        w = self.aim_k_ang * angle
                        w = float(np.clip(w, -self.aim_max_w, self.aim_max_w))

                        self.move_cmd.linear.x = 0.0
                        self.move_cmd.linear.y = 0.0
                        self.move_cmd.angular.z = w
                        self.vel_pub.publish(self.move_cmd)

                        print(f"[AIM] turning: x={x:.3f}, y={y:.3f}, angle={angle:.3f}, w={w:.3f}")
                        return

                    # ---- Step 2: 再前后让 x -> 0.25 ----
                    dx = x - self.aim_x_target
                    if abs(dx) > self.aim_x_tol:
                        v = self.aim_k_lin * dx
                        v = float(np.clip(v, -self.aim_max_v, self.aim_max_v))

                        self.move_cmd.linear.x = v
                        self.move_cmd.linear.y = 0.0
                        self.move_cmd.angular.z = 0.0
                        self.vel_pub.publish(self.move_cmd)

                        print(f"[AIM] translating: x={x:.3f}, y={y:.3f}, dx={dx:.3f}, v={v:.3f}")
                        return

                    # ---- Done: 停车 -> 进入下一步 ----
                    self.move_cmd.linear.x = 0.0
                    self.move_cmd.linear.y = 0.0
                    self.move_cmd.angular.z = 0.0
                    self.vel_pub.publish(self.move_cmd)

                    print(f"[AIM] aligned: x={x:.3f}, y={y:.3f} -> NEXT0")
                    self.machine_state = "NEXT0"
                    return



                case "NEXT0":
                    if self.set_group_pos([-1.0, 0.0, 0.8, -1.3]) == True:
                        print('NEXT0 control done!')
                        # self.machine_state = "NEXT1"
                        self.machine_state = "NEXT1"
                        #self.machine_state = "TEST1"
                        print('aruco: ', self.marker2base_Matrix)
                        # grasp / alignment offset in base frame (m)
                        
                        self.marker2base_Matrix[:3, 3] += self.marker_offset

                        self.aruco_update = False
                
                case "NEXT1":
                    angle = np.arctan2(self.marker2base_Matrix[1, 3], self.marker2base_Matrix[0, 3])
                    if self.set_group_pos([angle, -0.3, 0.9, -1.3]) == True:

                        print('NEXT1 control done!')

                        self.machine_state = "NEXT2"
    
                case "NEXT2":

                    # if len(self.joint_pos) == 7:
                    #     try:
                    #         trans = self.tf_buffer.lookup_transform(
                    #             "base_link",
                    #             "px100/fingers_link",
                    #             rclpy.time.Time()
                    #         )

                    #         p = trans.transform.translation
                    #         q = trans.transform.rotation

                    #         print("=== EE (fingers_link) pose in base ===")
                    #         print("position (m):", [p.x, p.y, p.z])
                    #         print("orientation (quat):", [q.x, q.y, q.z, q.w])
                    #         print("====================================")

                    #     except TransformException as e:
                    #         print("TF lookup failed:", e)
                    # time.sleep(100.0)
                    np.set_printoptions(precision=3)
                    print('base2marker_Matrix: ', self.marker2base_Matrix)
                    angle = np.arctan2(self.marker2base_Matrix[1, 3], self.marker2base_Matrix[0, 3])
                    guess2 = [angle, -0.1, 0.0, 0.0]
                    mlist, mflag = self.matrix_control(self.marker2base_Matrix, custom_guess=guess2)
                    print('mlist2:', mlist)
                    self.fk = self.joint_to_pose(mlist)
                    np.set_printoptions(precision=3)
                    print('fk', self.fk)
                    # error = self.marker2base_Matrix - self.fk
                    np.set_printoptions(precision=3)

                    # if self.set_group_pos([mlist[0], mlist[1], mlist[2], mlist[3]]) and self.release():
                    self.set_group_pos([mlist[0], mlist[1], mlist[2], mlist[3]])
                    time.sleep(3.0)
                    print('NEXT2 control done!')
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
                    self.set_group_pos([0.0, -0.2, 0.0, -1.2]) == True
                    time.sleep(3.0)
                    print('NEXT3 control done!')
                    self.machine_state = "NEXT4"
                        # time.sleep(3.0)
                case "NEXT4":
                    if self.go_sleep_pos() == True:
                        print('Go sleep pos done!')
                        self.machine_state = "WAIT"
                        # self.release()
                        # self.machine_state = "NEXT6"
                        # time.sleep(1.0)
                case "WAIT":

                case "PLACE":
                    print('PLACE control start!')
                    self.go_home_pos()
                    time.sleep(3.0)
                    self.release()
                    print('PLACE control done!')
                    self.machine_state = "QUIT"
                case 'QUIT':                      
                    self.pub_timer.destroy()
                    print(1/0)
                    return
                        
        pass


    def set_single_pos(self, name, pos, blocking=True):
        '''
        ### @param: name: joint name
        ### @param: pos: radian
        ### @param: blocking - whether the arm need to check current position 

        '''
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
                        check_pos = self.joint_pos[0]
                        cal_name = 'joint'
                    case "shoulder":
                        check_pos = self.joint_pos[1]
                        cal_name = 'joint'
                    case "elbow":
                        check_pos = self.joint_pos[2]
                        cal_name = 'joint'
                    case "wrist_angle":
                        check_pos = self.joint_pos[3]
                        cal_name = 'joint'
                    case "gripper":
                        check_pos = self.joint_pos[4]
                        cal_name = 'gripper'
                    case _:
                        print('unvalid name input!')

                match cal_name:
                    case "joint":
                        dis = np.abs(pos-check_pos)
                        if dis < thred:
                            return True
                        else:
                            print('single joint moving...')
                            return False                       
                    case "gripper":
                        return True

        pass


    def set_group_pos(self, pos_list, blocking=True):
        '''
        ### @param: group pos: radian
        ### @param: blocking - whether the arm need to check current position 
        '''
        if len(pos_list) != self.num_joints:
            print('unexpect length of list!')
        else:
            self.arm_group_command.name = "arm"
            self.arm_group_command.cmd = pos_list
            w = 0.22
            self.arm_group_command.cmd[0] += w
            self.group_pub.publish(self.arm_group_command)
            
            # self.move_cmd.linear.x = 0.0
            # self.move_cmd.linear.y = 0.0
            # self.cmd_pub.publish(self.move_cmd)
     
            thred = self.thred
            if blocking:
                if len(self.joint_pos) == 7:
                    check_pos = self.joint_pos
                    # print('current group pos:', check_pos)
                    if np.abs(pos_list[0]+w - check_pos[0]) < thred and np.abs(pos_list[1] - check_pos[1]) < thred and np.abs(pos_list[2] - check_pos[2]) < thred and np.abs(pos_list[3] - check_pos[3]) < thred:
                    # if np.abs(pos_list[1] - check_pos[1]) < thred and np.abs(pos_list[2] - check_pos[2]) < thred and np.abs(pos_list[3] - check_pos[3]) < thred:
                        return True
                    else:
                        if np.abs(pos_list[0] - check_pos[0]) >= thred:
                            pass
                        if np.abs(pos_list[1] - check_pos[1]) >= thred:
                            pass
                        if np.abs(pos_list[2] - check_pos[2]) >= thred:
                            pass
                        if np.abs(pos_list[3] - check_pos[3]) >= thred:
                            pass
                            return False            
            pass


    def joint_to_pose(self, joint_state):
        return mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, joint_state)


    def go_home_pos(self):
        state = self.set_group_pos([0.0, 0.0, 0.0, 0.0])
        return state


    def go_sleep_pos(self):
        state = self.set_group_pos([-1.6, -0.35, 0.7, 1.0])
        return state

    def matrix_control(self, T_sd, custom_guess=None, execute=True):
        """
        Position-only IK using vectorized coarse-to-fine grid search.
        Same input/output as before.
        """

        p_d = T_sd[0:3, 3]

        joint_lower = np.array([-1.5, -0.4, -1.1, -1.4])
        joint_upper = np.array([ 1.5,  0.9,  0.8, -0.8])

        pos_tol = 0.02  # 2 cm

        # ========= coarse grid =========
        steps = np.array([0.25, 0.2, 0.2, 0.25])

        grids = [
            np.arange(joint_lower[i], joint_upper[i], steps[i])
            for i in range(4)
        ]

        # 生成所有组合 (N,4)
        mesh = np.meshgrid(*grids, indexing="ij")
        theta_candidates = np.stack(
            [m.reshape(-1) for m in mesh], axis=1
        )  # shape: (N,4)

        N = theta_candidates.shape[0]

        best_err = np.inf
        best_theta = None

        # 单层 for：只剩 FK 没法 vectorize
        for k in range(N):
            theta = theta_candidates[k]

            Tsb = mr.FKinSpace(
                self.robot_des.M,
                self.robot_des.Slist,
                theta
            )
            p = Tsb[0:3, 3]
            err = np.linalg.norm(p - p_d)

            if err < best_err:
                best_err = err
                best_theta = theta.copy()
        print("粗搜索完成")
        print('best_theta: ', best_theta)
        print('best_err: ', best_err)
        # time.sleep(1.0)

        # 粗搜索失败
        if best_theta is None:
            return self.initial_guesses[0], False

        # ========= refine =========
        refine_range = 0.15
        refine_step = 0.03

        deltas = np.arange(-refine_range, refine_range, refine_step)
        mesh = np.meshgrid(deltas, deltas, deltas, deltas, indexing="ij")
        delta_candidates = np.stack(
            [m.reshape(-1) for m in mesh], axis=1
        )

        best_theta_f = best_theta.copy()
        best_err_f = best_err

        for d in delta_candidates:
            theta = best_theta + d
            theta = np.clip(theta, joint_lower, joint_upper)

            Tsb = mr.FKinSpace(
                self.robot_des.M,
                self.robot_des.Slist,
                theta
            )
            p = Tsb[0:3, 3]
            err = np.linalg.norm(p - p_d)

            if err < best_err_f:
                best_err_f = err
                best_theta_f = theta.copy()

        success = best_err_f < pos_tol
        print('success: ', success)
        print("细搜索完成")
        print('best_err_f: ', best_err_f)
        return best_theta_f, success



    def waist_control(self, pos):
        """
        lower limit = -1.5
        upper limit = 1.5
        """
        pos = float(pos)

        if pos < -1.5:
            pos = -1.5
        elif pos > 1.5:
            pos = 1.5

        state = self.set_single_pos('waist', pos)
        return state


    def shoulder_control(self, pos):
        """
        lower limit = -0.4
        upper limit = 0.9
        """
        pos = float(pos)

        if pos < -0.4:
            pos = -0.4
        elif pos > 0.9:
            pos = 0.9

        state = self.set_single_pos('shoulder', pos)
        return state
    
    def elbow_control(self, pos):
        '''
        lower limit = -1.1
        upper limit = 0.8
        '''
        pos = float(pos)

        if pos < -1.1:
            pos = -1.1
        elif pos > 0.8:
            pos = 0.8

        state = self.set_single_pos('elbow', pos)
        return state
    

    def wrist_control(self, pos):
        '''
        lower limit = -1.4
        upper limit = 1.8
        '''
        pos = float(pos)

        if pos < -1.4:
            pos = -1.4
        elif pos > 1.8:
            pos = 1.8

        state = self.set_single_pos('wrist_angle', pos)
        return state


    def gripper_controller(self, effort, delay: float):
        '''
        effort: release = 1.5
        effort: grasp = -0.6
        '''
        name = 'gripper'
        effort = float(effort)

        if len(self.joint_pos) == 7:
            gripper_state = self.set_single_pos(name, effort)
            time.sleep(delay)
            return gripper_state


    def set_pressure(self, pressure: float) -> None:
        """
        Set the amount of pressure that the gripper should use when grasping an object.
        :param pressure: a scaling factor from 0 to 1 where the pressure increases as
            the factor increases
        """
        self.gripper_value = self.gripper_pressure_lower_limit + pressure * (
            self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit
        )

    def release(self, delay: float = 1.0) -> None:
        """
        Open the gripper (when in 'pwm' control mode).
        :param delay: (optional) number of seconds to delay before returning control to the user
        """
        state = self.gripper_controller(1.5, delay)
        return state

    def grasp(self, pressure: float = 0.5, delay: float = 1.0) -> None:
        """
        Close the gripper (when in 'pwm' control mode).
        :param delay: (optional) number of seconds to delay before returning control to the user
        """
        state = self.gripper_controller(pressure, delay)
        return state


    def _wrap_theta_list(self, theta_list: list[np.ndarray]) -> list[np.ndarray]:
        """
        Wrap an array of joint commands to [-pi, pi) and between the joint limits.

        :param theta_list: array of floats to wrap
        :return: array of floats wrapped between [-pi, pi)
        """
        REV = 2 * np.pi
        theta_list = (theta_list + np.pi) % REV - np.pi
        for x in range(len(theta_list)):
            if round(theta_list[x], 3) < round(self.joint_lower_limits[x], 3):
                theta_list[x] += REV
            elif round(theta_list[x], 3) > round(self.joint_upper_limits[x], 3):
                theta_list[x] -= REV
        return theta_list