import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R


class ArucoTest(Node):

    def __init__(self):
        super().__init__("aruco_test")

        # ====================
        # Camera parameters
        # ====================
        # !!! 替换成你真实标定得到的内参 !!!
        self.camera_matrix = np.array([
            [909.6, 0.0, 650.7],
            [0.0, 909.8, 349.6],
            [0.0, 0.0, 1.0]
        ])

        self.dist_coeffs = np.zeros(5)

        # Marker 边长（真实尺寸，单位 m）
        self.marker_size = 0.04

        # ====================
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_cb,
            10
        )

        self.get_logger().info("OpenCV ArUco TEST node started.")

    # ====================
    def image_cb(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        DICT_LIST = [
            cv2.aruco.DICT_4X4_50,
            cv2.aruco.DICT_4X4_100,
            cv2.aruco.DICT_5X5_100,
            cv2.aruco.DICT_5X5_250,
            cv2.aruco.DICT_6X6_100,
            cv2.aruco.DICT_6X6_250,
        ]

        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        corners = None
        ids = None

        for d in DICT_LIST:
            aruco_dict = cv2.aruco.getPredefinedDictionary(d)
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                aruco_dict,
                parameters=parameters
            )

            if ids is not None:
                self.get_logger().info(f"Detected marker using dict = {d}")
                break

        if ids is None:
            cv2.imshow("Aruco Test", frame)
            cv2.waitKey(1)
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_size,
            self.camera_matrix,
            self.dist_coeffs,
        )

        for i, marker_id in enumerate(ids.flatten()):

            rvec = rvecs[i]
            tvec = tvecs[i]

            # -------- Rotation matrix --------
            rmat, _ = cv2.Rodrigues(rvec)

            # -------- Quaternion --------
            quat = R.from_matrix(rmat).as_quat()

            # -------- Euler ZYX (yaw, pitch, roll) --------
            euler = R.from_matrix(rmat).as_euler("zyx", degrees=True)

            pos = tvec.flatten()

            # -------- 4x4 Homogeneous matrix --------
            T = np.eye(4)
            T[:3, :3] = rmat
            T[:3, 3] = pos

            # -------- Print nicely --------
            self.get_logger().info("========================")
            self.get_logger().info(f"Marker ID: {marker_id}")

            self.get_logger().info(
                f"Position (camera frame) [m]: "
                f"x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}"
            )

            self.get_logger().info(
                f"Quaternion (x,y,z,w): "
                f"{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}"
            )

            self.get_logger().info(
                f"Euler ZYX (deg): "
                f"yaw={euler[0]:.1f}, pitch={euler[1]:.1f}, roll={euler[2]:.1f}"
            )

            self.get_logger().info("T_camera_marker =")
            self.get_logger().info("\n" + np.array2string(
                T,
                precision=3,
                suppress_small=True
            ))

            # -------- Draw --------
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            cv2.drawFrameAxes(
                frame,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                self.marker_size * 0.5
            )



        cv2.imshow("Aruco Test", frame)
        cv2.waitKey(1)



# =====================
def main():
    rclpy.init()
    node = ArucoTest()
    rclpy.spin(node)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
