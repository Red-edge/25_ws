import time
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointSingleCommand, JointGroupCommand


class ArmBasicTest(Node):
    def __init__(self):
        super().__init__("arm_basic_test")

        # ------------------------
        # ROS interfaces
        # ------------------------
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self.js_cb, 10
        )

        self.single_pub = self.create_publisher(
            JointSingleCommand, "/px100/commands/joint_single", 10
        )

        self.group_pub = self.create_publisher(
            JointGroupCommand, "/px100/commands/joint_group", 10
        )

        self.timer = self.create_timer(0.1, self.run_cb)

        # ------------------------
        # Internal states
        # ------------------------
        self.joint_pos = []
        self.thred = 0.05
        self.step = 0

        self.single_cmd = JointSingleCommand()
        self.group_cmd = JointGroupCommand()

        self.get_logger().info("Arm basic test node started.")

    # ========================
    # Joint state callback
    # ========================
    def js_cb(self, msg):
        if len(msg.name) >= 4:
            self.joint_pos = list(msg.position)

    # ========================
    # Command helpers
    # ========================
    def set_single(self, name, pos):
        cmd = JointSingleCommand()
        cmd.name = name
        cmd.cmd = float(pos)
        self.single_pub.publish(cmd)

    def set_group(self, pos_list):
        cmd = JointGroupCommand()
        cmd.name = "arm"
        cmd.cmd = list(pos_list)
        self.group_pub.publish(cmd)

    def open_gripper(self):
        self.set_single("gripper", 1.5)

    def close_gripper(self):
        self.set_single("gripper", -0.6)

    # ========================
    # Main test FSM
    # ========================
    def run_cb(self):
        if len(self.joint_pos) == 0:
            self.get_logger().info("Waiting for joint_states...")
            return

        match self.step:
            case 0:
                self.get_logger().info("Step 0: Open gripper")
                self.open_gripper()
                time.sleep(1.0)
                self.step += 1

            case 1:
                self.get_logger().info("Step 1: Go home pose")
                self.set_group([0.0, 0.0, 0.0, 0.0])
                time.sleep(2.0)
                self.step += 1

            case 2:
                self.get_logger().info("Step 2: Test waist")
                self.set_single("waist", 0.5)
                time.sleep(1.0)
                self.set_single("waist", -0.5)
                time.sleep(1.0)
                self.set_single("waist", 0.0)
                time.sleep(1.0)
                self.step += 1

            case 3:
                self.get_logger().info("Step 3: Test shoulder")
                self.set_single("shoulder", -0.3)
                time.sleep(1.0)
                self.set_single("shoulder", 0.3)
                time.sleep(1.0)
                self.set_single("shoulder", 0.0)
                time.sleep(1.0)
                self.step += 1

            case 4:
                self.get_logger().info("Step 4: Test elbow")
                self.set_single("elbow", 0.5)
                time.sleep(1.0)
                self.set_single("elbow", -0.5)
                time.sleep(1.0)
                self.set_single("elbow", 0.0)
                time.sleep(1.0)
                self.step += 1

            case 5:
                self.get_logger().info("Step 5: Test wrist")
                self.set_single("wrist_angle", 0.8)
                time.sleep(1.0)
                self.set_single("wrist_angle", -0.8)
                time.sleep(1.0)
                self.set_single("wrist_angle", 0.0)
                time.sleep(1.0)
                self.step += 1

            case 6:
                self.get_logger().info("Step 6: Go sleep pose")
                self.set_group([-1.4, -0.35, 0.7, 1.0])
                time.sleep(2.0)
                self.step += 1

            case 7:
                self.get_logger().info("Step 7: Close gripper")
                self.close_gripper()
                time.sleep(1.0)
                self.step += 1

            case 8:
                self.get_logger().info("Arm basic test finished.")
                self.timer.destroy()
                rclpy.shutdown()


def main():
    rclpy.init()
    node = ArmBasicTest()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
