# px100_pick/main.py

import rclpy
from pick_place_demo import *


def main(args=None):
    rclpy.init(args=args)

    node = PickPlace()
    node.get_logger().info("Pickup demo started.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
