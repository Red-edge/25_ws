#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from tb4_autonav_interfaces.msg import PickPlaceEvent


class PickPlaceCommTester(Node):
    def __init__(self):
        super().__init__('pick_place_comm_tester')

        # Publisher to send commands (like Nav node would)
        self.cmd_pub = self.create_publisher(PickPlaceEvent, '/PickPlaceEvent', 10)

        # Subscriber to receive feedback (optional, in case real nodes exist)
        self.status_sub = self.create_subscription(
            PickPlaceEvent,
            '/PickPlaceEvent',
            self.status_callback,
            10
        )

        self.current_state = "IDLE"  # IDLE -> SENT_PICK -> PICK_DONE -> SENT_PLACE -> DONE

        self.get_logger().info("PickPlaceCommTester started. Simulating navigation arrival at pick point...")

        # Start the test sequence after a short delay
        self.timer = self.create_timer(1.0, self.test_sequence_step)

        # Hold references to one-shot timers to avoid premature destruction
        self.pick_complete_timer = None
        self.place_complete_timer = None

    def test_sequence_step(self):
        if self.current_state == "IDLE":
            self.send_pick_command()
            self.current_state = "SENT_PICK"
            # Simulate pick completion after 3 seconds
            self.pick_complete_timer = self.create_timer(3.0, self.simulate_pick_complete, callback_group=None, reset_counter=True)
        elif self.current_state == "PICK_DONE":
            self.send_place_command()
            self.current_state = "SENT_PLACE"
            # Simulate place completion after 3 seconds
            self.place_complete_timer = self.create_timer(3.0, self.simulate_place_complete, callback_group=None, reset_counter=True)
        elif self.current_state == "DONE":
            self.get_logger().info("✅ Test completed successfully!")
            self.timer.cancel()

    def send_pick_command(self):
        msg = PickPlaceEvent()
        msg.status = 1  # ARRIVED_AT_PICK_POINT – NAV PAUSED, PICK STARTED
        self.cmd_pub.publish(msg)
        self.get_logger().info("[TEST] ➡️ Sent status=1 (start PICK)")

    def send_place_command(self):
        msg = PickPlaceEvent()
        msg.status = 3  # ARRIVED_AT_PLACE_POINT – NAV PAUSED, PLACE STARTED
        self.cmd_pub.publish(msg)
        self.get_logger().info("[TEST] ➡️ Sent status=3 (start PLACE)")

    def simulate_pick_complete(self):
        self.get_logger().info("[TEST] ⏳ 3s elapsed – simulating PICK COMPLETED (status=2)")
        msg = PickPlaceEvent()
        msg.status = 2
        self.cmd_pub.publish(msg)
        # Do NOT change state here; let callback handle it for consistency

    def simulate_place_complete(self):
        self.get_logger().info("[TEST] ⏳ 3s elapsed – simulating PLACE COMPLETED (status=4)")
        msg = PickPlaceEvent()
        msg.status = 4
        self.cmd_pub.publish(msg)
        # Do NOT change state here; let callback handle it

    def status_callback(self, msg):
        status_map = {
            0: "NAVIGATING",
            1: "ARRIVED_AT_PICK – PICK STARTED",
            2: "PICK COMPLETED",
            3: "ARRIVED_AT_PLACE – PLACE STARTED",
            4: "PLACE COMPLETED"
        }
        desc = status_map.get(msg.status, f"UNKNOWN({msg.status})")
        self.get_logger().info(f"[TEST] ⬅️ Received status={msg.status} → {desc}")

        # Only react to completion statuses if we're expecting them
        if msg.status == 2 and self.current_state == "SENT_PICK":
            self.get_logger().info("[TEST] 🟢 Pick confirmed complete. Proceeding to place...")
            self.current_state = "PICK_DONE"
            if self.pick_complete_timer:
                self.pick_complete_timer.cancel()  # optional cleanup
        elif msg.status == 4 and self.current_state == "SENT_PLACE":
            self.get_logger().info("[TEST] 🟢 Place confirmed complete.")
            self.current_state = "DONE"
            if self.place_complete_timer:
                self.place_complete_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceCommTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()