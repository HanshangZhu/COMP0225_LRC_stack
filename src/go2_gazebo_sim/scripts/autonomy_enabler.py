#!/usr/bin/env python3
"""
Publish a synthetic /joy message to keep localPlanner and pathFollower
in autonomy mode.  Both nodes check joy->axes[2] <= -0.1 to enable
autonomyMode.  Without a physical joystick this never happens, so the
entire navigation stack sits idle.

This node waits for the robot to stand up (startup_delay), then
publishes /joy at 10 Hz with axes[2] = -1.0.
"""
import rclpy                          # ROS 2 Python client library
from rclpy.node import Node           # base Node class
from sensor_msgs.msg import Joy       # joystick message type


class AutonomyEnabler(Node):
    def __init__(self):
        super().__init__("autonomy_enabler")

        # --- parameters ------------------------------------------------
        self.declare_parameter("startup_delay", 10.0)   # seconds before first publish
        self.declare_parameter("rate", 10.0)             # Hz for /joy publishing
        self.startup_delay = float(self.get_parameter("startup_delay").value)
        self.rate = float(self.get_parameter("rate").value)

        # --- state ------------------------------------------------------
        self.start_time = None                           # set on first timer callback
        self.enabled = False                             # set True after delay

        # --- publisher --------------------------------------------------
        self.joy_pub = self.create_publisher(Joy, "/joy", 10)

        # --- timer ------------------------------------------------------
        self.timer = self.create_timer(1.0 / self.rate, self.publish_joy)

        self.get_logger().info(
            f"Autonomy enabler: will activate in {self.startup_delay:.1f}s"
        )

    # ------------------------------------------------------------------
    def publish_joy(self):
        # set start_time on first callback (sim clock is 0 at __init__)
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # wait for robot stand-up to finish
        if elapsed < self.startup_delay:
            return

        if not self.enabled:
            self.enabled = True
            self.get_logger().info("Autonomy mode ENABLED via synthetic /joy")

        # build Joy message -----------------------------------------------
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        # axes layout (matches typical gamepad):
        #   [0] left-stick-X   (manual yaw)
        #   [1] left-stick-Y
        #   [2] left-trigger   ** <= -0.1 → autonomy ON **
        #   [3] right-stick-X  (manual left)
        #   [4] right-stick-Y  (manual fwd)
        #   [5] right-trigger  ** <= -0.1 → manual ON — keep >-0.1 **
        #   [6] dpad-X
        #   [7] dpad-Y
        msg.axes = [
            0.0,   # 0  left-stick X
            0.0,   # 1  left-stick Y
           -1.0,   # 2  left trigger  → enables autonomy
            0.0,   # 3  right-stick X
            0.0,   # 4  right-stick Y
            0.0,   # 5  right trigger → NOT manual mode
            0.0,   # 6  dpad X
            0.0,   # 7  dpad Y
        ]
        msg.buttons = [0] * 11        # no buttons pressed

        self.joy_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyEnabler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
