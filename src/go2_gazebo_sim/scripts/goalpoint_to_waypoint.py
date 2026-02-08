#!/usr/bin/env python3
"""
Bridge Goalpoint -> Waypoint for Gazebo.

RViz Goalpoint tool publishes to /goal_point, while local_planner expects /way_point.
This keeps Gazebo behavior aligned with the Unity stack.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class GoalpointToWaypoint(Node):
    def __init__(self):
        super().__init__("goalpoint_to_waypoint")
        self.sub = self.create_subscription(
            PointStamped,
            "/goal_point",
            self.goal_callback,
            10,
        )
        self.pub = self.create_publisher(PointStamped, "/way_point", 10)
        self.get_logger().info("Goalpoint bridge: /goal_point -> /way_point")

    def goal_callback(self, msg: PointStamped):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoalpointToWaypoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
