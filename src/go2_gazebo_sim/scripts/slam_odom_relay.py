#!/usr/bin/env python3
"""
Relay node: converts Point-LIO SLAM odometry to the format expected
by the autonomy stack.

Point-LIO publishes:
    /aft_mapped_to_init  (frame_id=camera_init, child_frame_id=aft_mapped)

The autonomy stack expects:
    /odom/ground_truth   (frame_id=world, child_frame_id=base_link)

This node subscribes to the SLAM output and republishes it with the
correct frame IDs so all downstream nodes (reactive_nav, geometric_frontier,
motion_monitor) work unchanged.

Usage:
    ros2 run go2_gazebo_sim slam_odom_relay.py
    # or with custom topics:
    ros2 run go2_gazebo_sim slam_odom_relay.py --ros-args \
        -p input_topic:=/aft_mapped_to_init \
        -p output_topic:=/odom/ground_truth
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class SlamOdomRelay(Node):
    def __init__(self):
        super().__init__("slam_odom_relay")

        self.declare_parameter("input_topic", "/aft_mapped_to_init")
        self.declare_parameter("output_topic", "/odom/ground_truth")
        self.declare_parameter("output_frame_id", "world")
        self.declare_parameter("output_child_frame_id", "base_link")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.output_frame = self.get_parameter("output_frame_id").value
        self.output_child_frame = self.get_parameter("output_child_frame_id").value

        self.sub = self.create_subscription(Odometry, input_topic, self.relay_cb, 10)
        self.pub = self.create_publisher(Odometry, output_topic, 10)

        self.msg_count = 0
        self.get_logger().info(
            f"SLAM odom relay: {input_topic} -> {output_topic} "
            f"(frame: {self.output_frame} -> {self.output_child_frame})"
        )

    def relay_cb(self, msg: Odometry):
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.output_frame
        out.child_frame_id = self.output_child_frame
        out.pose = msg.pose
        out.twist = msg.twist
        self.pub.publish(out)

        self.msg_count += 1
        if self.msg_count % 100 == 1:
            p = msg.pose.pose.position
            self.get_logger().info(
                f"Relayed {self.msg_count} msgs | "
                f"SLAM pose=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SlamOdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
