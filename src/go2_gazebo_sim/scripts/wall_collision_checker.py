#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8
import numpy as np

class WallCollisionChecker(Node):
    def __init__(self):
        super().__init__('wall_collision_checker')
        
        # Parameters
        self.declare_parameter('safety_dist', 0.6)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('stop_topic', '/stop')
        self.declare_parameter('check_angle_deg', 60.0) # Check +/- 30 degrees front
        
        self.safety_dist = self.get_parameter('safety_dist').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.stop_topic = self.get_parameter('stop_topic').value
        self.check_angle_rad = np.deg2rad(self.get_parameter('check_angle_deg').value) / 2.0

        self.sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.pub = self.create_publisher(Int8, self.stop_topic, 10)
        
        self.get_logger().info(f"Start detecting walls within {self.safety_dist}m on {self.scan_topic}...")

    def scan_callback(self, msg):
        # Convert ranges to numpy array
        ranges = np.array(msg.ranges)
        
        # Calculate angles
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        angles = angle_min + np.arange(len(ranges)) * angle_increment
        
        # Filter strictly front sector
        # Assuming 0 is front. If LIDAR is rotated, adjust here.
        # Usually 0 is front in standard ROSREP-103.
        
        # Handle wrap around if needed (usually angles are -PI to PI)
        # We want angles between -threshold and +threshold
        
        # Select indices where angle is within limits
        # Using simple boolean indexing since angles are monotonic usually
        front_indices = np.abs(angles) < self.check_angle_rad
        
        front_ranges = ranges[front_indices]
        
        # Filter out inf/nan
        front_ranges = front_ranges[np.isfinite(front_ranges)]
        front_ranges = front_ranges[front_ranges > 0.05] # Ignore self body
        
        stop_msg = Int8()
        
        if len(front_ranges) > 0:
            min_dist = np.min(front_ranges)
            if min_dist < self.safety_dist:
                stop_msg.data = 1
                self.get_logger().warn(f"Wall detected at {min_dist:.2f}m! Stopping.")
            else:
                stop_msg.data = 0
        else:
            stop_msg.data = 0
            
        self.pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallCollisionChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
