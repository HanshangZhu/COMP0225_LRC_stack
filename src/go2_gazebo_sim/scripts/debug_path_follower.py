#!/usr/bin/env python3
"""
Debug script to monitor pathFollower inputs and outputs
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int8, Float32
import math

class PathFollowerDebugger(Node):
    def __init__(self):
        super().__init__('path_follower_debugger')
        
        # State tracking
        self.odom_count = 0
        self.path_count = 0
        self.cmd_vel_count = 0
        self.last_odom_time = None
        self.last_path_size = 0
        self.last_cmd_vel = None
        
        # Subscriptions (same as pathFollower)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/path', self.path_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped, '/cmd_vel', self.cmd_vel_callback, 10)
        self.stop_sub = self.create_subscription(
            Int8, '/stop', self.stop_callback, 10)
        self.speed_sub = self.create_subscription(
            Float32, '/speed', self.speed_callback, 10)
        
        # Timer for periodic debug output
        self.timer = self.create_timer(2.0, self.debug_output)
        
        self.get_logger().info('PathFollower Debugger Started')
        
    def odom_callback(self, msg):
        self.odom_count += 1
        self.last_odom_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.last_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
    def path_callback(self, msg):
        self.path_count += 1
        self.last_path_size = len(msg.poses)
        if self.last_path_size > 0:
            end = msg.poses[-1].pose.position
            self.last_path_end = (end.x, end.y)
        
    def cmd_vel_callback(self, msg):
        self.cmd_vel_count += 1
        self.last_cmd_vel = msg.twist
        
    def stop_callback(self, msg):
        self.get_logger().warn(f'STOP signal received: {msg.data}')
        
    def speed_callback(self, msg):
        self.get_logger().info(f'Speed command: {msg.data}')
        
    def debug_output(self):
        self.get_logger().info('='*50)
        self.get_logger().info(f'ODOM: {self.odom_count} msgs, last_time={self.last_odom_time:.2f}' if self.last_odom_time else 'ODOM: No messages!')
        
        if hasattr(self, 'last_pos'):
            self.get_logger().info(f'  Position: ({self.last_pos[0]:.3f}, {self.last_pos[1]:.3f})')
        
        self.get_logger().info(f'PATH: {self.path_count} msgs, size={self.last_path_size}')
        if hasattr(self, 'last_path_end') and self.last_path_size > 1:
            self.get_logger().info(f'  End point (vehicle frame): ({self.last_path_end[0]:.3f}, {self.last_path_end[1]:.3f})')
            if hasattr(self, 'last_pos'):
                # Calculate distance to path end in vehicle frame (approximate)
                dist = math.sqrt(self.last_path_end[0]**2 + self.last_path_end[1]**2)
                self.get_logger().info(f'  Distance to path end: {dist:.3f}m')
        
        if self.cmd_vel_count > 0:
            self.get_logger().info(f'CMD_VEL: {self.cmd_vel_count} msgs')
            if self.last_cmd_vel:
                self.get_logger().info(f'  linear: ({self.last_cmd_vel.linear.x:.3f}, {self.last_cmd_vel.linear.y:.3f})')
                self.get_logger().info(f'  angular.z: {self.last_cmd_vel.angular.z:.3f}')
        else:
            self.get_logger().warn('CMD_VEL: *** NO MESSAGES - THIS IS THE PROBLEM! ***')
        
        self.get_logger().info('='*50)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
