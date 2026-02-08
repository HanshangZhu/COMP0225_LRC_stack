#!/usr/bin/env python3
"""
Simple Goal Follower - A minimal navigation script that walks toward a goal.
Bypasses complex autonomy stacks for debugging.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
import math
import numpy as np

class SimpleGoalFollower(Node):
    def __init__(self):
        super().__init__('simple_goal_follower')
        
        # Parameters
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('angle_tolerance', 0.1)
        
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        
        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_reached = False
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Optional: Subscribe to goal_point for dynamic goals
        self.goal_sub = self.create_subscription(
            PointStamped,
            '/goal_point',
            self.goal_callback,
            10
        )
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'Simple Goal Follower started. Goal: ({self.goal_x}, {self.goal_y})')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_callback(self, msg):
        self.goal_x = msg.point.x
        self.goal_y = msg.point.y
        self.goal_reached = False
        self.get_logger().info(f'New goal received: ({self.goal_x}, {self.goal_y})')

    def control_loop(self):
        cmd = Twist()
        
        # Calculate distance and angle to goal
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        angle_to_goal = math.atan2(dy, dx)
        
        # Angle difference (normalize to -pi, pi)
        angle_diff = angle_to_goal - self.current_yaw
        while angle_diff > math.pi:
            angle_diff -= 2*math.pi
        while angle_diff < -math.pi:
            angle_diff += 2*math.pi
        
        if distance < self.goal_tolerance:
            if not self.goal_reached:
                self.get_logger().info('Goal reached!')
                self.goal_reached = True
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif abs(angle_diff) > self.angle_tolerance:
            # Turn toward goal first
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        else:
            # Move forward
            cmd.linear.x = self.linear_speed
            cmd.angular.z = angle_diff * 0.5  # Proportional steering
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGoalFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
