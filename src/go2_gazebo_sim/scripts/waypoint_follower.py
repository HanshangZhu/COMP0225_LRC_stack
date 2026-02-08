#!/usr/bin/env python3
"""
Waypoint Follower - Follows a predefined trajectory through the L-corridor.
No planning, just sequential waypoint tracking.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        # Predefined waypoints for L-corridor (x, y, yaw_deg)
        # Robot starts at (2.0, 0.0), facing +X
        self.waypoints = [
            (4.0, 0.0),    # Move forward in corridor
            (6.0, 0.0),    # Continue forward
            (8.0, 0.0),    # Approach corner
            (9.0, 1.0),    # Start turning into vertical section
            (9.3, 3.0),    # Continue up
            (9.3, 5.5),    # Near end of corridor
        ]
        
        self.current_waypoint_idx = 0
        self.current_pose = None
        
        # Parameters
        self.linear_speed = 0.25  # m/s - conservative
        self.angular_speed = 0.4  # rad/s
        self.goal_tolerance = 0.4  # meters
        
        # Publishers/Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom/ground_truth', self.odom_callback, 10)
        
        # Control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'Waypoint Follower started with {len(self.waypoints)} waypoints')
        self.get_logger().info(f'Target: {self.waypoints[-1]}')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def get_yaw_from_quaternion(self, q):
        """Extract yaw from quaternion."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.current_pose is None:
            return
        
        if self.current_waypoint_idx >= len(self.waypoints):
            # All waypoints reached - stop
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return
        
        # Current position
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Target waypoint
        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        
        # Distance and angle to target
        dx = target_x - x
        dy = target_y - y
        distance = math.sqrt(dx*dx + dy*dy)
        target_yaw = math.atan2(dy, dx)
        
        # Angle error (normalized to [-pi, pi])
        yaw_error = target_yaw - yaw
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        # Check if waypoint reached
        if distance < self.goal_tolerance:
            self.get_logger().info(f'Waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)} reached: ({target_x:.1f}, {target_y:.1f})')
            self.current_waypoint_idx += 1
            
            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info('🎉 All waypoints reached! Goal complete!')
                cmd = Twist()
                self.cmd_pub.publish(cmd)
                return
            return
        
        # Simple control: rotate first, then move forward
        cmd = Twist()
        
        if abs(yaw_error) > 0.3:  # ~17 degrees
            # Rotate to face target
            cmd.angular.z = self.angular_speed if yaw_error > 0 else -self.angular_speed
            cmd.linear.x = 0.05  # Tiny forward motion while turning
        else:
            # Move towards target
            cmd.linear.x = min(self.linear_speed, distance * 0.5)
            cmd.angular.z = yaw_error * 1.5  # Proportional correction
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        cmd = Twist()
        node.cmd_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
