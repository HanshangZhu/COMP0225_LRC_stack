#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(PointStamped, '/goal_point', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        # Goal in World Frame: (9.35, 5.5)
        # Robot Spawn (Odom Origin) in World Frame: (2.0, 0.0)
        # Goal in Odom Frame: (7.35, 5.5)
        
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.point.x = 7.35
        msg.point.y = 5.5
        msg.point.z = 0.0
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing goal in odom frame: {msg.point.x}, {msg.point.y}')
        self.count += 1
        # Keep publishing for a while to ensure planner receives it
        if self.count > 10:
            self.get_logger().info('Goal published 10 times. Exiting.')
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
