#!/usr/bin/env python3
"""
Autonomous Frontier Exploration Node

This node implements true autonomous exploration by:
1. Subscribing to the visibility graph from FAR planner
2. Identifying frontier nodes (boundaries between known and unknown space)
3. Selecting the best frontier to explore (nearest, largest, etc.)
4. Publishing the selected frontier as the next goal

This replaces the need for manual goal specification and enables
truly autonomous exploration behavior.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from visibility_graph_msg.msg import Graph
import math

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        # Parameters
        self.declare_parameter('min_frontier_distance', 1.0)  # Min distance to consider a frontier
        self.declare_parameter('goal_reached_threshold', 2.0)  # Distance to consider goal reached
        self.declare_parameter('exploration_frame', 'odom')
        self.declare_parameter('goal_publish_rate', 2.0)  # Hz
        self.declare_parameter('goal_topic', '/goal_point')
        
        self.min_frontier_dist = self.get_parameter('min_frontier_distance').value
        self.goal_reached_thresh = self.get_parameter('goal_reached_threshold').value
        self.exploration_frame = self.get_parameter('exploration_frame').value
        self.goal_topic = self.get_parameter('goal_topic').value
        
        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.current_goal = None
        self.frontiers = []
        self.visited_frontiers = set()  # Track visited frontier IDs
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.vgraph_sub = self.create_subscription(
            Graph, '/robot_vgraph', self.vgraph_callback, 10)
        
        # Publisher
        self.goal_pub = self.create_publisher(
            PointStamped, self.goal_topic, 10)
        
        # Timer for periodic goal publishing
        rate = self.get_parameter('goal_publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.exploration_loop)
        
        self.get_logger().info('='*50)
        self.get_logger().info('Frontier Explorer Started!')
        self.get_logger().info('  - Autonomously exploring frontiers')
        self.get_logger().info('  - No manual goals needed')
        self.get_logger().info(f'  - Publishing goals to: {self.goal_topic}')
        self.get_logger().info('='*50)
        
    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z
        
    def vgraph_callback(self, msg: Graph):
        """Extract frontier nodes from visibility graph"""
        self.frontiers = []
        
        for node in msg.nodes:
            if node.is_frontier and not node.is_covered:
                frontier = {
                    'id': node.id,
                    'x': node.position.x,
                    'y': node.position.y,
                    'z': node.position.z,
                }
                self.frontiers.append(frontier)
                
    def get_distance(self, x, y):
        """Calculate distance from robot to point"""
        return math.sqrt((x - self.robot_x)**2 + (y - self.robot_y)**2)
    
    def select_best_frontier(self):
        """Select the best frontier to explore"""
        if not self.frontiers:
            return None
            
        # Filter out frontiers that are too close or already visited recently
        valid_frontiers = []
        for f in self.frontiers:
            dist = self.get_distance(f['x'], f['y'])
            if dist >= self.min_frontier_dist:
                # Check if this frontier was visited recently
                frontier_key = (round(f['x'], 1), round(f['y'], 1))
                if frontier_key not in self.visited_frontiers:
                    f['distance'] = dist
                    valid_frontiers.append(f)
        
        if not valid_frontiers:
            # If all frontiers are visited, reset visited set
            if self.frontiers:
                self.get_logger().info('All frontiers visited, resetting exploration memory')
                self.visited_frontiers.clear()
                return self.select_best_frontier()
            return None
            
        # Strategy: Select the nearest frontier
        # (Can be changed to other strategies like largest frontier, etc.)
        valid_frontiers.sort(key=lambda f: f['distance'])
        
        return valid_frontiers[0]
    
    def exploration_loop(self):
        """Main exploration loop"""
        # Check if we've reached current goal
        if self.current_goal:
            dist_to_goal = self.get_distance(
                self.current_goal['x'], 
                self.current_goal['y']
            )
            
            if dist_to_goal < self.goal_reached_thresh:
                # Mark as visited
                frontier_key = (round(self.current_goal['x'], 1), 
                               round(self.current_goal['y'], 1))
                self.visited_frontiers.add(frontier_key)
                self.get_logger().info(
                    f'Reached frontier at ({self.current_goal["x"]:.1f}, '
                    f'{self.current_goal["y"]:.1f}), selecting new frontier...')
                self.current_goal = None
        
        # Select new frontier if needed
        if self.current_goal is None:
            best_frontier = self.select_best_frontier()
            
            if best_frontier:
                self.current_goal = best_frontier
                self.get_logger().info(
                    f'New exploration target: ({best_frontier["x"]:.1f}, '
                    f'{best_frontier["y"]:.1f}) - distance: {best_frontier["distance"]:.1f}m')
            else:
                if len(self.frontiers) == 0:
                    self.get_logger().warn('No frontiers available yet, waiting for V-Graph...')
                else:
                    self.get_logger().info('Exploration complete! No more frontiers.')
                return
        
        # Publish current goal
        if self.current_goal:
            goal_msg = PointStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = self.exploration_frame
            goal_msg.point.x = self.current_goal['x']
            goal_msg.point.y = self.current_goal['y']
            goal_msg.point.z = self.current_goal['z']
            
            self.goal_pub.publish(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
