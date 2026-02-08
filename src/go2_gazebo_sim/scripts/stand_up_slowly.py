#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class StandUpSlowly(Node):
    def __init__(self):
        super().__init__('stand_up_slowly')
        
        # Publisher to the controller
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_group_effort_controller/joint_trajectory',
            10
        )
        
        # Joint names matching the controller config
        self.joint_names = [
            'lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint',
            'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint',
            'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint',
            'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint'
        ]
        
        # Target standing positions
        # Hip: 0.0, Thigh: 0.9, Calf: -1.8
        self.target_positions = [
            0.0, 0.9, -1.8,  # LF
            0.0, 0.9, -1.8,  # RF
            0.0, 0.9, -1.8,  # LH
            0.0, 0.9, -1.8   # RH
        ]
        
        self.get_logger().info('Waiting for controller to come up...')
        # Give time for controller to start
        self.timer = self.create_timer(3.0, self.publish_trajectory)
        self.published = False

    def publish_trajectory(self):
        if self.published:
            return
            
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.target_positions
        point.velocities = [0.0] * 12
        point.accelerations = [0.0] * 12
        
        # Take 5 seconds to reach the target (Slowly!)
        point.time_from_start = Duration(sec=5, nanosec=0)
        
        msg.points = [point]
        
        self.publisher_.publish(msg)
        self.get_logger().info('Published stand-up trajectory (5 seconds duration)')
        self.published = True
        
        # Exit after publishing
        # self.destroy_node()
        # rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = StandUpSlowly()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
