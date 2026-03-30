#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

class Converter(Node):
    def __init__(self):
        super().__init__('joint_trajectory_to_joint_state')
        self.sub = self.create_subscription(
            JointTrajectory,
            'joint_group_effort_controller/joint_trajectory',
            self.callback,
            10
        )
        self.pub = self.create_publisher(JointState, 'joint_command', 10)

    def callback(self, msg: JointTrajectory):
        if not msg.points:
            return
        out = JointState()
        out.header = msg.header
        out.name = msg.joint_names
        pt = msg.points[0]
        out.position = pt.positions
        out.velocity = pt.velocities
        out.effort = pt.effort
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    c = Converter()
    rclpy.spin(c)
    c.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
