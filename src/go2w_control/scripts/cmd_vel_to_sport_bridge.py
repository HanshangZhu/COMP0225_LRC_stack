#!/usr/bin/env python3
"""Bridge /cmd_vel (Twist) → /api/sport/request (Unitree Go2 Sport API).

Subscribes to /cmd_vel geometry_msgs/Twist and publishes velocity commands
to /api/sport/request using api_id=1008 (Move) at the incoming rate.
"""

import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request


class CmdVelToSportBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_sport_bridge')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('sport_topic', '/api/sport/request')

        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        sport_topic = str(self.get_parameter('sport_topic').value)

        self.sport_pub = self.create_publisher(Request, sport_topic, 10)
        self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_cb, 10)

        self.get_logger().info(
            f'cmd_vel→sport bridge: {cmd_vel_topic} → {sport_topic} (api_id=1008)')

    def _cmd_vel_cb(self, msg: Twist):
        req = Request()
        req.header.identity.api_id = 1008  # Move
        req.parameter = json.dumps({
            'x': msg.linear.x,
            'y': msg.linear.y,
            'z': msg.angular.z,
        })
        self.sport_pub.publish(req)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSportBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
