#!/usr/bin/env python3
"""
Spawn/update a small sphere in Gazebo at the frontier goal.
"""
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Pose
from gazebo_msgs.srv import SpawnEntity, DeleteEntity


SPHERE_SDF = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="frontier_goal_marker">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1 0.2 0.2 1</ambient>
          <diffuse>1 0.2 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class GazeboFrontierVisual(Node):
    def __init__(self):
        super().__init__("gazebo_frontier_visual")

        self.declare_parameter("goal_topic", "/frontier_goal")
        self.declare_parameter("entity_name", "frontier_goal_marker")
        self.declare_parameter("min_update_distance", 0.2)
        self.declare_parameter("min_update_sec", 1.0)

        self.goal_topic = self.get_parameter("goal_topic").value
        self.entity_name = self.get_parameter("entity_name").value
        self.min_update_distance = float(self.get_parameter("min_update_distance").value)
        self.min_update_sec = float(self.get_parameter("min_update_sec").value)

        self.last_x: Optional[float] = None
        self.last_y: Optional[float] = None
        self.last_time = None
        self.spawned = False

        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")
        self.delete_cli = self.create_client(DeleteEntity, "/delete_entity")

        self.sub = self.create_subscription(PointStamped, self.goal_topic, self.goal_cb, 10)

        self.get_logger().info("Gazebo frontier visual started")

    def goal_cb(self, msg: PointStamped):
        now = self.get_clock().now()
        if self.last_time is not None:
            if (now - self.last_time).nanoseconds / 1e9 < self.min_update_sec:
                return
        if self.last_x is not None and self.last_y is not None:
            if math.hypot(msg.point.x - self.last_x, msg.point.y - self.last_y) < self.min_update_distance:
                return

        self.last_x = msg.point.x
        self.last_y = msg.point.y
        self.last_time = now

        if not self.spawn_cli.service_is_ready():
            return
        if not self.delete_cli.service_is_ready():
            return

        if self.spawned:
            delete_req = DeleteEntity.Request()
            delete_req.name = self.entity_name
            self.delete_cli.call_async(delete_req)

        spawn_req = SpawnEntity.Request()
        spawn_req.name = self.entity_name
        spawn_req.xml = SPHERE_SDF
        spawn_req.robot_namespace = self.entity_name
        spawn_req.reference_frame = "world"
        pose = Pose()
        pose.position.x = float(msg.point.x)
        pose.position.y = float(msg.point.y)
        pose.position.z = 0.1
        pose.orientation.w = 1.0
        spawn_req.initial_pose = pose
        self.spawn_cli.call_async(spawn_req)
        self.spawned = True


def main(args=None):
    rclpy.init(args=args)
    node = GazeboFrontierVisual()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
