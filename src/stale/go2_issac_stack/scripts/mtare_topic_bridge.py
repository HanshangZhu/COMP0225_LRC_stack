#!/usr/bin/env python3
"""ROS2-side adapter between Isaac autonomy topics and M-TARE bridge topics."""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from typing import Any

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker


@dataclass(frozen=True)
class RobotMap:
    ns: str
    mtare_name: str


class MTareTopicBridge(Node):
    def __init__(self) -> None:
        super().__init__("mtare_topic_bridge")

        self.declare_parameter("robot_a_ns", "go2_1")
        self.declare_parameter("robot_b_ns", "go2_2")
        self.declare_parameter("robot_a_mtare_name", "wheeled0")
        self.declare_parameter("robot_b_mtare_name", "wheeled1")
        self.declare_parameter("hold_last_waypoint", True)
        self.declare_parameter("waypoint_stale_timeout_sec", 3.0)
        self.declare_parameter("waypoint_republish_hz", 5.0)
        self.declare_parameter("state_estimation_in_suffix", "state_estimation_at_scan")
        self.declare_parameter("state_estimation_out_suffix", "state_estimation_at_scan")
        self.declare_parameter("key_pose_in_suffix", "odom/nav")
        self.declare_parameter("key_pose_out_suffix", "key_pose_to_map")
        self.declare_parameter("registered_scan_in_suffix", "registered_scan")
        self.declare_parameter("registered_scan_out_suffix", "registered_scan")
        self.declare_parameter("terrain_map_in_suffix", "terrain_map")
        self.declare_parameter("terrain_map_out_suffix", "terrain_map")
        self.declare_parameter("terrain_map_ext_in_suffix", "terrain_map_ext")
        self.declare_parameter("terrain_map_ext_out_suffix", "terrain_map_ext")
        self.declare_parameter("waypoint_in_suffix", "way_point")
        self.declare_parameter("waypoint_out_suffix", "way_point_coord")
        self.declare_parameter("marker_frame_override", "")
        self.declare_parameter("waypoint_output_frame", "")

        self.robot_maps = [
            RobotMap(
                ns=str(self.get_parameter("robot_a_ns").value),
                mtare_name=str(self.get_parameter("robot_a_mtare_name").value),
            ),
            RobotMap(
                ns=str(self.get_parameter("robot_b_ns").value),
                mtare_name=str(self.get_parameter("robot_b_mtare_name").value),
            ),
        ]

        self.hold_last_waypoint = bool(self.get_parameter("hold_last_waypoint").value)
        self.waypoint_stale_timeout_sec = max(0.0, float(self.get_parameter("waypoint_stale_timeout_sec").value))
        self.waypoint_republish_hz = max(0.0, float(self.get_parameter("waypoint_republish_hz").value))
        self.state_estimation_in_suffix = self._suffix("state_estimation_in_suffix")
        self.state_estimation_out_suffix = self._suffix("state_estimation_out_suffix")
        self.key_pose_in_suffix = self._suffix("key_pose_in_suffix")
        self.key_pose_out_suffix = self._suffix("key_pose_out_suffix")
        self.registered_scan_in_suffix = self._suffix("registered_scan_in_suffix")
        self.registered_scan_out_suffix = self._suffix("registered_scan_out_suffix")
        self.terrain_map_in_suffix = self._suffix("terrain_map_in_suffix")
        self.terrain_map_out_suffix = self._suffix("terrain_map_out_suffix")
        self.terrain_map_ext_in_suffix = self._suffix("terrain_map_ext_in_suffix")
        self.terrain_map_ext_out_suffix = self._suffix("terrain_map_ext_out_suffix")
        self.waypoint_in_suffix = self._suffix("waypoint_in_suffix")
        self.waypoint_out_suffix = self._suffix("waypoint_out_suffix")
        self.marker_frame_override = str(self.get_parameter("marker_frame_override").value).strip()
        self.waypoint_output_frame = str(self.get_parameter("waypoint_output_frame").value).strip()

        self._latest_waypoint: dict[str, PointStamped] = {}
        self._latest_waypoint_time: dict[str, Any] = {}
        self._waypoint_stale_warned: dict[str, bool] = {}
        self._waypoint_out_pubs: dict[str, Any] = {}
        self._waypoint_marker_pubs: dict[str, Any] = {}
        self._waypoint_rx_count: dict[str, int] = {}
        self._waypoint_pub_count: dict[str, int] = {}
        self._relay_handles: list[Any] = []

        for robot in self.robot_maps:
            self._configure_robot(robot)

        if self.hold_last_waypoint and self.waypoint_republish_hz > 0.0:
            self.create_timer(1.0 / self.waypoint_republish_hz, self._republish_waypoints)
        self.create_timer(2.0, self._diagnostics_tick)

        mapping_text = ", ".join([f"{m.ns}<->{m.mtare_name}" for m in self.robot_maps])
        self.get_logger().warn(
            "DEPRECATED executable go2_issac_stack/mtare_topic_bridge.py in use. "
            "Switch launch/runtime to mtare_ros2/mtare_topic_bridge.py."
        )
        self.get_logger().info(
            "M-TARE topic bridge started | "
            f"maps={mapping_text} hold_last_waypoint={self.hold_last_waypoint} "
            f"stale_timeout={self.waypoint_stale_timeout_sec:.2f}s republish_hz={self.waypoint_republish_hz:.2f} "
            f"marker_frame={self.marker_frame_override or 'msg/default'} "
            f"waypoint_frame={self.waypoint_output_frame or 'msg/default'}"
        )

    def _suffix(self, parameter: str) -> str:
        return str(self.get_parameter(parameter).value).strip().strip("/")

    @staticmethod
    def _topic(prefix: str, suffix: str) -> str:
        if not suffix:
            return f"/{prefix}"
        return f"/{prefix}/{suffix}"

    def _configure_robot(self, robot: RobotMap) -> None:
        ns = robot.ns
        mtare = robot.mtare_name

        state_estimation_in = self._topic(ns, self.state_estimation_in_suffix)
        state_estimation_out = self._topic(mtare, self.state_estimation_out_suffix)
        key_pose_in = self._topic(ns, self.key_pose_in_suffix)
        key_pose_out = self._topic(mtare, self.key_pose_out_suffix)
        registered_scan_in = self._topic(ns, self.registered_scan_in_suffix)
        registered_scan_out = self._topic(mtare, self.registered_scan_out_suffix)
        terrain_map_in = self._topic(ns, self.terrain_map_in_suffix)
        terrain_map_out = self._topic(mtare, self.terrain_map_out_suffix)
        terrain_map_ext_in = self._topic(ns, self.terrain_map_ext_in_suffix)
        terrain_map_ext_out = self._topic(mtare, self.terrain_map_ext_out_suffix)
        waypoint_in = self._topic(mtare, self.waypoint_in_suffix)
        waypoint_out = self._topic(ns, self.waypoint_out_suffix)

        # ROS2 autonomy feeds -> M-TARE bridge feeds.
        self._relay_topic(
            Odometry,
            state_estimation_in,
            state_estimation_out,
        )
        self._relay_topic(
            Odometry,
            key_pose_in,
            key_pose_out,
        )
        self._relay_topic(
            PointCloud2,
            registered_scan_in,
            registered_scan_out,
        )
        self._relay_topic(
            PointCloud2,
            terrain_map_in,
            terrain_map_out,
        )
        self._relay_topic(
            PointCloud2,
            terrain_map_ext_in,
            terrain_map_ext_out,
        )

        self._waypoint_out_pubs[ns] = self.create_publisher(PointStamped, waypoint_out, 10)
        self._waypoint_marker_pubs[ns] = self.create_publisher(Marker, f"/{ns}/mtare_goal_marker", 10)
        self._waypoint_rx_count[ns] = 0
        self._waypoint_pub_count[ns] = 0
        self._waypoint_stale_warned[ns] = False
        self.create_subscription(
            PointStamped,
            waypoint_in,
            lambda msg, m=robot: self._waypoint_cb(msg, m),
            10,
        )
        self.get_logger().info(
            f"[{ns}] bridge map: "
            f"{state_estimation_in}->{state_estimation_out}, "
            f"{key_pose_in}->{key_pose_out}, "
            f"{registered_scan_in}->{registered_scan_out}, "
            f"{terrain_map_in}->{terrain_map_out}, "
            f"{terrain_map_ext_in}->{terrain_map_ext_out}, "
            f"{waypoint_in}->{waypoint_out}, "
            f"/{ns}/mtare_goal_marker"
        )

    def _relay_topic(self, msg_type: Any, in_topic: str, out_topic: str) -> None:
        pub = self.create_publisher(msg_type, out_topic, 10)
        sub = self.create_subscription(
            msg_type,
            in_topic,
            lambda msg, p=pub: p.publish(msg),
            10,
        )
        self._relay_handles.extend([pub, sub])

    def _waypoint_cb(self, msg: PointStamped, robot: RobotMap) -> None:
        ns = robot.ns
        if not self._waypoint_is_finite(msg):
            self.get_logger().warn(
                f"[{ns}] dropping non-finite waypoint from bridge input: "
                f"({msg.point.x}, {msg.point.y}, {msg.point.z})"
            )
            return
        self._waypoint_rx_count[ns] = self._waypoint_rx_count.get(ns, 0) + 1
        normalized = self._normalize_waypoint(msg)
        self._latest_waypoint[ns] = normalized
        self._latest_waypoint_time[ns] = self.get_clock().now()
        self._waypoint_stale_warned[ns] = False
        self._publish_waypoint_outputs(ns, normalized)
        rx_count = self._waypoint_rx_count[ns]
        if rx_count == 1 or rx_count % 50 == 0:
            self.get_logger().info(
                f"[{ns}] MTARE waypoint rx={rx_count} -> way_point_coord "
                f"({normalized.point.x:.2f}, {normalized.point.y:.2f}, {normalized.point.z:.2f})"
            )

    @staticmethod
    def _waypoint_is_finite(msg: PointStamped) -> bool:
        return (
            math.isfinite(float(msg.point.x))
            and math.isfinite(float(msg.point.y))
            and math.isfinite(float(msg.point.z))
        )

    def _normalize_waypoint(self, msg: PointStamped) -> PointStamped:
        out = copy.deepcopy(msg)
        if self.waypoint_output_frame:
            out.header.frame_id = self.waypoint_output_frame
        return out

    def _publish_waypoint_outputs(self, ns: str, msg: PointStamped) -> None:
        self._waypoint_out_pubs[ns].publish(msg)
        self._waypoint_pub_count[ns] = self._waypoint_pub_count.get(ns, 0) + 1
        marker = Marker()
        marker.header = msg.header
        if self.marker_frame_override:
            marker.header.frame_id = self.marker_frame_override
        elif not marker.header.frame_id:
            marker.header.frame_id = "world"
        marker.ns = "mtare_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = msg.point.x
        marker.pose.position.y = msg.point.y
        marker.pose.position.z = msg.point.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        marker.color.a = 0.95
        marker.color.r = 1.0
        marker.color.g = 0.35
        marker.color.b = 0.0
        self._waypoint_marker_pubs[ns].publish(marker)

    def _republish_waypoints(self) -> None:
        now = self.get_clock().now()
        for ns in list(self._latest_waypoint.keys()):
            stamp = self._latest_waypoint_time.get(ns)
            msg = self._latest_waypoint.get(ns)
            if stamp is None or msg is None:
                continue
            age_sec = (now - stamp).nanoseconds * 1e-9
            if self.waypoint_stale_timeout_sec > 0.0 and age_sec > self.waypoint_stale_timeout_sec:
                if not self._waypoint_stale_warned.get(ns, False):
                    self.get_logger().warn(
                        f"Dropping stale waypoint for {ns}: age={age_sec:.2f}s > {self.waypoint_stale_timeout_sec:.2f}s"
                    )
                    self._waypoint_stale_warned[ns] = True
                continue
            self._publish_waypoint_outputs(ns, msg)

    def _diagnostics_tick(self) -> None:
        now = self.get_clock().now()
        for robot in self.robot_maps:
            ns = robot.ns
            rx_count = self._waypoint_rx_count.get(ns, 0)
            pub_count = self._waypoint_pub_count.get(ns, 0)
            stamp = self._latest_waypoint_time.get(ns)
            age = (now - stamp).nanoseconds * 1e-9 if stamp is not None else -1.0
            self.get_logger().info(
                f"[{ns}] MTARE diagnostics: waypoint_rx={rx_count} waypoint_pub={pub_count} "
                f"last_waypoint_age={age:.2f}s"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MTareTopicBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
