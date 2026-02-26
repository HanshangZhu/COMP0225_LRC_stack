"""Shared dual-robot runtime helpers for Isaac launch files."""

from __future__ import annotations

from typing import Any

from launch_ros.actions import Node

from _stack_components import (
    build_isaac_topic_router_node,
    build_odom_tf_broadcaster_node,
    build_qos_bridge_node,
)


ROBOT_A_NS = "go2_1"
ROBOT_B_NS = "go2_2"


def robot_topics(ns: str) -> tuple[list[tuple[str, str]], str, str]:
    tf_remaps = [("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")]
    nav_odom_topic = f"/{ns}/odom/nav"
    planning_scan_topic = f"/{ns}/scan_3d"
    return tf_remaps, nav_odom_topic, planning_scan_topic


def build_slam_nodes(
    ns: str,
    use_sim_time: bool,
    use_fast_lio: bool,
    slam_config: str,
    nav_odom_topic: str,
) -> list[Any]:
    nodes: list[Any] = []

    if use_fast_lio:
        nodes.append(
            Node(
                package="go2_gazebo_sim",
                executable="pointcloud_adapter.py",
                namespace=ns,
                name="pointcloud_adapter",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_topic": f"/{ns}/registered_scan_reliable"},
                    {"output_topic": f"/{ns}/velodyne_points"},
                    {"num_rings": 16},
                ],
                output="screen",
            )
        )
        nodes.append(
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                namespace=ns,
                name="slam_node",
                parameters=[slam_config, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("/velodyne_points", f"/{ns}/velodyne_points"),
                    ("/imu/data", f"/{ns}/imu/data"),
                    ("/Odometry", f"/{ns}/Odometry"),
                ],
                output="screen",
            )
        )
        nodes.append(
            Node(
                package="go2_gazebo_sim",
                executable="slam_odom_relay.py",
                namespace=ns,
                name="slam_odom_relay",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_topic": f"/{ns}/Odometry"},
                    {"gt_topic": f"/{ns}/odom/ground_truth"},
                    {"output_topic": nav_odom_topic},
                    {"output_frame_id": "world"},
                    {"output_child_frame_id": "base_link"},
                    {"bootstrap_from_gt": True},
                    {"require_gt_for_alignment": True},
                ],
                output="screen",
            )
        )
    else:
        nodes.append(
            Node(
                package="go2_gazebo_sim",
                executable="slam_odom_relay.py",
                namespace=ns,
                name="gt_odom_relay",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_topic": f"/{ns}/odom/ground_truth"},
                    {"output_topic": nav_odom_topic},
                    {"output_frame_id": "world"},
                    {"output_child_frame_id": "base_link"},
                ],
                output="screen",
            )
        )

    return nodes


def build_core_bridge_nodes(
    ns: str,
    use_sim_time: bool,
    use_fast_lio: bool,
    slam_config: str,
    tf_remaps: list[tuple[str, str]],
    nav_odom_topic: str,
) -> list[Any]:
    isaac_topic_router_node = build_isaac_topic_router_node(
        ns=ns,
        use_sim_time=use_sim_time,
        extra_params={
            "input_odom_topic": f"/{ns}/odom",
            "output_odom_topic": f"/{ns}/odom/ground_truth",
            "input_pointcloud_topic": f"/{ns}/lidar/points",
            "output_pointcloud_topic": f"/{ns}/registered_scan",
            "input_imu_topic": f"/{ns}/imu",
            "output_imu_topic": f"/{ns}/imu/data",
            "input_cmd_vel_topic": f"/{ns}/cmd_vel",
            "output_cmd_vel_topic": f"/{ns}/isaac/cmd_vel",
        },
    )

    twist_bridge_node = Node(
        package="go2_gazebo_sim",
        executable="twist_bridge.py",
        namespace=ns,
        remappings=[("/cmd_vel_stamped", f"/{ns}/cmd_vel_stamped"), ("/cmd_vel", f"/{ns}/cmd_vel")],
        output="screen",
    )

    qos_bridge_node = build_qos_bridge_node(
        ns=ns,
        use_sim_time=use_sim_time,
        extra_params={
            "input_topic": f"/{ns}/registered_scan",
            "output_topic": f"/{ns}/registered_scan_reliable",
        },
    )

    world_to_odom_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=ns,
        name="world_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        remappings=tf_remaps,
        output="screen",
    )

    world_to_map_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=ns,
        name="world_to_map_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
        remappings=tf_remaps,
        output="screen",
    )

    odom_tf_broadcaster_node = build_odom_tf_broadcaster_node(
        ns=ns,
        use_sim_time=use_sim_time,
        extra_params={
            "odom_topic": nav_odom_topic,
            "parent_frame": "odom",
            "child_frame": "base_link",
            "max_publish_rate": 60.0,
        },
        remappings=tf_remaps,
    )

    return [
        isaac_topic_router_node,
        twist_bridge_node,
        qos_bridge_node,
        *build_slam_nodes(ns, use_sim_time, use_fast_lio, slam_config, nav_odom_topic),
        world_to_odom_tf_node,
        world_to_map_tf_node,
        odom_tf_broadcaster_node,
    ]
