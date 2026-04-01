#!/usr/bin/env python3
"""Single Go2W real-robot CFPA2 runtime on top of Cartographer SLAM.

Assumes Cartographer is already running (via ./go2w_ethernet_start.sh cartographer),
which provides:
  - TF: map → odom → body
  - /map (OccupancyGrid) from cartographer_occupancy_grid_node
  - /utlidar/transformed_cloud (from transform_everything)

This launch file adds the navigation stack:
  carto_odom_bridge → /robot/odom/nav (Odometry from TF)
  pointcloud_to_laserscan → /robot/scan_3d (LaserScan for default_nav local avoidance)
  navigation sub-launch → mapper + cfpa2 + default_nav
  safety sub-launch → wall_checker + autonomy_enabler
  twist_bridge → cmd_vel_activity_mux → /cmd_vel (sport API)

Shared pipeline layers (navigation, safety, observability) use sub-launches from go2w_config.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text: str) -> bool:
    return str(text).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _launch_setup(context):
    robot_ns = _get(context, "robot_namespace").strip().strip("/") or "robot"
    enable_manual_fallback = _as_bool(_get(context, "enable_manual_fallback"))
    external_mapper = _as_bool(_get(context, "external_mapper"))
    map_backend = _get(context, "map_backend").strip().lower() or "scan"
    waypoint_input_suffix = _get(context, "waypoint_input_suffix").strip()
    if not waypoint_input_suffix:
        waypoint_input_suffix = "/way_point_coord"
    if not waypoint_input_suffix.startswith("/"):
        waypoint_input_suffix = "/" + waypoint_input_suffix

    go2w_config_pkg = get_package_share_directory("go2w_config")
    cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")

    teleop_config = os.path.join(go2w_config_pkg, "config", "teleop", "teleop_twist_joy_go2w.yaml")

    # Sub-launch paths
    nav_launch = os.path.join(go2w_config_pkg, "launch", "navigation.launch.py")
    safety_launch = os.path.join(go2w_config_pkg, "launch", "safety.launch.py")
    obs_launch = os.path.join(go2w_config_pkg, "launch", "observability.launch.py")

    if map_backend not in {"scan", "carto_binary", "carto_2d"}:
        raise ValueError(f"Unsupported map_backend '{map_backend}' (expected scan, carto_binary or carto_2d)")

    actions = []

    # ── Real-specific: Static TF body → base_link (identity) ──
    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="body_to_base_link",
            arguments=[
                "--frame-id", "body", "--child-frame-id", "base_link",
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            ],
            parameters=[{"use_sim_time": False}],
            output="log",
        )
    )

    # ── Real-specific: Cartographer TF → Odometry bridge ──
    actions.append(
        Node(
            package="go2w_perception",
            executable="carto_odom_bridge.py",
            namespace=robot_ns,
            name="carto_odom_bridge",
            parameters=[
                {
                    "parent_frame": "map",
                    "child_frame": "body",
                    "output_topic": f"/{robot_ns}/odom/nav",
                    "output_frame_id": "map",
                    "output_child_frame_id": "base_link",
                    "rate": 50.0,
                }
            ],
            output="screen",
        )
    )

    # ── Real-specific: PointCloud → LaserScan ──
    actions.append(
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            namespace=robot_ns,
            name="pointcloud_to_laserscan",
            parameters=[
                {
                    "use_sim_time": False,
                    "target_frame": "base_link",
                    "transform_tolerance": 0.3,
                    "min_height": -0.25,
                    "max_height": 0.60,
                    "angle_min": -3.14159,
                    "angle_max": 3.14159,
                    "angle_increment": 0.006135923151543,
                    "scan_time": 0.1,
                    "range_min": 0.10,
                    "range_max": 8.0,
                    "use_inf": True,
                }
            ],
            remappings=[
                ("cloud_in", "/utlidar/transformed_cloud"),
                ("scan", f"/{robot_ns}/scan_3d_raw"),
            ],
            output="screen",
        )
    )

    # ── Real-specific: Rear self-hit filter ──
    actions.append(
        Node(
            package="go2_nav_algorithms",
            executable="scan_rear_filter",
            namespace=robot_ns,
            name="scan_rear_filter",
            parameters=[{"rear_blank_radius": 0.45}],
            remappings=[
                ("scan_in", f"/{robot_ns}/scan_3d_raw"),
                ("scan_out", f"/{robot_ns}/scan_3d"),
            ],
            output="screen",
        )
    )

    # ── Real-specific: carto_binary / carto_2d mapper fallback ──
    if not external_mapper and map_backend in {"carto_binary", "carto_2d"}:
        actions.append(
            Node(
                package="go2w_perception",
                executable="probability_grid_binarizer.py",
                namespace=robot_ns,
                name="probability_grid_binarizer",
                parameters=[
                    {
                        "input_topic": f"/{robot_ns}/map_prob",
                        "output_topic": f"/{robot_ns}/map",
                        "free_threshold": 25,
                        "occupied_threshold": 65,
                        "min_occupied_component_cells": 3,
                        "fill_holes": True,
                        "hole_neighbor_threshold": 7,
                    }
                ],
                output="screen",
            )
        )

    # ── Shared navigation sub-launch (mapper + cfpa2 + default_nav) ──
    # scan mapper is skipped when map_backend != "scan" (external_mapper handles it)
    use_external_mapper = external_mapper or map_backend != "scan"
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
            launch_arguments={
                "robot_namespace": robot_ns,
                "use_sim_time": "false",
                "map_frame": "map",
                "external_mapper": str(use_external_mapper).lower(),
                "broadcast_tf": "false",
                "remap_tf": "false",
                "scan_topic": f"/{robot_ns}/scan_3d",
                "odom_topic": f"/{robot_ns}/odom/nav",
                "waypoint_input_suffix": waypoint_input_suffix,
                "cfpa2_config": os.path.join(cfpa2_pkg, "config", "cfpa2_single_robot.yaml"),
                "cfpa2_w_ig": "0.5",
                "cfpa2_w_c": "0.8",
                "cfpa2_w_momentum": "2.5",
                "cfpa2_min_utility": "-1.0",
                "cfpa2_switch_hysteresis": "0.06",
                "max_linear_speed": "0.30",
                "require_settle_before_motion": "false",
                "nav_map_topic": f"/{robot_ns}/map",
            }.items(),
        )
    )

    # ── Shared safety sub-launch (wall checker + autonomy enabler) ──
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(safety_launch),
            launch_arguments={
                "robot_namespace": robot_ns,
                "use_sim_time": "false",
                "scan_topic": f"/{robot_ns}/scan_3d",
                "autonomy_startup_delay": "4.0",
            }.items(),
        )
    )

    # ── Shared observability sub-launch ──
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(obs_launch),
            launch_arguments={
                "robot_namespace": robot_ns,
                "use_sim_time": "false",
                "experiment_name": "real_go2w",
            }.items(),
        )
    )

    # ── Real-specific: Twist Bridge (routes to auto mux input) ──
    actions.append(
        Node(
            package="go2w_perception",
            executable="twist_bridge.py",
            namespace=robot_ns,
            name="twist_bridge",
            remappings=[
                ("cmd_vel_stamped", f"/{robot_ns}/cmd_vel_stamped"),
                ("cmd_vel", f"/{robot_ns}/cmd_vel_auto"),
            ],
            output="screen",
        )
    )

    # ── Real-specific: Auto/Manual velocity mux ──
    actions.append(
        Node(
            package="go2w_control",
            executable="cmd_vel_activity_mux.py",
            namespace=robot_ns,
            name="cmd_vel_activity_mux",
            parameters=[
                {
                    "auto_topic": f"/{robot_ns}/cmd_vel_auto",
                    "manual_topic": f"/{robot_ns}/cmd_vel_manual",
                    "output_topic": "/cmd_vel",
                    "status_topic": f"/{robot_ns}/control_source",
                    "publish_rate": 20.0,
                    "manual_timeout_sec": float(_get(context, "manual_timeout_sec")),
                    "auto_timeout_sec": float(_get(context, "auto_timeout_sec")),
                    "linear_activity_threshold": float(_get(context, "manual_linear_threshold")),
                    "angular_activity_threshold": float(_get(context, "manual_angular_threshold")),
                }
            ],
            output="screen",
        )
    )

    # ── Real-specific: cmd_vel → Sport API bridge ──
    use_obstacle_avoidance = _as_bool(_get(context, "obstacle_avoidance"))
    actions.append(
        Node(
            package="go2w_control",
            executable="cmd_vel_to_sport_bridge.py",
            name="cmd_vel_to_sport_bridge",
            parameters=[
                {"cmd_vel_topic": "/cmd_vel"},
                {"sport_topic": "/api/sport/request"},
                {"obstacle_avoidance": use_obstacle_avoidance},
            ],
            output="screen",
        )
    )

    # ── Real-specific: Optional joystick manual fallback ──
    if enable_manual_fallback:
        actions.extend(
            [
                Node(
                    package="joy",
                    executable="joy_node",
                    name="ps3_joy",
                    parameters=[
                        {
                            "dev": _get(context, "joy_dev"),
                            "deadzone": float(_get(context, "joy_deadzone")),
                            "autorepeat_rate": float(_get(context, "joy_autorepeat_rate")),
                        }
                    ],
                    output="screen",
                ),
                Node(
                    package="teleop_twist_joy",
                    executable="teleop_node",
                    namespace=robot_ns,
                    name="teleop_twist_joy_node",
                    parameters=[teleop_config, {"publish_stamped_twist": False}],
                    remappings=[
                        ("/joy", "/joy"),
                        ("/cmd_vel", f"/{robot_ns}/cmd_vel_manual"),
                    ],
                    output="screen",
                ),
            ]
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="robot"),
            DeclareLaunchArgument("external_mapper", default_value="false"),
            DeclareLaunchArgument("map_backend", default_value="scan"),
            DeclareLaunchArgument("waypoint_input_suffix", default_value="/way_point_coord"),
            DeclareLaunchArgument("enable_manual_fallback", default_value="true"),
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("joy_deadzone", default_value="0.12"),
            DeclareLaunchArgument("joy_autorepeat_rate", default_value="20.0"),
            DeclareLaunchArgument("manual_timeout_sec", default_value="0.35"),
            DeclareLaunchArgument("auto_timeout_sec", default_value="0.60"),
            DeclareLaunchArgument("manual_linear_threshold", default_value="0.02"),
            DeclareLaunchArgument("manual_angular_threshold", default_value="0.05"),
            DeclareLaunchArgument(
                "obstacle_avoidance",
                default_value="true",
                description="Use Unitree built-in obstacle avoidance (api_id=1003)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
