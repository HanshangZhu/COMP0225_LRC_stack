#!/usr/bin/env python3
"""Single Go2W real-robot runtime with TARE waypoint wiring.

This launch composes the existing CFPA2 real stack and then inserts:
  - tare_planner_node: /<ns>/way_point_seed -> /<ns>/way_point_tare
  - waypoint_mux: prefer /<ns>/way_point_tare, fallback /<ns>/way_point_coord
                 and publish /<ns>/way_point_coord_nav

Reactive nav consumes /<ns>/way_point_coord_nav via included launch override.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _launch_setup(context):
    robot_ns = _get(context, "robot_namespace").strip().strip("/") or "robot"

    go2_real_pkg = get_package_share_directory("go2_real_bringup")
    base_launch = os.path.join(go2_real_pkg, "launch", "single_go2w_real_cfpa2.launch.py")

    waypoint_seed_suffix = _get(context, "tare_seed_input_suffix").strip() or "/way_point_coord"
    if not waypoint_seed_suffix.startswith("/"):
        waypoint_seed_suffix = "/" + waypoint_seed_suffix

    tare_output_suffix = _get(context, "tare_output_suffix").strip() or "/way_point_tare"
    if not tare_output_suffix.startswith("/"):
        tare_output_suffix = "/" + tare_output_suffix

    waypoint_nav_suffix = _get(context, "tare_mux_output_suffix").strip() or "/way_point_coord_nav"
    if not waypoint_nav_suffix.startswith("/"):
        waypoint_nav_suffix = "/" + waypoint_nav_suffix

    include_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            "robot_namespace": robot_ns,
            "external_mapper": _get(context, "external_mapper"),
            "waypoint_input_suffix": waypoint_nav_suffix,
            "enable_manual_fallback": _get(context, "enable_manual_fallback"),
            "joy_dev": _get(context, "joy_dev"),
            "joy_deadzone": _get(context, "joy_deadzone"),
            "joy_autorepeat_rate": _get(context, "joy_autorepeat_rate"),
            "manual_timeout_sec": _get(context, "manual_timeout_sec"),
            "auto_timeout_sec": _get(context, "auto_timeout_sec"),
            "manual_linear_threshold": _get(context, "manual_linear_threshold"),
            "manual_angular_threshold": _get(context, "manual_angular_threshold"),
            "obstacle_avoidance": _get(context, "obstacle_avoidance"),
        }.items(),
    )

    tare_planner_node = Node(
        package="go2_tare_planner_ros2",
        executable="tare_planner_node",
        name="tare_planner_node",
        parameters=[
            {"use_sim_time": False},
            {"namespaces": [robot_ns]},
            {"input_topic_suffix": waypoint_seed_suffix},
            {"output_topic_suffix": tare_output_suffix},
            {"output_rate_hz": float(_get(context, "tare_output_rate_hz"))},
        ],
        output="screen",
    )

    tare_waypoint_mux = Node(
        package="go2_gazebo_sim",
        executable="waypoint_mux.py",
        name="tare_waypoint_mux",
        parameters=[
            {"use_sim_time": False},
            {"namespaces": [robot_ns]},
            {"primary_input_suffix": tare_output_suffix},
            {"fallback_input_suffix": "/way_point_coord"},
            {"output_suffix": waypoint_nav_suffix},
            {"primary_timeout_sec": float(_get(context, "tare_primary_timeout_sec"))},
            {"output_rate": float(_get(context, "tare_mux_output_rate_hz"))},
            {"hold_last_output": True},
            {"stamp_now": True},
        ],
        output="screen",
    )

    return [include_base, tare_planner_node, tare_waypoint_mux]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="robot"),
            DeclareLaunchArgument("external_mapper", default_value="false"),
            DeclareLaunchArgument("enable_manual_fallback", default_value="true"),
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("joy_deadzone", default_value="0.12"),
            DeclareLaunchArgument("joy_autorepeat_rate", default_value="20.0"),
            DeclareLaunchArgument("manual_timeout_sec", default_value="0.35"),
            DeclareLaunchArgument("auto_timeout_sec", default_value="0.60"),
            DeclareLaunchArgument("manual_linear_threshold", default_value="0.02"),
            DeclareLaunchArgument("manual_angular_threshold", default_value="0.05"),
            DeclareLaunchArgument("obstacle_avoidance", default_value="false"),
            DeclareLaunchArgument("tare_seed_input_suffix", default_value="/way_point_coord"),
            DeclareLaunchArgument("tare_output_suffix", default_value="/way_point_tare"),
            DeclareLaunchArgument("tare_mux_output_suffix", default_value="/way_point_coord_nav"),
            DeclareLaunchArgument("tare_primary_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument("tare_output_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("tare_mux_output_rate_hz", default_value="8.0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
