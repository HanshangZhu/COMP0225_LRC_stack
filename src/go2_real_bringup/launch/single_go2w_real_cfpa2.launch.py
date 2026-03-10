#!/usr/bin/env python3
"""Single Go2W real-robot CFPA2 runtime with reactive navigation and joystick fallback."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def _as_bool(text: str) -> bool:
    return str(text).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _launch_setup(context):
    robot_ns = _get(context, "robot_namespace").strip().strip("/") or "robot"
    rviz = _as_bool(_get(context, "rviz"))
    enable_manual_fallback = _as_bool(_get(context, "enable_manual_fallback"))

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    go2w_bringup_pkg = get_package_share_directory("go2w_bringup")
    point_lio_pkg = get_package_share_directory("point_lio_unilidar")
    cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")
    go2_nav_pkg = get_package_share_directory("go2_nav_algorithms")

    go2w_control_pkg = get_package_share_directory("go2w_control")
    reactive_nav_profile = os.path.join(go2w_control_pkg, "config", "reactive_nav_real_go2w.yaml")
    teleop_config = os.path.join(go2w_control_pkg, "config", "teleop_twist_joy_go2w.yaml")
    cfpa2_config = os.path.join(cfpa2_pkg, "config", "cfpa2_single_robot.yaml")
    mapper_profile = os.path.join(go2_nav_pkg, "config", "nav", "geometric_frontier_single.yaml")

    go2w_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(go2w_bringup_pkg, "launch", "go2w.launch.py")),
        launch_arguments={"rviz": "true" if rviz else "false", "sim": "false"}.items(),
    )

    slam_backend = GroupAction(
        [
            SetRemap(src="/registered_scan", dst=f"/{robot_ns}/registered_scan"),
            SetRemap(src="/state_estimation", dst=f"/{robot_ns}/state_estimation"),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(os.path.join(point_lio_pkg, "launch", "mapping_utlidar.launch")),
                launch_arguments={"rviz": "false"}.items(),
            ),
        ]
    )

    actions = [
        go2w_bringup,
        slam_backend,
        Node(
            package="go2w_control",
            executable="go2w_startup_mode.py",
            name="go2w_startup_mode",
            parameters=[
                {
                    "switch_joystick_service": "/switch_joystick",
                    "mode_service": "/mode",
                    "startup_mode": _get(context, "startup_mode"),
                    "call_switch_joystick": True,
                    "switch_joystick_flag": False,
                    "wait_timeout_sec": float(_get(context, "startup_wait_timeout_sec")),
                }
            ],
            output="screen",
        ),
        Node(
            package="go2w_perception",
            executable="slam_odom_relay.py",
            namespace=robot_ns,
            name="slam_odom_relay",
            parameters=[
                {
                    "use_sim_time": False,
                    "input_topic": f"/{robot_ns}/state_estimation",
                    "output_topic": f"/{robot_ns}/odom/nav",
                    "output_frame_id": "map",
                    "output_child_frame_id": "base_link",
                    "bootstrap_from_gt": False,
                }
            ],
            output="screen",
        ),
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
                    "min_height": 0.05,
                    "max_height": 0.70,
                    "angle_min": -3.14159,
                    "angle_max": 3.14159,
                    "angle_increment": 0.006135923151543,
                    "scan_time": 0.1,
                    "range_min": 0.2,
                    "range_max": 20.0,
                    "use_inf": True,
                }
            ],
            remappings=[
                ("cloud_in", f"/{robot_ns}/registered_scan"),
                ("scan", f"/{robot_ns}/scan_3d"),
            ],
            output="screen",
        ),
        Node(
            package="go2_nav_algorithms",
            executable="simple_scan_mapper_cpp",
            namespace=robot_ns,
            name="simple_scan_mapper_cpp",
            parameters=[
                mapper_profile,
                {"use_sim_time": False},
                {
                    "scan_topic": f"/{robot_ns}/scan_3d",
                    "odom_topic": f"/{robot_ns}/odom/nav",
                    "map_topic": f"/{robot_ns}/map",
                    "map_frame": "map",
                    "startup_delay": 0.0,
                    "max_scan_odom_dt": 0.10,
                    "max_range": 12.0,
                    "max_clear_distance": 2.0,
                },
            ],
            output="screen",
        ),
        Node(
            package="cfpa2_collaborative_autonomy",
            executable="cfpa2_single_robot_node",
            name="cfpa2_single_robot",
            parameters=[
                cfpa2_config,
                {
                    "robot_namespace": robot_ns,
                    "namespaces": [robot_ns],
                    "goal_topic_suffix": "/way_point_coord",
                    "marker_frame_override": "map",
                },
            ],
            output="screen",
        ),
        Node(
            package="go2w_control",
            executable="reactive_nav.py",
            namespace=robot_ns,
            name="reactive_nav",
            parameters=[
                reactive_nav_profile,
                {"use_sim_time": False},
                {
                    "frontier_replan_topic": f"/{robot_ns}/frontier_replan",
                    "stop_topic": f"/{robot_ns}/stop",
                },
            ],
            remappings=[
                ("/way_point", f"/{robot_ns}/way_point_coord"),
                ("/odom/ground_truth", f"/{robot_ns}/odom/nav"),
                ("/scan", f"/{robot_ns}/scan_3d"),
                ("/cmd_vel_stamped", f"/{robot_ns}/cmd_vel_stamped"),
                ("/nav_status", f"/{robot_ns}/nav_status"),
                ("/planned_path", f"/{robot_ns}/planned_path"),
                ("/robot_trajectory", f"/{robot_ns}/robot_trajectory"),
                ("/final_goal_marker", f"/{robot_ns}/final_goal_marker"),
            ],
            output="screen",
        ),
        Node(
            package="go2w_perception",
            executable="twist_bridge.py",
            namespace=robot_ns,
            name="twist_bridge",
            remappings=[
                ("/cmd_vel_stamped", f"/{robot_ns}/cmd_vel_stamped"),
                ("/cmd_vel", f"/{robot_ns}/cmd_vel_auto"),
            ],
            output="screen",
        ),
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
        ),
    ]

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
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("enable_manual_fallback", default_value="true"),
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("joy_deadzone", default_value="0.12"),
            DeclareLaunchArgument("joy_autorepeat_rate", default_value="20.0"),
            DeclareLaunchArgument("manual_timeout_sec", default_value="0.35"),
            DeclareLaunchArgument("auto_timeout_sec", default_value="0.60"),
            DeclareLaunchArgument("manual_linear_threshold", default_value="0.02"),
            DeclareLaunchArgument("manual_angular_threshold", default_value="0.05"),
            DeclareLaunchArgument("startup_mode", default_value="stand_up"),
            DeclareLaunchArgument("startup_wait_timeout_sec", default_value="30.0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
