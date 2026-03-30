#!/usr/bin/env python3
"""Dual real-robot CFPA2: two namespaced stacks + shared map fuser + coordinator.

Expects per-robot Cartographer (and optional Octomap) already running in separate
process groups with CYCLONEDDS bound to that robot's NIC. Those stacks must publish:
  /{ns}/utlidar/transformed_cloud
  TF: {ns}_map -> {ns}_odom -> {ns}_body

Sport API bridges (cmd_vel -> /api/sport/request) should run with the same DDS
profile as each robot so commands do not fan out to both units.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text: str) -> bool:
    return str(text).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _resolve_variant(arg_value: str, env_key: str, default: str) -> str:
    v = str(arg_value).strip().lower()
    if v == "auto":
        v = os.environ.get(env_key, default).strip().lower()
    if v not in {"go2", "go2w"}:
        return default
    return v


def _robot_frames(namespace: str) -> tuple[str, str, str]:
    ns = namespace.strip().strip("/")
    return (f"{ns}_map", f"{ns}_body", f"{ns}_base_link")


def _reactive_nav_yaml(variant: str) -> str:
    go2w_control_pkg = get_package_share_directory("go2w_control")
    if variant == "go2w":
        return os.path.join(go2w_control_pkg, "config", "reactive_nav_single_go2w.yaml")
    return os.path.join(go2w_control_pkg, "config", "reactive_nav_single.yaml")


def _teleop_yaml(_variant: str) -> str:
    # Only go2w teleop YAML is checked in; legged Go2 uses the same joystick layout.
    go2w_control_pkg = get_package_share_directory("go2w_control")
    return os.path.join(go2w_control_pkg, "config", "teleop_twist_joy_go2w.yaml")


def _add_robot_stack(
    *,
    namespace: str,
    variant: str,
    external_mapper: bool,
    obstacle_avoidance: bool,
    enable_manual: bool,
    joy_dev: str,
    waypoint_input_suffix: str,
) -> list:
    ns = namespace.strip().strip("/")
    map_frame, body_frame, base_frame = _robot_frames(ns)
    go2w_control_pkg = get_package_share_directory("go2w_control")
    nav_pkg = get_package_share_directory("go2_nav_algorithms")
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")

    reactive_nav_profile = _reactive_nav_yaml(variant)
    teleop_config = _teleop_yaml(variant)
    mapper_profile = os.path.join(go2_gazebo_pkg, "config", "nav", "simple_scan_mapper_single_go2w.yaml")

    if not waypoint_input_suffix.startswith("/"):
        waypoint_input_suffix = "/" + waypoint_input_suffix

    actions: list = []

    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"{ns}_body_to_base_link",
            arguments=[
                "--frame-id",
                body_frame,
                "--child-frame-id",
                base_frame,
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--qx",
                "0",
                "--qy",
                "0",
                "--qz",
                "0",
                "--qw",
                "1",
            ],
            parameters=[{"use_sim_time": False}],
            output="log",
        )
    )

    actions.append(
        Node(
            package="go2w_perception",
            executable="carto_odom_bridge.py",
            namespace=ns,
            name="carto_odom_bridge",
            parameters=[
                {
                    "parent_frame": map_frame,
                    "child_frame": body_frame,
                    "output_topic": f"/{ns}/odom/nav",
                    "output_frame_id": map_frame,
                    "output_child_frame_id": base_frame,
                    "rate": 50.0,
                }
            ],
            output="screen",
        )
    )

    actions.append(
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            namespace=ns,
            name="pointcloud_to_laserscan",
            parameters=[
                {
                    "use_sim_time": False,
                    "target_frame": base_frame,
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
                ("cloud_in", f"/{ns}/utlidar/transformed_cloud"),
                ("scan", f"/{ns}/scan_3d_raw"),
            ],
            output="screen",
        )
    )

    actions.append(
        Node(
            package="go2_nav_algorithms",
            executable="scan_rear_filter",
            namespace=ns,
            name="scan_rear_filter",
            parameters=[{"rear_blank_radius": 0.45}],
            remappings=[
                ("scan_in", f"/{ns}/scan_3d_raw"),
                ("scan_out", f"/{ns}/scan_3d"),
            ],
            output="screen",
        )
    )

    if not external_mapper:
        actions.append(
            Node(
                package="go2_nav_algorithms",
                executable="simple_scan_mapper_cpp",
                namespace=ns,
                name="simple_scan_mapper_cpp",
                parameters=[
                    os.path.join(nav_pkg, "config", "nav", "geometric_frontier_single.yaml"),
                    mapper_profile,
                    {
                        "use_sim_time": False,
                        "scan_topic": f"/{ns}/scan_3d",
                        "odom_topic": f"/{ns}/odom/nav",
                        "map_topic": f"/{ns}/map",
                        "map_frame": map_frame,
                        "broadcast_tf": False,
                        "startup_delay": 0.0,
                        "scan_frame": base_frame,
                    },
                ],
                output="screen",
            )
        )

    # Global frontier assignment is cfpa2_coordinator_node (not cfpa2_single_robot_node).

    actions.append(
        Node(
            package="go2w_nav",
            executable="reactive_nav.py",
            namespace=ns,
            name="reactive_nav",
            parameters=[
                reactive_nav_profile,
                {"use_sim_time": False},
                {
                    "max_linear_speed": 0.30,
                    "require_settle_before_motion": False,
                    "frontier_replan_topic": f"/{ns}/frontier_replan",
                    "stop_topic": f"/{ns}/stop",
                    "map_topic": f"/{ns}/map",
                },
            ],
            remappings=[
                ("/way_point", f"/{ns}{waypoint_input_suffix}"),
                ("/odom/ground_truth", f"/{ns}/odom/nav"),
                ("/scan", f"/{ns}/scan_3d"),
                ("/cmd_vel_stamped", f"/{ns}/cmd_vel_stamped"),
                ("/nav_status", f"/{ns}/nav_status"),
                ("/planned_path", f"/{ns}/planned_path"),
                ("/robot_trajectory", f"/{ns}/robot_trajectory"),
                ("/final_goal_marker", f"/{ns}/final_goal_marker"),
            ],
            output="screen",
        )
    )

    actions.append(
        Node(
            package="go2w_perception",
            executable="twist_bridge.py",
            namespace=ns,
            name="twist_bridge",
            remappings=[
                ("cmd_vel_stamped", f"/{ns}/cmd_vel_stamped"),
                ("cmd_vel", f"/{ns}/cmd_vel_auto"),
            ],
            output="screen",
        )
    )

    actions.append(
        Node(
            package="go2w_control",
            executable="cmd_vel_activity_mux.py",
            namespace=ns,
            name="cmd_vel_activity_mux",
            parameters=[
                {
                    "auto_topic": f"/{ns}/cmd_vel_auto",
                    "manual_topic": f"/{ns}/cmd_vel_manual",
                    "output_topic": f"/{ns}/cmd_vel_to_robot",
                    "status_topic": f"/{ns}/control_source",
                    "publish_rate": 20.0,
                    "manual_timeout_sec": 0.35,
                    "auto_timeout_sec": 0.60,
                    "linear_activity_threshold": 0.02,
                    "angular_activity_threshold": 0.05,
                }
            ],
            output="screen",
        )
    )

    if enable_manual:
        # joy_node is added once in _launch_setup (shared /joy) so two robots do not open js0 twice.
        actions.append(
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                namespace=ns,
                name="teleop_twist_joy_node",
                parameters=[teleop_config, {"publish_stamped_twist": False}],
                remappings=[
                    ("/joy", "/joy"),
                    ("/cmd_vel", f"/{ns}/cmd_vel_manual"),
                ],
                output="screen",
            )
        )

    return actions


def _launch_setup(context):
    ns_a = _get(context, "robot_a_namespace").strip().strip("/") or "robot_a"
    ns_b = _get(context, "robot_b_namespace").strip().strip("/") or "robot_b"
    variant_a = _resolve_variant(_get(context, "robot_a_variant"), "ROBOT_A_VARIANT", "go2w")
    variant_b = _resolve_variant(_get(context, "robot_b_variant"), "ROBOT_B_VARIANT", "go2")
    external_mapper = _as_bool(_get(context, "external_mapper"))
    obstacle_avoidance = _as_bool(_get(context, "obstacle_avoidance"))
    enable_a = _as_bool(_get(context, "enable_manual_fallback_robot_a"))
    enable_b = _as_bool(_get(context, "enable_manual_fallback_robot_b"))
    joy_dev = _get(context, "joy_dev")
    waypoint_suffix = _get(context, "waypoint_input_suffix").strip() or "/way_point_coord"
    use_shared_map = _as_bool(_get(context, "use_shared_map"))
    shared_map_topic = _get(context, "shared_map_topic").strip() or "/disco_slam/global_map"
    shared_map_frame = _get(context, "shared_map_frame_id").strip()
    map_a_topic = _get(context, "shared_map_input_a").strip() or f"/{ns_a}/map"
    map_b_topic = _get(context, "shared_map_input_b").strip() or f"/{ns_b}/map"

    cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")
    coord_cfg = os.path.join(cfpa2_pkg, "config", "cfpa2_coordinator.yaml")
    map_a_frame, _, _ = _robot_frames(ns_a)

    actions: list = []
    if enable_a or enable_b:
        actions.append(
            Node(
                package="joy",
                executable="joy_node",
                name="shared_joy",
                parameters=[
                    {
                        "dev": joy_dev,
                        "deadzone": 0.12,
                        "autorepeat_rate": 20.0,
                    }
                ],
                output="screen",
            )
        )

    actions.extend(
        _add_robot_stack(
            namespace=ns_a,
            variant=variant_a,
            external_mapper=external_mapper,
            obstacle_avoidance=obstacle_avoidance,
            enable_manual=enable_a,
            joy_dev=joy_dev,
            waypoint_input_suffix=waypoint_suffix,
        )
    )
    actions.extend(
        _add_robot_stack(
            namespace=ns_b,
            variant=variant_b,
            external_mapper=external_mapper,
            obstacle_avoidance=obstacle_avoidance,
            enable_manual=enable_b,
            joy_dev=joy_dev,
            waypoint_input_suffix=waypoint_suffix,
        )
    )

    if use_shared_map:
        fuser_params = {
            "use_sim_time": False,
            "map_a_topic": map_a_topic,
            "map_b_topic": map_b_topic,
            "output_topic": shared_map_topic,
            "publish_rate": 2.0,
            "frame_id": shared_map_frame,
        }
        actions.append(
            Node(
                package="go2_gazebo_sim",
                executable="shared_map_fuser.py",
                name="shared_map_fuser",
                parameters=[fuser_params],
                output="screen",
            )
        )

    actions.append(
        Node(
            package="cfpa2_collaborative_autonomy",
            executable="cfpa2_coordinator_node",
            name="cfpa2_coordinator",
            parameters=[
                coord_cfg,
                {
                    "namespaces": [ns_a, ns_b],
                    "algorithm_mode": "cfpa2",
                    "use_shared_map": use_shared_map,
                    "shared_map_topic": shared_map_topic,
                    "shared_map_wait_sec": 8.0,
                    "marker_frame_override": map_a_frame if shared_map_frame == "" else shared_map_frame,
                    "goal_topic_suffix": "/way_point_coord",
                    "output_mode": "waypoint_coord",
                },
            ],
            output="screen",
        )
    )

    # obstacle_avoidance reserved for future per-robot sport API variants (same api_id today).
    _ = obstacle_avoidance

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_a_namespace", default_value="robot_a"),
            DeclareLaunchArgument("robot_b_namespace", default_value="robot_b"),
            DeclareLaunchArgument(
                "robot_a_variant",
                default_value="auto",
                description="go2w | go2 | auto (auto uses env ROBOT_A_VARIANT, default go2w)",
            ),
            DeclareLaunchArgument(
                "robot_b_variant",
                default_value="auto",
                description="go2w | go2 | auto (auto uses env ROBOT_B_VARIANT, default go2)",
            ),
            DeclareLaunchArgument("external_mapper", default_value="false"),
            DeclareLaunchArgument("obstacle_avoidance", default_value="false"),
            DeclareLaunchArgument(
                "enable_manual_fallback_robot_a",
                default_value="true",
                description="Joystick teleop for robot A (uses /dev/input/js0)",
            ),
            DeclareLaunchArgument(
                "enable_manual_fallback_robot_b",
                default_value="false",
                description="Set true only with a second joystick device (change joy_dev)",
            ),
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("waypoint_input_suffix", default_value="/way_point_coord"),
            DeclareLaunchArgument("use_shared_map", default_value="true"),
            DeclareLaunchArgument("shared_map_topic", default_value="/disco_slam/global_map"),
            DeclareLaunchArgument(
                "shared_map_frame_id",
                default_value="",
                description="If non-empty, override fused map header frame_id",
            ),
            DeclareLaunchArgument(
                "shared_map_input_a",
                default_value="",
                description="Defaults to /{robot_a_namespace}/map if empty",
            ),
            DeclareLaunchArgument(
                "shared_map_input_b",
                default_value="",
                description="Defaults to /{robot_b_namespace}/map if empty",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
