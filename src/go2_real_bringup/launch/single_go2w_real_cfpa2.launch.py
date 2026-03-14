#!/usr/bin/env python3
"""Single Go2W real-robot CFPA2 runtime on top of Cartographer SLAM.

Assumes Cartographer is already running (via ./go2w_ethernet_start.sh cartographer),
which provides:
  - TF: map → odom → body
  - /map (OccupancyGrid) from cartographer_occupancy_grid_node
  - /utlidar/transformed_cloud (from transform_everything)

This launch file adds the navigation stack:
  carto_odom_bridge → /robot/odom/nav (Odometry from TF)
  pointcloud_to_laserscan → /robot/scan_3d (LaserScan for reactive_nav local avoidance)
  cfpa2_single_robot_node → frontier detection on /map
  reactive_nav → A* planning on /map + local avoidance on scan_3d → cmd_vel_stamped
  twist_bridge → cmd_vel_activity_mux → /cmd_vel (sport API)
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


def _launch_setup(context):
    robot_ns = _get(context, "robot_namespace").strip().strip("/") or "robot"
    enable_manual_fallback = _as_bool(_get(context, "enable_manual_fallback"))

    go2w_control_pkg = get_package_share_directory("go2w_control")
    cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")

    # Planning configs — match demo.sh pipeline
    reactive_nav_profile = os.path.join(go2w_control_pkg, "config", "reactive_nav_single_go2w.yaml")
    cfpa2_config = os.path.join(cfpa2_pkg, "config", "cfpa2_single_robot.yaml")
    teleop_config = os.path.join(go2w_control_pkg, "config", "teleop_twist_joy_go2w.yaml")

    actions = []

    # ── Static TF: body → base_link (identity) ──
    # Needed by pointcloud_to_laserscan to transform cloud from body frame
    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="body_to_base_link",
            arguments=["--frame-id", "body", "--child-frame-id", "base_link",
                        "--x", "0", "--y", "0", "--z", "0",
                        "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1"],
            parameters=[{"use_sim_time": False}],
            output="log",
        )
    )

    # ── Cartographer TF → Odometry bridge ──
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

    # ── PointCloud → LaserScan (for reactive_nav local avoidance) ──
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
                    "min_height": 0.05,
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
                ("scan", f"/{robot_ns}/scan_3d"),
            ],
            output="screen",
        )
    )

    # ── 2D Occupancy Grid Mapper (ray-traced free cells for frontier detection) ──
    # Cartographer's 3D occupancy grid is an x-ray projection with zero free cells.
    # simple_scan_mapper_cpp builds a proper 2D grid from LaserScan + TF.
    nav_pkg = get_package_share_directory("go2_nav_algorithms")
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    mapper_profile = os.path.join(go2_gazebo_pkg, "config", "nav", "simple_scan_mapper_single_go2w.yaml")
    actions.append(
        Node(
            package="go2_nav_algorithms",
            executable="simple_scan_mapper_cpp",
            namespace=robot_ns,
            name="simple_scan_mapper_cpp",
            parameters=[
                os.path.join(nav_pkg, "config", "nav", "geometric_frontier_single.yaml"),
                mapper_profile,
                {
                    "use_sim_time": False,
                    "scan_topic": f"/{robot_ns}/scan_3d",
                    "odom_topic": f"/{robot_ns}/odom/nav",
                    "map_topic": f"/{robot_ns}/map",
                    "map_frame": "map",
                    "broadcast_tf": False,  # Cartographer provides TF; don't conflict
                    "startup_delay": 0.0,
                    "scan_frame": "base_link",
                },
            ],
            output="screen",
        )
    )

    # ── CFPA2 Frontier Exploration (weights match demo.sh) ──
    actions.append(
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
                    # demo.sh weights
                    "cfpa2_w_ig": 0.5,
                    "cfpa2_w_c": 0.8,
                    "cfpa2_w_momentum": 2.5,
                    "cfpa2_min_utility": -1.0,
                },
            ],
            # CFPA2 subscribes to /{ns}/map which simple_scan_mapper_cpp publishes directly
            output="screen",
        )
    )

    # ── Reactive Navigation (same planner params as demo.sh, capped speed for real robot) ──
    actions.append(
        Node(
            package="go2w_control",
            executable="reactive_nav.py",
            namespace=robot_ns,
            name="reactive_nav",
            parameters=[
                reactive_nav_profile,
                {"use_sim_time": False},
                {
                    # Real-robot overrides
                    "max_linear_speed": 0.15,  # Start conservative, increase after testing
                    "require_settle_before_motion": False,  # Carto pose jitter defeats settle gate
                    "frontier_replan_topic": f"/{robot_ns}/frontier_replan",
                    "stop_topic": f"/{robot_ns}/stop",
                    # Use simple_scan_mapper's /robot/map for planning (not Carto's /map)
                    "map_topic": f"/{robot_ns}/map",
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
        )
    )

    # ── Twist Bridge: TwistStamped → Twist ──
    actions.append(
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
        )
    )

    # ── Auto/Manual velocity mux → /cmd_vel (sport API) ──
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

    # ── cmd_vel → Sport API bridge (Go2 doesn't natively subscribe to /cmd_vel) ──
    actions.append(
        Node(
            package="go2w_control",
            executable="cmd_vel_to_sport_bridge.py",
            name="cmd_vel_to_sport_bridge",
            parameters=[
                {"cmd_vel_topic": "/cmd_vel"},
                {"sport_topic": "/api/sport/request"},
            ],
            output="screen",
        )
    )

    # ── Optional: Joystick manual fallback ──
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
            DeclareLaunchArgument("enable_manual_fallback", default_value="true"),
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("joy_deadzone", default_value="0.12"),
            DeclareLaunchArgument("joy_autorepeat_rate", default_value="20.0"),
            DeclareLaunchArgument("manual_timeout_sec", default_value="0.35"),
            DeclareLaunchArgument("auto_timeout_sec", default_value="0.60"),
            DeclareLaunchArgument("manual_linear_threshold", default_value="0.02"),
            DeclareLaunchArgument("manual_angular_threshold", default_value="0.05"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
