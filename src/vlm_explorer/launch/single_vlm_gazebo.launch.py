#!/usr/bin/env python3
"""Single Go2W VLM-in-the-loop Gazebo exploration launch with Cartographer SLAM.

Composes:
  1. Existing single_go2w_gazebo_cfpa2.launch.py (Gazebo + Go2W + CFPA2 nav)
     - with enable_slam=false so we provide SLAM via Cartographer
  2. Cartographer 3D SLAM (replaces ground truth odom relay)
  3. VLM explorer layer (skeleton extractor, map renderer, green detector, VLM coordinator)

Usage:
  ros2 launch vlm_explorer single_vlm_gazebo.launch.py
  ros2 launch vlm_explorer single_vlm_gazebo.launch.py vlm_model:=gpt-4o-mini
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, key: str) -> str:
    return LaunchConfiguration(key).perform(context)


def _launch_setup(context):
    use_sim_time = _as_bool(_get(context, "use_sim_time"))
    vlm_enabled = _as_bool(_get(context, "vlm_enabled"))
    vlm_model = _get(context, "vlm_model").strip()
    vlm_replan_sec = float(_get(context, "vlm_replan_sec"))
    vlm_delay = float(_get(context, "vlm_delay"))
    gui = _get(context, "gui")
    rviz = _get(context, "rviz")
    robot_ns = _get(context, "robot_namespace").strip().strip("/") or "robot"

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    vlm_pkg = get_package_share_directory("vlm_explorer")

    world = _get(context, "world").strip()
    if not world:
        world = os.path.join(vlm_pkg, "worlds", "vlm_exploration.world")

    tf_remaps = [("/tf", f"/{robot_ns}/tf"), ("/tf_static", f"/{robot_ns}/tf_static")]
    carto_cfg_dir = os.path.join(vlm_pkg, "config")

    actions = []

    # ── 1. Base launch: Gazebo + Go2W + CFPA2 (SLAM disabled) ────────
    #    We disable SLAM so the base launch doesn't start gt_odom_relay.
    #    Cartographer will provide /{ns}/odom/nav instead.
    single_launch = os.path.join(
        go2_gazebo_pkg, "launch", "single_go2w_gazebo_cfpa2.launch.py"
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(single_launch),
            launch_arguments={
                "robot_namespace": robot_ns,
                "use_sim_time": str(use_sim_time).lower(),
                "gui": gui,
                "rviz": "false",
                "cleanup_stale": "false",
                "enable_slam": "false",  # We provide SLAM via Cartographer
                "use_fast_lio": "false",
                "world": world,
                "spawn_x": _get(context, "spawn_x"),
                "spawn_y": _get(context, "spawn_y"),
                "spawn_yaw": _get(context, "spawn_yaw"),
                "cfpa2_w_ig": _get(context, "cfpa2_w_ig"),
                "cfpa2_w_c": _get(context, "cfpa2_w_c"),
                "cfpa2_w_momentum": _get(context, "cfpa2_w_momentum"),
                "cfpa2_min_utility": _get(context, "cfpa2_min_utility"),
            }.items(),
        )
    )

    # ── RViz with camera + VLM displays ──────────────────────────────
    if _as_bool(rviz):
        rviz_config = os.path.join(vlm_pkg, "rviz", "single_vlm_gazebo.rviz")
        actions.append(
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2_vlm",
                        arguments=["-d", rviz_config],
                        parameters=[{"use_sim_time": use_sim_time}],
                        output="screen",
                    )
                ],
            )
        )

    # ── 2. Cartographer 3D SLAM ──────────────────────────────────────
    # CRITICAL: Start AFTER pose guard finishes to avoid teleportation
    # jumps corrupting the SLAM map.
    #   assets spawn @5s, pose_guard_hold=12s → guard ends @~17s
    #   robot_actions (nav/control) start @16s
    # Start Cartographer at 20s: pose guard done, sensors streaming,
    # robot is standing still — clean initial scan for first submap.
    carto_delay = 20.0

    carto_nodes = []

    # Cartographer node
    carto_nodes.append(
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            namespace=robot_ns,
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "-configuration_directory", carto_cfg_dir,
                "-configuration_basename", "cartographer_sim_3d.lua",
            ],
            remappings=tf_remaps + [
                ("points2", f"/{robot_ns}/registered_scan_reliable"),
                ("imu", f"/{robot_ns}/imu/data"),
            ],
            output="screen",
        )
    )

    # No cartographer_occupancy_grid_node needed — the base launch's
    # simple_scan_mapper_cpp builds /{ns}/map from the 2D scan + odom.
    # Cartographer just provides SLAM odom; the mapper handles the grid.

    # carto_odom_bridge: convert Cartographer TF (map→imu) to Odometry
    # The nav stack expects /{ns}/odom/nav with frame_id=world, child=base_link
    carto_nodes.append(
        Node(
            package="go2w_perception",
            executable="carto_odom_bridge.py",
            name="carto_odom_bridge",
            namespace=robot_ns,
            parameters=[
                {"use_sim_time": use_sim_time},
                {"parent_frame": "map"},
                {"child_frame": "imu"},
                {"output_topic": f"/{robot_ns}/odom/nav"},
                {"output_frame_id": "world"},
                {"output_child_frame_id": "base_link"},
                {"rate": 50.0},
            ],
            remappings=tf_remaps,
            output="screen",
        )
    )

    actions.append(TimerAction(period=carto_delay, actions=carto_nodes))

    # ── 3. VLM nodes — delayed until Cartographer + nav stabilize ────
    map_topic = f"/{robot_ns}/map"
    vlm_nodes = []

    vlm_nodes.append(
        Node(
            package="vlm_explorer",
            executable="skeleton_extractor_node",
            name="skeleton_extractor",
            parameters=[
                {"use_sim_time": use_sim_time},
                {
                    "map_topic": map_topic,
                    "skeleton_marker_topic": "/vlm/skeleton_markers",
                    "skeleton_image_topic": "/vlm/skeleton_image",
                    "frame_id": "world",
                    "rate": 1.0,
                    "free_threshold": 50,
                    "downsample": 2,
                },
            ],
            output="screen",
        )
    )

    vlm_nodes.append(
        Node(
            package="vlm_explorer",
            executable="map_renderer_node",
            name="map_renderer",
            parameters=[
                {"use_sim_time": use_sim_time},
                {
                    "map_topic": map_topic,
                    "robot_namespaces": [robot_ns],
                    "skeleton_image_topic": "/vlm/skeleton_image",
                    "green_detections_topic": "/vlm/green_detections",
                    "rendered_map_topic": "/vlm/rendered_map",
                    "scene_json_topic": "/vlm/scene_json",
                    "frame_id": "world",
                    "rate": 1.0,
                },
            ],
            output="screen",
        )
    )

    vlm_nodes.append(
        Node(
            package="vlm_explorer",
            executable="green_marker_detector_node",
            name="green_marker_detector",
            parameters=[
                {"use_sim_time": use_sim_time},
                {
                    "robot_namespaces": [robot_ns],
                    "detections_topic": "/vlm/green_detections",
                    "rate": 2.0,
                    "hsv_h_low": 35,
                    "hsv_h_high": 85,
                    "hsv_s_low": 80,
                    "hsv_v_low": 80,
                    "min_blob_pixels": 200,
                    "assumed_depth_m": 2.0,
                    "camera_hfov_rad": 2.0944,
                    "dedup_radius_m": 0.8,
                },
            ],
            output="screen",
        )
    )

    vlm_nodes.append(
        Node(
            package="vlm_explorer",
            executable="vlm_coordinator_node",
            name="vlm_coordinator",
            parameters=[
                {"use_sim_time": use_sim_time},
                {
                    "robot_namespaces": [robot_ns],
                    "rendered_map_topic": "/vlm/rendered_map",
                    "scene_json_topic": "/vlm/scene_json",
                    "green_detections_topic": "/vlm/green_detections",
                    "goal_topic_suffix": "/vlm_way_point",
                    "frame_id": "world",
                    "replan_period_sec": vlm_replan_sec,
                    "vlm_enabled": vlm_enabled,
                    "vlm_model": vlm_model,
                    "vlm_temperature": 0.2,
                    "vlm_max_tokens": 1024,
                    "vlm_max_retries": 3,
                    "green_reach_radius_m": 1.0,
                },
            ],
            output="screen",
        )
    )

    actions.append(TimerAction(period=vlm_delay, actions=vlm_nodes))

    return actions


def generate_launch_description():
    vlm_pkg = get_package_share_directory("vlm_explorer")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="robot"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "vlm_enabled",
                default_value="true",
                description="Enable VLM queries (needs OPENAI_API_KEY). Falls back to dummy planner if unset.",
            ),
            DeclareLaunchArgument("vlm_model", default_value="gpt-4o"),
            DeclareLaunchArgument("vlm_replan_sec", default_value="20.0"),
            DeclareLaunchArgument(
                "vlm_delay",
                default_value="35.0",
                description="Seconds to wait before starting VLM nodes (let Cartographer + nav stabilize)",
            ),
            DeclareLaunchArgument("spawn_x", default_value="4.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("cfpa2_w_ig", default_value="0.5"),
            DeclareLaunchArgument("cfpa2_w_c", default_value="0.8"),
            DeclareLaunchArgument("cfpa2_w_momentum", default_value="2.5"),
            DeclareLaunchArgument("cfpa2_min_utility", default_value="-1.0"),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(vlm_pkg, "worlds", "vlm_exploration.world"),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
