"""CMU autonomy stack wiring for single-robot Gazebo runs."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))

from _stack_components import (
    build_autonomy_enabler_node,
    build_frontier_recovery_node,
    build_gazebo_frontier_visual_node,
    build_geometric_frontier_node,
    build_goalpoint_bridge_node,
    build_motion_monitor_node,
    build_reactive_nav_node,
    build_wall_checker_node,
)


def generate_launch_description():
    """Launch CMU autonomy stack adapted for Gazebo."""

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    enable_far_planner = LaunchConfiguration("enable_far_planner", default="false")
    enable_simple_frontier = LaunchConfiguration("enable_simple_frontier", default="false")
    enable_geometric_frontier = LaunchConfiguration("enable_geometric_frontier", default="true")
    enable_gazebo_frontier_visual = LaunchConfiguration("enable_gazebo_frontier_visual", default="true")

    try:
        local_planner_pkg = get_package_share_directory("local_planner")
    except Exception:
        local_planner_pkg = ""

    terrain_analysis_node = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrain_analysis",
        parameters=[
            {
                "use_sim_time": True,
                "scanVoxelSize": 0.1,
                "decayTime": 2.0,
                "noDecayDis": 4.0,
                "clearingDis": 8.0,
                "useSorting": True,
                "quantileZ": 0.25,
                "vehicleHeight": 0.4,
                "terrainVoxelSize": 0.05,
                "terrainVoxelHalfWidth": 100,
                "noDataObstacle": False,
                "clearDyObs": True,
                "terrainFrame": "odom",
                "minRelZ": -0.3,
                "maxRelZ": 1.0,
            }
        ],
        remappings=[("/state_estimation", "/odom/ground_truth"), ("/registered_scan", "/registered_scan_reliable")],
        output="screen",
    )

    local_planner_node = Node(
        package="local_planner",
        executable="localPlanner",
        name="local_planner",
        parameters=[
            {
                "use_sim_time": True,
                "pathFolder": os.path.join(local_planner_pkg, "paths") if local_planner_pkg else "",
                "vehicleLength": 0.6,
                "vehicleWidth": 0.35,
                "sensorOffsetX": 0.0,
                "sensorOffsetY": 0.0,
                "twoWayDrive": True,
                "laserVoxelSize": 0.05,
                "terrainVoxelSize": 0.05,
                "useTerrainAnalysis": False,
                "checkObstacle": True,
                "adjacentRange": 4.0,
                "obstacleHeightThre": 0.15,
                "groundHeightThre": 0.15,
                "maxSpeed": 1.0,
                "autonomyMode": True,
                "autonomySpeed": 1.0,
                "goalClearRange": 0.5,
            }
        ],
        remappings=[("/state_estimation", "/odom/ground_truth"), ("/registered_scan", "/registered_scan_reliable")],
        output="screen",
    )

    path_follower_node = Node(
        package="local_planner",
        executable="pathFollower",
        name="path_follower",
        parameters=[
            {
                "use_sim_time": True,
                "sensorOffsetX": 0.0,
                "sensorOffsetY": 0.0,
                "twoWayDrive": True,
                "lookAheadDis": 0.8,
                "yawRateGain": 4.0,
                "maxYawRate": 1.0,
                "maxSpeed": 0.5,
                "maxAccel": 0.5,
                "autonomyMode": True,
                "autonomySpeed": 0.4,
            }
        ],
        remappings=[("/state_estimation", "/odom/ground_truth")],
        output="screen",
    )

    far_planner_node = Node(
        package="far_planner",
        executable="far_planner",
        name="far_planner",
        parameters=[
            {
                "use_sim_time": True,
                "world_frame": "odom",
                "robot_frame": "base_link",
                "sensor_range": 3.5,
                "is_attempt_autoswitch": True,
                "is_viewpoint_extend": True,
                "is_terrain_planner": False,
                "util/terrain_free_Z": -0.01,
            }
        ],
        remappings=[
            ("/odom_world", "/odom"),
            ("/scan_cloud", "/registered_scan_reliable"),
            ("/terrain_cloud", "/terrain_map"),
            ("/terrain_local_cloud", "/terrain_map"),
        ],
        output="screen",
        condition=IfCondition(enable_far_planner),
    )

    frontier_explorer_node = Node(
        package="go2_gazebo_sim",
        executable="frontier_explorer.py",
        name="frontier_explorer",
        parameters=[
            {
                "use_sim_time": True,
                "exploration_frame": "odom",
                "goal_topic": "/way_point",
                "min_frontier_distance": 1.0,
                "goal_reached_threshold": 2.0,
                "goal_publish_rate": 2.0,
            }
        ],
        output="screen",
        condition=IfCondition(enable_far_planner),
    )

    simple_frontier_node = Node(
        package="go2_gazebo_sim",
        executable="simple_frontier_nav.py",
        name="simple_frontier_nav",
        parameters=[
            {
                "use_sim_time": True,
                "terrain_topic": "/terrain_map",
                "odom_topic": "/odom/ground_truth",
                "waypoint_topic": "/way_point",
                "free_intensity_max": 0.15,
                "min_distance": 1.0,
                "max_distance": 4.0,
                "publish_rate": 1.0,
                "reselect_interval": 3.0,
                "max_points_sample": 2000,
            }
        ],
        output="screen",
        condition=IfCondition(enable_simple_frontier),
    )

    geometric_frontier_node = build_geometric_frontier_node(
        ns=None,
        use_sim_time=use_sim_time,
        profile="geometric_frontier_single.yaml",
        extra_params={
            "scan_topic": "/scan",
            "odom_topic": "/odom/ground_truth",
            "map_frame": "odom",
            "map_topic": "/map",
            "frontier_goal_topic": "/way_point",
            "frontier_marker_topic": "/frontier_goal_marker",
            "frontier_regions_topic": "/frontier_markers",
            "frontier_replan_topic": "/frontier_replan",
        },
        condition=IfCondition(enable_geometric_frontier),
    )

    gazebo_frontier_visual_node = build_gazebo_frontier_visual_node(
        ns=None,
        use_sim_time=use_sim_time,
        extra_params={
            "goal_topic": "/frontier_goal",
            "entity_name": "frontier_goal_marker",
            "min_update_distance": 0.2,
            "min_update_sec": 1.0,
        },
        condition=IfCondition(enable_gazebo_frontier_visual),
    )

    motion_monitor_node = build_motion_monitor_node(
        ns=None,
        use_sim_time=use_sim_time,
        extra_params={
            "odom_topic": "/odom/ground_truth",
            "stop_topic": "/stop",
            "report_rate": 1.0,
            "move_threshold": 0.05,
        },
    )

    wall_checker_node = build_wall_checker_node(
        ns=None,
        use_sim_time=use_sim_time,
        extra_params={
            "safety_dist": 0.2,
            "check_angle_deg": 20.0,
            "scan_topic": "/scan",
            "stop_topic": "/stop",
        },
    )

    frontier_recovery_node = build_frontier_recovery_node(
        ns=None,
        use_sim_time=use_sim_time,
        extra_params={
            "min_frontier_distance": 1.0,
            "trigger_stop_duration": 1.5,
            "cooldown_sec": 6.0,
            "publish_rate": 2.0,
        },
    )

    goalpoint_bridge_node = build_goalpoint_bridge_node(ns=None, use_sim_time=use_sim_time)

    autonomy_enabler_node = build_autonomy_enabler_node(
        ns=None,
        use_sim_time=use_sim_time,
        extra_params={
            "startup_delay": 15.0,
            "rate": 10.0,
        },
    )

    reactive_nav_node = build_reactive_nav_node(
        ns=None,
        use_sim_time=use_sim_time,
        profile="reactive_nav_single.yaml",
        extra_params={
            "frontier_replan_topic": "/frontier_replan",
            "stop_topic": "/stop",
        },
    )

    delayed_autonomy = TimerAction(
        period=3.0,
        actions=[
            wall_checker_node,
            frontier_recovery_node,
            goalpoint_bridge_node,
            simple_frontier_node,
            geometric_frontier_node,
            gazebo_frontier_visual_node,
            motion_monitor_node,
            autonomy_enabler_node,
            reactive_nav_node,
        ],
    )

    delayed_far_planner = TimerAction(
        period=5.0,
        actions=[
            far_planner_node,
            frontier_explorer_node,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("enable_far_planner", default_value="false"),
            DeclareLaunchArgument("enable_simple_frontier", default_value="false"),
            DeclareLaunchArgument("enable_geometric_frontier", default_value="true"),
            DeclareLaunchArgument("enable_gazebo_frontier_visual", default_value="true"),
            delayed_autonomy,
            delayed_far_planner,
        ]
    )
