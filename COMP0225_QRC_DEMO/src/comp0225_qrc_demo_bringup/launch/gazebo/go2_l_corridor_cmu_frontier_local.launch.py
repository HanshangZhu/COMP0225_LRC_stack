import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

THIS_LAUNCH_DIR = os.path.dirname(__file__)
if THIS_LAUNCH_DIR not in sys.path:
    sys.path.append(THIS_LAUNCH_DIR)
from pipeline_components import build_pointcloud_to_laserscan_node, build_simple_scan_mapper_cpp_node


"""Gazebo single-Go2 stack with frontier goal generation + reactive local control.

Pipeline:
- Gazebo + stand-up
- Odom relay (/odom/ground_truth by default) -> /state_estimation
- Optional SLAM (FAST-LIO) in parallel
- Frontier planner (simple_scan_mapper + geometric_frontier) -> /way_point
- Reactive nav + twist bridge -> /cmd_vel for gazebo_ros2_control
"""


def generate_launch_description():
    demo_pkg = get_package_share_directory("comp0225_qrc_demo_bringup")
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    autonomous = LaunchConfiguration("autonomous")
    use_slam = LaunchConfiguration("use_slam")
    robot_variant = LaunchConfiguration("robot_variant")
    robot_name = LaunchConfiguration("robot_name")
    cleanup_stale = LaunchConfiguration("cleanup_stale")

    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_heading = LaunchConfiguration("spawn_heading")

    slam_package = LaunchConfiguration("slam_package")
    slam_executable = LaunchConfiguration("slam_executable")
    slam_odom_topic = LaunchConfiguration("slam_odom_topic")
    slam_enabled = IfCondition(
        PythonExpression(["'", autonomous, "' == 'true' and '", use_slam, "' == 'true'"])
    )

    frontier_profile = os.path.join(demo_pkg, "config", "nav", "geometric_frontier_single.yaml")
    slam_config = os.path.join(demo_pkg, "config", "slam", "pointlio_gazebo.yaml")
    reactive_nav_config = os.path.join(go2_gazebo_pkg, "config", "nav", "reactive_nav_single.yaml")
    wall_checker_config = os.path.join(go2_gazebo_pkg, "config", "nav", "wall_checker.yaml")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(demo_pkg, "launch", "gazebo", "go2_l_corridor.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "gui": gui,
            "rviz": rviz,
            "robot_variant": robot_variant,
            "robot_name": robot_name,
            "cleanup_stale": cleanup_stale,
            "world_init_x": spawn_x,
            "world_init_y": spawn_y,
            "world_init_z": spawn_z,
            "world_init_heading": spawn_heading,
        }.items(),
    )

    qos_bridge_node = Node(
        package="comp0225_qrc_demo_bringup",
        executable="qos_bridge.py",
        name="qos_bridge",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/registered_scan",
                "output_topic": "/registered_scan_reliable",
            }
        ],
        output="screen",
    )

    pointcloud_adapter_node = Node(
        package="comp0225_qrc_demo_bringup",
        executable="pointcloud_adapter.py",
        name="pointcloud_adapter",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": "/registered_scan_reliable"},
            {"output_topic": "/velodyne_points"},
            {"num_rings": 16},
        ],
        output="screen",
    )

    slam_node = Node(
        package=slam_package,
        executable=slam_executable,
        name="slam_node",
        parameters=[slam_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("/velodyne_points", "/velodyne_points"),
            ("/imu/data", "/imu/data"),
            ("/Odometry", "/Odometry"),
        ],
        output="screen",
        condition=slam_enabled,
    )

    slam_relay_node = Node(
        package="comp0225_qrc_demo_bringup",
        executable="slam_odom_relay.py",
        name="slam_odom_relay",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": slam_odom_topic},
            {"output_topic": "/state_estimation"},
            {"output_frame_id": "map"},
            {"output_child_frame_id": "base_link"},
            {"bootstrap_from_gt": True},
            {"gt_topic": "/odom/ground_truth"},
            {"require_gt_for_alignment": True},
        ],
        output="screen",
        condition=IfCondition(autonomous),
    )

    pointcloud_to_laserscan = build_pointcloud_to_laserscan_node(
        ns=None,
        use_sim_time=use_sim_time,
        extra_params={
            "target_frame": "base_link",
            "transform_tolerance": 0.3,
            "min_height": 0.05,
            "max_height": 0.60,
            "range_min": 0.2,
            "range_max": 12.0,
        },
        remappings=[("cloud_in", "/registered_scan_reliable"), ("scan", "/scan")],
        condition=IfCondition(autonomous),
    )

    simple_scan_mapper_node = build_simple_scan_mapper_cpp_node(
        ns=None,
        use_sim_time=use_sim_time,
        profile="geometric_frontier_single.yaml",
        extra_params={
            "scan_topic": "/scan",
            "odom_topic": "/state_estimation",
            "map_topic": "/map",
            "map_frame": "map",
            "startup_delay": 0.0,
            "max_scan_odom_dt": 0.1,
        },
        name="simple_scan_mapper_cpp",
        condition=IfCondition(autonomous),
    )

    geometric_frontier_node = Node(
        package="go2_nav_algorithms",
        executable="simple_frontier_explorer.py",
        name="geometric_frontier",
        parameters=[
            frontier_profile,
            {
                "use_sim_time": use_sim_time,
                "odom_topic": "/state_estimation",
                "map_topic": "/map",
                "prefer_costmap": False,
                "costmap_topic": "",
                "frontier_goal_topic": "/way_point",
                "frontier_marker_topic": "/frontier_goal_marker",
                "frontier_regions_topic": "/frontier_markers",
                "frontier_replan_topic": "/frontier_replan",
                "startup_delay": 0.0,
                "max_map_odom_dt": 1.0,
            },
        ],
        output="screen",
        condition=IfCondition(autonomous),
    )

    autonomy_enabler_node = Node(
        package="comp0225_qrc_demo_bringup",
        executable="autonomy_enabler.py",
        name="autonomy_enabler",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"startup_delay": 8.0},
            {"rate": 10.0},
            {"wait_for_waypoint": True},
        ],
        output="screen",
        condition=IfCondition(autonomous),
    )

    twist_bridge_node = Node(
        package="comp0225_qrc_demo_bringup",
        executable="twist_bridge.py",
        name="twist_bridge",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(autonomous),
    )

    # Reactive controller path for Gazebo frontier following.
    reactive_nav_node = Node(
        package="go2_gazebo_sim",
        executable="reactive_nav.py",
        name="reactive_nav",
        parameters=[
            reactive_nav_config,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("/odom/ground_truth", "/state_estimation")],
        output="screen",
        condition=IfCondition(autonomous),
    )

    wall_checker_node = Node(
        package="go2_gazebo_sim",
        executable="wall_collision_checker.py",
        name="wall_collision_checker",
        parameters=[
            wall_checker_config,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        condition=IfCondition(autonomous),
    )

    delayed_perception = TimerAction(period=10.0, actions=[qos_bridge_node, pointcloud_adapter_node])
    delayed_slam = TimerAction(period=14.0, actions=[slam_node], condition=slam_enabled)

    delayed_autonomy = TimerAction(
        period=22.0,
        actions=[
            slam_relay_node,
            pointcloud_to_laserscan,
            simple_scan_mapper_node,
            geometric_frontier_node,
            autonomy_enabler_node,
            twist_bridge_node,
            wall_checker_node,
            reactive_nav_node,
        ],
        condition=IfCondition(autonomous),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
            DeclareLaunchArgument("gui", default_value="true", description="Run Gazebo GUI"),
            DeclareLaunchArgument("rviz", default_value="false", description="Run RViz"),
            DeclareLaunchArgument("cleanup_stale", default_value="true", description="Kill stale Gazebo processes before launch"),
            DeclareLaunchArgument("autonomous", default_value="true", description="Enable CMU-style autonomy stack"),
            DeclareLaunchArgument("use_slam", default_value="false", description="Run FAST-LIO in parallel"),
            DeclareLaunchArgument("robot_variant", default_value="go2w", description="Robot model variant: go2 or go2w"),
            DeclareLaunchArgument("robot_name", default_value="go2w", description="Gazebo entity name"),
            DeclareLaunchArgument("spawn_x", default_value="2.5", description="Spawn X coordinate"),
            DeclareLaunchArgument("spawn_y", default_value="0.0", description="Spawn Y coordinate"),
            DeclareLaunchArgument("spawn_z", default_value="0.45", description="Spawn Z coordinate"),
            DeclareLaunchArgument("spawn_heading", default_value="0.0", description="Spawn Heading (yaw)"),
            DeclareLaunchArgument("slam_package", default_value="fast_lio", description="SLAM package (used when use_slam=true)"),
            DeclareLaunchArgument("slam_executable", default_value="fastlio_mapping", description="SLAM executable"),
            DeclareLaunchArgument("slam_odom_topic", default_value="/odom/ground_truth", description="Odometry topic relayed into /state_estimation"),
            gazebo_launch,
            delayed_perception,
            delayed_slam,
            delayed_autonomy,
        ]
    )
