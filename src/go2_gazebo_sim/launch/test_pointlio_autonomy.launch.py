"""
Test Point-LIO SLAM + Full Autonomy Stack.

This launch file replaces the "cheating" ground-truth odometry with real
SLAM-based odometry from Point-LIO, then runs the full autonomy stack.

Pipeline:
    Gazebo + Robot  →  Stand Up  →  QoS Bridge  →  Point-LIO SLAM
                                                        |
                                                        ▼
                                                slam_odom_relay
                                            (/aft_mapped_to_init → /odom/ground_truth)
                                                        |
                                                        ▼
                                               CMU Autonomy Stack
                                          (geometric_frontier + reactive_nav)

IMPORTANT: go2_l_corridor.launch.py still starts the EKF nodes and p3d plugin.
The p3d plugin publishes /odom/ground_truth but our slam_odom_relay will
also publish to /odom/ground_truth. To avoid conflicts, we remap the relay
to a separate topic and let downstream nodes use that. See the remappings below.

Usage:
    ros2 launch go2_gazebo_sim test_pointlio_autonomy.launch.py

    # Monitor SLAM health:
    ros2 topic hz /aft_mapped_to_init
    # Check drift:
    tail -f /tmp/odom_comparison_*.csv
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# =====================================================================
# CONFIGURATION: Change these to match whichever SLAM package you built
# =====================================================================
SLAM_PACKAGE = "fast_lio"          # or "point_lio_unilidar"
SLAM_EXECUTABLE = "fastlio_mapping"  # or "pointlio_mapping"
# FAST-LIO publishes odom to /Odometry; Point-LIO to /aft_mapped_to_init
SLAM_ODOM_TOPIC = "/Odometry"        # or "/aft_mapped_to_init"


def generate_launch_description():
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")

    gui = LaunchConfiguration("gui")
    use_slam_odom = LaunchConfiguration("use_slam_odom")

    # The SLAM odom topic that downstream nodes will consume.
    # If use_slam_odom is true, this comes from the relay; otherwise ground truth.
    slam_odom_topic = "/slam/odom"

    # ----------------------------------------------------------------
    # 1. Gazebo Sim (unchanged)
    # ----------------------------------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_gazebo_pkg, "launch", "go2_l_corridor.launch.py")
        ),
        launch_arguments={
            "gui": gui,
            "rviz": "true",
        }.items(),
    )

    # ----------------------------------------------------------------
    # 2. Stand Up
    # ----------------------------------------------------------------
    stand_up_node = Node(
        package="go2_gazebo_sim",
        executable="stand_up_slowly.py",
        name="stand_up_slowly",
        output="screen",
    )
    delayed_stand_up = TimerAction(period=8.0, actions=[stand_up_node])

    # ----------------------------------------------------------------
    # 3. Bridges
    # ----------------------------------------------------------------
    qos_bridge_node = Node(
        package="go2_gazebo_sim",
        executable="qos_bridge.py",
        name="qos_bridge",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[{
            "use_sim_time": True,
            "target_frame": "odom",
            "transform_tolerance": 1.0,
            "min_height": 0.15,
            "max_height": 1.0,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.006135923151543,
            "scan_time": 0.1,
            "range_min": 0.2,
            "range_max": 20.0,
            "use_inf": True,
        }],
        remappings=[
            ("cloud_in", "/registered_scan"),
            ("scan", "/scan"),
        ],
        output="screen",
    )

    twist_bridge_node = Node(
        package="go2_gazebo_sim",
        executable="twist_bridge.py",
        name="twist_bridge",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # ----------------------------------------------------------------
    # 4. Point-LIO SLAM
    # ----------------------------------------------------------------
    slam_config = os.path.join(
        go2_gazebo_pkg, "config", "slam", "pointlio_gazebo.yaml"
    )

    pointlio_node = Node(
        package=SLAM_PACKAGE,
        executable=SLAM_EXECUTABLE,
        name="slam_node",
        parameters=[
            slam_config,
            {
                "use_sim_time": True,
                "use_imu_as_input": False,
                "prop_at_freq_of_imu": True,
                "check_satu": True,
                "init_map_size": 10,
                "point_filter_num": 4,
                "space_down_sample": True,
                "filter_size_surf": 0.5,
                "filter_size_map": 0.5,
                "cube_side_length": 1000.0,
                "runtime_pos_log_enable": False,
            },
        ],
        output="screen",
    )

    # ----------------------------------------------------------------
    # 5. SLAM Odom Relay
    # Converts Point-LIO output → /slam/odom with correct frame IDs
    # ----------------------------------------------------------------
    slam_relay_node = Node(
        package="go2_gazebo_sim",
        executable="slam_odom_relay.py",
        name="slam_odom_relay",
        parameters=[
            {"use_sim_time": True},
            {"input_topic": SLAM_ODOM_TOPIC},
            {"output_topic": slam_odom_topic},
            {"output_frame_id": "world"},
            {"output_child_frame_id": "base_link"},
        ],
        output="screen",
    )

    # ----------------------------------------------------------------
    # 6. Autonomy Stack (using SLAM odom instead of ground truth)
    # ----------------------------------------------------------------

    # Wall checker
    wall_checker_node = Node(
        package="go2_gazebo_sim",
        executable="wall_collision_checker.py",
        name="wall_collision_checker",
        parameters=[
            os.path.join(go2_gazebo_pkg, "config", "nav", "wall_checker.yaml"),
            {"use_sim_time": True},
        ],
        output="screen",
    )

    # Geometric frontier — remap odom to SLAM source
    frontier_config = os.path.join(
        go2_gazebo_pkg, "config", "nav", "geometric_frontier_single.yaml"
    )
    geometric_frontier_node = Node(
        package="go2_gazebo_sim",
        executable="geometric_frontier.py",
        name="geometric_frontier",
        parameters=[
            frontier_config,
            {
                "use_sim_time": True,
                "odom_topic": slam_odom_topic,   # Use SLAM odom!
            },
        ],
        output="screen",
    )

    # Reactive nav — remap odom to SLAM source
    reactive_nav_config = os.path.join(
        go2_gazebo_pkg, "config", "nav", "reactive_nav_single.yaml"
    )
    reactive_nav_node = Node(
        package="go2_gazebo_sim",
        executable="reactive_nav.py",
        name="reactive_nav",
        parameters=[
            reactive_nav_config,
            {"use_sim_time": True},
        ],
        remappings=[
            ("/odom/ground_truth", slam_odom_topic),   # Use SLAM odom!
        ],
        output="screen",
    )

    # ----------------------------------------------------------------
    # 7. Odom Comparison (SLAM vs Ground Truth)
    # ----------------------------------------------------------------
    odom_comparison_node = Node(
        package="go2_gazebo_sim",
        executable="odom_comparison.py",
        name="odom_comparison",
        parameters=[
            {"use_sim_time": True},
            {"gt_topic": "/odom/ground_truth"},
            {"slam_topic": slam_odom_topic},
            {"report_rate": 1.0},
        ],
        output="screen",
    )

    # ----------------------------------------------------------------
    # Timing
    # ----------------------------------------------------------------
    delayed_bridges = TimerAction(
        period=10.0,
        actions=[qos_bridge_node, pointcloud_to_laserscan, twist_bridge_node],
    )

    delayed_slam = TimerAction(
        period=14.0,
        actions=[pointlio_node, slam_relay_node],
    )

    delayed_autonomy = TimerAction(
        period=20.0,
        actions=[
            wall_checker_node,
            geometric_frontier_node,
            reactive_nav_node,
            odom_comparison_node,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("use_slam_odom", default_value="true",
                              description="Use SLAM odom instead of ground truth"),

        # 1. Gazebo
        gazebo_launch,

        # 2. Stand up
        delayed_stand_up,

        # 3. Bridges
        delayed_bridges,

        # 4. SLAM
        delayed_slam,

        # 5. Autonomy
        delayed_autonomy,
    ])
