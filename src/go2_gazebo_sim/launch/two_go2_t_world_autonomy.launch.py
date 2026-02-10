import os
import sys

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))

from _stack_components import (
    build_autonomy_enabler_node,
    build_dual_robot_stack,
    build_geometric_frontier_node,
    build_namespaced_robot_description,
    build_pointcloud_to_laserscan_node,
    build_qos_bridge_node,
    build_reactive_nav_node,
    build_rviz_node,
)


def _robot_autonomy_actions(ns: str, use_sim_time):
    tf_remaps = [("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")]

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

    pointcloud_to_laserscan_node = build_pointcloud_to_laserscan_node(
        ns=ns,
        use_sim_time=use_sim_time,
        extra_params={
            "target_frame": f"{ns}/base_link",
        },
        remappings=tf_remaps
        + [
            ("cloud_in", f"/{ns}/registered_scan_reliable"),
            ("scan", f"/{ns}/scan"),
        ],
    )

    geometric_frontier_node = build_geometric_frontier_node(
        ns=ns,
        use_sim_time=use_sim_time,
        profile="geometric_frontier_dual.yaml",
        extra_params={
            "scan_topic": f"/{ns}/scan",
            "odom_topic": f"/{ns}/odom/ground_truth",
            "map_topic": f"/{ns}/map",
            "frontier_goal_topic": f"/{ns}/way_point",
            "frontier_marker_topic": f"/{ns}/frontier_goal_marker",
            "frontier_regions_topic": f"/{ns}/frontier_markers",
            "frontier_replan_topic": f"/{ns}/frontier_replan",
        },
        remappings=tf_remaps,
    )

    autonomy_enabler_node = build_autonomy_enabler_node(
        ns=ns,
        use_sim_time=use_sim_time,
        extra_params={"startup_delay": 12.0, "rate": 10.0},
        remappings=[("/way_point", f"/{ns}/way_point"), ("/joy", f"/{ns}/joy")],
    )

    reactive_nav_node = build_reactive_nav_node(
        ns=ns,
        use_sim_time=use_sim_time,
        profile="reactive_nav_dual.yaml",
        extra_params={
            "frontier_replan_topic": f"/{ns}/frontier_replan",
            "stop_topic": f"/{ns}/stop",
        },
        remappings=[
            ("/way_point", f"/{ns}/way_point"),
            ("/odom/ground_truth", f"/{ns}/odom/ground_truth"),
            ("/scan", f"/{ns}/scan"),
            ("/cmd_vel_stamped", f"/{ns}/cmd_vel_stamped"),
            ("/nav_status", f"/{ns}/nav_status"),
        ],
    )

    return [
        TimerAction(
            period=14.0,
            actions=[
                twist_bridge_node,
                qos_bridge_node,
                pointcloud_to_laserscan_node,
                geometric_frontier_node,
                autonomy_enabler_node,
                reactive_nav_node,
            ],
        )
    ]


def generate_launch_description():
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    go2_config_pkg = get_package_share_directory("go2_config")
    champ_base_pkg = get_package_share_directory("champ_base")
    champ_gazebo_pkg = get_package_share_directory("champ_gazebo")

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    cleanup_stale = LaunchConfiguration("cleanup_stale")
    world = LaunchConfiguration("world")
    robot_a_spawn_x = LaunchConfiguration("robot_a_spawn_x")
    robot_a_spawn_y = LaunchConfiguration("robot_a_spawn_y")
    robot_a_spawn_yaw = LaunchConfiguration("robot_a_spawn_yaw")
    robot_b_spawn_x = LaunchConfiguration("robot_b_spawn_x")
    robot_b_spawn_y = LaunchConfiguration("robot_b_spawn_y")
    robot_b_spawn_yaw = LaunchConfiguration("robot_b_spawn_yaw")

    gazebo_config = os.path.join(champ_gazebo_pkg, "config", "gazebo.yaml")

    rviz_config_robot_a = os.path.join(go2_gazebo_pkg, "rviz", "dual_robot_a.rviz")
    rviz_config_robot_b = os.path.join(go2_gazebo_pkg, "rviz", "dual_robot_b.rviz")

    description_path = os.path.join(go2_gazebo_pkg, "urdf", "go2_description_3d_lidar.xacro")
    doc = xacro.process_file(description_path)
    base_robot_description = doc.documentElement.toxml()

    ros_control_robot_a = os.path.join(go2_gazebo_pkg, "config", "ros_control", "ros_control_robot_a.yaml")
    ros_control_robot_b = os.path.join(go2_gazebo_pkg, "config", "ros_control", "ros_control_robot_b.yaml")

    robot_description_a = build_namespaced_robot_description(base_robot_description, "robot_a", ros_control_robot_a)
    robot_description_b = build_namespaced_robot_description(base_robot_description, "robot_b", ros_control_robot_b)

    joints_config = os.path.join(go2_config_pkg, "config", "joints", "joints.yaml")
    links_config = os.path.join(go2_config_pkg, "config", "links", "links.yaml")
    gait_config = os.path.join(go2_config_pkg, "config", "gait", "gait.yaml")
    ekf_base_to_footprint = os.path.join(champ_base_pkg, "config", "ekf", "base_to_footprint.yaml")
    ekf_footprint_to_odom = os.path.join(champ_base_pkg, "config", "ekf", "footprint_to_odom.yaml")

    start_gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world,
            "--ros-args",
            "--params-file",
            gazebo_config,
        ],
        output="screen",
    )

    cleanup_stale_processes = ExecuteProcess(
        condition=IfCondition(cleanup_stale),
        cmd=[
            "bash",
            "-lc",
            "pkill -f 'gzserver -s libgazebo_ros_init.so -s libgazebo_ros_factory.so' || true; "
            "pkill -f '^gzclient$' || true; "
            "sleep 1",
        ],
        output="screen",
    )

    start_gazebo_client = ExecuteProcess(condition=IfCondition(gui), cmd=["gzclient"], output="screen")

    rviz_node_robot_a = build_rviz_node(rviz_config_robot_a, use_sim_time, condition=IfCondition(rviz), name="rviz2_robot_a")
    rviz_node_robot_b = build_rviz_node(rviz_config_robot_b, use_sim_time, condition=IfCondition(rviz), name="rviz2_robot_b")

    dual_coverage_visualizer_node = Node(
        package="go2_gazebo_sim",
        executable="dual_map_coverage_visualizer.py",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_a_map_topic": "/robot_a/map"},
            {"robot_b_map_topic": "/robot_b/map"},
            {"robot_a_odom_topic": "/robot_a/odom/ground_truth"},
            {"robot_b_odom_topic": "/robot_b/odom/ground_truth"},
            {"marker_topic": "/dual_robot/coverage_markers"},
            {"marker_frame": "world"},
            {"publish_rate": 1.0},
            {"min_map_value": 0},
            {"cell_stride": 1},
            {"robot_a_alpha": 0.20},
            {"robot_b_alpha": 0.20},
        ],
        output="screen",
    )

    robot_a_actions = build_dual_robot_stack(
        ns="robot_a",
        spawn_x=robot_a_spawn_x,
        spawn_y=robot_a_spawn_y,
        spawn_yaw=robot_a_spawn_yaw,
        use_sim_time=use_sim_time,
        robot_description=robot_description_a,
        joints_config=joints_config,
        links_config=links_config,
        gait_config=gait_config,
        ekf_base_to_footprint=ekf_base_to_footprint,
        ekf_footprint_to_odom=ekf_footprint_to_odom,
    ) + _robot_autonomy_actions("robot_a", use_sim_time)

    robot_b_actions = build_dual_robot_stack(
        ns="robot_b",
        spawn_x=robot_b_spawn_x,
        spawn_y=robot_b_spawn_y,
        spawn_yaw=robot_b_spawn_yaw,
        use_sim_time=use_sim_time,
        robot_description=robot_description_b,
        joints_config=joints_config,
        links_config=links_config,
        gait_config=gait_config,
        ekf_base_to_footprint=ekf_base_to_footprint,
        ekf_footprint_to_odom=ekf_footprint_to_odom,
    ) + _robot_autonomy_actions("robot_b", use_sim_time)

    robot_status_monitor_node = Node(
        package="go2_gazebo_sim",
        executable="robot_status_monitor.py",
        name="robot_status_monitor",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"namespaces": ["robot_a", "robot_b"]},
            {"report_rate": 1.0},
            {"json_output": False},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "cleanup_stale",
                default_value="true",
                description="Kill stale Gazebo server/client processes before starting dual run.",
            ),
            DeclareLaunchArgument("robot_a_spawn_x", default_value="1.0"),
            DeclareLaunchArgument("robot_a_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_a_spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("robot_b_spawn_x", default_value="18.0"),
            DeclareLaunchArgument("robot_b_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_b_spawn_yaw", default_value="3.14159"),
            DeclareLaunchArgument("world", default_value=os.path.join(go2_gazebo_pkg, "worlds", "3.world")),
            cleanup_stale_processes,
            TimerAction(period=1.0, actions=[start_gazebo_server]),
            start_gazebo_client,
            rviz_node_robot_a,
            rviz_node_robot_b,
            dual_coverage_visualizer_node,
            TimerAction(period=2.0, actions=robot_a_actions),
            TimerAction(period=10.0, actions=robot_b_actions),
            TimerAction(period=25.0, actions=[robot_status_monitor_node]),
        ]
    )
