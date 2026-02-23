import os
import sys

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
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


def _wait_controllers_loaded(ns: str):
    return ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                "until ros2 control list_controllers -c "
                f"/{ns}/controller_manager 2>/dev/null | "
                "awk '"
                f"/{ns}_joint_states_controller/ && tolower($0) ~ /(inactive|active|configured)/ {{a=1}} "
                f"/{ns}_joint_group_effort_controller/ && tolower($0) ~ /(inactive|active|configured)/ {{b=1}} "
                "END {exit !(a && b)}'; "
                "do sleep 0.25; done"
            ),
        ],
        output="screen",
    )


def _robot_autonomy_actions(ns: str, use_sim_time, use_fast_lio, slam_config):
    tf_remaps = [("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")]
    nav_odom_topic = f"/{ns}/odom/nav"
    planning_scan_topic = f"/{ns}/scan_3d"

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

    pointcloud_adapter_node = Node(
        package="go2_gazebo_sim",
        executable="pointcloud_adapter.py",
        namespace=ns,
        name="pointcloud_adapter",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": f"/{ns}/registered_scan_reliable"},
            {"output_topic": f"/{ns}/velodyne_points"},
            {"num_rings": 16},
        ],
        output="screen",
        condition=IfCondition(use_fast_lio),
    )

    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        namespace=ns,
        name="slam_node",
        parameters=[slam_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("/velodyne_points", f"/{ns}/velodyne_points"),
            ("/imu/data", f"/{ns}/imu/data"),
            ("/Odometry", f"/{ns}/Odometry"),
        ],
        output="screen",
        condition=IfCondition(use_fast_lio),
    )

    slam_odom_relay_node = Node(
        package="go2_gazebo_sim",
        executable="slam_odom_relay.py",
        namespace=ns,
        name="slam_odom_relay",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": f"/{ns}/Odometry"},
            {"gt_topic": f"/{ns}/odom/ground_truth"},
            {"output_topic": nav_odom_topic},
            {"output_frame_id": "world"},
            {"output_child_frame_id": "base_link"},
            {"bootstrap_from_gt": True},
            {"require_gt_for_alignment": True},
        ],
        output="screen",
        condition=IfCondition(use_fast_lio),
    )

    gt_odom_relay_node = Node(
        package="go2_gazebo_sim",
        executable="slam_odom_relay.py",
        namespace=ns,
        name="gt_odom_relay",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": f"/{ns}/odom/ground_truth"},
            {"output_topic": nav_odom_topic},
            {"output_frame_id": "world"},
            {"output_child_frame_id": "base_link"},
        ],
        output="screen",
        condition=UnlessCondition(use_fast_lio),
    )

    pointcloud_to_laserscan_node = build_pointcloud_to_laserscan_node(
        ns=ns,
        use_sim_time=use_sim_time,
        extra_params={
            "target_frame": "base_link",
            # Keep a conservative 2D slice to reduce over/under-wall artifacts.
            "min_height": 0.05,
            "max_height": 0.60,
            "range_max": 12.0,
        },
        remappings=tf_remaps
        + [
            ("cloud_in", f"/{ns}/registered_scan_reliable"),
            ("scan", planning_scan_topic),
        ],
    )

    geometric_frontier_node = build_geometric_frontier_node(
        ns=ns,
        use_sim_time=use_sim_time,
        profile="geometric_frontier_dual.yaml",
        extra_params={
            "scan_topic": planning_scan_topic,
            "odom_topic": nav_odom_topic,
            "map_topic": f"/{ns}/map",
            # Keep per-robot frontier maps/markers, but route goals through
            # centralized assignment below.
            "frontier_goal_topic": f"/{ns}/way_point_raw",
            "frontier_marker_topic": f"/{ns}/frontier_goal_marker",
            "frontier_regions_topic": f"/{ns}/frontier_markers",
            "frontier_replan_topic": f"/{ns}/frontier_replan",
            "max_range": 12.0,
        },
        remappings=tf_remaps,
    )

    autonomy_enabler_node = build_autonomy_enabler_node(
        ns=ns,
        use_sim_time=use_sim_time,
        extra_params={"startup_delay": 8.0, "rate": 10.0},
        remappings=[("/way_point", f"/{ns}/way_point_coord"), ("/joy", f"/{ns}/joy")],
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
            ("/way_point", f"/{ns}/way_point_coord"),
            ("/odom/ground_truth", nav_odom_topic),
            ("/scan", planning_scan_topic),
            ("/cmd_vel_stamped", f"/{ns}/cmd_vel_stamped"),
            ("/nav_status", f"/{ns}/nav_status"),
        ],
    )

    return [
        TimerAction(
            period=16.0,
            actions=[
                twist_bridge_node,
                qos_bridge_node,
                pointcloud_adapter_node,
                fast_lio_node,
                slam_odom_relay_node,
                gt_odom_relay_node,
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
    use_fast_lio = LaunchConfiguration("use_fast_lio")
    use_shared_map = LaunchConfiguration("use_shared_map")
    shared_map_topic = LaunchConfiguration("shared_map_topic")
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

    robot_description_a = build_namespaced_robot_description(
        base_robot_description,
        "robot_a",
        ros_control_robot_a,
    )
    robot_description_b = build_namespaced_robot_description(
        base_robot_description,
        "robot_b",
        ros_control_robot_b,
    )

    joints_config = os.path.join(go2_config_pkg, "config", "joints", "joints.yaml")
    links_config = os.path.join(go2_config_pkg, "config", "links", "links.yaml")
    gait_config = os.path.join(go2_config_pkg, "config", "gait", "gait.yaml")
    ekf_base_to_footprint = os.path.join(champ_base_pkg, "config", "ekf", "base_to_footprint.yaml")
    ekf_footprint_to_odom = os.path.join(champ_base_pkg, "config", "ekf", "footprint_to_odom.yaml")
    slam_config = os.path.join(go2_gazebo_pkg, "config", "slam", "pointlio_gazebo.yaml")

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
            "SELF=$$; PARENT=$PPID; "
            "kill_pattern(){ "
            "  PATTERN=\"$1\"; "
            "  for PID in $(pgrep -f \"$PATTERN\" 2>/dev/null || true); do "
            "    [ \"$PID\" = \"$SELF\" ] && continue; "
            "    [ \"$PID\" = \"$PARENT\" ] && continue; "
            "    kill \"$PID\" 2>/dev/null || true; "
            "  done; "
            "}; "
            "kill_pattern '[g]zserver'; "
            "kill_pattern '(^|/)gzclient( |$)'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/dual_map_coverage_visualizer.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/geometric_frontier.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/reactive_nav.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/autonomy_enabler.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/twist_bridge.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/qos_bridge.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/robot_status_monitor.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/initial_pose_guard.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/pointcloud_adapter.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/slam_odom_relay.py'; "
            "kill_pattern '/fast_lio/lib/fast_lio/fastlio_mapping'; "
            "kill_pattern '/pointcloud_to_laserscan_node'; "
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
            {"robot_a_odom_topic": "/robot_a/odom/nav"},
            {"robot_b_odom_topic": "/robot_b/odom/nav"},
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

    multi_robot_goal_assigner_node = Node(
        package="go2_gazebo_sim",
        executable="multi_robot_goal_assigner.py",
        name="multi_robot_goal_assigner",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"namespaces": ["robot_a", "robot_b"]},
            {"publish_rate": 2.0},
            {"beta": 0.18},
            {"sensor_range": 3.5},
            {"frontier_stride": 2},
            {"max_targets": 800},
            {"switch_hysteresis": 0.05},
            {"switch_min_dist": 0.35},
            {"goal_topic_suffix": "/way_point_coord"},
            {"use_shared_map": use_shared_map},
            {"shared_map_topic": shared_map_topic},
            {"shared_map_wait_sec": 8.0},
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
        joint_state_spawner_delay_sec=1.0,
        effort_spawner_delay_sec=1.2,
        standup_delay_sec=4.0,
        pose_guard_hold_sec=12.0,
        activate_controllers_on_spawn=True,
    ) + _robot_autonomy_actions("robot_a", use_sim_time, use_fast_lio, slam_config)

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
        joint_state_spawner_delay_sec=1.6,
        effort_spawner_delay_sec=1.8,
        standup_delay_sec=4.8,
        pose_guard_hold_sec=13.0,
        activate_controllers_on_spawn=True,
    ) + _robot_autonomy_actions("robot_b", use_sim_time, use_fast_lio, slam_config)

    robot_status_monitor_node = Node(
        package="go2_gazebo_sim",
        executable="robot_status_monitor.py",
        name="robot_status_monitor",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"namespaces": ["robot_a", "robot_b"]},
            {"report_rate": 0.1},
            {"json_output": False},
        ],
        remappings=[
            ("/robot_a/odom/ground_truth", "/robot_a/odom/nav"),
            ("/robot_b/odom/ground_truth", "/robot_b/odom/nav"),
            ("/robot_a/way_point", "/robot_a/way_point_coord"),
            ("/robot_b/way_point", "/robot_b/way_point_coord"),
        ],
        output="screen",
    )

    wait_robot_a_controllers_loaded = _wait_controllers_loaded("robot_a")
    wait_robot_b_controllers_loaded = _wait_controllers_loaded("robot_b")

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
            DeclareLaunchArgument(
                "use_fast_lio",
                default_value="false",
                description="Use FAST-LIO odometry for autonomy odom input (default false uses GT odom relay).",
            ),
            DeclareLaunchArgument(
                "use_shared_map",
                default_value="false",
                description="Use a shared map backend topic (e.g., Disco-SLAM) for coordinated assignment.",
            ),
            DeclareLaunchArgument(
                "shared_map_topic",
                default_value="/disco_slam/global_map",
                description="Shared occupancy grid topic published by backend.",
            ),
            DeclareLaunchArgument("robot_a_spawn_x", default_value="1.0"),
            DeclareLaunchArgument("robot_a_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_a_spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("robot_b_spawn_x", default_value="18.0"),
            DeclareLaunchArgument("robot_b_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_b_spawn_yaw", default_value="3.14159"),
            DeclareLaunchArgument("world", default_value=os.path.join(go2_gazebo_pkg, "worlds", "3.world")),
            cleanup_stale_processes,
            TimerAction(period=3.0, actions=[start_gazebo_server]),
            # Delay GUI/visual tools so stale-process cleanup cannot kill the
            # newly launched instances in this same launch.
            TimerAction(period=6.0, actions=[start_gazebo_client]),
            TimerAction(
                period=7.0,
                actions=[rviz_node_robot_a, rviz_node_robot_b, dual_coverage_visualizer_node],
            ),
            TimerAction(period=5.0, actions=robot_a_actions + [wait_robot_a_controllers_loaded]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_robot_a_controllers_loaded,
                    on_exit=robot_b_actions + [wait_robot_b_controllers_loaded],
                )
            ),
            TimerAction(period=20.0, actions=[multi_robot_goal_assigner_node]),
            TimerAction(period=18.0, actions=[robot_status_monitor_node]),
        ]
    )
