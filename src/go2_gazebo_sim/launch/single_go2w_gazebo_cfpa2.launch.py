#!/usr/bin/env python3
"""Single-Go2W Gazebo CFPA2 launch aligned with the real-robot single-stack."""

from __future__ import annotations

import os
import shlex
import sys

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.join(get_package_share_directory("go2_nav_algorithms"), "launch"))
from modules.assets import build_dual_robot_stack, build_namespaced_robot_description
from modules.orchestration import build_rviz_node
from modules.slam import build_slam_odom_relay_node
from pipeline_components import build_pointcloud_to_laserscan_node, build_simple_scan_mapper_cpp_node


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, key: str) -> str:
    return LaunchConfiguration(key).perform(context)


def _build_cleanup_stale_command() -> str:
    patterns = [
        "ros2 launch go2_gazebo_sim single_go2w_gazebo_cfpa2.launch.py",
        "ros2 launch go2_gazebo_sim dual_go2_modular.launch.py",
        "ros2 launch go2_gazebo_sim dual_go2w_modular.launch.py",
        "[g]zserver",
        "(^|/)gzclient( |$)",
        "(^|/)gazebo( |$)",
        "/go2_nav_algorithms/lib/go2_nav_algorithms/simple_scan_mapper_cpp",
        "/go2w_perception/lib/go2w_perception/qos_bridge.py",
        "/go2w_control/lib/go2w_control/reactive_nav.py",
        "/go2w_control/lib/go2w_control/autonomy_enabler.py",
        "/go2w_perception/lib/go2w_perception/twist_bridge.py",
        "/go2w_control/lib/go2w_control/go2w_hybrid_cmd_router.py",
        "/go2w_spawn/lib/go2w_spawn/initial_pose_guard.py",
        "/go2w_spawn/lib/go2w_spawn/spawn_entity_direct.py",
        "/go2w_perception/lib/go2w_perception/pointcloud_adapter.py",
        "/go2w_perception/lib/go2w_perception/slam_odom_relay.py",
        "/fast_lio/lib/fast_lio/fastlio_mapping",
        "/cfpa2_collaborative_autonomy/lib/cfpa2_collaborative_autonomy/cfpa2_single_robot_node",
        "/champ_base/lib/champ_base/quadruped_controller_node",
        "/champ_base/lib/champ_base/state_estimation_node",
        "/robot_localization/ekf_node",
        "/robot_state_publisher",
        "/champ_gazebo/lib/champ_gazebo/contact_sensor",
        "/opt/ros/.*/lib/controller_manager/spawner",
        "/pointcloud_to_laserscan_node",
    ]

    command = [
        "SELF=$$; PARENT=$PPID; ",
        "kill_pattern(){ ",
        "  PATTERN=\"$1\"; SIGNAL=\"$2\"; ",
        "  for PID in $(pgrep -f \"$PATTERN\" 2>/dev/null || true); do ",
        "    [ \"$PID\" = \"$SELF\" ] && continue; ",
        "    [ \"$PID\" = \"$PARENT\" ] && continue; ",
        "    kill -\"$SIGNAL\" \"$PID\" 2>/dev/null || true; ",
        "  done; ",
        "}; ",
    ]
    for pattern in patterns:
        command.append(f"kill_pattern {shlex.quote(pattern)} TERM; ")
    command.append("sleep 1; ")
    for pattern in patterns:
        command.append(f"kill_pattern {shlex.quote(pattern)} KILL; ")
    command.append("sleep 0.5")
    return "".join(command)


def _launch_setup(context):
    use_sim_time = _as_bool(_get(context, "use_sim_time"))
    gui = _as_bool(_get(context, "gui"))
    rviz = _as_bool(_get(context, "rviz"))
    cleanup_stale = _as_bool(_get(context, "cleanup_stale"))
    enable_assets = _as_bool(_get(context, "enable_assets"))
    enable_perception = _as_bool(_get(context, "enable_perception"))
    enable_slam = _as_bool(_get(context, "enable_slam"))
    enable_control = _as_bool(_get(context, "enable_control"))
    enable_navigation = _as_bool(_get(context, "enable_navigation"))
    use_fast_lio = _as_bool(_get(context, "use_fast_lio"))
    pointcloud_noise_enabled = _as_bool(_get(context, "pointcloud_noise_enabled"))
    pointcloud_noise_mean = _get(context, "pointcloud_noise_mean").strip() or "0.0"
    pointcloud_noise_stddev = _get(context, "pointcloud_noise_stddev").strip() or "0.015"

    robot_ns = _get(context, "robot_namespace").strip().strip("/") or "robot"
    world = _get(context, "world")
    spawn_x = _get(context, "spawn_x")
    spawn_y = _get(context, "spawn_y")
    spawn_yaw = _get(context, "spawn_yaw")
    cfpa2_w_ig = float(_get(context, "cfpa2_w_ig"))
    cfpa2_w_c = float(_get(context, "cfpa2_w_c"))
    cfpa2_w_momentum = float(_get(context, "cfpa2_w_momentum"))
    cfpa2_min_utility = float(_get(context, "cfpa2_min_utility"))

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    champ_base_pkg = get_package_share_directory("champ_base")
    champ_gazebo_pkg = get_package_share_directory("champ_gazebo")
    cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")

    gazebo_config = os.path.join(champ_gazebo_pkg, "config", "gazebo.yaml")
    rviz_config = os.path.join(go2_gazebo_pkg, "rviz", "single_go2w_gazebo_cfpa2.rviz")
    go2w_control_pkg = get_package_share_directory("go2w_control")
    reactive_nav_profile = os.path.join(go2w_control_pkg, "config", "reactive_nav_single_go2w.yaml")
    mapper_profile = os.path.join(go2_gazebo_pkg, "config", "nav", "simple_scan_mapper_single_go2w.yaml")
    hybrid_motion_config = os.path.join(go2w_control_pkg, "config", "go2w_hybrid_motion.yaml")
    slam_config = os.path.join(go2_gazebo_pkg, "config", "slam", "pointlio_gazebo.yaml")
    cfpa2_config = os.path.join(cfpa2_pkg, "config", "cfpa2_single_robot.yaml")

    base_robot_description = xacro.process_file(
        os.path.join(go2_gazebo_pkg, "urdf", "go2w", "go2w_description_3d_lidar.xacro"),
        mappings={
            "pointcloud_noise_enabled": "true" if pointcloud_noise_enabled else "false",
            "pointcloud_noise_mean": pointcloud_noise_mean,
            "pointcloud_noise_stddev": pointcloud_noise_stddev,
        },
    ).documentElement.toxml()
    robot_description = build_namespaced_robot_description(
        base_robot_description,
        robot_ns,
        os.path.join(go2_gazebo_pkg, "config", "ros_control", "ros_control_go2w_robot.yaml"),
    )

    joints_config = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "joints.yaml")
    links_config = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "links.yaml")
    gait_config = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "gait.yaml")
    ekf_base_to_footprint = os.path.join(champ_base_pkg, "config", "ekf", "base_to_footprint.yaml")
    ekf_footprint_to_odom = os.path.join(champ_base_pkg, "config", "ekf", "footprint_to_odom.yaml")

    actions = [
        LogInfo(
            msg=(
                "[single_go2w_gazebo_cfpa2] "
                f"ns={robot_ns} assets={enable_assets} perception={enable_perception} "
                f"slam={enable_slam} control={enable_control} navigation={enable_navigation} "
                f"use_fast_lio={use_fast_lio} pointcloud_noise={pointcloud_noise_enabled} "
                f"pointcloud_noise_stddev={pointcloud_noise_stddev}"
            )
        )
    ]

    if cleanup_stale:
        actions.append(ExecuteProcess(cmd=["bash", "-lc", _build_cleanup_stale_command()], output="screen"))

    actions.append(
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
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
            ],
        )
    )

    if gui:
        actions.append(TimerAction(period=6.0, actions=[ExecuteProcess(cmd=["gzclient"], output="screen")]))

    if rviz:
        actions.append(
            TimerAction(
                period=7.0,
                actions=[build_rviz_node(rviz_config, use_sim_time, name="rviz2_single_go2w")],
            )
        )

    if enable_assets:
        actions.append(
            TimerAction(
                period=5.0,
                actions=build_dual_robot_stack(
                    ns=robot_ns,
                    spawn_x=spawn_x,
                    spawn_y=spawn_y,
                    spawn_yaw=spawn_yaw,
                    use_sim_time=use_sim_time,
                    robot_description=robot_description,
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
                    stand_up_joint_preset="go2w",
                    cmd_vel_input_topic="cmd_vel_legged",
                    wheel_controller_name=f"{robot_ns}_wheel_velocity_controller",
                    rsp_publish_frequency=100.0,
                ),
            )
        )

    robot_actions = []
    nav_odom_topic = f"/{robot_ns}/odom/nav"
    planning_scan_topic = f"/{robot_ns}/scan_3d"
    perception_cloud_topic = f"/{robot_ns}/registered_scan_reliable"

    if enable_control:
        robot_actions.append(
            Node(
                package="go2w_perception",
                executable="twist_bridge.py",
                namespace=robot_ns,
                remappings=[
                    ("/cmd_vel_stamped", f"/{robot_ns}/cmd_vel_stamped"),
                    ("/cmd_vel", f"/{robot_ns}/cmd_vel"),
                ],
                output="screen",
            )
        )
        robot_actions.append(
            Node(
                package="go2w_control",
                executable="go2w_hybrid_cmd_router.py",
                namespace=robot_ns,
                name="go2w_hybrid_cmd_router",
                parameters=[
                    hybrid_motion_config,
                    {"wheel_command_topic": f"{robot_ns}_wheel_velocity_controller/commands"},
                ],
                output="screen",
            )
        )
        # Hard sensor-level obstacle gating (replicates Unitree ObstaclesAvoidClient).
        # Publishes Int8(1) on /{ns}/stop when a wall patch is detected in front,
        # which reactive_nav reads as external_stop → immediate zero velocity.
        robot_actions.append(
            Node(
                package="go2w_control",
                executable="wall_collision_checker.py",
                namespace=robot_ns,
                name="wheel_wall_collision_checker",
                parameters=[
                    os.path.join(go2w_control_pkg, "config", "wall_checker.yaml"),
                    {"use_sim_time": use_sim_time},
                    {"scan_topic": planning_scan_topic},
                    {"stop_topic": f"/{robot_ns}/stop"},
                    {"mode_topic": "mobility_mode"},
                    # Legged mode defaults
                    {"safety_dist": 0.32},
                    {"check_angle_deg": 40.0},
                    {"min_close_points": 4},
                    # Wheel mode: wider cone and farther detection
                    {"wheel_safety_dist": 0.55},
                    {"wheel_check_angle_deg": 80.0},
                    {"wheel_min_close_points": 4},
                ],
                output="screen",
            )
        )

    if enable_perception:
        robot_actions.append(
            Node(
                package="go2w_perception",
                executable="qos_bridge.py",
                namespace=robot_ns,
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_topic": f"/{robot_ns}/registered_scan"},
                    {"output_topic": f"/{robot_ns}/registered_scan_reliable"},
                ],
                output="screen",
            )
        )

    if enable_perception and enable_slam and use_fast_lio:
        robot_actions.append(
            Node(
                package="go2w_perception",
                executable="pointcloud_adapter.py",
                namespace=robot_ns,
                name="pointcloud_adapter",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_topic": perception_cloud_topic},
                    {"output_topic": f"/{robot_ns}/velodyne_points"},
                    {"num_rings": 16},
                ],
                output="screen",
            )
        )
        robot_actions.append(
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                namespace=robot_ns,
                name="slam_node",
                parameters=[slam_config, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("/velodyne_points", f"/{robot_ns}/velodyne_points"),
                    ("/imu/data", f"/{robot_ns}/imu/data"),
                    ("/Odometry", f"/{robot_ns}/Odometry"),
                    ("/cloud_registered_body", f"/{robot_ns}/cloud_registered_body"),
                ],
                output="screen",
            )
        )

    if enable_slam:
        if use_fast_lio:
            robot_actions.append(
                build_slam_odom_relay_node(
                    ns=robot_ns,
                    use_sim_time=use_sim_time,
                    name="slam_odom_relay",
                    input_topic=f"/{robot_ns}/Odometry",
                    gt_topic=f"/{robot_ns}/odom/ground_truth",
                    output_topic=nav_odom_topic,
                    output_frame_id="world",
                    output_child_frame_id="base_link",
                    bootstrap_from_gt=True,
                    require_gt_for_alignment=True,
                )
            )
        else:
            robot_actions.append(
                build_slam_odom_relay_node(
                    ns=robot_ns,
                    use_sim_time=use_sim_time,
                    name="gt_odom_relay",
                    input_topic=f"/{robot_ns}/odom/ground_truth",
                    output_topic=nav_odom_topic,
                    output_frame_id="world",
                    output_child_frame_id="base_link",
                )
            )

    if enable_perception:
        # When Fast-LIO is active, use its motion-undistorted cloud.
        # Static TF imu→body (identity) on namespaced /robot/tf_static
        # connects Fast-LIO's 'body' frame to URDF chain:
        #   body → imu → base → base_link
        # No conflict with Fast-LIO's camera_init→body on global /tf
        # since pointcloud_to_laserscan only listens on /robot/tf.
        if use_fast_lio:
            scan_cloud_topic = f"/{robot_ns}/cloud_registered_body"
            robot_actions.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    namespace=robot_ns,
                    name="imu_to_body_tf",
                    arguments=["--frame-id", "imu", "--child-frame-id", "body",
                               "--x", "0", "--y", "0", "--z", "0",
                               "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1"],
                    remappings=[
                        ("/tf_static", f"/{robot_ns}/tf_static"),
                    ],
                    parameters=[{"use_sim_time": use_sim_time}],
                    output="log",
                )
            )
        else:
            scan_cloud_topic = perception_cloud_topic

        robot_actions.append(
            build_pointcloud_to_laserscan_node(
                ns=robot_ns,
                use_sim_time=use_sim_time,
                extra_params={
                    "target_frame": "base_link",
                    # LiDAR is at ~0.12m above base_link with 13° pitch.
                    # 0.05 filters out ground reflections and robot leg hits.
                    "min_height": 0.05,
                    "max_height": 0.60,
                    "range_min": 0.10,
                    "range_max": 8.0,
                },
                remappings=[
                    ("/tf", f"/{robot_ns}/tf"),
                    ("/tf_static", f"/{robot_ns}/tf_static"),
                    ("cloud_in", scan_cloud_topic),
                    ("scan", planning_scan_topic),
                ],
            )
        )


    if enable_navigation:
        robot_actions.extend(
            [
                Node(
                    package="go2_nav_algorithms",
                    executable="simple_scan_mapper_cpp",
                    namespace=robot_ns,
                    name="simple_scan_mapper_cpp",
                    parameters=[
                        os.path.join(
                            get_package_share_directory("go2_nav_algorithms"),
                            "config",
                            "nav",
                            "geometric_frontier_single.yaml",
                        ),
                        mapper_profile,
                        {
                            "use_sim_time": use_sim_time,
                            "scan_topic": planning_scan_topic,
                            "odom_topic": nav_odom_topic,
                            "map_topic": f"/{robot_ns}/map",
                            "map_frame": "world",
                            "startup_delay": 0.0,
                            # TF-based scan projection — interpolates odom at
                            # exact scan timestamps via tf2_ros::Buffer.
                            "scan_frame": "base_link",
                        },
                    ],
                    remappings=[("/tf", f"/{robot_ns}/tf"), ("/tf_static", f"/{robot_ns}/tf_static")],
                    output="screen",
                ),
                Node(
                    package="cfpa2_collaborative_autonomy",
                    executable="cfpa2_single_robot_node",
                    name="cfpa2_single_robot",
                    parameters=[
                        cfpa2_config,
                        {
                            "use_sim_time": use_sim_time,
                            "robot_namespace": robot_ns,
                            "namespaces": [robot_ns],
                            "goal_topic_suffix": "/way_point_coord",
                            "marker_frame_override": "world",
                            "cfpa2_w_ig": cfpa2_w_ig,
                            "cfpa2_w_c": cfpa2_w_c,
                            "cfpa2_w_momentum": cfpa2_w_momentum,
                            "cfpa2_min_utility": cfpa2_min_utility,
                        },
                    ],
                    output="screen",
                ),
            ]
        )

    if enable_control:
        robot_actions.extend(
            [
                Node(
                    package="go2w_control",
                    executable="autonomy_enabler.py",
                    namespace=robot_ns,
                    name="autonomy_enabler",
                    parameters=[{"use_sim_time": use_sim_time}, {"startup_delay": 8.0}, {"rate": 10.0}],
                    remappings=[("/way_point", f"/{robot_ns}/way_point_coord"), ("/joy", f"/{robot_ns}/joy")],
                    output="screen",
                ),
                Node(
                    package="go2w_control",
                    executable="reactive_nav.py",
                    namespace=robot_ns,
                    name="reactive_nav",
                    parameters=[
                        reactive_nav_profile,
                        {"use_sim_time": use_sim_time},
                        {
                            "frontier_replan_topic": f"/{robot_ns}/frontier_replan",
                            "stop_topic": f"/{robot_ns}/stop",
                        },
                    ],
                    remappings=[
                        ("/way_point", f"/{robot_ns}/way_point_coord"),
                        ("/odom/ground_truth", nav_odom_topic),
                        ("/scan", planning_scan_topic),
                        ("/cmd_vel_stamped", f"/{robot_ns}/cmd_vel_stamped"),
                        ("/nav_status", f"/{robot_ns}/nav_status"),
                        ("/planned_path", f"/{robot_ns}/planned_path"),
                        ("/robot_trajectory", f"/{robot_ns}/robot_trajectory"),
                        ("/final_goal_marker", f"/{robot_ns}/final_goal_marker"),
                        ("/robot_pose_marker", f"/{robot_ns}/robot_pose_marker"),
                    ],
                    output="screen",
                ),
            ]
        )
    # Exploration experiment logger — writes time-series CSV to /tmp
    robot_actions.append(
        Node(
            package="go2w_observability",
            executable="exploration_metrics_logger.py",
            namespace=robot_ns,
            name="exploration_metrics_logger",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"namespaces": [robot_ns]},
                {"experiment_name": "single_go2w"},
                {"log_rate": 1.0},
                {"output_dir": "/tmp"},
            ],
            output="screen",
        )
    )

    if robot_actions:
        actions.append(TimerAction(period=16.0, actions=robot_actions))

    return actions


def generate_launch_description():
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="robot"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("cleanup_stale", default_value="true"),
            DeclareLaunchArgument("enable_assets", default_value="true"),
            DeclareLaunchArgument("enable_perception", default_value="true"),
            DeclareLaunchArgument("enable_slam", default_value="true"),
            DeclareLaunchArgument("use_fast_lio", default_value="true"),
            DeclareLaunchArgument("pointcloud_noise_enabled", default_value="false"),
            DeclareLaunchArgument("pointcloud_noise_mean", default_value="0.0"),
            DeclareLaunchArgument("pointcloud_noise_stddev", default_value="0.015"),
            DeclareLaunchArgument("enable_control", default_value="true"),
            DeclareLaunchArgument("enable_navigation", default_value="true"),
            DeclareLaunchArgument("spawn_x", default_value="1.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("world", default_value=os.path.join(go2_gazebo_pkg, "worlds", "3.world")),
            DeclareLaunchArgument("cfpa2_w_ig", default_value="1.0", description="CFPA2 info-gain weight"),
            DeclareLaunchArgument("cfpa2_w_c", default_value="0.6", description="CFPA2 distance-cost weight"),
            DeclareLaunchArgument("cfpa2_w_momentum", default_value="0.8", description="CFPA2 momentum bonus weight"),
            DeclareLaunchArgument("cfpa2_min_utility", default_value="-0.5", description="CFPA2 min utility to assign a frontier (below = stop)"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
