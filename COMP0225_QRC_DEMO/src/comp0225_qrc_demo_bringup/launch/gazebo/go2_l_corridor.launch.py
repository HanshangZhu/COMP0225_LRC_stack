import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _strip_comments(node):
    for child in list(node.childNodes):
        if child.nodeType == child.COMMENT_NODE:
            node.removeChild(child)
        else:
            _strip_comments(child)


def _as_bool(context, key):
    return LaunchConfiguration(key).perform(context).strip().lower() in {"1", "true", "yes", "on"}


def _as_float(context, key, default):
    try:
        return float(LaunchConfiguration(key).perform(context))
    except (TypeError, ValueError):
        return float(default)


def _build_robot_config(demo_pkg, robot_variant):
    variant = robot_variant.strip().lower()
    if variant != "go2w":
        raise RuntimeError(
            "Single-robot Gazebo startup is wired for robot_variant:=go2w only. "
            "Set robot_variant:=go2w."
        )

    return {
        "description_path": os.path.join(demo_pkg, "urdf", "go2w", "go2w.urdf.xacro"),
        "joints_config": os.path.join(demo_pkg, "config", "deps", "go2w_config", "joints.yaml"),
        "links_config": os.path.join(demo_pkg, "config", "deps", "go2w_config", "links.yaml"),
        "gait_config": os.path.join(demo_pkg, "config", "deps", "champ_base", "gait_go2w.yaml"),
    }


def _build_launch_actions(context):
    demo_pkg = get_package_share_directory("comp0225_qrc_demo_bringup")
    champ_base_pkg = get_package_share_directory("champ_base")
    champ_gazebo_pkg = get_package_share_directory("champ_gazebo")

    use_sim_time = _as_bool(context, "use_sim_time")
    gui_enabled = _as_bool(context, "gui")
    rviz_enabled = _as_bool(context, "rviz")
    cleanup_stale = _as_bool(context, "cleanup_stale")
    robot_name = LaunchConfiguration("robot_name").perform(context).strip() or "go2w"
    robot_variant = LaunchConfiguration("robot_variant").perform(context)

    spawn_x = _as_float(context, "world_init_x", 2.5)
    spawn_y = _as_float(context, "world_init_y", 0.0)
    spawn_z = max(_as_float(context, "world_init_z", 0.45), 0.45)
    spawn_yaw = _as_float(context, "world_init_heading", 0.0)

    world_path = os.path.join(demo_pkg, "worlds", "l_corridor.world")
    gazebo_config = os.path.join(champ_gazebo_pkg, "config", "gazebo.yaml")
    rviz_path = os.path.join(demo_pkg, "rviz", "autonomy.rviz")

    robot_config = _build_robot_config(demo_pkg, robot_variant)
    doc = xacro.process_file(robot_config["description_path"])
    _strip_comments(doc)
    robot_description = doc.documentElement.toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            {"use_tf_static": False},
            {"publish_frequency": 200.0},
            {"ignore_timestamp": True},
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {"joint_controller_topic": "/joint_group_effort_controller/joint_trajectory"},
            {"urdf": ParameterValue(robot_description, value_type=str)},
            robot_config["joints_config"],
            robot_config["links_config"],
            robot_config["gait_config"],
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    state_estimator_node = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"orientation_from_imu": False},
            {"urdf": ParameterValue(robot_description, value_type=str)},
            robot_config["joints_config"],
            robot_config["links_config"],
            robot_config["gait_config"],
        ],
    )

    base_to_footprint_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="base_to_footprint_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": "base_link"},
            {"use_sim_time": use_sim_time},
            os.path.join(champ_base_pkg, "config", "ekf", "base_to_footprint.yaml"),
        ],
        remappings=[("odometry/filtered", "odom/local")],
    )

    footprint_to_odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="footprint_to_odom_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": "base_link"},
            {"use_sim_time": use_sim_time},
            os.path.join(champ_base_pkg, "config", "ekf", "footprint_to_odom.yaml"),
        ],
        remappings=[("odometry/filtered", "odom")],
    )

    spawn_entity_node = Node(
        package="go2_gazebo_sim",
        executable="spawn_entity_direct.py",
        output="screen",
        arguments=[
            "--entity",
            robot_name,
            "--topic",
            "/robot_description",
            "--x",
            f"{spawn_x:.6f}",
            "--y",
            f"{spawn_y:.6f}",
            "--z",
            f"{spawn_z:.6f}",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            f"{spawn_yaw:.6f}",
        ],
    )

    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "joint_states_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    load_joint_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "joint_group_effort_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    wait_joint_states_ready = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "until ros2 topic echo /joint_states --once >/dev/null 2>&1; do sleep 0.25; done",
        ],
        output="screen",
    )

    stand_up_node = Node(
        package="comp0225_qrc_demo_bringup",
        executable="stand_up_slowly.py",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"controller_wait_sec": 4.0},
            {"phase1_sec": 6.0},
            {"phase2_sec": 12.0},
            {"phase3_sec": 18.0},
            {"knee_bend_ratio": 0.80},
            {"joint_controller_topic": "/joint_group_effort_controller/joint_trajectory"},
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_path],
        output="screen",
    )

    actions = []

    if cleanup_stale:
        actions.append(
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    "pkill -9 -f '[g]zserver' || true; "
                    "pkill -9 -f '(^|/)gzclient( |$)' || true; "
                    "sleep 1",
                ],
                output="screen",
            )
        )

    actions.extend(
        [
            robot_state_publisher_node,
            quadruped_controller_node,
            state_estimator_node,
            base_to_footprint_ekf,
            footprint_to_odom_ekf,
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
                            world_path,
                            "--ros-args",
                            "--params-file",
                            gazebo_config,
                        ],
                        output="screen",
                    )
                ],
            ),
            spawn_entity_node,
            TimerAction(period=1.0, actions=[load_joint_state_controller]),
            TimerAction(period=1.2, actions=[load_joint_effort_controller]),
            TimerAction(period=4.0, actions=[wait_joint_states_ready]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_joint_states_ready,
                    on_exit=[stand_up_node],
                )
            ),
        ]
    )

    if gui_enabled:
        actions.append(TimerAction(period=6.0, actions=[ExecuteProcess(cmd=["gzclient"], output="screen")]))

    if rviz_enabled:
        actions.append(TimerAction(period=7.0, actions=[rviz_node]))

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
            DeclareLaunchArgument("gui", default_value="true", description="Run Gazebo GUI"),
            DeclareLaunchArgument("rviz", default_value="true", description="Run RViz"),
            DeclareLaunchArgument(
                "cleanup_stale",
                default_value="true",
                description="Kill stale Gazebo processes before launch",
            ),
            DeclareLaunchArgument(
                "robot_variant",
                default_value="go2w",
                description="Robot model variant. Only go2w is supported by this launch.",
            ),
            DeclareLaunchArgument("robot_name", default_value="go2w", description="Gazebo entity name"),
            DeclareLaunchArgument("world_init_x", default_value="2.5"),
            DeclareLaunchArgument("world_init_y", default_value="0.0"),
            DeclareLaunchArgument("world_init_z", default_value="0.45"),
            DeclareLaunchArgument("world_init_heading", default_value="0.0"),
            OpaqueFunction(function=_build_launch_actions),
        ]
    )
