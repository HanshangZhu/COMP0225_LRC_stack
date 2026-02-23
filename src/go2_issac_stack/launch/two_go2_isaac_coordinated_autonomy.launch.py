import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.join(get_package_prefix("go2_issac_stack"), "lib", "go2_issac_stack"))
from isaac_robot_stack import build_isaac_robot_stack

from _stack_components import (
    build_autonomy_enabler_node,
    build_geometric_frontier_node,
    build_isaac_topic_router_node,
    build_odom_tf_broadcaster_node,
    build_pointcloud_to_laserscan_node,
    build_qos_bridge_node,
    build_reactive_nav_node,
    build_rviz_node,
    build_simple_scan_mapper_node,
)


ROBOT_A_NS = "go2_1"
ROBOT_B_NS = "go2_2"


def _robot_topics(ns: str):
    tf_remaps = [("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")]
    nav_odom_topic = f"/{ns}/odom/nav"
    planning_scan_topic = f"/{ns}/scan_3d"
    return tf_remaps, nav_odom_topic, planning_scan_topic


def _build_slam_nodes(ns: str, use_sim_time, use_fast_lio, slam_config, nav_odom_topic: str):
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

    return [
        pointcloud_adapter_node,
        fast_lio_node,
        slam_odom_relay_node,
        gt_odom_relay_node,
    ]


def _build_core_bridge_nodes(
    ns: str,
    use_sim_time,
    use_fast_lio,
    slam_config,
    tf_remaps,
    nav_odom_topic: str,
):
    isaac_topic_router_node = build_isaac_topic_router_node(
        ns=ns,
        use_sim_time=use_sim_time,
        extra_params={
            "input_odom_topic": f"/{ns}/odom",
            "output_odom_topic": f"/{ns}/odom/ground_truth",
            "input_pointcloud_topic": f"/{ns}/lidar/points",
            "output_pointcloud_topic": f"/{ns}/registered_scan",
            "input_imu_topic": f"/{ns}/imu",
            "output_imu_topic": f"/{ns}/imu/data",
            "input_cmd_vel_topic": f"/{ns}/cmd_vel",
            "output_cmd_vel_topic": f"/{ns}/isaac/cmd_vel",
        },
    )

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

    # REP-105 split:
    # - static world->odom anchor (global frame)
    # - dynamic odom->base_link from odometry (local continuous frame)
    world_to_odom_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=ns,
        name="world_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        remappings=tf_remaps,
        output="screen",
    )

    odom_tf_broadcaster_node = build_odom_tf_broadcaster_node(
        ns=ns,
        use_sim_time=use_sim_time,
        extra_params={
            "odom_topic": nav_odom_topic,
            "parent_frame": "odom",
            "child_frame": "base_link",
            "max_publish_rate": 60.0,
        },
        remappings=tf_remaps,
    )

    return [
        isaac_topic_router_node,
        twist_bridge_node,
        qos_bridge_node,
        *_build_slam_nodes(ns, use_sim_time, use_fast_lio, slam_config, nav_odom_topic),
        world_to_odom_tf_node,
        odom_tf_broadcaster_node,
    ]


def _build_exploration_nodes(
    ns: str,
    use_sim_time,
    use_cpp_mapper,
    tf_remaps,
    nav_odom_topic: str,
    planning_scan_topic: str,
    frontier_costmap_topic,
    frontier_prefer_costmap,
    frontier_costmap_stale_sec,
    planning_scan_min_height,
    planning_scan_max_height,
    planning_scan_range_max,
    mapper_update_rate,
    frontier_update_rate,
):
    # Exploration Algorithm Contract (per robot):
    # - Inputs: `planning_scan_topic` and `nav_odom_topic`.
    # - Goal output for centralized assignment path: `/<ns>/way_point_raw`.
    # - Assigned goal input for controller: `/<ns>/way_point_coord`.
    pointcloud_to_laserscan_node = build_pointcloud_to_laserscan_node(
        ns=ns,
        use_sim_time=use_sim_time,
        extra_params={
            "target_frame": "base_link",
            "transform_tolerance": 2.0,
            "min_height": ParameterValue(planning_scan_min_height, value_type=float),
            "max_height": ParameterValue(planning_scan_max_height, value_type=float),
            "range_max": ParameterValue(planning_scan_range_max, value_type=float),
        },
        remappings=tf_remaps
        + [
            ("cloud_in", f"/{ns}/registered_scan_reliable"),
            ("scan", planning_scan_topic),
        ],
    )

    mapper_params = {
        "scan_topic": planning_scan_topic,
        "odom_topic": nav_odom_topic,
        "map_topic": f"/{ns}/map",
        "map_frame": "world",
        "startup_delay": 0.0,
        "update_rate": ParameterValue(mapper_update_rate, value_type=float),
        # Isaac bridge jitter can exceed tight bounds; keep permissive.
        "max_scan_odom_dt": 1.00,
        "odom_history_sec": 2.0,
    }
    simple_scan_mapper_cpp_node = build_simple_scan_mapper_node(
        ns=ns,
        use_sim_time=use_sim_time,
        profile="geometric_frontier_dual.yaml",
        extra_params=mapper_params,
        executable="simple_scan_mapper_cpp",
        name="simple_scan_mapper_cpp",
        condition=IfCondition(use_cpp_mapper),
    )
    simple_scan_mapper_py_node = build_simple_scan_mapper_node(
        ns=ns,
        use_sim_time=use_sim_time,
        profile="geometric_frontier_dual.yaml",
        extra_params=mapper_params,
        executable="simple_scan_mapper.py",
        name="simple_scan_mapper_py",
        condition=UnlessCondition(use_cpp_mapper),
    )

    frontier_explorer_node = build_geometric_frontier_node(
        ns=ns,
        use_sim_time=use_sim_time,
        profile="geometric_frontier_dual.yaml",
        extra_params={
            "odom_topic": nav_odom_topic,
            "map_topic": f"/{ns}/map",
            "costmap_topic": frontier_costmap_topic,
            "prefer_costmap": ParameterValue(frontier_prefer_costmap, value_type=bool),
            "costmap_stale_sec": ParameterValue(frontier_costmap_stale_sec, value_type=float),
            "frontier_goal_topic": f"/{ns}/way_point_raw",
            "frontier_marker_topic": f"/{ns}/frontier_goal_marker",
            "frontier_regions_topic": f"/{ns}/frontier_markers",
            "frontier_replan_topic": f"/{ns}/frontier_replan",
            "map_frame": "world",
            "startup_delay": 0.0,
            "update_rate": ParameterValue(frontier_update_rate, value_type=float),
            "max_map_odom_dt": 5.00,
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
        pointcloud_to_laserscan_node,
        simple_scan_mapper_cpp_node,
        simple_scan_mapper_py_node,
        frontier_explorer_node,
        autonomy_enabler_node,
        reactive_nav_node,
    ]


def _robot_autonomy_actions(
    ns: str,
    use_sim_time,
    use_fast_lio,
    use_cpp_mapper,
    slam_config,
    startup_delay,
    frontier_costmap_topic,
    frontier_prefer_costmap,
    frontier_costmap_stale_sec,
    enable_dynamic_physics,
    enable_champ_stack,
    planning_scan_min_height,
    planning_scan_max_height,
    planning_scan_range_max,
    mapper_update_rate,
    frontier_update_rate,
    base_robot_description,
):
    tf_remaps, nav_odom_topic, planning_scan_topic = _robot_topics(ns)

    core_bridge_nodes = _build_core_bridge_nodes(
        ns=ns,
        use_sim_time=use_sim_time,
        use_fast_lio=use_fast_lio,
        slam_config=slam_config,
        tf_remaps=tf_remaps,
        nav_odom_topic=nav_odom_topic,
    )
    exploration_nodes = _build_exploration_nodes(
        ns=ns,
        use_sim_time=use_sim_time,
        use_cpp_mapper=use_cpp_mapper,
        tf_remaps=tf_remaps,
        nav_odom_topic=nav_odom_topic,
        planning_scan_topic=planning_scan_topic,
        frontier_costmap_topic=frontier_costmap_topic,
        frontier_prefer_costmap=frontier_prefer_costmap,
        frontier_costmap_stale_sec=frontier_costmap_stale_sec,
        planning_scan_min_height=planning_scan_min_height,
        planning_scan_max_height=planning_scan_max_height,
        planning_scan_range_max=planning_scan_range_max,
        mapper_update_rate=mapper_update_rate,
        frontier_update_rate=frontier_update_rate,
    )

    this_package = get_package_share_directory("go2_config")
    joints_config = os.path.join(this_package, "config", "joints", "joints.yaml")
    links_config = os.path.join(this_package, "config", "links", "links.yaml")
    gait_config = os.path.join(this_package, "config", "gait", "gait.yaml")
    champ_base_pkg = get_package_share_directory("champ_base")
    ekf_base_to_footprint = os.path.join(champ_base_pkg, "config", "ekf", "base_to_footprint.yaml")
    ekf_footprint_to_odom = os.path.join(champ_base_pkg, "config", "ekf", "footprint_to_odom.yaml")

    champ_nodes = build_isaac_robot_stack(
        ns=ns,
        use_sim_time=use_sim_time,
        robot_description=base_robot_description,
        joints_config=joints_config,
        links_config=links_config,
        gait_config=gait_config,
        ekf_base_to_footprint=ekf_base_to_footprint,
        ekf_footprint_to_odom=ekf_footprint_to_odom,
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    enable_dynamic_physics,
                    "' == 'true' and '",
                    enable_champ_stack,
                    "' == 'true'",
                ]
            )
        ),
    )

    bridge_node = Node(
        package="go2_issac_stack",
        executable="joint_trajectory_to_joint_state.py",
        namespace=ns,
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    enable_dynamic_physics,
                    "' == 'true' and '",
                    enable_champ_stack,
                    "' == 'true'",
                ]
            )
        ),
    )

    return [
        TimerAction(
            period=startup_delay,
            actions=[
                *core_bridge_nodes,
                *exploration_nodes,
                *champ_nodes,
                bridge_node,
            ],
        )
    ]


def generate_launch_description():
    go2_issac_pkg = get_package_share_directory("go2_issac_stack")
    go2_issac_prefix = get_package_prefix("go2_issac_stack")
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    go2_description_pkg = get_package_share_directory("go2_description")

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    cleanup_stale = LaunchConfiguration("cleanup_stale")
    start_isaac_sim = LaunchConfiguration("start_isaac_sim")
    isaac_headless = LaunchConfiguration("isaac_headless")
    isaac_loop_hz = LaunchConfiguration("isaac_loop_hz")
    isaac_renderer = LaunchConfiguration("isaac_renderer")
    isaac_lidar_rays = LaunchConfiguration("isaac_lidar_rays")
    isaac_pointcloud_hz = LaunchConfiguration("isaac_pointcloud_hz")
    isaac_robot_usd = LaunchConfiguration("isaac_robot_usd")
    isaac_robot_lidar_prim = LaunchConfiguration("isaac_robot_lidar_prim")
    isaac_lidar_mode = LaunchConfiguration("isaac_lidar_mode")
    isaac_rtx_lidar_config = LaunchConfiguration("isaac_rtx_lidar_config")
    isaac_rtx_frame_skip = LaunchConfiguration("isaac_rtx_frame_skip")
    isaac_rtx_full_scan = LaunchConfiguration("isaac_rtx_full_scan")
    enable_dynamic_physics = LaunchConfiguration("enable_dynamic_physics")
    robot_a_spawn_x = LaunchConfiguration("robot_a_spawn_x")
    robot_a_spawn_y = LaunchConfiguration("robot_a_spawn_y")
    robot_a_spawn_yaw = LaunchConfiguration("robot_a_spawn_yaw")
    robot_b_spawn_x = LaunchConfiguration("robot_b_spawn_x")
    robot_b_spawn_y = LaunchConfiguration("robot_b_spawn_y")
    robot_b_spawn_yaw = LaunchConfiguration("robot_b_spawn_yaw")
    isaac_sim_command = LaunchConfiguration("isaac_sim_command")
    use_fast_lio = LaunchConfiguration("use_fast_lio")
    use_cpp_mapper = LaunchConfiguration("use_cpp_mapper")
    use_shared_map = LaunchConfiguration("use_shared_map")
    shared_map_topic = LaunchConfiguration("shared_map_topic")
    autonomy_start_delay = LaunchConfiguration("autonomy_start_delay")
    rviz_tf_namespace = LaunchConfiguration("rviz_tf_namespace")
    frontier_costmap_topic_a = LaunchConfiguration("frontier_costmap_topic_a")
    frontier_costmap_topic_b = LaunchConfiguration("frontier_costmap_topic_b")
    frontier_prefer_costmap = LaunchConfiguration("frontier_prefer_costmap")
    frontier_costmap_stale_sec = LaunchConfiguration("frontier_costmap_stale_sec")
    planning_scan_min_height = LaunchConfiguration("planning_scan_min_height")
    planning_scan_max_height = LaunchConfiguration("planning_scan_max_height")
    planning_scan_range_max = LaunchConfiguration("planning_scan_range_max")
    mapper_update_rate = LaunchConfiguration("mapper_update_rate")
    frontier_update_rate = LaunchConfiguration("frontier_update_rate")
    enable_goal_assigner = LaunchConfiguration("enable_goal_assigner")
    enable_status_monitor = LaunchConfiguration("enable_status_monitor")
    enable_coverage_visualizer = LaunchConfiguration("enable_coverage_visualizer")
    enable_champ_stack = LaunchConfiguration("enable_champ_stack")
    physics_device = LaunchConfiguration("physics_device")

    rviz_config = os.path.join(go2_issac_pkg, "rviz", "dual_isaac_autonomy.rviz")
    slam_config = os.path.join(go2_issac_pkg, "config", "slam", "pointlio_isaac.yaml")
    default_isaac_bringup_script = os.path.join(go2_issac_prefix, "lib", "go2_issac_stack", "isaac_t_world_dual_bringup.py")
    default_world_file = os.path.join(go2_gazebo_pkg, "worlds", "t_dual_corridor.world")
    default_robot_urdf = os.path.join(go2_description_pkg, "urdf", "go2_description.urdf")

    default_isaac_sim_command = PythonExpression(
        [
            "'python3 "
            + default_isaac_bringup_script
            + " --world-file "
            + default_world_file
            + " --robot-urdf "
            + default_robot_urdf
            + " --robot-a-namespace "
            + ROBOT_A_NS
            + " --robot-b-namespace "
            + ROBOT_B_NS
            + " --robot-a-spawn-x ' + '",
            robot_a_spawn_x,
            "'",
            " + ' --robot-a-spawn-y ' + '",
            robot_a_spawn_y,
            "'",
            " + ' --robot-a-spawn-yaw ' + '",
            robot_a_spawn_yaw,
            "'",
            " + ' --robot-b-spawn-x ' + '",
            robot_b_spawn_x,
            "'",
            " + ' --robot-b-spawn-y ' + '",
            robot_b_spawn_y,
            "'",
            " + ' --robot-b-spawn-yaw ' + '",
            robot_b_spawn_yaw,
            "'",
            " + ' --loop-hz ' + '",
            isaac_loop_hz,
            "'",
            " + ' --renderer ' + '",
            isaac_renderer,
            "'",
            " + ' --lidar-rays ' + '",
            isaac_lidar_rays,
            "'",
            " + ' --pointcloud-hz ' + '",
            isaac_pointcloud_hz,
            "'",
            " + (' --robot-usd ' + '",
            isaac_robot_usd,
            "' if '",
            isaac_robot_usd,
            "' != '' else '')",
            " + (' --robot-lidar-prim ' + '",
            isaac_robot_lidar_prim,
            "' if '",
            isaac_robot_lidar_prim,
            "' != '' else '')",
            " + ' --lidar-mode ' + '",
            isaac_lidar_mode,
            "'",
            " + ' --rtx-lidar-config ' + '",
            isaac_rtx_lidar_config,
            "'",
            " + ' --rtx-frame-skip ' + '",
            isaac_rtx_frame_skip,
            "'",
            " + (' --rtx-full-scan' if '",
            isaac_rtx_full_scan,
            "' == 'true' else '')",
            " + (' --enable-dynamic-physics' if '",
            enable_dynamic_physics,
            "' == 'true' else '')",
            " + (' --headless' if '",
            isaac_headless,
            "' == 'true' else '')",
            " + ' --physics-device ' + '",
            physics_device,
            "'",
        ]
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
            "kill_pattern '[i]saac-sim.sh'; "
            "kill_pattern '[i]saac_t_world_dual_bringup.py'; "
            "kill_pattern '[p]ython.*go2_issac_stack/lib/go2_issac_stack/isaac_topic_router.py'; "
            "kill_pattern '/go2_issac_stack/lib/go2_issac_stack/odom_tf_broadcaster.py'; "
            "kill_pattern '/go2_issac_stack/lib/go2_issac_stack/simple_scan_mapper.py'; "
            "kill_pattern '/go2_issac_stack/lib/go2_issac_stack/simple_scan_mapper_cpp'; "
            "kill_pattern '/go2_issac_stack/lib/go2_issac_stack/simple_frontier_explorer.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/geometric_frontier.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/reactive_nav.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/autonomy_enabler.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/twist_bridge.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/qos_bridge.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/multi_robot_goal_assigner.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/robot_status_monitor.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/pointcloud_adapter.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/slam_odom_relay.py'; "
            "kill_pattern '/static_transform_publisher.*world.*odom'; "
            "kill_pattern '/fast_lio/lib/fast_lio/fastlio_mapping'; "
            "kill_pattern '/pointcloud_to_laserscan_node'; "
            "sleep 0.5",
        ],
        output="screen",
    )

    start_isaac_sim_process = ExecuteProcess(
        condition=IfCondition(start_isaac_sim),
        cmd=["bash", "-lc", isaac_sim_command],
        output="screen",
    )

    # RViz reads one robot TF namespace by default to avoid frame-id collisions
    # from two robots sharing `base_link` names.
    rviz_tf_topic = ["/", rviz_tf_namespace, "/tf"]
    rviz_tf_static_topic = ["/", rviz_tf_namespace, "/tf_static"]
    rviz_node = build_rviz_node(
        rviz_config,
        use_sim_time,
        condition=IfCondition(rviz),
        remappings=[("/tf", rviz_tf_topic), ("/tf_static", rviz_tf_static_topic)],
        name="rviz2_dual_isaac",
    )

    dual_coverage_visualizer_node = Node(
        package="go2_gazebo_sim",
        executable="dual_map_coverage_visualizer.py",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_a_map_topic": f"/{ROBOT_A_NS}/map"},
            {"robot_b_map_topic": f"/{ROBOT_B_NS}/map"},
            {"robot_a_odom_topic": f"/{ROBOT_A_NS}/odom/nav"},
            {"robot_b_odom_topic": f"/{ROBOT_B_NS}/odom/nav"},
            {"marker_topic": "/dual_robot/coverage_markers"},
            {"marker_frame": "world"},
            {"publish_rate": 1.0},
            {"min_map_value": 0},
            {"cell_stride": 1},
            {"robot_a_alpha": 0.20},
            {"robot_b_alpha": 0.20},
        ],
        output="screen",
        condition=IfCondition(enable_coverage_visualizer),
    )

    multi_robot_goal_assigner_node = Node(
        package="go2_gazebo_sim",
        executable="multi_robot_goal_assigner.py",
        name="multi_robot_goal_assigner",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"namespaces": [ROBOT_A_NS, ROBOT_B_NS]},
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
        condition=IfCondition(enable_goal_assigner),
    )

    robot_status_monitor_node = Node(
        package="go2_gazebo_sim",
        executable="robot_status_monitor.py",
        name="robot_status_monitor",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"namespaces": [ROBOT_A_NS, ROBOT_B_NS]},
            {"report_rate": 0.1},
            {"json_output": False},
        ],
        remappings=[
            (f"/{ROBOT_A_NS}/odom/ground_truth", f"/{ROBOT_A_NS}/odom/nav"),
            (f"/{ROBOT_B_NS}/odom/ground_truth", f"/{ROBOT_B_NS}/odom/nav"),
            (f"/{ROBOT_A_NS}/way_point", f"/{ROBOT_A_NS}/way_point_coord"),
            (f"/{ROBOT_B_NS}/way_point", f"/{ROBOT_B_NS}/way_point_coord"),
        ],
        output="screen",
        condition=IfCondition(enable_status_monitor),
    )

    description_path = os.path.join(get_package_share_directory("go2_gazebo_sim"), "urdf", "go2_description_3d_lidar.xacro")
    doc = xacro.process_file(description_path)
    base_robot_description = doc.documentElement.toxml()

    robot_actions = []
    for ns, costmap_topic in (
        (ROBOT_A_NS, frontier_costmap_topic_a),
        (ROBOT_B_NS, frontier_costmap_topic_b),
    ):
        robot_actions.extend(
            _robot_autonomy_actions(
                ns,
                use_sim_time,
                use_fast_lio,
                use_cpp_mapper,
                slam_config,
                autonomy_start_delay,
                costmap_topic,
                frontier_prefer_costmap,
                frontier_costmap_stale_sec,
                enable_dynamic_physics,
                enable_champ_stack,
                planning_scan_min_height,
                planning_scan_max_height,
                planning_scan_range_max,
                mapper_update_rate,
                frontier_update_rate,
                base_robot_description,
            )
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "cleanup_stale",
                default_value="true",
                description="Kill stale Isaac/autonomy bridge processes before start.",
            ),
            DeclareLaunchArgument(
                "start_isaac_sim",
                default_value="true",
                description="Launch internal dual-robot Isaac bringup command.",
            ),
            DeclareLaunchArgument("isaac_headless", default_value="true"),
            DeclareLaunchArgument("isaac_loop_hz", default_value="120.0"),
            DeclareLaunchArgument("isaac_renderer", default_value="HydraStorm"),
            DeclareLaunchArgument("isaac_lidar_rays", default_value="360"),
            DeclareLaunchArgument("isaac_pointcloud_hz", default_value="5.0"),
            DeclareLaunchArgument(
                "isaac_robot_usd",
                default_value="",
                description="Path to Unitree go2.usd (preferred). Empty keeps URDF fallback.",
            ),
            DeclareLaunchArgument(
                "isaac_robot_lidar_prim",
                default_value="",
                description="Optional lidar prim path in robot USD, absolute or relative under robot prim.",
            ),
            DeclareLaunchArgument("isaac_lidar_mode", default_value="rtx"),
            DeclareLaunchArgument(
                "isaac_rtx_lidar_config",
                default_value="Unitree_L1",
                description="RTX lidar profile name (e.g. Unitree_L1, Unitree/Unitree_L1, NVIDIA/Debug_Rotary).",
            ),
            DeclareLaunchArgument("isaac_rtx_frame_skip", default_value="-1"),
            DeclareLaunchArgument("isaac_rtx_full_scan", default_value="false"),
            DeclareLaunchArgument("enable_dynamic_physics", default_value="false"),
            DeclareLaunchArgument("robot_a_spawn_x", default_value="1.0"),
            DeclareLaunchArgument("robot_a_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_a_spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("robot_b_spawn_x", default_value="18.0"),
            DeclareLaunchArgument("robot_b_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_b_spawn_yaw", default_value="3.14159"),
            DeclareLaunchArgument(
                "physics_device",
                default_value="0",
                description="0 for GPU physics, -1 for CPU physics.",
            ),
            DeclareLaunchArgument(
                "isaac_sim_command",
                default_value=default_isaac_sim_command,
                description="Shell command used when start_isaac_sim=true.",
            ),
            DeclareLaunchArgument(
                "use_fast_lio",
                default_value="false",
                description="Use FAST-LIO odometry for autonomy odom input.",
            ),
            DeclareLaunchArgument(
                "use_cpp_mapper",
                default_value="true",
                description="Use C++ scan mapper for higher throughput; disable for Python fallback.",
            ),
            DeclareLaunchArgument(
                "use_shared_map",
                default_value="false",
                description="Use shared occupancy grid backend topic for coordinated assignment.",
            ),
            DeclareLaunchArgument(
                "shared_map_topic",
                default_value="/disco_slam/global_map",
                description="Shared occupancy grid topic published by backend.",
            ),
            DeclareLaunchArgument(
                "autonomy_start_delay",
                default_value="8.0",
                description="Delay before per-robot autonomy pipeline starts.",
            ),
            DeclareLaunchArgument(
                "rviz_tf_namespace",
                default_value=ROBOT_A_NS,
                description="Robot namespace whose TF tree RViz should subscribe to (go2_1 or go2_2).",
            ),
            DeclareLaunchArgument(
                "frontier_costmap_topic_a",
                default_value=f"/{ROBOT_A_NS}/global_costmap/costmap",
                description="Robot A inflated Nav2 costmap topic used by frontier planner.",
            ),
            DeclareLaunchArgument(
                "frontier_costmap_topic_b",
                default_value=f"/{ROBOT_B_NS}/global_costmap/costmap",
                description="Robot B inflated Nav2 costmap topic used by frontier planner.",
            ),
            DeclareLaunchArgument(
                "frontier_prefer_costmap",
                default_value="true",
                description="Prefer inflated costmap over raw map for safer frontier goals.",
            ),
            DeclareLaunchArgument(
                "frontier_costmap_stale_sec",
                default_value="2.0",
                description="Fallback to raw map when latest costmap message is older than this.",
            ),
            DeclareLaunchArgument(
                "planning_scan_min_height",
                default_value="-1.0",
                description="Lower z clip used when converting point cloud to planning LaserScan.",
            ),
            DeclareLaunchArgument(
                "planning_scan_max_height",
                default_value="2.0",
                description="Upper z clip used when converting point cloud to planning LaserScan. "
                            "Set high (2.0) so RTX lidar beams at ±45° elevation pass through "
                            "even for wall hits at max range.",
            ),
            DeclareLaunchArgument(
                "planning_scan_range_max",
                default_value="12.0",
                description="Max range used for planning LaserScan projection.",
            ),
            DeclareLaunchArgument(
                "mapper_update_rate",
                default_value="2.0",
                description="Map update loop rate for simple_scan_mapper nodes.",
            ),
            DeclareLaunchArgument(
                "frontier_update_rate",
                default_value="2.0",
                description="Frontier planner update loop rate.",
            ),
            DeclareLaunchArgument(
                "enable_goal_assigner",
                default_value="true",
                description="Enable centralized multi-robot goal assignment.",
            ),
            DeclareLaunchArgument(
                "enable_status_monitor",
                default_value="true",
                description="Enable robot status monitor node.",
            ),
            DeclareLaunchArgument(
                "enable_coverage_visualizer",
                default_value="true",
                description="Enable dual-map coverage visualization node.",
            ),
            DeclareLaunchArgument(
                "enable_champ_stack",
                default_value="true",
                description="Enable CHAMP/robot-localization stack when dynamic physics is enabled.",
            ),
            cleanup_stale_processes,
            TimerAction(period=1.0, actions=[start_isaac_sim_process]),
            TimerAction(period=2.0, actions=robot_actions),
            TimerAction(period=8.0, actions=[multi_robot_goal_assigner_node]),
            TimerAction(period=10.0, actions=[robot_status_monitor_node, dual_coverage_visualizer_node, rviz_node]),
        ]
    )
