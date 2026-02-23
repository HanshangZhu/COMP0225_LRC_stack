import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.append(os.path.dirname(__file__))

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


ROBOT_NS = "go2_1"


def generate_launch_description():
    pkg = get_package_share_directory("go2_issac_stack")
    pkg_prefix = get_package_prefix("go2_issac_stack")
    gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
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
    isaac_rtx_frame_skip = LaunchConfiguration("isaac_rtx_frame_skip")
    isaac_rtx_full_scan = LaunchConfiguration("isaac_rtx_full_scan")
    isaac_sim_command = LaunchConfiguration("isaac_sim_command")
    use_fast_lio = LaunchConfiguration("use_fast_lio")
    use_cpp_mapper = LaunchConfiguration("use_cpp_mapper")
    startup_delay = LaunchConfiguration("startup_delay")
    run_readiness_gate = LaunchConfiguration("run_readiness_gate")
    readiness_timeout_sec = LaunchConfiguration("readiness_timeout_sec")
    readiness_required_coverage = LaunchConfiguration("readiness_required_coverage")
    frontier_costmap_topic = LaunchConfiguration("frontier_costmap_topic")
    frontier_prefer_costmap = LaunchConfiguration("frontier_prefer_costmap")
    frontier_costmap_stale_sec = LaunchConfiguration("frontier_costmap_stale_sec")
    planning_scan_min_height = LaunchConfiguration("planning_scan_min_height")
    planning_scan_max_height = LaunchConfiguration("planning_scan_max_height")
    planning_scan_range_max = LaunchConfiguration("planning_scan_range_max")

    rviz_config = os.path.join(pkg, "rviz", "dual_isaac_autonomy.rviz")
    slam_config = os.path.join(pkg, "config", "slam", "pointlio_isaac.yaml")
    default_isaac_bringup_script = os.path.join(pkg_prefix, "lib", "go2_issac_stack", "isaac_t_world_bringup.py")
    default_world_file = os.path.join(gazebo_pkg, "worlds", "t_dual_corridor.world")
    default_robot_urdf = os.path.join(go2_description_pkg, "urdf", "go2_description.urdf")
    default_isaac_sim_command = PythonExpression(
        [
            "'python3 "
            + default_isaac_bringup_script
            + " --ros-namespace "
            + ROBOT_NS
            + " --world-file "
            + default_world_file
            + " --robot-urdf "
            + default_robot_urdf
            + " --loop-hz ' + '",
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
            " + ' --rtx-frame-skip ' + '",
            isaac_rtx_frame_skip,
            "'",
            " + (' --rtx-full-scan' if '",
            isaac_rtx_full_scan,
            "' == 'true' else '')",
            " + (' --headless' if '",
            isaac_headless,
            "' == 'true' else '')",
        ]
    )

    tf_remaps = [("/tf", f"/{ROBOT_NS}/tf"), ("/tf_static", f"/{ROBOT_NS}/tf_static")]
    nav_odom_topic = f"/{ROBOT_NS}/odom/nav"
    planning_scan_topic = f"/{ROBOT_NS}/scan_3d"

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
            "kill_pattern '[i]saac_t_world_bringup.py'; "
            "kill_pattern '/go2_issac_stack/lib/go2_issac_stack/isaac_topic_router.py'; "
            "kill_pattern '/go2_issac_stack/lib/go2_issac_stack/odom_tf_broadcaster.py'; "
            "kill_pattern '/go2_issac_stack/lib/go2_issac_stack/readiness_gate.py'; "
            "kill_pattern '/go2_issac_stack/lib/go2_issac_stack/simple_scan_mapper.py'; "
            "kill_pattern '/go2_issac_stack/lib/go2_issac_stack/simple_scan_mapper_cpp'; "
            "kill_pattern '/go2_issac_stack/lib/go2_issac_stack/simple_frontier_explorer.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/geometric_frontier.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/reactive_nav.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/autonomy_enabler.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/twist_bridge.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/qos_bridge.py'; "
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

    isaac_topic_router_node = build_isaac_topic_router_node(
        ns=ROBOT_NS,
        use_sim_time=use_sim_time,
        extra_params={
            "input_odom_topic": f"/{ROBOT_NS}/odom",
            "output_odom_topic": f"/{ROBOT_NS}/odom/ground_truth",
            "input_pointcloud_topic": f"/{ROBOT_NS}/lidar/points",
            "output_pointcloud_topic": f"/{ROBOT_NS}/registered_scan",
            "input_imu_topic": f"/{ROBOT_NS}/imu",
            "output_imu_topic": f"/{ROBOT_NS}/imu/data",
            "input_cmd_vel_topic": f"/{ROBOT_NS}/cmd_vel",
            "output_cmd_vel_topic": f"/{ROBOT_NS}/isaac/cmd_vel",
        },
    )

    twist_bridge_node = Node(
        package="go2_gazebo_sim",
        executable="twist_bridge.py",
        namespace=ROBOT_NS,
        remappings=[("/cmd_vel_stamped", f"/{ROBOT_NS}/cmd_vel_stamped"), ("/cmd_vel", f"/{ROBOT_NS}/cmd_vel")],
        output="screen",
    )

    qos_bridge_node = build_qos_bridge_node(
        ns=ROBOT_NS,
        use_sim_time=use_sim_time,
        extra_params={
            "input_topic": f"/{ROBOT_NS}/registered_scan",
            "output_topic": f"/{ROBOT_NS}/registered_scan_reliable",
        },
    )

    pointcloud_adapter_node = Node(
        package="go2_gazebo_sim",
        executable="pointcloud_adapter.py",
        namespace=ROBOT_NS,
        name="pointcloud_adapter",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": f"/{ROBOT_NS}/registered_scan_reliable"},
            {"output_topic": f"/{ROBOT_NS}/velodyne_points"},
            {"num_rings": 16},
        ],
        output="screen",
        condition=IfCondition(use_fast_lio),
    )

    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        namespace=ROBOT_NS,
        name="slam_node",
        parameters=[slam_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("/velodyne_points", f"/{ROBOT_NS}/velodyne_points"),
            ("/imu/data", f"/{ROBOT_NS}/imu/data"),
            ("/Odometry", f"/{ROBOT_NS}/Odometry"),
        ],
        output="screen",
        condition=IfCondition(use_fast_lio),
    )

    slam_odom_relay_node = Node(
        package="go2_gazebo_sim",
        executable="slam_odom_relay.py",
        namespace=ROBOT_NS,
        name="slam_odom_relay",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": f"/{ROBOT_NS}/Odometry"},
            {"gt_topic": f"/{ROBOT_NS}/odom/ground_truth"},
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
        namespace=ROBOT_NS,
        name="gt_odom_relay",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": f"/{ROBOT_NS}/odom/ground_truth"},
            {"output_topic": nav_odom_topic},
            {"output_frame_id": "world"},
            {"output_child_frame_id": "base_link"},
        ],
        output="screen",
        condition=UnlessCondition(use_fast_lio),
    )

    # REP-105 split:
    # - static world->odom anchor (global frame)
    # - dynamic odom->base_link from odometry (local continuous frame)
    world_to_odom_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        remappings=tf_remaps,
        output="screen",
    )

    # TF bridge: publish odom->base_link transform from nav odometry so RViz
    # and TF consumers have a valid local frame tree.
    odom_tf_broadcaster_node = build_odom_tf_broadcaster_node(
        ns=ROBOT_NS,
        use_sim_time=use_sim_time,
        extra_params={
            "odom_topic": nav_odom_topic,
            "parent_frame": "odom",
            "child_frame": "base_link",
            "max_publish_rate": 60.0,
        },
        remappings=tf_remaps,
    )

    # Exploration Algorithm Contract (single robot):
    # - Input topics: `planning_scan_topic` and `nav_odom_topic`.
    # - Goal output topic consumed downstream: `/<ns>/way_point`.
    # - If you replace the algorithm, keep these topic contracts (or update
    #   remappings/params in one place here) so the rest of the stack is unchanged.
    pointcloud_to_laserscan_node = build_pointcloud_to_laserscan_node(
        ns=ROBOT_NS,
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
            ("cloud_in", f"/{ROBOT_NS}/registered_scan_reliable"),
            ("scan", planning_scan_topic),
        ],
    )

    # Mapping plugin slot: choose C++ mapper by default for throughput, or set
    # `use_cpp_mapper:=false` to fall back to the Python implementation.
    mapper_params = {
        "scan_topic": planning_scan_topic,
        "odom_topic": nav_odom_topic,
        "map_topic": f"/{ROBOT_NS}/map",
        "map_frame": "world",
        "startup_delay": 0.0,
        # Isaac bridge jitter can exceed tight bounds; keep permissive.
        "max_scan_odom_dt": 1.00,
        "odom_history_sec": 2.0,
    }
    simple_scan_mapper_cpp_node = build_simple_scan_mapper_node(
        ns=ROBOT_NS,
        use_sim_time=use_sim_time,
        profile="geometric_frontier_dual.yaml",
        extra_params=mapper_params,
        executable="simple_scan_mapper_cpp",
        name="simple_scan_mapper_cpp",
        condition=IfCondition(use_cpp_mapper),
    )
    simple_scan_mapper_py_node = build_simple_scan_mapper_node(
        ns=ROBOT_NS,
        use_sim_time=use_sim_time,
        profile="geometric_frontier_dual.yaml",
        extra_params=mapper_params,
        executable="simple_scan_mapper.py",
        name="simple_scan_mapper_py",
        condition=UnlessCondition(use_cpp_mapper),
    )

    # Planner plugin slot: replace `frontier_explorer_node` with any planner
    # that publishes goals to `/<ns>/way_point` and optionally frontier markers.
    # By default this planner prefers Nav2's inflated costmap topic and falls
    # back to raw map if the costmap stream is missing/stale.
    frontier_explorer_node = build_geometric_frontier_node(
        ns=ROBOT_NS,
        use_sim_time=use_sim_time,
        profile="geometric_frontier_dual.yaml",
        extra_params={
            "odom_topic": nav_odom_topic,
            "map_topic": f"/{ROBOT_NS}/map",
            "costmap_topic": frontier_costmap_topic,
            "prefer_costmap": ParameterValue(frontier_prefer_costmap, value_type=bool),
            "costmap_stale_sec": ParameterValue(frontier_costmap_stale_sec, value_type=float),
            "frontier_goal_topic": f"/{ROBOT_NS}/way_point",
            "frontier_marker_topic": f"/{ROBOT_NS}/frontier_goal_marker",
            "frontier_regions_topic": f"/{ROBOT_NS}/frontier_markers",
            "frontier_replan_topic": f"/{ROBOT_NS}/frontier_replan",
            "map_frame": "world",
            "startup_delay": 0.0,
            "max_map_odom_dt": 1.00,
        },
        remappings=tf_remaps,
    )

    autonomy_enabler_node = build_autonomy_enabler_node(
        ns=ROBOT_NS,
        use_sim_time=use_sim_time,
        extra_params={"startup_delay": 8.0, "rate": 10.0},
        remappings=[("/way_point", f"/{ROBOT_NS}/way_point"), ("/joy", f"/{ROBOT_NS}/joy")],
    )

    # Controller plugin slot: replace `reactive_nav_node` with a controller that
    # consumes the planner goal topic and publishes `/<ns>/cmd_vel_stamped`.
    reactive_nav_node = build_reactive_nav_node(
        ns=ROBOT_NS,
        use_sim_time=use_sim_time,
        profile="reactive_nav_dual.yaml",
        extra_params={
            "frontier_replan_topic": f"/{ROBOT_NS}/frontier_replan",
            "stop_topic": f"/{ROBOT_NS}/stop",
        },
        remappings=[
            ("/way_point", f"/{ROBOT_NS}/way_point"),
            ("/odom/ground_truth", nav_odom_topic),
            ("/scan", planning_scan_topic),
            ("/cmd_vel_stamped", f"/{ROBOT_NS}/cmd_vel_stamped"),
            ("/nav_status", f"/{ROBOT_NS}/nav_status"),
        ],
    )

    status_monitor_node = Node(
        package="go2_gazebo_sim",
        executable="robot_status_monitor.py",
        name="robot_status_monitor",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"namespaces": [ROBOT_NS]},
            {"report_rate": 0.2},
            {"json_output": False},
        ],
        remappings=[
            (f"/{ROBOT_NS}/odom/ground_truth", f"/{ROBOT_NS}/odom/nav"),
            (f"/{ROBOT_NS}/way_point", f"/{ROBOT_NS}/way_point"),
        ],
        output="screen",
    )

    readiness_gate_node = Node(
        package="go2_issac_stack",
        executable="readiness_gate.py",
        name="single_robot_readiness_gate",
        parameters=[
            {"startup_timeout_sec": ParameterValue(readiness_timeout_sec, value_type=float)},
            {"required_coverage_ratio": ParameterValue(readiness_required_coverage, value_type=float)},
            {"odom_topic": f"/{ROBOT_NS}/odom/nav"},
            {"imu_topic": f"/{ROBOT_NS}/imu/data"},
            {"pointcloud_topic": f"/{ROBOT_NS}/registered_scan"},
            {"cmd_topic": f"/{ROBOT_NS}/isaac/cmd_vel"},
            {"map_topic": f"/{ROBOT_NS}/map"},
            {"nav_status_topic": f"/{ROBOT_NS}/nav_status"},
        ],
        output="screen",
        condition=IfCondition(run_readiness_gate),
    )

    # RViz must read the same TF topic namespace used by the robot stack.
    rviz_node = build_rviz_node(
        rviz_config,
        use_sim_time,
        condition=IfCondition(rviz),
        remappings=tf_remaps,
        name="rviz2_single_isaac",
    )

    core_bridge_nodes = [
        isaac_topic_router_node,
        twist_bridge_node,
        qos_bridge_node,
        pointcloud_adapter_node,
        fast_lio_node,
        slam_odom_relay_node,
        gt_odom_relay_node,
        world_to_odom_tf_node,
        odom_tf_broadcaster_node,
    ]
    # Keep exploration nodes isolated so swapping algorithms only changes this list.
    exploration_nodes = [
        pointcloud_to_laserscan_node,
        simple_scan_mapper_cpp_node,
        simple_scan_mapper_py_node,
        frontier_explorer_node,
        autonomy_enabler_node,
        reactive_nav_node,
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("cleanup_stale", default_value="true"),
            DeclareLaunchArgument("start_isaac_sim", default_value="true"),
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
            DeclareLaunchArgument("isaac_rtx_frame_skip", default_value="-1"),
            DeclareLaunchArgument("isaac_rtx_full_scan", default_value="false"),
            DeclareLaunchArgument(
                "isaac_sim_command",
                default_value=default_isaac_sim_command,
            ),
            DeclareLaunchArgument("use_fast_lio", default_value="false"),
            DeclareLaunchArgument(
                "use_cpp_mapper",
                default_value="true",
                description="Use C++ scan mapper for higher throughput; disable for Python fallback.",
            ),
            DeclareLaunchArgument("startup_delay", default_value="8.0"),
            DeclareLaunchArgument("run_readiness_gate", default_value="true"),
            DeclareLaunchArgument("readiness_timeout_sec", default_value="180.0"),
            DeclareLaunchArgument("readiness_required_coverage", default_value="0.80"),
            DeclareLaunchArgument(
                "frontier_costmap_topic",
                default_value=f"/{ROBOT_NS}/global_costmap/costmap",
                description="Inflated Nav2 costmap topic used by frontier planner when available.",
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
                description="Upper z clip used when converting point cloud to planning LaserScan.",
            ),
            DeclareLaunchArgument(
                "planning_scan_range_max",
                default_value="12.0",
                description="Max range used for planning LaserScan projection.",
            ),
            cleanup_stale_processes,
            TimerAction(period=1.0, actions=[start_isaac_sim_process]),
            TimerAction(
                period=startup_delay,
                actions=[
                    *core_bridge_nodes,
                    *exploration_nodes,
                    status_monitor_node,
                    readiness_gate_node,
                    rviz_node,
                ],
            ),
        ]
    )
