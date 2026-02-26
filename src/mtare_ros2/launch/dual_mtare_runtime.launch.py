import os
import shlex
import sys
from typing import Any

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.join(get_package_share_directory("go2_issac_stack"), "launch"))
sys.path.append(os.path.join(get_package_share_directory("go2_nav_algorithms"), "launch"))

from _dual_runtime_core import ROBOT_A_NS, ROBOT_B_NS, build_core_bridge_nodes, robot_topics
from _stack_components import (
    build_autonomy_enabler_node,
    build_geometric_frontier_node,
    build_rviz_node,
)
from pipeline_components import (
    build_goal_assigner_passthrough_node,
    build_pointcloud_to_laserscan_node,
    build_simple_scan_mapper_cpp_node,
    build_waypoint_mux_node,
)


def _as_bool(text: str) -> bool:
    return str(text).strip().lower() in {"1", "true", "yes", "on"}


def _normalize_backend(value: str) -> str:
    normalized = str(value).strip().lower()
    if normalized in {"", "auto"}:
        return "auto"
    if normalized in {"mtare_ros2", "mtare"}:
        return "mtare_ros2"
    if normalized in {"ros1_mtare", "ros1"}:
        return "ros1_mtare"
    if normalized in {"far_ros2", "far"}:
        return "far_ros2"
    if normalized in {"tare_ros2_exact", "exact"}:
        return "tare_ros2_exact"
    if normalized in {"none", "off"}:
        return "none"
    raise ValueError(
        "Unsupported planner_backend "
        f"'{value}'. Use one of: auto, mtare_ros2, ros1_mtare, far_ros2, tare_ros2_exact, none."
    )


def _build_reactive_nav_node(
    *,
    ns: str,
    use_sim_time: bool,
    reactive_nav_yaml: str,
    nav_odom_topic: str,
    planning_scan_topic: str,
    reactive_nav_startup_delay: float,
) -> Node:
    return Node(
        package="go2_gazebo_sim",
        executable="reactive_nav.py",
        namespace=ns,
        name="reactive_nav",
        parameters=[
            reactive_nav_yaml,
            {"use_sim_time": use_sim_time},
            {
                "frontier_replan_topic": f"/{ns}/frontier_replan",
                "stop_topic": f"/{ns}/stop",
                "startup_delay": reactive_nav_startup_delay,
            },
        ],
        remappings=[
            ("/way_point", f"/{ns}/way_point_coord"),
            ("/odom/ground_truth", nav_odom_topic),
            ("/scan", planning_scan_topic),
            ("/cmd_vel_stamped", f"/{ns}/cmd_vel_stamped"),
            ("/nav_status", f"/{ns}/nav_status"),
        ],
        output="screen",
    )


def _build_mtare_feeder_nodes(
    *,
    ns: str,
    use_sim_time: bool,
    nav_odom_topic: str,
) -> list[Any]:
    scan_topic = f"/{ns}/registered_scan_reliable"

    sensor_scan_generation_node = Node(
        package="sensor_scan_generation",
        executable="sensorScanGeneration",
        name=f"{ns}_sensor_scan_generation",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/state_estimation", nav_odom_topic),
            ("/registered_scan", scan_topic),
            ("/state_estimation_at_scan", f"/{ns}/state_estimation_at_scan"),
            ("/sensor_scan", f"/{ns}/sensor_scan"),
        ],
        output="screen",
    )

    terrain_analysis_node = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name=f"{ns}_terrain_analysis",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/state_estimation", nav_odom_topic),
            ("/registered_scan", scan_topic),
            ("/joy", f"/{ns}/joy"),
            ("/map_clearing", f"/{ns}/map_clearing"),
            ("/terrain_map", f"/{ns}/terrain_map"),
        ],
        output="screen",
    )

    terrain_analysis_ext_node = Node(
        package="terrain_analysis_ext",
        executable="terrainAnalysisExt",
        name=f"{ns}_terrain_analysis_ext",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/state_estimation", nav_odom_topic),
            ("/registered_scan", scan_topic),
            ("/joy", f"/{ns}/joy"),
            ("/cloud_clearing", f"/{ns}/cloud_clearing"),
            ("/terrain_map", f"/{ns}/terrain_map"),
            ("/terrain_map_ext", f"/{ns}/terrain_map_ext"),
        ],
        output="screen",
    )

    return [sensor_scan_generation_node, terrain_analysis_node, terrain_analysis_ext_node]


def _build_control_nodes(
    *,
    ns: str,
    use_sim_time: bool,
    tf_remaps: list[tuple[str, str]],
    nav_odom_topic: str,
    planning_scan_topic: str,
    reactive_nav_yaml: str,
    reactive_nav_startup_delay: float,
    planning_scan_min_height: float,
    planning_scan_max_height: float,
    planning_scan_range_min: float,
    planning_scan_range_max: float,
) -> list[Any]:
    pointcloud_to_laserscan_node = build_pointcloud_to_laserscan_node(
        ns=ns,
        use_sim_time=use_sim_time,
        extra_params={
            "target_frame": "base_link",
            "transform_tolerance": 2.0,
            "min_height": planning_scan_min_height,
            "max_height": planning_scan_max_height,
            "range_min": planning_scan_range_min,
            "range_max": planning_scan_range_max,
        },
        remappings=tf_remaps
        + [
            ("cloud_in", f"/{ns}/registered_scan_reliable"),
            ("scan", planning_scan_topic),
        ],
    )

    reactive_nav_node = _build_reactive_nav_node(
        ns=ns,
        use_sim_time=use_sim_time,
        reactive_nav_yaml=reactive_nav_yaml,
        nav_odom_topic=nav_odom_topic,
        planning_scan_topic=planning_scan_topic,
        reactive_nav_startup_delay=reactive_nav_startup_delay,
    )

    autonomy_enabler_node = build_autonomy_enabler_node(
        ns=ns,
        use_sim_time=use_sim_time,
        # Do not gate joy on first waypoint in this runtime to avoid planner/controller deadlocks.
        extra_params={"startup_delay": 8.0, "rate": 10.0, "wait_for_waypoint": False},
        remappings=[("/way_point", f"/{ns}/way_point_coord"), ("/joy", f"/{ns}/joy")],
    )

    return [pointcloud_to_laserscan_node, reactive_nav_node, autonomy_enabler_node]


def _build_mtare_map_viz_nodes(
    *,
    ns: str,
    use_sim_time: bool,
    tf_remaps: list[tuple[str, str]],
    nav_odom_topic: str,
    planning_scan_topic: str,
) -> list[Any]:
    mapper_node = build_simple_scan_mapper_cpp_node(
        ns=ns,
        use_sim_time=use_sim_time,
        profile="geometric_frontier_dual.yaml",
        name="simple_scan_mapper_cpp",
        extra_params={
            "scan_topic": planning_scan_topic,
            "odom_topic": nav_odom_topic,
            "map_topic": f"/{ns}/map",
            "map_frame": "world",
            "startup_delay": 0.0,
            "update_rate": 2.0,
            "max_scan_odom_dt": 0.0,
            "odom_history_sec": 2.0,
        },
        remappings=tf_remaps,
    )
    return [mapper_node]


def _build_far_goal_source_nodes(
    *,
    ns: str,
    use_sim_time: bool,
    tf_remaps: list[tuple[str, str]],
    nav_odom_topic: str,
    planning_scan_topic: str,
) -> list[Any]:
    mapper_node = build_simple_scan_mapper_cpp_node(
        ns=ns,
        use_sim_time=use_sim_time,
        profile="geometric_frontier_dual.yaml",
        name="simple_scan_mapper_cpp",
        extra_params={
            "scan_topic": planning_scan_topic,
            "odom_topic": nav_odom_topic,
            "map_topic": f"/{ns}/map",
            "map_frame": "world",
            "startup_delay": 0.0,
            "update_rate": 2.0,
            "max_scan_odom_dt": 0.0,
            "odom_history_sec": 2.0,
        },
        remappings=tf_remaps,
    )

    frontier_goal_source_node = build_geometric_frontier_node(
        ns=ns,
        use_sim_time=use_sim_time,
        profile="geometric_frontier_dual.yaml",
        name="frontier_far_goal_source",
        extra_params={
            "map_topic": f"/{ns}/map",
            "odom_topic": nav_odom_topic,
            "frontier_goal_topic": f"/{ns}/goal_point_raw",
            "frontier_marker_topic": f"/{ns}/frontier_goal_marker",
            "frontier_regions_topic": f"/{ns}/frontier_markers",
            "frontier_replan_topic": f"/{ns}/frontier_replan",
            "map_frame": "world",
            "startup_delay": 0.0,
            "update_rate": 2.0,
            "max_map_odom_dt": 0.0,
            "prefer_costmap": False,
            "costmap_topic": "",
        },
        remappings=tf_remaps,
    )

    return [mapper_node, frontier_goal_source_node]


def _build_far_goal_hold_node(*, use_sim_time: bool) -> Node:
    return build_goal_assigner_passthrough_node(
        use_sim_time=use_sim_time,
        namespaces=[ROBOT_A_NS, ROBOT_B_NS],
        input_topic_suffix="/goal_point_raw",
        output_topic_suffix="/goal_point",
        publish_rate=5.0,
        hold_last=True,
        name="far_goal_point_hold",
    )


def _build_far_waypoint_mux_node(*, use_sim_time: bool) -> Node:
    return build_waypoint_mux_node(
        use_sim_time=use_sim_time,
        namespaces=[ROBOT_A_NS, ROBOT_B_NS],
        primary_input_suffix="/way_point_far",
        fallback_input_suffix="/goal_point",
        output_suffix="/way_point_coord",
        primary_timeout_sec=1.0,
        output_rate=8.0,
        hold_last_output=True,
        stamp_now=True,
        name="far_waypoint_mux",
    )


def _build_mtare_coordinator_node(
    *,
    config_yaml: str,
    use_sim_time: bool,
    output_mode: str,
    goal_topic_suffix: str,
    use_shared_map: bool,
    shared_map_topic: str,
    shared_map_wait_sec: float,
    overlap_weight: float,
    communication_timeout_sec: float,
    prediction_horizon_sec: float,
    pursuit_weight: float,
    pursuit_switch_margin: float,
    exploration_gain_radius_cells: int,
    meeting_min_distance: float,
    teammate_stale_ttl_sec: float,
    marker_frame_override: str,
) -> Node:
    return Node(
        package="mtare_ros2",
        executable="mtare_coordinator.py",
        name="mtare_coordinator",
        parameters=[
            config_yaml,
            {"use_sim_time": use_sim_time},
            {"namespaces": [ROBOT_A_NS, ROBOT_B_NS]},
            {"goal_topic_suffix": goal_topic_suffix},
            {"output_mode": output_mode},
            {"algorithm_mode": "mui_tare"},
            {"use_shared_map": use_shared_map},
            {"shared_map_topic": shared_map_topic},
            {"shared_map_wait_sec": shared_map_wait_sec},
            {"overlap_weight": overlap_weight},
            {"communication_timeout_sec": communication_timeout_sec},
            {"prediction_horizon_sec": prediction_horizon_sec},
            {"pursuit_weight": pursuit_weight},
            {"pursuit_switch_margin": pursuit_switch_margin},
            {"exploration_gain_radius_cells": exploration_gain_radius_cells},
            {"meeting_min_distance": meeting_min_distance},
            {"teammate_stale_ttl_sec": teammate_stale_ttl_sec},
            {"marker_frame_override": marker_frame_override},
        ],
        output="screen",
    )


def _build_mtare_behavior_executive_node(*, use_sim_time: bool) -> Node:
    return Node(
        package="mtare_ros2",
        executable="mtare_behavior_executive_cpp",
        name="mtare_behavior_executive_cpp",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"namespaces": [ROBOT_A_NS, ROBOT_B_NS]},
            {"tare_input_suffix": "/way_point_tare"},
            {"waypoint_output_suffix": "/way_point_coord"},
            {"planner_mode_output_suffix": "/planner_mode"},
            {"enable_hysteresis_guard": False},
            {"enable_switch_lock_guard": False},
            {"enable_stale_guard": False},
            {"source_timeout_sec": 2.0},
            {"output_rate_hz": 8.0},
        ],
        output="screen",
    )


def _build_graph_decoder_node(*, use_sim_time: bool, use_shared_graph_bus: bool) -> Node:
    remappings = []
    if use_shared_graph_bus:
        remappings = [
            ("/robot_vgraph", "/mtare/robot_vgraph"),
            ("decoded_vgraph", "/mtare/decoded_vgraph"),
        ]
    return Node(
        package="graph_decoder",
        executable="graph_decoder",
        name="graph_decoder_exact",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=remappings,
        output="screen",
    )


def _build_far_planner_node(
    *,
    ns: str,
    use_sim_time: bool,
    far_config_yaml: str,
    world_frame: str,
    robot_id: int,
    nav_odom_topic: str,
    tf_remaps: list[tuple[str, str]],
    use_shared_graph_bus: bool,
) -> Node:
    params: list[Any] = [
        far_config_yaml,
        {"use_sim_time": use_sim_time},
        {"world_frame": world_frame},
        {"graph_msger/robot_id": robot_id},
    ]

    graph_remappings = [
        ("/robot_vgraph", "/mtare/robot_vgraph"),
        ("/decoded_vgraph", "/mtare/decoded_vgraph"),
    ] if use_shared_graph_bus else [
        ("/robot_vgraph", f"/{ns}/robot_vgraph"),
        ("/decoded_vgraph", f"/{ns}/decoded_vgraph"),
    ]

    return Node(
        package="go2_far_planner",
        executable="far_planner",
        namespace=ns,
        name="far_planner",
        parameters=params,
        remappings=[
            ("/odom_world", nav_odom_topic),
            ("/terrain_cloud", f"/{ns}/terrain_map_ext"),
            ("/scan_cloud", f"/{ns}/terrain_map"),
            ("/terrain_local_cloud", f"/{ns}/registered_scan"),
            ("/goal_point", f"/{ns}/goal_point"),
            ("/way_point", f"/{ns}/way_point_far"),
            ("/joy", f"/{ns}/joy"),
            ("/navigation_boundary", f"/{ns}/navigation_boundary"),
            ("/runtime", f"/{ns}/far_runtime"),
            ("/planning_time", f"/{ns}/far_planning_time"),
            *graph_remappings,
            *tf_remaps,
        ],
        output="screen",
    )


def _launch_setup(context):
    go2_issac_pkg = get_package_share_directory("go2_issac_stack")
    go2_issac_prefix = get_package_prefix("go2_issac_stack")

    workspace_root = os.path.dirname(os.path.dirname(go2_issac_prefix))
    default_mtare_ros1_ws = os.path.join(workspace_root, "src", "mtare_ros1_ws")

    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    use_fast_lio = _as_bool(LaunchConfiguration("use_fast_lio").perform(context))
    rviz_enabled = _as_bool(LaunchConfiguration("rviz").perform(context))
    start_isaac_sim = _as_bool(LaunchConfiguration("start_isaac_sim").perform(context))
    enable_mtare_process_manager = _as_bool(LaunchConfiguration("enable_mtare_process_manager").perform(context))

    mtare_scenario = LaunchConfiguration("mtare_scenario").perform(context).strip() or "indoor_go2_bridge"
    mtare_ros1_ws_path = LaunchConfiguration("mtare_ros1_ws_path").perform(context).strip() or default_mtare_ros1_ws
    mtare_bridge_start_delay = float(LaunchConfiguration("mtare_bridge_start_delay").perform(context))
    ros1_setup_bash = LaunchConfiguration("ros1_setup_bash").perform(context).strip() or "/opt/ros/noetic/setup.bash"
    planner_backend = _normalize_backend(LaunchConfiguration("planner_backend").perform(context))
    far_planner_config = LaunchConfiguration("far_planner_config").perform(context).strip() or "default"
    far_world_frame = LaunchConfiguration("far_world_frame").perform(context).strip() or "map"
    fastdds_transport = LaunchConfiguration("fastdds_transport").perform(context).strip() or "udp"
    rviz_tf_namespace = LaunchConfiguration("rviz_tf_namespace").perform(context)

    planning_scan_min_height = float(LaunchConfiguration("planning_scan_min_height").perform(context))
    planning_scan_max_height = float(LaunchConfiguration("planning_scan_max_height").perform(context))
    planning_scan_range_min = float(LaunchConfiguration("planning_scan_range_min").perform(context))
    planning_scan_range_max = float(LaunchConfiguration("planning_scan_range_max").perform(context))
    reactive_nav_startup_delay = float(LaunchConfiguration("reactive_nav_startup_delay").perform(context))
    mtare_waypoint_stale_timeout_sec = float(LaunchConfiguration("mtare_waypoint_stale_timeout_sec").perform(context))
    mtare_waypoint_republish_hz = float(LaunchConfiguration("mtare_waypoint_republish_hz").perform(context))
    mtare_marker_frame = LaunchConfiguration("mtare_marker_frame").perform(context).strip()
    mtare_waypoint_output_frame = LaunchConfiguration("mtare_waypoint_output_frame").perform(context).strip()
    enable_mtare_map_viz = _as_bool(LaunchConfiguration("enable_mtare_map_viz").perform(context))
    mtare_use_shared_map = _as_bool(LaunchConfiguration("mtare_use_shared_map").perform(context))
    require_shared_graph = _as_bool(LaunchConfiguration("require_shared_graph").perform(context))
    mtare_shared_map_topic = LaunchConfiguration("mtare_shared_map_topic").perform(context).strip()
    mtare_shared_map_wait_sec = float(LaunchConfiguration("mtare_shared_map_wait_sec").perform(context))
    mtare_overlap_weight = float(LaunchConfiguration("mtare_overlap_weight").perform(context))
    mtare_communication_timeout_sec = float(LaunchConfiguration("mtare_communication_timeout_sec").perform(context))
    mtare_prediction_horizon_sec = float(LaunchConfiguration("mtare_prediction_horizon_sec").perform(context))
    mtare_pursuit_weight = float(LaunchConfiguration("mtare_pursuit_weight").perform(context))
    mtare_pursuit_switch_margin = float(LaunchConfiguration("mtare_pursuit_switch_margin").perform(context))
    mtare_exploration_gain_radius_cells = int(LaunchConfiguration("mtare_exploration_gain_radius_cells").perform(context))
    mtare_meeting_min_distance = float(LaunchConfiguration("mtare_meeting_min_distance").perform(context))
    mtare_teammate_stale_ttl_sec = float(LaunchConfiguration("mtare_teammate_stale_ttl_sec").perform(context))

    slam_config = os.path.join(go2_issac_pkg, "config", "slam", "pointlio_isaac.yaml")
    reactive_nav_yaml = os.path.join(go2_issac_pkg, "config", "nav", "reactive_nav_dual.yaml")
    mtare_coordinator_yaml = os.path.join(get_package_share_directory("mtare_ros2"), "config", "mtare_ros2.yaml")

    ros1_ws_setup = os.path.join(mtare_ros1_ws_path, "devel", "setup.bash")
    ros1_available = os.path.isfile(ros1_setup_bash) and os.path.isfile(ros1_ws_setup)
    if planner_backend == "auto":
        selected_backend = "far_ros2"
    else:
        selected_backend = planner_backend

    manage_ros1_processes = enable_mtare_process_manager and selected_backend == "ros1_mtare"
    use_mtare_topic_bridge = selected_backend in ("ros1_mtare", "tare_ros2_exact")
    use_mtare_ros2_coordinator = selected_backend == "mtare_ros2"
    use_tare_ros2_exact = selected_backend == "tare_ros2_exact"

    if manage_ros1_processes and not ros1_available:
        raise RuntimeError(
            "planner_backend=ros1_mtare requested but ROS1 runtime is unavailable. "
            f"Missing one or both files: ros1_setup_bash={ros1_setup_bash}, ros1_ws_setup={ros1_ws_setup}. "
            "Install/provide ROS1 setup and built M-TARE ws, or explicitly select planner_backend:=far_ros2."
        )

    required_exact_packages = ("go2_tare_planner_ros2", "go2_far_planner")
    shared_graph_packages = ("graph_decoder", "visibility_graph_msg")
    missing_exact_packages: list[str] = []
    missing_shared_graph_packages: list[str] = []
    if use_tare_ros2_exact:
        for pkg_name in required_exact_packages:
            try:
                get_package_share_directory(pkg_name)
            except PackageNotFoundError:
                missing_exact_packages.append(pkg_name)
        for pkg_name in shared_graph_packages:
            try:
                get_package_share_directory(pkg_name)
            except PackageNotFoundError:
                missing_shared_graph_packages.append(pkg_name)
        if missing_exact_packages:
            raise RuntimeError(
                "planner_backend=tare_ros2_exact missing required package(s): "
                + ", ".join(missing_exact_packages)
                + ". Build/install them before launching exact backend."
            )
        if missing_shared_graph_packages and require_shared_graph:
            raise RuntimeError(
                "planner_backend=tare_ros2_exact requires shared graph packages "
                "(set require_shared_graph:=false for a degraded dev run). Missing: "
                + ", ".join(missing_shared_graph_packages)
            )

    use_shared_graph_bus = use_tare_ros2_exact and not missing_shared_graph_packages
    use_far_ros2_planner = selected_backend in {"far_ros2", "tare_ros2_exact"}

    far_config_yaml = ""
    far_config_available = False
    if use_far_ros2_planner:
        try:
            far_pkg = get_package_share_directory("go2_far_planner")
            candidate_far_yaml = os.path.join(far_pkg, "config", f"{far_planner_config}.yaml")
            if os.path.isfile(candidate_far_yaml):
                far_config_yaml = candidate_far_yaml
            else:
                far_config_yaml = os.path.join(far_pkg, "config", "default.yaml")
            far_config_available = os.path.isfile(far_config_yaml)
        except PackageNotFoundError:
            far_config_available = False

        if not far_config_available:
            if use_tare_ros2_exact:
                raise RuntimeError(
                    "planner_backend=tare_ros2_exact requested but go2_far_planner config is unavailable. "
                    "Verify go2_far_planner package build/install and far_planner_config value."
                )
            use_far_ros2_planner = False
            selected_backend = "none"

    robot_actions: list[Any] = []
    far_planner_nodes: list[Any] = []
    for ns in (ROBOT_A_NS, ROBOT_B_NS):
        tf_remaps, nav_odom_topic, planning_scan_topic = robot_topics(ns)
        core_bridge_nodes = build_core_bridge_nodes(
            ns=ns,
            use_sim_time=use_sim_time,
            use_fast_lio=use_fast_lio,
            slam_config=slam_config,
            tf_remaps=tf_remaps,
            nav_odom_topic=nav_odom_topic,
        )
        control_nodes = _build_control_nodes(
            ns=ns,
            use_sim_time=use_sim_time,
            tf_remaps=tf_remaps,
            nav_odom_topic=nav_odom_topic,
            planning_scan_topic=planning_scan_topic,
            reactive_nav_yaml=reactive_nav_yaml,
            reactive_nav_startup_delay=reactive_nav_startup_delay,
            planning_scan_min_height=planning_scan_min_height,
            planning_scan_max_height=planning_scan_max_height,
            planning_scan_range_min=planning_scan_range_min,
            planning_scan_range_max=planning_scan_range_max,
        )
        per_robot_actions = [*core_bridge_nodes, *control_nodes]
        if selected_backend in {"ros1_mtare", "far_ros2", "tare_ros2_exact"}:
            per_robot_actions.extend(
                _build_mtare_feeder_nodes(
                    ns=ns,
                    use_sim_time=use_sim_time,
                    nav_odom_topic=nav_odom_topic,
                )
            )
        if use_mtare_ros2_coordinator or use_tare_ros2_exact or (use_mtare_topic_bridge and enable_mtare_map_viz):
            per_robot_actions.extend(
                _build_mtare_map_viz_nodes(
                    ns=ns,
                    use_sim_time=use_sim_time,
                    tf_remaps=tf_remaps,
                    nav_odom_topic=nav_odom_topic,
                    planning_scan_topic=planning_scan_topic,
                )
            )
        if use_far_ros2_planner and far_config_available and not use_tare_ros2_exact:
            per_robot_actions.extend(
                _build_far_goal_source_nodes(
                    ns=ns,
                    use_sim_time=use_sim_time,
                    tf_remaps=tf_remaps,
                    nav_odom_topic=nav_odom_topic,
                    planning_scan_topic=planning_scan_topic,
                )
            )

        robot_actions.append(
            TimerAction(
                period=2.0,
                actions=per_robot_actions,
            )
        )

        if use_far_ros2_planner and far_config_available:
            far_planner_nodes.append(
                _build_far_planner_node(
                    ns=ns,
                    use_sim_time=use_sim_time,
                    far_config_yaml=far_config_yaml,
                    world_frame=far_world_frame,
                    robot_id=1 if ns == ROBOT_A_NS else 2,
                    nav_odom_topic=nav_odom_topic,
                    tf_remaps=tf_remaps,
                    use_shared_graph_bus=use_shared_graph_bus,
                )
            )

    rviz_config = os.path.join(go2_issac_pkg, "rviz", "dual_isaac_autonomy.rviz")
    rviz_node = build_rviz_node(
        rviz_config,
        use_sim_time,
        condition=IfCondition(str(rviz_enabled).lower()),
        remappings=[
            ("/tf", ["/", rviz_tf_namespace, "/tf"]),
            ("/tf_static", ["/", rviz_tf_namespace, "/tf_static"]),
        ],
        name="rviz2_dual_isaac_mtare",
    )

    mtare_bridge_node = Node(
        package="mtare_ros2",
        executable="mtare_topic_bridge.py",
        name="mtare_topic_bridge",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_a_ns": ROBOT_A_NS},
            {"robot_b_ns": ROBOT_B_NS},
            {"robot_a_mtare_name": "wheeled0"},
            {"robot_b_mtare_name": "wheeled1"},
            {"state_estimation_in_suffix": "state_estimation_at_scan"},
            {"state_estimation_out_suffix": "state_estimation_at_scan"},
            # Keep key-pose in the scan-synchronized map frame generated by sensorScanGeneration.
            {"key_pose_in_suffix": "state_estimation_at_scan"},
            {"key_pose_out_suffix": "key_pose_to_map"},
            # Feed planner with scan-synchronized cloud from sensorScanGeneration.
            {"registered_scan_in_suffix": "sensor_scan"},
            {"registered_scan_out_suffix": "registered_scan"},
            {"terrain_map_in_suffix": "terrain_map"},
            {"terrain_map_out_suffix": "terrain_map"},
            {"terrain_map_ext_in_suffix": "terrain_map_ext"},
            {"terrain_map_ext_out_suffix": "terrain_map_ext"},
            {"waypoint_in_suffix": "way_point"},
            {"waypoint_out_suffix": "way_point_coord"},
            {"hold_last_waypoint": True},
            {"waypoint_stale_timeout_sec": mtare_waypoint_stale_timeout_sec},
            {"waypoint_republish_hz": mtare_waypoint_republish_hz},
            {"marker_frame_override": mtare_marker_frame},
            {"waypoint_output_frame": mtare_waypoint_output_frame},
        ],
        output="screen",
        condition=IfCondition(str(use_mtare_topic_bridge).lower()),
    )

    coordinator_output_mode = "exact_split" if use_tare_ros2_exact else "waypoint_coord"
    mtare_coordinator_node = _build_mtare_coordinator_node(
        config_yaml=mtare_coordinator_yaml,
        use_sim_time=use_sim_time,
        output_mode=coordinator_output_mode,
        goal_topic_suffix="/way_point_coord",
        use_shared_map=mtare_use_shared_map,
        shared_map_topic=mtare_shared_map_topic,
        shared_map_wait_sec=mtare_shared_map_wait_sec,
        overlap_weight=mtare_overlap_weight,
        communication_timeout_sec=mtare_communication_timeout_sec,
        prediction_horizon_sec=mtare_prediction_horizon_sec,
        pursuit_weight=mtare_pursuit_weight,
        pursuit_switch_margin=mtare_pursuit_switch_margin,
        exploration_gain_radius_cells=mtare_exploration_gain_radius_cells,
        meeting_min_distance=mtare_meeting_min_distance,
        teammate_stale_ttl_sec=mtare_teammate_stale_ttl_sec,
        marker_frame_override=mtare_marker_frame,
    )
    mtare_behavior_executive_node = _build_mtare_behavior_executive_node(use_sim_time=use_sim_time)
    graph_decoder_node = _build_graph_decoder_node(
        use_sim_time=use_sim_time, use_shared_graph_bus=use_shared_graph_bus
    )

    roscore_cmd = f"source {shlex.quote(ros1_setup_bash)} && roscore"
    roscore_process = ExecuteProcess(
        cmd=["bash", "-lc", roscore_cmd],
        output="screen",
        condition=IfCondition(str(manage_ros1_processes).lower()),
    )

    mtare_launch_cmd = " && ".join(
        [
            f"source {shlex.quote(ros1_setup_bash)}",
            f"source {shlex.quote(os.path.join(mtare_ros1_ws_path, 'devel', 'setup.bash'))}",
            (
                "roslaunch --wait tare_planner explore_dual_go2_bridge.launch "
                f"scenario:={shlex.quote(mtare_scenario)} robot_num:=2 rviz:=false"
            ),
        ]
    )
    mtare_launch_process = ExecuteProcess(
        cmd=["bash", "-lc", mtare_launch_cmd],
        output="screen",
        condition=IfCondition(str(manage_ros1_processes).lower()),
    )

    ros1_bridge_cmd = " && ".join(
        [
            f"source {shlex.quote(ros1_setup_bash)}",
            "source /opt/ros/humble/setup.bash",
            f"source {shlex.quote(os.path.join(mtare_ros1_ws_path, 'devel', 'setup.bash'))}",
            f"source {shlex.quote(os.path.join(workspace_root, 'install', 'setup.bash'))}",
            "ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics --bridge-all-2to1-topics",
        ]
    )
    ros1_bridge_process = ExecuteProcess(
        cmd=["bash", "-lc", ros1_bridge_cmd],
        output="screen",
        condition=IfCondition(str(manage_ros1_processes).lower()),
    )

    summary = (
        "[mtare_bridge] "
        f"start_isaac_sim={start_isaac_sim} "
        f"use_fast_lio={use_fast_lio} "
        f"enable_mtare_process_manager={enable_mtare_process_manager} "
        f"manage_ros1_processes={manage_ros1_processes} "
        f"use_mtare_topic_bridge={use_mtare_topic_bridge} "
        f"use_mtare_ros2_coordinator={use_mtare_ros2_coordinator} "
        f"use_tare_ros2_exact={use_tare_ros2_exact} "
        f"planner_backend={planner_backend} "
        f"selected_backend={selected_backend} "
        f"require_shared_graph={require_shared_graph} "
        f"use_shared_graph_bus={use_shared_graph_bus} "
        f"missing_shared_graph_packages={missing_shared_graph_packages} "
        f"ros1_available={ros1_available} "
        f"fastdds_transport={fastdds_transport} "
        f"reactive_nav_startup_delay={reactive_nav_startup_delay:.1f} "
        f"mtare_waypoint_stale_timeout_sec={mtare_waypoint_stale_timeout_sec:.1f} "
        f"mtare_use_shared_map={mtare_use_shared_map} "
        f"mtare_overlap_weight={mtare_overlap_weight:.3f} "
        f"mtare_communication_timeout_sec={mtare_communication_timeout_sec:.2f} "
        f"mtare_pursuit_weight={mtare_pursuit_weight:.3f} "
        f"mtare_pursuit_switch_margin={mtare_pursuit_switch_margin:.3f} "
        f"enable_mtare_map_viz={enable_mtare_map_viz} "
        f"scenario={mtare_scenario} "
        f"mtare_ros1_ws_path={mtare_ros1_ws_path} "
        f"ros1_setup_bash={ros1_setup_bash}"
    )

    planner_actions: list[Any] = []
    if far_planner_nodes and not use_tare_ros2_exact:
        planner_actions.append(
            TimerAction(
                period=2.0,
                actions=[
                    _build_far_goal_hold_node(use_sim_time=use_sim_time),
                    _build_far_waypoint_mux_node(use_sim_time=use_sim_time),
                ],
            )
        )
        planner_actions.append(TimerAction(period=mtare_bridge_start_delay + 4.0, actions=far_planner_nodes))
    if use_tare_ros2_exact:
        planner_actions.append(TimerAction(period=2.0, actions=[mtare_behavior_executive_node]))
        if use_shared_graph_bus:
            planner_actions.append(TimerAction(period=2.0, actions=[graph_decoder_node]))
        planner_actions.append(TimerAction(period=8.0, actions=[mtare_coordinator_node]))
        planner_actions.append(TimerAction(period=mtare_bridge_start_delay + 4.0, actions=far_planner_nodes))
    elif use_mtare_ros2_coordinator:
        planner_actions.append(TimerAction(period=8.0, actions=[mtare_coordinator_node]))

    mtare_bridge_period = mtare_bridge_start_delay + 4.0 if manage_ros1_processes else 2.0

    return [
        LogInfo(msg=summary),
        *(
            [
                LogInfo(
                    msg=(
                        "[mtare_bridge] planner_backend=tare_ros2_exact running in degraded mode: "
                        f"shared graph packages missing={missing_shared_graph_packages} "
                        "(require_shared_graph:=false)."
                    )
                )
            ]
            if use_tare_ros2_exact and missing_shared_graph_packages and not require_shared_graph
            else []
        ),
        *robot_actions,
        TimerAction(period=mtare_bridge_start_delay, actions=[roscore_process]),
        TimerAction(period=mtare_bridge_start_delay + 2.0, actions=[mtare_launch_process]),
        TimerAction(period=mtare_bridge_start_delay + 4.0, actions=[ros1_bridge_process]),
        TimerAction(period=mtare_bridge_period, actions=[mtare_bridge_node]),
        *planner_actions,
        TimerAction(period=mtare_bridge_start_delay + 6.0, actions=[rviz_node]),
    ]


def generate_launch_description():
    go2_issac_prefix = get_package_prefix("go2_issac_stack")
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    go2_description_pkg = get_package_share_directory("go2_description")
    workspace_root = os.path.dirname(os.path.dirname(go2_issac_prefix))
    default_mtare_ros1_ws = os.path.join(workspace_root, "src", "mtare_ros1_ws")

    isaac_headless = LaunchConfiguration("isaac_headless")
    isaac_lidar_mode = LaunchConfiguration("isaac_lidar_mode")
    fastdds_transport = LaunchConfiguration("fastdds_transport")
    default_isaac_bringup_script = os.path.join(go2_issac_prefix, "lib", "go2_issac_stack", "isaac_t_world_dual_bringup.py")
    default_world_file = os.path.join(go2_gazebo_pkg, "worlds", "t_dual_corridor.world")
    default_robot_urdf = os.path.join(go2_description_pkg, "urdf", "go2_description.urdf")

    default_isaac_sim_command = PythonExpression(
        [
            "'python3 ",
            default_isaac_bringup_script,
            " --world-file ",
            default_world_file,
            " --robot-urdf ",
            default_robot_urdf,
            " --robot-a-namespace ",
            ROBOT_A_NS,
            " --robot-b-namespace ",
            ROBOT_B_NS,
            " --lidar-mode ",
            isaac_lidar_mode,
            "'",
            " + (' --headless' if '",
            isaac_headless,
            "' == 'true' else '')",
        ]
    )

    start_isaac_sim_process = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("start_isaac_sim")),
        cmd=["bash", "-lc", LaunchConfiguration("isaac_sim_command")],
        output="screen",
    )

    fastdds_udp_condition = IfCondition(PythonExpression(["'", fastdds_transport, "' == 'udp'"]))
    set_fastdds_udp_transport = SetEnvironmentVariable(
        name="FASTDDS_BUILTIN_TRANSPORTS",
        value="UDPv4",
        condition=fastdds_udp_condition,
    )
    set_fastdds_rmw = SetEnvironmentVariable(
        name="RMW_IMPLEMENTATION",
        value="rmw_fastrtps_cpp",
        condition=fastdds_udp_condition,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_tf_namespace", default_value=ROBOT_A_NS),
            DeclareLaunchArgument("start_isaac_sim", default_value="true"),
            DeclareLaunchArgument("isaac_headless", default_value="true"),
            DeclareLaunchArgument(
                "isaac_lidar_mode",
                default_value="cpu",
                description="Isaac dual bringup lidar backend: cpu | rtx.",
            ),
            DeclareLaunchArgument(
                "isaac_sim_command",
                default_value=default_isaac_sim_command,
                description="Shell command used when start_isaac_sim=true.",
            ),
            DeclareLaunchArgument(
                "use_fast_lio",
                default_value="false",
                description="Use FAST-LIO odometry for /<ns>/odom/nav.",
            ),
            DeclareLaunchArgument("planning_scan_min_height", default_value="-1.0"),
            DeclareLaunchArgument("planning_scan_max_height", default_value="2.0"),
            DeclareLaunchArgument("planning_scan_range_min", default_value="0.05"),
            DeclareLaunchArgument("planning_scan_range_max", default_value="12.0"),
            DeclareLaunchArgument(
                "reactive_nav_startup_delay",
                default_value="8.0",
                description="Startup gate (seconds) for reactive_nav in M-TARE runtime.",
            ),
            DeclareLaunchArgument(
                "mtare_scenario",
                default_value="indoor_go2_bridge",
                description="M-TARE scenario config name loaded by ROS1 launch.",
            ),
            DeclareLaunchArgument(
                "mtare_waypoint_stale_timeout_sec",
                default_value="20.0",
                description="Drop hold-last waypoint when older than this timeout (<=0 disables dropping).",
            ),
            DeclareLaunchArgument(
                "mtare_waypoint_republish_hz",
                default_value="5.0",
                description="Republish frequency for hold-last waypoint forwarding.",
            ),
            DeclareLaunchArgument(
                "mtare_marker_frame",
                default_value="world",
                description="Optional frame override for MTARE goal markers.",
            ),
            DeclareLaunchArgument(
                "mtare_waypoint_output_frame",
                default_value="world",
                description="Optional frame override for outgoing /<ns>/way_point_coord.",
            ),
            DeclareLaunchArgument(
                "enable_mtare_map_viz",
                default_value="true",
                description="Enable simple scan mapper in ros1_mtare mode for RViz occupancy map visualization.",
            ),
            DeclareLaunchArgument(
                "enable_mtare_process_manager",
                default_value="true",
                description="Manage roscore, ROS1 M-TARE launch and ros1_bridge from this launch.",
            ),
            DeclareLaunchArgument(
                "mtare_ros1_ws_path",
                default_value=default_mtare_ros1_ws,
                description="ROS1 catkin workspace root containing tare_planner.",
            ),
            DeclareLaunchArgument(
                "mtare_bridge_start_delay",
                default_value="10.0",
                description="Delay before managed M-TARE/bridge processes are started.",
            ),
            DeclareLaunchArgument(
                "ros1_setup_bash",
                default_value="/opt/ros/noetic/setup.bash",
                description="ROS1 setup.bash path used by managed roscore/roslaunch/ros1_bridge commands.",
            ),
            DeclareLaunchArgument(
                "planner_backend",
                default_value="far_ros2",
                description=(
                    "Planner backend: mtare_ros2 | ros1_mtare | far_ros2 | tare_ros2_exact | "
                    "none | auto (auto picks far_ros2)."
                ),
            ),
            DeclareLaunchArgument(
                "require_shared_graph",
                default_value="true",
                description=(
                    "When planner_backend=tare_ros2_exact, fail-fast unless graph_decoder and "
                    "visibility_graph_msg are available."
                ),
            ),
            DeclareLaunchArgument(
                "mtare_use_shared_map",
                default_value="false",
                description="Enable shared-map planning input for mtare_ros2 coordinator.",
            ),
            DeclareLaunchArgument(
                "mtare_shared_map_topic",
                default_value="/disco_slam/global_map",
                description="Shared map topic for mtare_ros2 coordinator when mtare_use_shared_map=true.",
            ),
            DeclareLaunchArgument(
                "mtare_shared_map_wait_sec",
                default_value="8.0",
                description="Wait before fail-open fallback when shared map is unavailable.",
            ),
            DeclareLaunchArgument(
                "mtare_overlap_weight",
                default_value="1.0",
                description="Mutual-exclusion overlap penalty weight for mtare_ros2 coordinator.",
            ),
            DeclareLaunchArgument(
                "mtare_communication_timeout_sec",
                default_value="6.0",
                description="Trigger pursuit when teammate updates are older than this timeout.",
            ),
            DeclareLaunchArgument(
                "mtare_prediction_horizon_sec",
                default_value="4.0",
                description="Prediction horizon for stale teammate pursuit target estimation.",
            ),
            DeclareLaunchArgument(
                "mtare_pursuit_weight",
                default_value="2.0",
                description="Relative utility weight for pursuit strategy in mtare_ros2 coordinator.",
            ),
            DeclareLaunchArgument(
                "mtare_pursuit_switch_margin",
                default_value="0.10",
                description="Required pursuit-vs-exploration utility margin before switching to pursuit.",
            ),
            DeclareLaunchArgument(
                "mtare_exploration_gain_radius_cells",
                default_value="4",
                description="Radius (cells) used to estimate frontier information gain in mtare_ros2 coordinator.",
            ),
            DeclareLaunchArgument(
                "mtare_meeting_min_distance",
                default_value="1.5",
                description="Minimum pursuit meeting distance (meters) to avoid trivial rendezvous goals.",
            ),
            DeclareLaunchArgument(
                "mtare_teammate_stale_ttl_sec",
                default_value="120.0",
                description="Ignore teammate pursuit beyond this stale age (seconds).",
            ),
            DeclareLaunchArgument(
                "far_planner_config",
                default_value="default",
                description=(
                    "Config basename from go2_far_planner/config/*.yaml when planner_backend="
                    "far_ros2 or tare_ros2_exact."
                ),
            ),
            DeclareLaunchArgument(
                "far_world_frame",
                default_value="map",
                description="world_frame parameter override for far_planner when planner_backend=far_ros2.",
            ),
            DeclareLaunchArgument(
                "fastdds_transport",
                default_value="udp",
                description="FastDDS transport policy: udp | auto. Use udp to avoid SHM port-lock errors.",
            ),
            set_fastdds_udp_transport,
            set_fastdds_rmw,
            TimerAction(period=1.0, actions=[start_isaac_sim_process]),
            OpaqueFunction(function=_launch_setup),
        ]
    )
