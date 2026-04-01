import copy
import os
import sys
from typing import Any

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro

sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.join(get_package_prefix("go2_issac_stack"), "lib", "go2_issac_stack"))
sys.path.append(os.path.join(get_package_share_directory("go2_nav_algorithms"), "launch"))
from isaac_robot_stack import build_isaac_robot_stack

from _pipeline_registry import load_pipeline_runtime
from _dual_runtime_core import (
    ROBOT_A_NS,
    ROBOT_B_NS,
    build_core_bridge_nodes as _build_core_bridge_nodes_shared,
    robot_topics as _robot_topics_shared,
)
from _stack_components import (
    build_autonomy_enabler_node,
    build_isaac_topic_router_node,
    build_odom_tf_broadcaster_node,
    build_qos_bridge_node,
    build_rviz_node,
)
from pipeline_components import build_pointcloud_to_laserscan_node, build_simple_scan_mapper_cpp_node


def _as_bool(text: str) -> bool:
    return str(text).strip().lower() in {"1", "true", "yes", "on"}


def _optional_float(text: str) -> float | None:
    value = str(text).strip()
    if not value:
        return None
    return float(value)


def _optional_bool(text: str) -> bool | None:
    value = str(text).strip()
    if not value:
        return None
    return _as_bool(value)


def _robot_topics(ns: str) -> tuple[list[tuple[str, str]], str, str]:
    return _robot_topics_shared(ns)


def _build_slam_nodes(
    ns: str,
    use_sim_time: bool,
    use_fast_lio: bool,
    slam_config: str,
    nav_odom_topic: str,
) -> list[Any]:
    nodes: list[Any] = []

    if use_fast_lio:
        nodes.append(
            Node(
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
            )
        )
        nodes.append(
            Node(
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
            )
        )
        nodes.append(
            Node(
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
            )
        )
    else:
        nodes.append(
            Node(
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
            )
        )

    return nodes


def _build_core_bridge_nodes(
    ns: str,
    use_sim_time: bool,
    use_fast_lio: bool,
    slam_config: str,
    tf_remaps: list[tuple[str, str]],
    nav_odom_topic: str,
) -> list[Any]:
    return _build_core_bridge_nodes_shared(
        ns=ns,
        use_sim_time=use_sim_time,
        use_fast_lio=use_fast_lio,
        slam_config=slam_config,
        tf_remaps=tf_remaps,
        nav_odom_topic=nav_odom_topic,
    )


def _build_frontier_node(
    *,
    ns: str,
    use_sim_time: bool,
    tf_remaps: list[tuple[str, str]],
    frontier_module: dict[str, Any],
    use_legacy_yaml_fallback: bool,
    legacy_frontier_yaml: str,
    nav_odom_topic: str,
    frontier_costmap_topic: str,
    frontier_prefer_costmap: bool,
    frontier_costmap_stale_sec: float,
    mapping_params: dict[str, Any],
) -> Node:
    module_params = copy.deepcopy(frontier_module.get("params", {}))
    runtime_params = {
        "odom_topic": nav_odom_topic,
        "map_topic": f"/{ns}/map",
        "costmap_topic": frontier_costmap_topic,
        "prefer_costmap": frontier_prefer_costmap,
        "costmap_stale_sec": frontier_costmap_stale_sec,
        "frontier_goal_topic": f"/{ns}/way_point_raw",
        "frontier_marker_topic": f"/{ns}/frontier_goal_marker",
        "frontier_regions_topic": f"/{ns}/frontier_markers",
        "frontier_replan_topic": f"/{ns}/frontier_replan",
        "map_frame": "world",
        "startup_delay": 0.0,
        "update_rate": float(mapping_params.get("frontier_update_rate", 2.0)),
        "max_map_odom_dt": float(mapping_params.get("max_map_odom_dt", 0.5)),
    }

    params: list[Any] = []
    if use_legacy_yaml_fallback:
        params.append(legacy_frontier_yaml)
    params.append({"use_sim_time": use_sim_time})
    if isinstance(module_params, dict) and module_params:
        params.append(module_params)
    params.append(runtime_params)

    node_cfg = frontier_module["node"]
    return Node(
        package=node_cfg["package"],
        executable=node_cfg["executable"],
        namespace=ns,
        name=f"frontier_{frontier_module['id']}",
        parameters=params,
        remappings=tf_remaps,
        output="screen",
    )


def _build_mapper_node(
    *,
    ns: str,
    use_sim_time: bool,
    tf_remaps: list[tuple[str, str]],
    planning_scan_topic: str,
    nav_odom_topic: str,
    use_legacy_yaml_fallback: bool,
    mapping_params: dict[str, Any],
) -> Node:
    mapper_common = {
        "scan_topic": planning_scan_topic,
        "odom_topic": nav_odom_topic,
        "map_topic": f"/{ns}/map",
        "map_frame": "world",
        "startup_delay": 0.0,
        "update_rate": float(mapping_params.get("mapper_update_rate", 2.0)),
        "max_scan_odom_dt": float(mapping_params.get("max_scan_odom_dt", 0.1)),
        "odom_history_sec": float(mapping_params.get("odom_history_sec", 2.0)),
        "resolution": float(mapping_params.get("resolution", 0.1)),
        "width": int(mapping_params.get("width", 400)),
        "height": int(mapping_params.get("height", 400)),
        "origin_x": float(mapping_params.get("origin_x", -20.0)),
        "origin_y": float(mapping_params.get("origin_y", -20.0)),
        "max_range": float(mapping_params.get("max_range", 6.0)),
        "max_clear_distance": float(mapping_params.get("max_clear_distance", 2.0)),
    }

    profile = "geometric_frontier_dual.yaml" if use_legacy_yaml_fallback else None
    return build_simple_scan_mapper_cpp_node(
        ns=ns,
        use_sim_time=use_sim_time,
        profile=profile,
        extra_params=mapper_common,
        remappings=tf_remaps,
        name="simple_scan_mapper_cpp",
    )


def _build_default_nav_node(
    *,
    ns: str,
    use_sim_time: bool,
    use_legacy_yaml_fallback: bool,
    legacy_reactive_yaml: str,
    default_nav_params: dict[str, Any],
    nav_odom_topic: str,
    planning_scan_topic: str,
) -> Node:
    params: list[Any] = []
    if use_legacy_yaml_fallback:
        params.append(legacy_reactive_yaml)
    params.append({"use_sim_time": use_sim_time})
    params.append(copy.deepcopy(default_nav_params))
    params.append(
        {
            "frontier_replan_topic": f"/{ns}/frontier_replan",
            "stop_topic": f"/{ns}/stop",
        }
    )

    return Node(
        package="go2_gazebo_sim",
        executable="default_nav.py",
        namespace=ns,
        name="default_nav",
        parameters=params,
        remappings=[
            ("/way_point", f"/{ns}/way_point_coord"),
            ("/odom/ground_truth", nav_odom_topic),
            ("/scan", planning_scan_topic),
            ("/cmd_vel_stamped", f"/{ns}/cmd_vel_stamped"),
            ("/nav_status", f"/{ns}/nav_status"),
        ],
        output="screen",
    )


def _build_exploration_nodes(
    *,
    ns: str,
    use_sim_time: bool,
    tf_remaps: list[tuple[str, str]],
    nav_odom_topic: str,
    planning_scan_topic: str,
    frontier_costmap_topic: str,
    frontier_prefer_costmap: bool,
    frontier_costmap_stale_sec: float,
    planning_scan_min_height: float,
    planning_scan_max_height: float,
    planning_scan_range_min: float,
    planning_scan_range_max: float,
    frontier_module: dict[str, Any],
    use_legacy_yaml_fallback: bool,
    legacy_frontier_yaml: str,
    legacy_reactive_yaml: str,
    mapping_params: dict[str, Any],
    default_nav_params: dict[str, Any],
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

    mapper_node = _build_mapper_node(
        ns=ns,
        use_sim_time=use_sim_time,
        tf_remaps=tf_remaps,
        planning_scan_topic=planning_scan_topic,
        nav_odom_topic=nav_odom_topic,
        use_legacy_yaml_fallback=use_legacy_yaml_fallback,
        mapping_params=mapping_params,
    )

    frontier_explorer_node = _build_frontier_node(
        ns=ns,
        use_sim_time=use_sim_time,
        tf_remaps=tf_remaps,
        frontier_module=frontier_module,
        use_legacy_yaml_fallback=use_legacy_yaml_fallback,
        legacy_frontier_yaml=legacy_frontier_yaml,
        nav_odom_topic=nav_odom_topic,
        frontier_costmap_topic=frontier_costmap_topic,
        frontier_prefer_costmap=frontier_prefer_costmap,
        frontier_costmap_stale_sec=frontier_costmap_stale_sec,
        mapping_params=mapping_params,
    )

    autonomy_enabler_node = build_autonomy_enabler_node(
        ns=ns,
        use_sim_time=use_sim_time,
        extra_params={"startup_delay": 8.0, "rate": 10.0},
        remappings=[("/way_point", f"/{ns}/way_point_coord"), ("/joy", f"/{ns}/joy")],
    )

    default_nav_node = _build_default_nav_node(
        ns=ns,
        use_sim_time=use_sim_time,
        use_legacy_yaml_fallback=use_legacy_yaml_fallback,
        legacy_reactive_yaml=legacy_reactive_yaml,
        default_nav_params=default_nav_params,
        nav_odom_topic=nav_odom_topic,
        planning_scan_topic=planning_scan_topic,
    )

    return [
        pointcloud_to_laserscan_node,
        mapper_node,
        frontier_explorer_node,
        autonomy_enabler_node,
        default_nav_node,
    ]


def _robot_autonomy_actions(
    *,
    ns: str,
    use_sim_time: bool,
    use_fast_lio: bool,
    slam_config: str,
    startup_delay: float,
    frontier_costmap_topic: str,
    frontier_prefer_costmap: bool,
    frontier_costmap_stale_sec: float,
    enable_dynamic_physics: bool,
    enable_champ_stack: bool,
    planning_scan_min_height: float,
    planning_scan_max_height: float,
    planning_scan_range_min: float,
    planning_scan_range_max: float,
    base_robot_description: str,
    frontier_module: dict[str, Any],
    use_legacy_yaml_fallback: bool,
    legacy_frontier_yaml: str,
    legacy_reactive_yaml: str,
    mapping_params: dict[str, Any],
    default_nav_params: dict[str, Any],
) -> list[Any]:
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
        tf_remaps=tf_remaps,
        nav_odom_topic=nav_odom_topic,
        planning_scan_topic=planning_scan_topic,
        frontier_costmap_topic=frontier_costmap_topic,
        frontier_prefer_costmap=frontier_prefer_costmap,
        frontier_costmap_stale_sec=frontier_costmap_stale_sec,
        planning_scan_min_height=planning_scan_min_height,
        planning_scan_max_height=planning_scan_max_height,
        planning_scan_range_min=planning_scan_range_min,
        planning_scan_range_max=planning_scan_range_max,
        frontier_module=frontier_module,
        use_legacy_yaml_fallback=use_legacy_yaml_fallback,
        legacy_frontier_yaml=legacy_frontier_yaml,
        legacy_reactive_yaml=legacy_reactive_yaml,
        mapping_params=mapping_params,
        default_nav_params=default_nav_params,
    )

    champ_nodes: list[Any] = []
    bridge_node: list[Any] = []
    if enable_dynamic_physics and enable_champ_stack:
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
            condition=None,
        )

        bridge_node = [
            Node(
                package="go2_issac_stack",
                executable="joint_trajectory_to_joint_state.py",
                namespace=ns,
                output="screen",
            )
        ]

    return [
        TimerAction(
            period=startup_delay,
            actions=[
                *core_bridge_nodes,
                *exploration_nodes,
                *champ_nodes,
                *bridge_node,
            ],
        )
    ]


def _build_goal_assigner_node(
    *,
    use_sim_time: bool,
    goal_assigner_module: dict[str, Any],
    namespaces: list[str],
    use_shared_map: bool,
    shared_map_topic: str,
    enable_goal_assigner: bool,
) -> Node:
    if not enable_goal_assigner:
        return Node(
            package="go2_gazebo_sim",
            executable="goal_assigner_passthrough.py",
            name="goal_assigner_passthrough_disabled",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"namespaces": namespaces},
                {"input_topic_suffix": "/way_point_raw"},
                {"output_topic_suffix": "/way_point_assigned"},
                {"publish_rate": 0.0},
                {"hold_last": True},
            ],
            output="screen",
        )

    module_node = goal_assigner_module["node"]
    module_params = copy.deepcopy(goal_assigner_module.get("params", {}))
    runtime_params = {
        "namespaces": namespaces,
        "goal_topic_suffix": "/way_point_assigned",
    }

    if module_node["executable"] == "multi_robot_goal_assigner.py":
        runtime_params.update(
            {
                "use_shared_map": use_shared_map,
                "shared_map_topic": shared_map_topic,
                "shared_map_wait_sec": 8.0,
            }
        )
    if module_node["executable"] == "goal_assigner_passthrough.py":
        runtime_params.update(
            {
                "input_topic_suffix": "/way_point_raw",
                "output_topic_suffix": "/way_point_assigned",
            }
        )

    params = [{"use_sim_time": use_sim_time}]
    if isinstance(module_params, dict) and module_params:
        params.append(module_params)
    params.append(runtime_params)

    return Node(
        package=module_node["package"],
        executable=module_node["executable"],
        name=f"goal_assigner_{goal_assigner_module['id']}",
        parameters=params,
        output="screen",
    )


def _build_global_planner_node(
    *,
    use_sim_time: bool,
    global_planner_module: dict[str, Any],
    namespaces: list[str],
) -> Node:
    mode = str(global_planner_module.get("mode", "")).strip().lower()
    module_params = copy.deepcopy(global_planner_module.get("params", {}))

    params = [
        {"use_sim_time": use_sim_time},
        {"namespaces": namespaces},
        {"input_topic_suffix": "/way_point_assigned"},
        {"output_topic_suffix": "/way_point_coord"},
    ]
    if isinstance(module_params, dict) and module_params:
        params.append(module_params)

    if mode == "passthrough":
        return Node(
            package="go2_gazebo_sim",
            executable="goal_assigner_passthrough.py",
            name="global_planner_none_passthrough",
            parameters=params,
            output="screen",
        )

    module_node = global_planner_module["node"]
    return Node(
        package=module_node["package"],
        executable=module_node["executable"],
        name=f"global_planner_{global_planner_module['id']}",
        parameters=params,
        output="screen",
    )


def _launch_setup(context):
    go2_issac_pkg = get_package_share_directory("go2_issac_stack")

    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    rviz_enabled = _as_bool(LaunchConfiguration("rviz").perform(context))
    use_fast_lio = _as_bool(LaunchConfiguration("use_fast_lio").perform(context))
    use_shared_map = _as_bool(LaunchConfiguration("use_shared_map").perform(context))
    enable_goal_assigner = _as_bool(LaunchConfiguration("enable_goal_assigner").perform(context))
    enable_status_monitor = _as_bool(LaunchConfiguration("enable_status_monitor").perform(context))
    enable_coverage_visualizer = _as_bool(LaunchConfiguration("enable_coverage_visualizer").perform(context))
    enable_frontier_distance_plot = _as_bool(LaunchConfiguration("enable_frontier_distance_plot").perform(context))
    enable_dynamic_physics = _as_bool(LaunchConfiguration("enable_dynamic_physics").perform(context))
    enable_champ_stack = _as_bool(LaunchConfiguration("enable_champ_stack").perform(context))
    use_legacy_yaml_fallback = _as_bool(LaunchConfiguration("use_legacy_yaml_fallback").perform(context))

    autonomy_start_delay = float(LaunchConfiguration("autonomy_start_delay").perform(context))
    planning_scan_min_height = float(LaunchConfiguration("planning_scan_min_height").perform(context))
    planning_scan_max_height = float(LaunchConfiguration("planning_scan_max_height").perform(context))
    planning_scan_range_min = float(LaunchConfiguration("planning_scan_range_min").perform(context))
    planning_scan_range_max = float(LaunchConfiguration("planning_scan_range_max").perform(context))
    frontier_costmap_stale_sec = float(LaunchConfiguration("frontier_costmap_stale_sec").perform(context))
    frontier_distance_sample_rate = float(LaunchConfiguration("frontier_distance_sample_rate").perform(context))
    frontier_distance_plot_write_rate = float(LaunchConfiguration("frontier_distance_plot_write_rate").perform(context))

    frontier_costmap_topic_a = LaunchConfiguration("frontier_costmap_topic_a").perform(context)
    frontier_costmap_topic_b = LaunchConfiguration("frontier_costmap_topic_b").perform(context)
    frontier_prefer_costmap = _as_bool(LaunchConfiguration("frontier_prefer_costmap").perform(context))
    shared_map_topic = LaunchConfiguration("shared_map_topic").perform(context)
    rviz_tf_namespace = LaunchConfiguration("rviz_tf_namespace").perform(context)
    frontier_distance_plot_path = LaunchConfiguration("frontier_distance_plot_path").perform(context)

    pipeline_config_json = LaunchConfiguration("pipeline_config_json").perform(context)
    frontier_planner_id = LaunchConfiguration("frontier_planner_id").perform(context).strip()
    goal_assigner_id = LaunchConfiguration("goal_assigner_id").perform(context).strip()
    global_planner_id = LaunchConfiguration("global_planner_id").perform(context).strip()

    legacy_frontier_yaml = os.path.join(go2_issac_pkg, "config", "nav", "geometric_frontier_dual.yaml")
    legacy_reactive_yaml = os.path.join(go2_issac_pkg, "config", "nav", "default_nav_dual.yaml")

    mapping_overrides = {
        "max_scan_odom_dt": _optional_float(LaunchConfiguration("max_scan_odom_dt").perform(context)),
        "max_map_odom_dt": _optional_float(LaunchConfiguration("max_map_odom_dt").perform(context)),
        "odom_history_sec": _optional_float(LaunchConfiguration("odom_history_sec").perform(context)),
        "mapper_update_rate": _optional_float(LaunchConfiguration("mapper_update_rate").perform(context)),
        "frontier_update_rate": _optional_float(LaunchConfiguration("frontier_update_rate").perform(context)),
    }
    default_nav_overrides = {
        "max_linear_speed": _optional_float(LaunchConfiguration("default_nav_max_linear_speed").perform(context)),
        "obstacle_slow_dist": _optional_float(LaunchConfiguration("default_nav_obstacle_slow_dist").perform(context)),
        "obstacle_stop_dist": _optional_float(LaunchConfiguration("default_nav_obstacle_stop_dist").perform(context)),
        "planner_grid_radius": _optional_float(LaunchConfiguration("default_nav_planner_grid_radius").perform(context)),
        "planner_goal_clip_distance": _optional_float(
            LaunchConfiguration("default_nav_planner_goal_clip_distance").perform(context)
        ),
        "wall_scan_enabled": _optional_bool(LaunchConfiguration("default_nav_wall_scan_enabled").perform(context)),
    }

    pipeline_runtime = load_pipeline_runtime(
        pipeline_config_json=pipeline_config_json,
        frontier_planner_id=frontier_planner_id,
        goal_assigner_id=goal_assigner_id,
        global_planner_id=global_planner_id,
        use_legacy_yaml_fallback=use_legacy_yaml_fallback,
        legacy_frontier_yaml=legacy_frontier_yaml,
        legacy_default_nav_yaml=legacy_reactive_yaml,
        mapping_overrides=mapping_overrides,
        default_nav_overrides=default_nav_overrides,
    )

    selected_ids = pipeline_runtime["selected_ids"]
    mapping_params = pipeline_runtime["mapping_params"]
    default_nav_params = pipeline_runtime["default_nav_params"]
    frontier_module = pipeline_runtime["frontier_module"]
    goal_assigner_module = pipeline_runtime["goal_assigner_module"]
    global_planner_module = pipeline_runtime["global_planner_module"]

    summary = (
        "[pipeline] "
        f"config={pipeline_config_json} "
        f"frontier={selected_ids['frontier_planner_id']} "
        f"goal_assigner={selected_ids['goal_assigner_id']} "
        f"global={selected_ids['global_planner_id']} "
        f"legacy_yaml_fallback={use_legacy_yaml_fallback} "
        f"fallback_sources={pipeline_runtime['fallback_sources'] if pipeline_runtime['fallback_sources'] else 'none'} "
        f"mapping(max_scan_odom_dt={mapping_params.get('max_scan_odom_dt')}, "
        f"max_map_odom_dt={mapping_params.get('max_map_odom_dt')}, "
        f"odom_history_sec={mapping_params.get('odom_history_sec')}, "
        f"mapper_update_rate={mapping_params.get('mapper_update_rate')}, "
        f"frontier_update_rate={mapping_params.get('frontier_update_rate')}) "
        f"default_nav(max_linear_speed={default_nav_params.get('max_linear_speed')}, "
        f"obstacle_slow_dist={default_nav_params.get('obstacle_slow_dist')}, "
        f"obstacle_stop_dist={default_nav_params.get('obstacle_stop_dist')}, "
        f"planner_grid_radius={default_nav_params.get('planner_grid_radius')}, "
        f"planner_goal_clip_distance={default_nav_params.get('planner_goal_clip_distance')}, "
        f"wall_scan_enabled={default_nav_params.get('wall_scan_enabled')})"
    )

    slam_config = os.path.join(go2_issac_pkg, "config", "slam", "pointlio_isaac.yaml")
    description_path = os.path.join(get_package_share_directory("go2_gazebo_sim"), "urdf", "go2_description_3d_lidar.xacro")
    doc = xacro.process_file(description_path)
    base_robot_description = doc.documentElement.toxml()

    robot_actions: list[Any] = []
    for ns, costmap_topic in (
        (ROBOT_A_NS, frontier_costmap_topic_a),
        (ROBOT_B_NS, frontier_costmap_topic_b),
    ):
        robot_actions.extend(
            _robot_autonomy_actions(
                ns=ns,
                use_sim_time=use_sim_time,
                use_fast_lio=use_fast_lio,
                slam_config=slam_config,
                startup_delay=autonomy_start_delay,
                frontier_costmap_topic=costmap_topic,
                frontier_prefer_costmap=frontier_prefer_costmap,
                frontier_costmap_stale_sec=frontier_costmap_stale_sec,
                enable_dynamic_physics=enable_dynamic_physics,
                enable_champ_stack=enable_champ_stack,
                planning_scan_min_height=planning_scan_min_height,
                planning_scan_max_height=planning_scan_max_height,
                planning_scan_range_min=planning_scan_range_min,
                planning_scan_range_max=planning_scan_range_max,
                base_robot_description=base_robot_description,
                frontier_module=frontier_module,
                use_legacy_yaml_fallback=use_legacy_yaml_fallback,
                legacy_frontier_yaml=legacy_frontier_yaml,
                legacy_reactive_yaml=legacy_reactive_yaml,
                mapping_params=mapping_params,
                default_nav_params=default_nav_params,
            )
        )

    goal_assigner_node = _build_goal_assigner_node(
        use_sim_time=use_sim_time,
        goal_assigner_module=goal_assigner_module,
        namespaces=[ROBOT_A_NS, ROBOT_B_NS],
        use_shared_map=use_shared_map,
        shared_map_topic=shared_map_topic,
        enable_goal_assigner=enable_goal_assigner,
    )

    global_planner_node = _build_global_planner_node(
        use_sim_time=use_sim_time,
        global_planner_module=global_planner_module,
        namespaces=[ROBOT_A_NS, ROBOT_B_NS],
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
        name="rviz2_dual_isaac",
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
        condition=IfCondition(str(enable_status_monitor).lower()),
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
        condition=IfCondition(str(enable_coverage_visualizer).lower()),
    )

    frontier_distance_plotter_node = Node(
        package="go2_issac_stack",
        executable="frontier_distance_plotter.py",
        name="frontier_distance_plotter",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"namespaces": [ROBOT_A_NS, ROBOT_B_NS]},
            {"odom_topic_suffix": "/odom/nav"},
            {"frontier_marker_topic_suffix": "/frontier_goal_marker"},
            {"sample_rate_hz": frontier_distance_sample_rate},
            {"plot_write_rate_hz": frontier_distance_plot_write_rate},
            {"output_path": frontier_distance_plot_path},
        ],
        output="screen",
        condition=IfCondition(str(enable_frontier_distance_plot).lower()),
    )

    return [
        LogInfo(msg=summary),
        TimerAction(period=2.0, actions=robot_actions),
        TimerAction(period=8.0, actions=[goal_assigner_node, global_planner_node]),
        TimerAction(
            period=10.0,
            actions=[
                robot_status_monitor_node,
                frontier_distance_plotter_node,
                dual_coverage_visualizer_node,
                rviz_node,
            ],
        ),
    ]


def generate_launch_description():
    go2_issac_pkg = get_package_share_directory("go2_issac_stack")
    go2_issac_prefix = get_package_prefix("go2_issac_stack")
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    go2_description_pkg = get_package_share_directory("go2_description")

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
    physics_device = LaunchConfiguration("physics_device")

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
            "kill_pattern '/go2_nav_algorithms/lib/go2_nav_algorithms/simple_scan_mapper_cpp'; "
            "kill_pattern '/go2_issac_stack/lib/go2_issac_stack/simple_frontier_explorer.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/geometric_frontier.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/default_nav.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/autonomy_enabler.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/twist_bridge.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/qos_bridge.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/multi_robot_goal_assigner.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/goal_assigner_passthrough.py'; "
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

    default_pipeline_json = os.path.join(go2_issac_pkg, "config", "pipeline", "isaac_dual_pipeline.json")

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
                description="RTX lidar profile name.",
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
                description="Upper z clip used when converting point cloud to planning LaserScan.",
            ),
            DeclareLaunchArgument(
                "planning_scan_range_min",
                default_value="0.05",
                description="Min range used for planning LaserScan projection.",
            ),
            DeclareLaunchArgument(
                "planning_scan_range_max",
                default_value="12.0",
                description="Max range used for planning LaserScan projection.",
            ),
            DeclareLaunchArgument(
                "pipeline_config_json",
                default_value=default_pipeline_json,
                description="Pipeline JSON registry path for planner/assigner/global modules and params.",
            ),
            DeclareLaunchArgument(
                "frontier_planner_id",
                default_value="",
                description="Frontier planner module id override. Empty uses JSON defaults.frontier_planner_id.",
            ),
            DeclareLaunchArgument(
                "goal_assigner_id",
                default_value="",
                description="Goal assigner module id override. Empty uses JSON defaults.goal_assigner_id.",
            ),
            DeclareLaunchArgument(
                "global_planner_id",
                default_value="",
                description="Global planner module id override. Empty uses JSON defaults.global_planner_id.",
            ),
            DeclareLaunchArgument(
                "use_legacy_yaml_fallback",
                default_value="true",
                description="If true, fill missing JSON params from legacy YAML profiles.",
            ),
            DeclareLaunchArgument("max_scan_odom_dt", default_value=""),
            DeclareLaunchArgument("max_map_odom_dt", default_value=""),
            DeclareLaunchArgument("odom_history_sec", default_value=""),
            DeclareLaunchArgument("mapper_update_rate", default_value=""),
            DeclareLaunchArgument("frontier_update_rate", default_value=""),
            DeclareLaunchArgument("default_nav_max_linear_speed", default_value=""),
            DeclareLaunchArgument("default_nav_obstacle_slow_dist", default_value=""),
            DeclareLaunchArgument("default_nav_obstacle_stop_dist", default_value=""),
            DeclareLaunchArgument("default_nav_planner_grid_radius", default_value=""),
            DeclareLaunchArgument("default_nav_planner_goal_clip_distance", default_value=""),
            DeclareLaunchArgument("default_nav_wall_scan_enabled", default_value=""),
            DeclareLaunchArgument(
                "enable_goal_assigner",
                default_value="true",
                description="Enable goal-assigner module; false forces passthrough raw->assigned bridge.",
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
                "enable_frontier_distance_plot",
                default_value="false",
                description="Enable matplotlib PNG plot of robot distance-to-frontier over time.",
            ),
            DeclareLaunchArgument(
                "frontier_distance_plot_path",
                default_value="/tmp/frontier_distance_over_time.png",
                description="Output PNG path for frontier-distance plotter.",
            ),
            DeclareLaunchArgument(
                "frontier_distance_sample_rate",
                default_value="2.0",
                description="Sampling rate (Hz) for distance-to-frontier measurements.",
            ),
            DeclareLaunchArgument(
                "frontier_distance_plot_write_rate",
                default_value="0.5",
                description="Plot write rate (Hz) for updating the PNG file.",
            ),
            DeclareLaunchArgument(
                "enable_champ_stack",
                default_value="true",
                description="Enable CHAMP/robot-localization stack when dynamic physics is enabled.",
            ),
            cleanup_stale_processes,
            TimerAction(period=1.0, actions=[start_isaac_sim_process]),
            OpaqueFunction(function=_launch_setup),
        ]
    )
