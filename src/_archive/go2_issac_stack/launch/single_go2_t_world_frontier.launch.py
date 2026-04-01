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

sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.join(get_package_share_directory("go2_nav_algorithms"), "launch"))

from _pipeline_registry import load_pipeline_runtime
from _stack_components import (
    build_autonomy_enabler_node,
    build_isaac_topic_router_node,
    build_odom_tf_broadcaster_node,
    build_qos_bridge_node,
    build_rviz_node,
)
from pipeline_components import build_pointcloud_to_laserscan_node, build_simple_scan_mapper_cpp_node


ROBOT_NS = "go2_1"


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


def _build_mapper_node(
    *,
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
        "map_topic": f"/{ROBOT_NS}/map",
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
        ns=ROBOT_NS,
        use_sim_time=use_sim_time,
        profile=profile,
        extra_params=mapper_common,
        remappings=tf_remaps,
        name="simple_scan_mapper_cpp",
    )


def _build_frontier_node(
    *,
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
        "map_topic": f"/{ROBOT_NS}/map",
        "costmap_topic": frontier_costmap_topic,
        "prefer_costmap": frontier_prefer_costmap,
        "costmap_stale_sec": frontier_costmap_stale_sec,
        "frontier_goal_topic": f"/{ROBOT_NS}/way_point_raw",
        "frontier_marker_topic": f"/{ROBOT_NS}/frontier_goal_marker",
        "frontier_regions_topic": f"/{ROBOT_NS}/frontier_markers",
        "frontier_replan_topic": f"/{ROBOT_NS}/frontier_replan",
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
        namespace=ROBOT_NS,
        name=f"frontier_{frontier_module['id']}",
        parameters=params,
        remappings=tf_remaps,
        output="screen",
    )


def _build_default_nav_node(
    *,
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
            "frontier_replan_topic": f"/{ROBOT_NS}/frontier_replan",
            "stop_topic": f"/{ROBOT_NS}/stop",
        }
    )

    return Node(
        package="go2_gazebo_sim",
        executable="default_nav.py",
        namespace=ROBOT_NS,
        name="default_nav",
        parameters=params,
        remappings=[
            ("/way_point", f"/{ROBOT_NS}/way_point_coord"),
            ("/odom/ground_truth", nav_odom_topic),
            ("/scan", planning_scan_topic),
            ("/cmd_vel_stamped", f"/{ROBOT_NS}/cmd_vel_stamped"),
            ("/nav_status", f"/{ROBOT_NS}/nav_status"),
        ],
        output="screen",
    )


def _build_goal_assigner_node(
    *,
    use_sim_time: bool,
    goal_assigner_module: dict[str, Any],
    enable_goal_assigner: bool,
    use_shared_map: bool,
    shared_map_topic: str,
) -> Node:
    if not enable_goal_assigner:
        return Node(
            package="go2_gazebo_sim",
            executable="goal_assigner_passthrough.py",
            name="goal_assigner_passthrough_disabled",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"namespaces": [ROBOT_NS]},
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
        "namespaces": [ROBOT_NS],
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
) -> Node:
    mode = str(global_planner_module.get("mode", "")).strip().lower()
    module_params = copy.deepcopy(global_planner_module.get("params", {}))

    params = [
        {"use_sim_time": use_sim_time},
        {"namespaces": [ROBOT_NS]},
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
    pkg = get_package_share_directory("go2_issac_stack")

    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    use_fast_lio = _as_bool(LaunchConfiguration("use_fast_lio").perform(context))
    rviz_enabled = _as_bool(LaunchConfiguration("rviz").perform(context))
    run_readiness_gate = _as_bool(LaunchConfiguration("run_readiness_gate").perform(context))
    enable_goal_assigner = _as_bool(LaunchConfiguration("enable_goal_assigner").perform(context))
    use_shared_map = _as_bool(LaunchConfiguration("use_shared_map").perform(context))
    use_legacy_yaml_fallback = _as_bool(LaunchConfiguration("use_legacy_yaml_fallback").perform(context))

    startup_delay = float(LaunchConfiguration("startup_delay").perform(context))
    readiness_timeout_sec = float(LaunchConfiguration("readiness_timeout_sec").perform(context))
    readiness_required_coverage = float(LaunchConfiguration("readiness_required_coverage").perform(context))
    frontier_costmap_stale_sec = float(LaunchConfiguration("frontier_costmap_stale_sec").perform(context))
    planning_scan_min_height = float(LaunchConfiguration("planning_scan_min_height").perform(context))
    planning_scan_max_height = float(LaunchConfiguration("planning_scan_max_height").perform(context))
    planning_scan_range_min = float(LaunchConfiguration("planning_scan_range_min").perform(context))
    planning_scan_range_max = float(LaunchConfiguration("planning_scan_range_max").perform(context))

    frontier_costmap_topic = LaunchConfiguration("frontier_costmap_topic").perform(context)
    frontier_prefer_costmap = _as_bool(LaunchConfiguration("frontier_prefer_costmap").perform(context))
    shared_map_topic = LaunchConfiguration("shared_map_topic").perform(context)

    pipeline_config_json = LaunchConfiguration("pipeline_config_json").perform(context)
    frontier_planner_id = LaunchConfiguration("frontier_planner_id").perform(context).strip()
    goal_assigner_id = LaunchConfiguration("goal_assigner_id").perform(context).strip()
    global_planner_id = LaunchConfiguration("global_planner_id").perform(context).strip()

    legacy_frontier_yaml = os.path.join(pkg, "config", "nav", "geometric_frontier_dual.yaml")
    legacy_reactive_yaml = os.path.join(pkg, "config", "nav", "default_nav_dual.yaml")

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

    tf_remaps = [("/tf", f"/{ROBOT_NS}/tf"), ("/tf_static", f"/{ROBOT_NS}/tf_static")]
    nav_odom_topic = f"/{ROBOT_NS}/odom/nav"
    planning_scan_topic = f"/{ROBOT_NS}/scan_3d"

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

    slam_nodes: list[Any] = []
    slam_config = os.path.join(pkg, "config", "slam", "pointlio_isaac.yaml")
    if use_fast_lio:
        slam_nodes.extend(
            [
                Node(
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
                ),
                Node(
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
                ),
                Node(
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
                ),
            ]
        )
    else:
        slam_nodes.append(
            Node(
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
            )
        )

    world_to_odom_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        remappings=tf_remaps,
        output="screen",
    )

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

    pointcloud_to_laserscan_node = build_pointcloud_to_laserscan_node(
        ns=ROBOT_NS,
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
            ("cloud_in", f"/{ROBOT_NS}/registered_scan_reliable"),
            ("scan", planning_scan_topic),
        ],
    )

    mapper_node = _build_mapper_node(
        use_sim_time=use_sim_time,
        tf_remaps=tf_remaps,
        planning_scan_topic=planning_scan_topic,
        nav_odom_topic=nav_odom_topic,
        use_legacy_yaml_fallback=use_legacy_yaml_fallback,
        mapping_params=mapping_params,
    )

    frontier_node = _build_frontier_node(
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

    goal_assigner_node = _build_goal_assigner_node(
        use_sim_time=use_sim_time,
        goal_assigner_module=goal_assigner_module,
        enable_goal_assigner=enable_goal_assigner,
        use_shared_map=use_shared_map,
        shared_map_topic=shared_map_topic,
    )

    global_planner_node = _build_global_planner_node(
        use_sim_time=use_sim_time,
        global_planner_module=global_planner_module,
    )

    autonomy_enabler_node = build_autonomy_enabler_node(
        ns=ROBOT_NS,
        use_sim_time=use_sim_time,
        extra_params={"startup_delay": 8.0, "rate": 10.0},
        remappings=[("/way_point", f"/{ROBOT_NS}/way_point_coord"), ("/joy", f"/{ROBOT_NS}/joy")],
    )

    default_nav_node = _build_default_nav_node(
        use_sim_time=use_sim_time,
        use_legacy_yaml_fallback=use_legacy_yaml_fallback,
        legacy_reactive_yaml=legacy_reactive_yaml,
        default_nav_params=default_nav_params,
        nav_odom_topic=nav_odom_topic,
        planning_scan_topic=planning_scan_topic,
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
            (f"/{ROBOT_NS}/way_point", f"/{ROBOT_NS}/way_point_coord"),
        ],
        output="screen",
    )

    readiness_gate_node = Node(
        package="go2_issac_stack",
        executable="readiness_gate.py",
        name="single_robot_readiness_gate",
        parameters=[
            {"startup_timeout_sec": readiness_timeout_sec},
            {"required_coverage_ratio": readiness_required_coverage},
            {"odom_topic": f"/{ROBOT_NS}/odom/nav"},
            {"imu_topic": f"/{ROBOT_NS}/imu/data"},
            {"pointcloud_topic": f"/{ROBOT_NS}/registered_scan"},
            {"cmd_topic": f"/{ROBOT_NS}/isaac/cmd_vel"},
            {"map_topic": f"/{ROBOT_NS}/map"},
            {"nav_status_topic": f"/{ROBOT_NS}/nav_status"},
        ],
        output="screen",
        condition=IfCondition(str(run_readiness_gate).lower()),
    )

    rviz_config = os.path.join(pkg, "rviz", "dual_isaac_autonomy.rviz")
    rviz_node = build_rviz_node(
        rviz_config,
        use_sim_time,
        condition=IfCondition(str(rviz_enabled).lower()),
        remappings=tf_remaps,
        name="rviz2_single_isaac",
    )

    return [
        LogInfo(msg=summary),
        TimerAction(
            period=startup_delay,
            actions=[
                isaac_topic_router_node,
                twist_bridge_node,
                qos_bridge_node,
                *slam_nodes,
                world_to_odom_tf_node,
                odom_tf_broadcaster_node,
                pointcloud_to_laserscan_node,
                mapper_node,
                frontier_node,
                goal_assigner_node,
                global_planner_node,
                autonomy_enabler_node,
                default_nav_node,
                status_monitor_node,
                readiness_gate_node,
                rviz_node,
            ],
        ),
    ]


def generate_launch_description():
    pkg = get_package_share_directory("go2_issac_stack")
    pkg_prefix = get_package_prefix("go2_issac_stack")
    gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
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
    isaac_rtx_frame_skip = LaunchConfiguration("isaac_rtx_frame_skip")
    isaac_rtx_full_scan = LaunchConfiguration("isaac_rtx_full_scan")
    isaac_sim_command = LaunchConfiguration("isaac_sim_command")

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
            "kill_pattern '/go2_nav_algorithms/lib/go2_nav_algorithms/simple_scan_mapper_cpp'; "
            "kill_pattern '/go2_issac_stack/lib/go2_issac_stack/simple_frontier_explorer.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/geometric_frontier.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/default_nav.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/autonomy_enabler.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/twist_bridge.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/qos_bridge.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/robot_status_monitor.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/pointcloud_adapter.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/slam_odom_relay.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/multi_robot_goal_assigner.py'; "
            "kill_pattern '/go2_gazebo_sim/lib/go2_gazebo_sim/goal_assigner_passthrough.py'; "
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

    default_pipeline_json = os.path.join(pkg, "config", "pipeline", "isaac_single_pipeline.json")

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
            DeclareLaunchArgument("enable_goal_assigner", default_value="true"),
            DeclareLaunchArgument("use_shared_map", default_value="false"),
            DeclareLaunchArgument("shared_map_topic", default_value="/disco_slam/global_map"),
            cleanup_stale_processes,
            TimerAction(period=1.0, actions=[start_isaac_sim_process]),
            OpaqueFunction(function=_launch_setup),
        ]
    )
