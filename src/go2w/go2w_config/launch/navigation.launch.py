#!/usr/bin/env python3
"""Shared navigation sub-launch: mapper + CFPA2 frontier + local planner.

Included by both sim and real top-level launch files with platform-appropriate args.

nav_backend:
  reactive  — reactive_nav.py (default, our grid-based local planner)
  far       — CMU autonomy stack: terrain_analysis + far_planner + localPlanner/pathFollower
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml_params(yaml_path: str) -> dict:
    """Load a ROS2 YAML param file and return the ros__parameters dict.

    CMU autonomy stack YAML files are keyed by unqualified node name
    (e.g. ``far_planner:``), which doesn't match when the node is
    launched in a namespace (``/robot/far_planner``).  This helper
    strips the outer node-name key and returns just the parameter dict
    so it can be merged into the launch ``parameters`` list directly.
    """
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f) or {}
    # Expect { node_name: { ros__parameters: { ... } } }
    for _node_name, inner in data.items():
        if isinstance(inner, dict) and "ros__parameters" in inner:
            return dict(inner["ros__parameters"])
    # Fallback: return everything flat
    return data


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, key: str) -> str:
    return LaunchConfiguration(key).perform(context)


def _setup(context):
    robot_ns = _get(context, "robot_namespace")
    use_sim_time = _as_bool(_get(context, "use_sim_time"))
    map_frame = _get(context, "map_frame")
    external_mapper = _as_bool(_get(context, "external_mapper"))
    broadcast_tf = _as_bool(_get(context, "broadcast_tf"))
    remap_tf = _as_bool(_get(context, "remap_tf"))
    nav_backend = _get(context, "nav_backend").strip().lower() or "reactive"

    scan_topic = _get(context, "scan_topic") or f"/{robot_ns}/scan_3d"
    odom_topic = _get(context, "odom_topic") or f"/{robot_ns}/odom/nav"
    map_topic = _get(context, "map_topic") or f"/{robot_ns}/map"
    waypoint_suffix = _get(context, "waypoint_input_suffix") or "/way_point_coord"

    # 3D point cloud topic for FAR terrain analysis (must be in map frame)
    registered_scan_topic = _get(context, "registered_scan_topic").strip() or f"/{robot_ns}/registered_scan_reliable"

    cfpa2_w_ig = float(_get(context, "cfpa2_w_ig"))
    cfpa2_w_c = float(_get(context, "cfpa2_w_c"))
    cfpa2_w_momentum = float(_get(context, "cfpa2_w_momentum"))
    cfpa2_min_utility = float(_get(context, "cfpa2_min_utility"))
    cfpa2_goal_topic_suffix = _get(context, "cfpa2_goal_topic_suffix").strip() or "/way_point_coord"
    max_linear_speed_str = _get(context, "max_linear_speed").strip()
    require_settle_str = _get(context, "require_settle_before_motion").strip()
    nav_map_topic_str = _get(context, "nav_map_topic").strip()
    switch_hysteresis_str = _get(context, "cfpa2_switch_hysteresis").strip()

    # FAR-specific params
    far_max_speed = float(_get(context, "far_max_speed"))
    far_robot_id = int(_get(context, "far_robot_id"))

    nav_config = _get(context, "nav_config")
    mapper_config = _get(context, "mapper_config")
    cfpa2_config = _get(context, "cfpa2_config")

    tf_remaps = []
    if remap_tf:
        tf_remaps = [
            ("/tf", f"/{robot_ns}/tf"),
            ("/tf_static", f"/{robot_ns}/tf_static"),
        ]

    actions = []

    # ── Mapper ──
    if not external_mapper:
        nav_pkg = get_package_share_directory("go2_nav_algorithms")
        actions.append(
            Node(
                package="go2_nav_algorithms",
                executable="simple_scan_mapper_cpp",
                namespace=robot_ns,
                name="simple_scan_mapper_cpp",
                parameters=[
                    os.path.join(nav_pkg, "config", "nav", "geometric_frontier_single.yaml"),
                    mapper_config,
                    {
                        "use_sim_time": use_sim_time,
                        "scan_topic": scan_topic,
                        "odom_topic": odom_topic,
                        "map_topic": map_topic,
                        "map_frame": map_frame,
                        "startup_delay": 0.0,
                        "scan_frame": "base_link",
                        "broadcast_tf": broadcast_tf,
                    },
                ],
                remappings=tf_remaps[:] if remap_tf else [],
                output="screen",
            )
        )

    # ── CFPA2 Frontier ──
    cfpa2_params = {
        "use_sim_time": use_sim_time,
        "robot_namespace": robot_ns,
        "namespaces": [robot_ns],
        "goal_topic_suffix": cfpa2_goal_topic_suffix,
        "marker_frame_override": map_frame,
        "cfpa2_w_ig": cfpa2_w_ig,
        "cfpa2_w_c": cfpa2_w_c,
        "cfpa2_w_momentum": cfpa2_w_momentum,
        "cfpa2_min_utility": cfpa2_min_utility,
    }
    if switch_hysteresis_str:
        cfpa2_params["switch_hysteresis"] = float(switch_hysteresis_str)

    actions.append(
        Node(
            package="cfpa2_collaborative_autonomy",
            executable="cfpa2_single_robot_node",
            name="cfpa2_single_robot",
            parameters=[cfpa2_config, cfpa2_params],
            output="screen",
        )
    )

    # ── Local Planner Backend ──
    if nav_backend == "far":
        actions.extend(
            _build_far_stack(
                robot_ns=robot_ns,
                use_sim_time=use_sim_time,
                map_frame=map_frame,
                odom_topic=odom_topic,
                registered_scan_topic=registered_scan_topic,
                waypoint_suffix=waypoint_suffix,
                tf_remaps=tf_remaps,
                max_speed=far_max_speed,
                robot_id=far_robot_id,
            )
        )
    else:
        # Default: reactive_nav
        nav_extra = {
            "frontier_replan_topic": f"/{robot_ns}/frontier_replan",
            "stop_topic": f"/{robot_ns}/stop",
        }
        if max_linear_speed_str:
            nav_extra["max_linear_speed"] = float(max_linear_speed_str)
        if require_settle_str:
            nav_extra["require_settle_before_motion"] = _as_bool(require_settle_str)
        if nav_map_topic_str:
            nav_extra["map_topic"] = nav_map_topic_str

        nav_remappings = [
            ("/way_point", f"/{robot_ns}{waypoint_suffix}"),
            ("/odom/ground_truth", odom_topic),
            ("/scan", scan_topic),
            ("/cmd_vel_stamped", f"/{robot_ns}/cmd_vel_stamped"),
            ("/nav_status", f"/{robot_ns}/nav_status"),
            ("/planned_path", f"/{robot_ns}/planned_path"),
            ("/robot_trajectory", f"/{robot_ns}/robot_trajectory"),
            ("/final_goal_marker", f"/{robot_ns}/final_goal_marker"),
            ("/robot_pose_marker", f"/{robot_ns}/robot_pose_marker"),
        ]

        actions.append(
            Node(
                package="go2w_nav",
                executable="reactive_nav.py",
                namespace=robot_ns,
                name="reactive_nav",
                parameters=[nav_config, {"use_sim_time": use_sim_time}, nav_extra],
                remappings=nav_remappings,
                output="screen",
            )
        )

    return actions


def _build_far_stack(
    *,
    robot_ns: str,
    use_sim_time: bool,
    map_frame: str,
    odom_topic: str,
    registered_scan_topic: str,
    waypoint_suffix: str,
    tf_remaps: list,
    max_speed: float,
    robot_id: int,
) -> list:
    """Build the CMU autonomy FAR planner stack with full namespacing.

    Topic flow:
      CFPA2 → /{ns}{waypoint_suffix} (goal_point)
        → far_planner → /{ns}/way_point (intermediate route waypoints)
          → localPlanner → pathFollower → /{ns}/cmd_vel_stamped

    Terrain pipeline:
      /{ns}/registered_scan_map (map-frame 3D cloud, provided by caller)
        → sensorScanGeneration → /{ns}/sensor_scan
        → terrainAnalysis       → /{ns}/terrain_map
        → terrainAnalysisExt    → /{ns}/terrain_map_ext
    """
    ns = robot_ns
    far_pkg = get_package_share_directory("far_planner")
    local_planner_pkg = get_package_share_directory("local_planner")

    nodes = []

    # ── Terrain analysis pipeline ──
    # These nodes assume the input point cloud is in map frame (no TF lookups).
    nodes.append(
        Node(
            package="sensor_scan_generation",
            executable="sensorScanGeneration",
            namespace=ns,
            name="sensor_scan_generation",
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[
                ("/state_estimation", odom_topic),
                ("/registered_scan", registered_scan_topic),
                ("/state_estimation_at_scan", f"/{ns}/state_estimation_at_scan"),
                ("/sensor_scan", f"/{ns}/sensor_scan"),
            ] + tf_remaps,
            output="screen",
        )
    )

    nodes.append(
        Node(
            package="terrain_analysis",
            executable="terrainAnalysis",
            namespace=ns,
            name="terrain_analysis",
            parameters=[{
                "use_sim_time": use_sim_time,
                # Default maxRelZ=0.2 cuts off wall points above 0.2m
                # relative to vehicle, leaving only floor + wall-base
                # fragments with ambiguous 0.2-0.3 intensity.  Raising
                # to 0.8 captures full wall height so intensity becomes
                # bimodal: floor ≈ 0, walls ≈ 0.5+.
                "maxRelZ": 0.8,
            }],
            remappings=[
                ("/state_estimation", odom_topic),
                ("/registered_scan", registered_scan_topic),
                ("/joy", f"/{ns}/joy"),
                ("/map_clearing", f"/{ns}/map_clearing"),
                ("/terrain_map", f"/{ns}/terrain_map"),
            ],
            output="screen",
        )
    )

    nodes.append(
        Node(
            package="terrain_analysis_ext",
            executable="terrainAnalysisExt",
            namespace=ns,
            name="terrain_analysis_ext",
            parameters=[{
                "use_sim_time": use_sim_time,
            }],
            remappings=[
                ("/state_estimation", odom_topic),
                ("/registered_scan", registered_scan_topic),
                ("/joy", f"/{ns}/joy"),
                ("/cloud_clearing", f"/{ns}/cloud_clearing"),
                ("/terrain_map", f"/{ns}/terrain_map"),
                ("/terrain_map_ext", f"/{ns}/terrain_map_ext"),
            ],
            output="screen",
        )
    )

    # ── FAR planner (global route planner) ──
    # Reads goal_point from CFPA2/VLM mux, outputs intermediate way_point for localPlanner.
    # Has its own TF lookups — handles frame transforms internally via world_frame param.
    nodes.append(
        Node(
            package="far_planner",
            executable="far_planner",
            namespace=ns,
            name="far_planner",
            parameters=[
                # Load YAML params stripped of the node-name key so they
                # apply correctly under any ROS2 namespace.
                _load_yaml_params(os.path.join(far_pkg, "config", "default.yaml")),
                {
                    "use_sim_time": use_sim_time,
                    "world_frame": map_frame,
                    "graph_msger/robot_id": robot_id,
                    # Converge distance must exceed pathFollower stopDisThre (0.2m)
                    # so FAR declares waypoint reached before pathFollower stalls.
                    "g_planner/converge_distance": 0.5,
                    # terrain_free_Z: points with intensity (height above local
                    # ground) below this are "free", above are obstacles for
                    # V-Graph contour detection.  MUST be < localPlanner's
                    # obstacleHeightThre so FAR is stricter than local planning.
                    "util/terrain_free_Z": 0.15,
                    # obs_inflate_size: voxels to inflate obstacle contours (CMU default=1).
                    "util/obs_inflate_size": 1,
                },
            ],
            remappings=[
                ("/odom_world", odom_topic),
                ("/terrain_cloud", f"/{ns}/terrain_map_ext"),
                ("/scan_cloud", f"/{ns}/terrain_map"),
                ("/terrain_local_cloud", registered_scan_topic),
                ("/goal_point", f"/{ns}{waypoint_suffix}"),
                ("/way_point", f"/{ns}/way_point"),
                ("/joy", f"/{ns}/joy"),
                ("/navigation_boundary", f"/{ns}/navigation_boundary"),
                ("/runtime", f"/{ns}/far_runtime"),
                ("/planning_time", f"/{ns}/far_planning_time"),
                # Per-robot graph exchange (use shared bus for multi-robot coordination)
                ("/robot_vgraph", f"/{ns}/robot_vgraph"),
                ("/decoded_vgraph", f"/{ns}/decoded_vgraph"),
            ] + tf_remaps,
            output="screen",
        )
    )

    # ── localPlanner (local trajectory planner) ──
    # No TF lookups — assumes point cloud and odometry in same frame.
    # Reads way_point from FAR, generates local path for pathFollower.
    nodes.append(
        Node(
            package="local_planner",
            executable="localPlanner",
            namespace=ns,
            name="localPlanner",
            parameters=[{
                "use_sim_time": use_sim_time,
                "pathFolder": os.path.join(local_planner_pkg, "paths"),
                "vehicleLength": 0.3,
                "vehicleWidth": 0.7,
                "sensorOffsetX": 0.0,
                "sensorOffsetY": 0.0,
                "twoWayDrive": False,
                "laserVoxelSize": 0.05,
                "terrainVoxelSize": 0.2,
                "useTerrainAnalysis": True,
                "checkObstacle": True,
                "checkRotObstacle": False,
                "adjacentRange": 3.0,
                "obstacleHeightThre": 0.3,
                "groundHeightThre": 0.1,
                "costHeightThre": 0.1,
                "costScore": 0.02,
                "useCost": False,
                "pointPerPathThre": 2,
                "minRelZ": -0.5,
                "maxRelZ": 0.25,
                "maxSpeed": max_speed,
                "dirWeight": 0.02,
                "dirThre": 90.0,
                "dirToVehicle": False,
                "pathScale": 0.75,
                "minPathScale": 0.5,
                "pathScaleStep": 0.25,
                "pathScaleBySpeed": True,
                "minPathRange": 1.0,
                "pathRangeStep": 0.5,
                "pathRangeBySpeed": True,
                "pathCropByGoal": True,
                "autonomyMode": True,
                "autonomySpeed": max_speed,
                "joyToSpeedDelay": 2.0,
                "joyToCheckObstacleDelay": 5.0,
                "goalClearRange": 0.5,
                "goalX": 0.0,
                "goalY": 0.0,
            }],
            remappings=[
                ("/state_estimation", odom_topic),
                ("/registered_scan", registered_scan_topic),
                ("/way_point", f"/{ns}/way_point"),
                ("/terrain_map", f"/{ns}/terrain_map"),
                ("/overall_map", f"/{ns}/terrain_map"),
                ("/joy", f"/{ns}/joy"),
                ("/path", f"/{ns}/local_path"),
                ("/freePaths", f"/{ns}/free_paths"),
            ],
            output="screen",
        )
    )

    # ── pathFollower (pure-pursuit path execution) ──
    # Publishes TwistStamped on cmd_vel (remapped to /{ns}/cmd_vel_stamped).
    nodes.append(
        Node(
            package="local_planner",
            executable="pathFollower",
            namespace=ns,
            name="pathFollower",
            parameters=[{
                "use_sim_time": use_sim_time,
                "sensorOffsetX": 0.0,
                "sensorOffsetY": 0.0,
                "pubSkipNum": 1,
                "twoWayDrive": False,
                "lookAheadDis": 0.5,
                "yawRateGain": 1.5,
                "stopYawRateGain": 1.5,
                "maxYawRate": 80.0,
                "maxSpeed": max_speed,
                "maxAccel": 2.0,
                "switchTimeThre": 1.0,
                "dirDiffThre": 0.4,
                "omniDirDiffThre": 1.5,
                "noRotSpeed": 10.0,
                "stopDisThre": 0.3,
                "slowDwnDisThre": 0.75,
                "useInclRateToSlow": False,
                "inclRateThre": 120.0,
                "slowRate1": 0.25,
                "slowRate2": 0.5,
                "slowTime1": 2.0,
                "slowTime2": 2.0,
                "useInclToStop": False,
                "inclThre": 45.0,
                "stopTime": 5.0,
                "noRotAtStop": False,
                "noRotAtGoal": True,
                "autonomyMode": True,
                "autonomySpeed": max_speed,
                "joyToSpeedDelay": 2.0,
                "goalCloseDis": 0.4,
                "is_real_robot": False,
            }],
            remappings=[
                ("/state_estimation", odom_topic),
                ("/path", f"/{ns}/local_path"),
                ("/cmd_vel", f"/{ns}/cmd_vel_stamped"),
                ("/joy", f"/{ns}/joy"),
                ("/speed", f"/{ns}/speed"),
                ("/stop", f"/{ns}/stop"),
            ],
            output="screen",
        )
    )

    # ── Static TFs for CMU local planner convention ──
    # sensor→vehicle: identity (LiDAR is approximately centered on Go2W)
    # sensor→camera: 90° rotation for camera convention (used by localPlanner visualization)
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace=ns,
            name="far_vehicle_tf",
            arguments=["0", "0", "0", "0", "0", "0", "sensor", "vehicle"],
            remappings=tf_remaps,
            output="screen",
        )
    )
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace=ns,
            name="far_camera_tf",
            arguments=["0", "0", "0", "-1.5707963", "0", "-1.5707963", "sensor", "camera"],
            remappings=tf_remaps,
            output="screen",
        )
    )

    return nodes


def generate_launch_description():
    go2w_config_pkg = get_package_share_directory("go2w_config")
    cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="robot"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("map_frame", default_value="world"),
            DeclareLaunchArgument("external_mapper", default_value="false"),
            DeclareLaunchArgument("broadcast_tf", default_value="true"),
            DeclareLaunchArgument("remap_tf", default_value="true"),
            DeclareLaunchArgument(
                "nav_backend",
                default_value="reactive",
                description="Local planner backend: reactive (grid A*) or far (CMU autonomy stack)",
            ),
            DeclareLaunchArgument("scan_topic", default_value=""),
            DeclareLaunchArgument("odom_topic", default_value=""),
            DeclareLaunchArgument("map_topic", default_value=""),
            DeclareLaunchArgument(
                "registered_scan_topic",
                default_value="",
                description="3D PointCloud2 for FAR terrain analysis (must be in map frame for Cartographer mode)",
            ),
            DeclareLaunchArgument("waypoint_input_suffix", default_value="/way_point_coord"),
            DeclareLaunchArgument(
                "nav_config",
                default_value=os.path.join(go2w_config_pkg, "config", "nav", "reactive_nav_single_go2w.yaml"),
            ),
            DeclareLaunchArgument(
                "mapper_config",
                default_value=os.path.join(go2w_config_pkg, "config", "nav", "simple_scan_mapper_single_go2w.yaml"),
            ),
            DeclareLaunchArgument(
                "cfpa2_config",
                default_value=os.path.join(cfpa2_pkg, "config", "cfpa2_single_robot.yaml"),
            ),
            DeclareLaunchArgument("cfpa2_w_ig", default_value="1.0"),
            DeclareLaunchArgument("cfpa2_w_c", default_value="0.6"),
            DeclareLaunchArgument("cfpa2_w_momentum", default_value="0.8"),
            DeclareLaunchArgument("cfpa2_min_utility", default_value="-0.5"),
            DeclareLaunchArgument("cfpa2_switch_hysteresis", default_value=""),
            DeclareLaunchArgument("cfpa2_goal_topic_suffix", default_value="/way_point_coord"),
            DeclareLaunchArgument("max_linear_speed", default_value=""),
            DeclareLaunchArgument("require_settle_before_motion", default_value=""),
            DeclareLaunchArgument("nav_map_topic", default_value=""),
            DeclareLaunchArgument("far_max_speed", default_value="0.5"),
            DeclareLaunchArgument(
                "far_robot_id",
                default_value="0",
                description="Robot ID for FAR graph_msger (unique per robot in multi-robot setups)",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
