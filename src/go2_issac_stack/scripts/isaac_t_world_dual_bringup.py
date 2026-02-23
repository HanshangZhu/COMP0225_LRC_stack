#!/usr/bin/env python3
"""Isaac Sim bringup for dual Go2 exploration in t_world.

This process owns one Isaac stage with two kinematic Go2 robots and bridges
ROS topics for both namespaces.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time

from dataclasses import dataclass
from pathlib import Path

from isaacsim import SimulationApp

import isaac_t_world_bringup as single

try:
    import numpy as np
except Exception:  # pragma: no cover - runtime fallback when numpy unavailable
    np = None


@dataclass
class RobotRuntime:
    namespace: str
    pose: single.Pose2D
    pose_prim_path: str
    robot_prim_path: str
    cmd_linear: float = 0.0
    cmd_angular: float = 0.0
    last_cmd_wall_t: float = 0.0
    prev_lin: float = 0.0
    next_pointcloud_t: float = 0.0
    lidar_mode: str = "cpu"
    rtx_sensor_path: str = ""
    rtx_graph_path: str = ""
    rtx_render_product: object | None = None
    odom_pub: object | None = None
    imu_pub: object | None = None
    pc_pub: object | None = None
    cmd_sub: object | None = None
    walls: list[single.WallBox] = None
    max_linear_speed: float = 0.6
    max_angular_speed: float = 1.5
    robot_radius: float = 0.28
    robot_z: float = 0.33
    lidar_angles: tuple[float, ...] = ()
    lidar_angles_np: object | None = None
    wall_cache: object | None = None
    sim_time: float = 0.0
    pointcloud_period: float = 0.2
    next_pointcloud_t: float = 0.0


def _build_wall_cache(walls: list[single.WallBox]) -> object | None:
    if np is None:
        return None
    if not walls:
        return None
    cx = np.asarray([w.cx for w in walls], dtype=np.float32)
    cy = np.asarray([w.cy for w in walls], dtype=np.float32)
    half_x = np.asarray([0.5 * w.sx for w in walls], dtype=np.float32)
    half_y = np.asarray([0.5 * w.sy for w in walls], dtype=np.float32)
    yaw = np.asarray([w.yaw for w in walls], dtype=np.float32)
    return {
        "cx": cx,
        "cy": cy,
        "half_x": half_x,
        "half_y": half_y,
        "cos_yaw": np.cos(yaw),
        "sin_yaw": np.sin(yaw),
    }

class DualRobotBridge:
    def __init__(self, robot_specs: dict[str, RobotRuntime], rcl_node: object):
        from geometry_msgs.msg import Twist
        from rosgraph_msgs.msg import Clock
        self.robot_specs = robot_specs
        self.node = rcl_node
        self.latest_clock_msg = None
        self.latest_clock_sec = None
        self.clock_sub = self.node.create_subscription(Clock, "/clock", self._clock_cb, 10)
        for ns, robot in self.robot_specs.items():
            # Bind closures to capture the namespace
            def make_cb(r):
                return lambda msg: self._cmd_cb(r, msg)
            robot.cmd_sub = self.node.create_subscription(
                Twist, f"/{ns}/isaac/cmd_vel", make_cb(robot), 10
            )

    def _clock_cb(self, msg):
        self.latest_clock_msg = msg.clock
        self.latest_clock_sec = float(msg.clock.sec) + float(msg.clock.nanosec) / 1e9

    def _cmd_cb(self, robot: RobotRuntime, msg: Twist):
        robot.cmd_linear = float(max(-robot.max_linear_speed, min(robot.max_linear_speed, msg.linear.x)))
        robot.cmd_angular = float(max(-robot.max_angular_speed, min(robot.max_angular_speed, msg.angular.z)))
        robot.last_cmd_wall_t = time.monotonic()

    def step(self, dt: float, dynamic_physics: bool, cmd_timeout: float):
        now = time.monotonic()
        for ns, robot in self.robot_specs.items():
            linear = 0.0
            angular = 0.0
            # 1. Update Kinematics (if not dynamic)
            if not dynamic_physics:
                cmd_stale = (robot.last_cmd_wall_t <= 0.0) or ((now - robot.last_cmd_wall_t) > cmd_timeout)
                linear = 0.0 if cmd_stale else robot.cmd_linear
                angular = 0.0 if cmd_stale else robot.cmd_angular

                # Kinematic update with wall collision
                new_yaw = single._angle_wrap(robot.pose.yaw + angular * dt)
                step_x = linear * math.cos(robot.pose.yaw) * dt
                step_y = linear * math.sin(robot.pose.yaw) * dt
                new_x = robot.pose.x + step_x
                new_y = robot.pose.y + step_y

                colliding = False
                for wall in robot.walls:
                    if single._circle_hits_box(new_x, new_y, robot.robot_radius, wall):
                        colliding = True
                        break
                
                if colliding:
                    new_x, new_y = robot.pose.x, robot.pose.y
                
                robot.pose.x = new_x
                robot.pose.y = new_y
                robot.pose.yaw = new_yaw
            
            # 2. Update sim time and handle CPU Lidar
            # Keep CPU lidar stamps in the same time domain as odom/TF by
            # tracking Isaac /clock (falls back to local integration until
            # /clock is available).
            if self.latest_clock_sec is not None:
                robot.sim_time = self.latest_clock_sec
            else:
                robot.sim_time += dt

            if not dynamic_physics:
                self._publish_kinematic_odom_imu(robot, dt, linear, angular)

            if robot.lidar_mode == "cpu":
                if robot.pointcloud_period > 0.0 and robot.next_pointcloud_t <= 0.0:
                    robot.next_pointcloud_t = robot.sim_time
                if robot.pointcloud_period <= 0.0 or robot.sim_time >= robot.next_pointcloud_t:
                    self._publish_cpu_pointcloud(robot)
                    if robot.pointcloud_period > 0.0:
                        robot.next_pointcloud_t = robot.sim_time + robot.pointcloud_period

    def _publish_cpu_pointcloud(self, robot: RobotRuntime):
        from std_msgs.msg import Header
        from sensor_msgs.msg import PointField
        from sensor_msgs_py import point_cloud2

        header = Header()
        if self.latest_clock_msg is not None:
            header.stamp.sec = int(self.latest_clock_msg.sec)
            header.stamp.nanosec = int(self.latest_clock_msg.nanosec)
        else:
            header.stamp = single._stamp_from_seconds(type(header.stamp), robot.sim_time)
        header.frame_id = "base_link"
        points = []

        distances = self._raycast_robot_vectorized(robot)
        if distances is None:
            for a in robot.lidar_angles:
                dist = self._raycast_robot(robot, a)
                if dist is not None:
                    points.append((dist * math.cos(a), dist * math.sin(a), 0.25, 255.0))
        else:
            if robot.lidar_angles_np is None and np is not None:
                robot.lidar_angles_np = np.asarray(robot.lidar_angles, dtype=np.float32)
            if np is not None and robot.lidar_angles_np is not None:
                finite = np.isfinite(distances)
                if np.any(finite):
                    d = distances[finite]
                    a = robot.lidar_angles_np[finite]
                    x = d * np.cos(a)
                    y = d * np.sin(a)
                    points = list(zip(x.tolist(), y.tolist(), [0.25] * len(x), [255.0] * len(x)))

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg = point_cloud2.create_cloud(header, fields, points)
        robot.pc_pub.publish(msg)

    def _publish_kinematic_odom_imu(self, robot: RobotRuntime, dt: float, linear_speed: float, angular_speed: float):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu

        # Odom follows the synthetic kinematic state used by CPU lidar.
        odom = Odometry()
        if self.latest_clock_msg is not None:
            odom.header.stamp.sec = int(self.latest_clock_msg.sec)
            odom.header.stamp.nanosec = int(self.latest_clock_msg.nanosec)
        else:
            odom.header.stamp = single._stamp_from_seconds(type(odom.header.stamp), robot.sim_time)
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = robot.pose.x
        odom.pose.pose.position.y = robot.pose.y
        odom.pose.pose.position.z = robot.robot_z
        qx, qy, qz, qw = single._quat_from_yaw(robot.pose.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear_speed
        odom.twist.twist.angular.z = angular_speed
        if robot.odom_pub is not None:
            robot.odom_pub.publish(odom)

        # Lightweight IMU proxy (orientation + angular + coarse linear acc).
        imu = Imu()
        if self.latest_clock_msg is not None:
            imu.header.stamp.sec = int(self.latest_clock_msg.sec)
            imu.header.stamp.nanosec = int(self.latest_clock_msg.nanosec)
        else:
            imu.header.stamp = single._stamp_from_seconds(type(imu.header.stamp), robot.sim_time)
        imu.header.frame_id = "imu_link"
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.angular_velocity.z = angular_speed
        lin_acc = 0.0 if dt <= 1e-6 else (linear_speed - robot.prev_lin) / dt
        robot.prev_lin = linear_speed
        imu.linear_acceleration.x = lin_acc
        imu.linear_acceleration.z = 9.81
        if robot.imu_pub is not None:
            robot.imu_pub.publish(imu)

    def _raycast_robot_vectorized(self, robot: RobotRuntime):
        if np is None:
            return None
        if robot.wall_cache is None:
            robot.wall_cache = _build_wall_cache(robot.walls)
        cache = robot.wall_cache
        if cache is None:
            return None
        if robot.lidar_angles_np is None:
            robot.lidar_angles_np = np.asarray(robot.lidar_angles, dtype=np.float32)

        angles = robot.pose.yaw + robot.lidar_angles_np
        dx = np.cos(angles)
        dy = np.sin(angles)

        # Transform ray origin and direction into each wall local frame.
        ox = np.float32(robot.pose.x)
        oy = np.float32(robot.pose.y)
        c = cache["cos_yaw"]
        s = cache["sin_yaw"]
        lox = c * (ox - cache["cx"]) + s * (oy - cache["cy"])
        loy = -s * (ox - cache["cx"]) + c * (oy - cache["cy"])

        ldx = c[:, None] * dx[None, :] + s[:, None] * dy[None, :]
        ldy = -s[:, None] * dx[None, :] + c[:, None] * dy[None, :]

        eps = np.float32(1e-8)
        hx = cache["half_x"][:, None]
        hy = cache["half_y"][:, None]
        lox2 = lox[:, None]
        loy2 = loy[:, None]

        near_x = np.abs(ldx) < eps
        near_y = np.abs(ldy) < eps

        x_in = (lox2 >= -hx) & (lox2 <= hx)
        y_in = (loy2 >= -hy) & (loy2 <= hy)
        valid_x = (~near_x) | x_in
        valid_y = (~near_y) | y_in

        inv_ldx = np.where(near_x, np.float32(0.0), np.float32(1.0) / ldx)
        inv_ldy = np.where(near_y, np.float32(0.0), np.float32(1.0) / ldy)

        tx1 = (-hx - lox2) * inv_ldx
        tx2 = (hx - lox2) * inv_ldx
        ty1 = (-hy - loy2) * inv_ldy
        ty2 = (hy - loy2) * inv_ldy

        tmin_x = np.where(near_x, -np.inf, np.minimum(tx1, tx2))
        tmax_x = np.where(near_x, np.inf, np.maximum(tx1, tx2))
        tmin_y = np.where(near_y, -np.inf, np.minimum(ty1, ty2))
        tmax_y = np.where(near_y, np.inf, np.maximum(ty1, ty2))

        tmin = np.maximum(tmin_x, tmin_y)
        tmax = np.minimum(tmax_x, tmax_y)

        valid = valid_x & valid_y & (tmax >= 0.0) & (tmin <= tmax)
        dist = np.where(valid, np.maximum(0.0, tmin), np.inf)
        best = np.min(dist, axis=0)
        best = np.where((best >= 0.2) & (best <= 12.0), best, np.inf)
        return best

    def _raycast_robot(self, robot: RobotRuntime, local_angle: float) -> float | None:
        world_angle = robot.pose.yaw + local_angle
        dx, dy = math.cos(world_angle), math.sin(world_angle)
        best = None
        for wall in robot.walls:
            t = single._ray_box_intersection_2d(robot.pose.x, robot.pose.y, dx, dy, wall)
            if t is not None and 0.2 <= t <= 12.0:
                if best is None or t < best:
                    best = t
        return best


def _make_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Isaac Sim dual t_world bringup for go2_issac_stack",
        parents=[single._make_base_arg_parser()],
    )
    parser.set_defaults(
        loop_hz=120.0,
        lidar_rays=360,
        lidar_mode="rtx",
        prefer_existing_lidar=False,
    )
    parser.add_argument(
        "--robots-config",
        default='[{"namespace":"go2_1","x":1.0,"y":0.0,"yaw":0.0},{"namespace":"go2_2","x":18.0,"y":0.0,"yaw":3.14159}]',
        help="JSON array of robot configs",
    )
    parser.add_argument("--robot-a-namespace", default="go2_1")
    parser.add_argument("--robot-b-namespace", default="go2_2")
    parser.add_argument("--robot-a-spawn-x", type=float, default=1.0)
    parser.add_argument("--robot-a-spawn-y", type=float, default=0.0)
    parser.add_argument("--robot-a-spawn-yaw", type=float, default=0.0)
    parser.add_argument("--robot-b-spawn-x", type=float, default=18.0)
    parser.add_argument("--robot-b-spawn-y", type=float, default=0.0)
    parser.add_argument("--robot-b-spawn-yaw", type=float, default=3.14159)
    parser.add_argument(
        "--physics-device",
        type=int,
        default=0,
        help="CUDA device ID for physics; -1 forces CPU physics",
    )
    return parser


def _debug_graph_timestamp_input(graph_path: str, node_name: str) -> None:
    """Print timestamp wiring state for one OmniGraph ROS publisher node."""
    try:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        node_path = f"{graph_path}/{node_name}"
        prim = stage.GetPrimAtPath(node_path)
        if not prim.IsValid():
            print(f"[DEBUG_TS_GRAPH] missing prim: {node_path}", flush=True)
            return

        attr = prim.GetAttribute("inputs:timeStamp")
        if not attr.IsValid():
            print(f"[DEBUG_TS_GRAPH] no timeStamp input attr: {node_path}", flush=True)
            return

        value = attr.Get()
        connections = [str(path) for path in attr.GetConnections()]
        print(
            f"[DEBUG_TS_GRAPH] {node_path} timeStamp value={value} connections={connections}",
            flush=True,
        )
    except Exception as exc:
        print(f"[DEBUG_TS_GRAPH] probe failed for {graph_path}/{node_name}: {exc}", flush=True)


def _create_ros2_base_graph(
    ns: str,
    robot_prim_path: str,
    pose_prim_path: str,
    max_linear: float,
    max_angular: float,
    enable_articulation_control: bool,
):
    """Creates an OmniGraph Action Graph that natively handles ROS 2 Twist subscription and Odometry/IMU/TF publishing."""
    import omni.graph.core as og
    import omni.usd
    from isaacsim.core.utils.prims import set_targets
    
    graph_path = f"/{ns}_ros2_base_graph"
    
    create_nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("OdomPub", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("ImuPub", "isaacsim.ros2.bridge.ROS2PublishImu"),
    ]
    connect = [
        ("OnPlaybackTick.outputs:tick", "OdomPub.inputs:execIn"),
        ("Context.outputs:context", "OdomPub.inputs:context"),
        ("SimTime.outputs:simulationTime", "OdomPub.inputs:timeStamp"),
        ("OnPlaybackTick.outputs:tick", "ImuPub.inputs:execIn"),
        ("Context.outputs:context", "ImuPub.inputs:context"),
        ("SimTime.outputs:simulationTime", "ImuPub.inputs:timeStamp"),
    ]
    set_values = [
        ("Context.inputs:useDomainIDEnvVar", True),
        ("OdomPub.inputs:topicName", "odom"),
        ("OdomPub.inputs:odomFrameId", "odom"),
        ("OdomPub.inputs:chassisFrameId", "base_link"),
        ("OdomPub.inputs:nodeNamespace", ns),
        ("ImuPub.inputs:topicName", "imu"),
        ("ImuPub.inputs:frameId", "imu_link"),
        ("ImuPub.inputs:nodeNamespace", ns),
    ]

    if enable_articulation_control:
        create_nodes.extend(
            [
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ]
        )
        connect.extend(
            [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("SimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("SubscribeJointState.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ]
        )
        set_values.extend(
            [
                ("PublishJointState.inputs:topicName", "joint_states"),
                ("PublishJointState.inputs:nodeNamespace", ns),
                ("SubscribeJointState.inputs:topicName", "joint_command"),
                ("SubscribeJointState.inputs:nodeNamespace", ns),
            ]
        )

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: create_nodes,
            og.Controller.Keys.CONNECT: connect,
            og.Controller.Keys.SET_VALUES: set_values,
        },
    )
    
    stage = omni.usd.get_context().get_stage()
    
    target_art_path = ""
    if enable_articulation_control:
        # Robustly find the articulation root
        from pxr import UsdPhysics, Usd

        target_art_path = robot_prim_path
        prim = stage.GetPrimAtPath(robot_prim_path)
        if prim.IsValid() and not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            for desc in Usd.PrimRange(prim):
                if desc.HasAPI(UsdPhysics.ArticulationRootAPI):
                    target_art_path = str(desc.GetPath())
                    break

        # Bind publisher/controller only in dynamic articulation mode.
        set_targets(
            prim=stage.GetPrimAtPath(f"{graph_path}/PublishJointState"),
            attribute="inputs:targetPrim",
            target_prim_paths=[target_art_path],
        )
        set_targets(
            prim=stage.GetPrimAtPath(f"{graph_path}/ArticulationController"),
            attribute="inputs:targetPrim",
            target_prim_paths=[target_art_path],
        )
    
    # 2. Bind Odometry Publisher to the motion-driving prim.
    # In kinematic mode we move `pose_prim_path` directly, so odom must read
    # that prim (binding to robot_prim_path only yields local identity).
    odom_chassis_prim = robot_prim_path if enable_articulation_control else pose_prim_path
    set_targets(
        prim=stage.GetPrimAtPath(f"{graph_path}/OdomPub"),
        attribute="inputs:chassisPrim",
        target_prim_paths=[odom_chassis_prim],
    )
    
    # 3. Bind IMU Publisher to the actual imu_link inside the robot
    # Note: the standard Go2 URDF puts imu_link directly under base
    imu_prim_path = f"{robot_prim_path}/imu_link"
    if not stage.GetPrimAtPath(imu_prim_path).IsValid():
        # Fallback to base_link if imu_link is missing
        imu_prim_path = f"{robot_prim_path}/base_link"
        
    set_targets(
        prim=stage.GetPrimAtPath(f"{graph_path}/ImuPub"),
        attribute="inputs:imuPrim",
        target_prim_paths=[imu_prim_path],
    )
    
    if enable_articulation_control:
        print(
            f"[{ns}] Created OmniGraph Action Graph mapping /joint_command -> Articulation and publishing /odom, /imu. "
            f"Articulation root: {target_art_path}"
        )
    else:
        print(
            f"[{ns}] Created OmniGraph Action Graph publishing /odom and /imu (kinematic mode) "
            f"| odom_chassis={odom_chassis_prim}"
        )
    _debug_graph_timestamp_input(graph_path, "OdomPub")
    _debug_graph_timestamp_input(graph_path, "ImuPub")
    if enable_articulation_control:
        _debug_graph_timestamp_input(graph_path, "PublishJointState")
    sys.stdout.flush()

def _create_ros_global_graph():
    """Creates a global ROS 2 graph for shared publishers like /clock."""
    import omni.graph.core as og
    graph_path = "/Global_ROS2_Graph"
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ClockPub", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ClockPub.inputs:execIn"),
                ("Context.outputs:context", "ClockPub.inputs:context"),
                ("SimTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Context.inputs:useDomainIDEnvVar", True),
            ],
        },
    )
    print("[isaac_dual_bringup] Created Global ROS 2 Graph for /clock")
    _debug_graph_timestamp_input(graph_path, "ClockPub")
    sys.stdout.flush()


def _parse_robot_specs(args) -> list[tuple[str, single.Pose2D]]:
    # Preserve existing CLI precedence: explicit robot-a flags beat robots-config JSON.
    if "--robot-a-namespace" in sys.argv:
        return [
            (args.robot_a_namespace, single.Pose2D(args.robot_a_spawn_x, args.robot_a_spawn_y, args.robot_a_spawn_yaw)),
            (args.robot_b_namespace, single.Pose2D(args.robot_b_spawn_x, args.robot_b_spawn_y, args.robot_b_spawn_yaw)),
        ]

    specs: list[tuple[str, single.Pose2D]] = []
    config_list = json.loads(args.robots_config)
    for cfg in config_list:
        specs.append(
            (
                cfg.get("namespace", "go2"),
                single.Pose2D(cfg.get("x", 0.0), cfg.get("y", 0.0), cfg.get("yaw", 0.0)),
            )
        )
    return specs


def _import_robot_prim(stage, args, ns: str, robot_usd: Path | None, robot_urdf: Path | None) -> str:
    if robot_usd is not None:
        imported = single._import_go2_usd_to_stage(
            stage,
            ns=ns,
            usd_path=robot_usd,
            enable_dynamic_physics=args.enable_dynamic_physics,
        )
        print(f"[isaac_dual_bringup] Referenced Go2 USD at {imported} for ns={ns}", flush=True)
        return imported

    imported = single._import_go2_to_stage(
        stage,
        ns=ns,
        urdf_path=robot_urdf,
        enable_dynamic_physics=args.enable_dynamic_physics,
        snapshot_before=True,
    )
    print(f"[isaac_dual_bringup] Imported Go2 URDF at {imported} for ns={ns}", flush=True)
    return imported


def _create_robot_runtime(stage, args, ns: str, spawn_pose: single.Pose2D, walls: list[single.WallBox], imported_path: str) -> RobotRuntime:
    pose_prim_path, robot_prim_path, attached, direct_pose = single._attach_robot_to_pose_carrier(stage, imported_path, ns)
    if not pose_prim_path.startswith("/World/") or not robot_prim_path.startswith("/World/"):
        raise RuntimeError(
            f"invalid robot prim resolution for ns={ns}: pose_prim={pose_prim_path} robot_prim={robot_prim_path}"
        )
    print(
        f"[isaac_dual_bringup] ns={ns} pose prim: {pose_prim_path} | robot prim: {robot_prim_path} "
        f"| carrier_attached={attached} | direct_pose={direct_pose}",
        flush=True,
    )

    single._set_robot_pose(stage, pose_prim_path, spawn_pose, args.robot_z)

    rays = max(90, args.lidar_rays)
    lidar_angles = tuple(-math.pi + (2.0 * math.pi * i / rays) for i in range(rays))
    lidar_angles_np = None
    if np is not None:
        lidar_angles_np = np.asarray(lidar_angles, dtype=np.float32)

    return RobotRuntime(
        namespace=ns,
        pose=spawn_pose,
        pose_prim_path=pose_prim_path,
        robot_prim_path=robot_prim_path,
        lidar_mode=args.lidar_mode,
        walls=walls,
        wall_cache=_build_wall_cache(walls),
        max_linear_speed=args.max_linear_speed,
        max_angular_speed=args.max_angular_speed,
        robot_radius=args.robot_radius,
        robot_z=args.robot_z,
        lidar_angles=lidar_angles,
        lidar_angles_np=lidar_angles_np,
        pointcloud_period=0.0 if args.pointcloud_hz <= 0.0 else (1.0 / args.pointcloud_hz),
    )


def _configure_robot_rtx_lidar(args, stage, robot: RobotRuntime, ns: str, frame_skip: int) -> None:
    sensor_path = None
    cfg_name = ""
    sensor_source = "created"

    if args.prefer_existing_lidar:
        sensor_path = single._resolve_existing_lidar_sensor_path(
            stage,
            robot_prim_path=robot.robot_prim_path,
            lidar_prim_hint=args.robot_lidar_prim,
        )
    if sensor_path:
        graph_path, render_product = single._create_rtx_graph_for_sensor(
            ros_namespace=ns,
            sensor_path=sensor_path,
            frame_skip=frame_skip,
            full_scan=args.rtx_full_scan,
            lidar_config=args.rtx_lidar_config,
        )
        cfg_name = "authored_sensor"
        sensor_source = "authored"
    else:
        sensor_path, graph_path, render_product, cfg_name = single._create_rtx_lidar_pipeline(
            ros_namespace=ns,
            robot_prim_path=robot.robot_prim_path,
            lidar_config=args.rtx_lidar_config,
            frame_skip=frame_skip,
            full_scan=args.rtx_full_scan,
        )
        sensor_source = "created"

    robot.lidar_mode = "rtx"
    robot.rtx_sensor_path = sensor_path
    robot.rtx_graph_path = graph_path
    robot.rtx_render_product = render_product
    print(
        "[isaac_dual_bringup] RTX lidar enabled | "
        f"ns={ns} sensor={sensor_path} source={sensor_source} config={cfg_name} "
        f"frame_skip={frame_skip} full_scan={args.rtx_full_scan}",
        flush=True,
    )


def main() -> int:
    args = _make_arg_parser().parse_args()
    world_file = Path(args.world_file).expanduser().resolve()
    robot_usd = Path(args.robot_usd).expanduser().resolve() if str(args.robot_usd).strip() else None
    robot_urdf = Path(args.robot_urdf).expanduser().resolve() if str(args.robot_urdf).strip() else None

    
    if not world_file.exists():
        print(f"[isaac_dual_bringup] ERROR: world file not found: {world_file}", file=sys.stderr)
        return 2
    if robot_usd is not None and not robot_usd.exists():
        print(f"[isaac_dual_bringup] ERROR: robot usd not found: {robot_usd}", file=sys.stderr)
        return 2
    if robot_urdf is not None and not robot_urdf.exists():
        print(f"[isaac_dual_bringup] ERROR: robot urdf not found: {robot_urdf}", file=sys.stderr)
        return 2
    if robot_usd is None and robot_urdf is None:
        print("[isaac_dual_bringup] ERROR: provide --robot-usd or --robot-urdf", file=sys.stderr)
        return 2

    walls = single.parse_world_boxes(world_file)
    if not walls:
        print(f"[isaac_dual_bringup] ERROR: no wall boxes parsed from {world_file}", file=sys.stderr)
        return 2

    # Boot Isaac Kit first; omni extensions require this runtime context.
    sim_app = SimulationApp(
        {
            "headless": bool(args.headless),
            "renderer": args.renderer,
            "anti_aliasing": 0,
            "hide_ui": bool(args.headless),
            "physics_device": args.physics_device,
        }
    )

    import omni.kit.app
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_manager.set_extension_enabled_immediate("isaacsim.robot.wheeled_robots", True)
    ext_manager.set_extension_enabled_immediate("isaacsim.core.nodes", True)
    # RTX lidar sensor pipeline extensions — without these the sensor
    # OmniGraph fires but the ray tracing pipeline never initialises.
    for ext_name in [
        "isaacsim.sensors.rtx",
        "omni.sensors.nv.lidar",
        "omni.sensors.nv.materials",
        "omni.sensors.nv.common",
    ]:
        try:
            ext_manager.set_extension_enabled_immediate(ext_name, True)
        except Exception:
            pass  # Not all may exist in every Isaac Sim version

    # Import Isaac/ROS modules only after SimulationApp is created.
    import omni.timeline
    import omni.usd
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, PointCloud2

    stage_ctx = omni.usd.get_context()
    stage_ctx.new_stage()
    stage = stage_ctx.get_stage()
    single._setup_stage(stage, walls=walls, world_name=world_file.name)

    try:
        specs = _parse_robot_specs(args)
    except Exception as exc:
        print(f"[isaac_dual_bringup] ERROR: invalid JSON config: {exc}", file=sys.stderr)
        return 2


    asset_used = f"usd={robot_usd}" if robot_usd is not None else f"urdf={robot_urdf}"
    robot_specs: dict[str, RobotRuntime] = {}
    for ns, spawn_pose in specs:
        imported = _import_robot_prim(stage, args, ns, robot_usd, robot_urdf)
        robot_specs[ns] = _create_robot_runtime(stage, args, ns, spawn_pose, walls, imported)

    world_robot_paths = single._list_world_robot_paths(stage)
    print(f"[isaac_dual_bringup] world go2 prims: {world_robot_paths}", flush=True)

    rclpy.init(args=None)
    rcl_node = rclpy.create_node("isaac_dual_bridge")
    bridge = DualRobotBridge(robot_specs, rcl_node)

    for ns, robot in robot_specs.items():
        if not args.enable_dynamic_physics:
            robot.odom_pub = rcl_node.create_publisher(Odometry, f"/{ns}/odom", 10)
            robot.imu_pub = rcl_node.create_publisher(Imu, f"/{ns}/imu", 10)
        robot.pc_pub = rcl_node.create_publisher(PointCloud2, f"/{ns}/lidar/points", 10)

    

    # Try enabling RTX lidar per robot; fallback independently on failures.
    if args.lidar_mode == "rtx":
        auto_skip = 0
        if args.pointcloud_hz > 0.0:
            auto_skip = max(0, int(round(max(args.loop_hz, 1.0) / args.pointcloud_hz)) - 1)
        frame_skip = args.rtx_frame_skip if args.rtx_frame_skip >= 0 else auto_skip
        for ns, robot in robot_specs.items():
            try:
                _configure_robot_rtx_lidar(args, stage, robot, ns, frame_skip)
            except Exception as exc:
                robot.lidar_mode = "cpu"
                print(
                    "[isaac_dual_bringup] WARNING: failed to enable RTX lidar; falling back to CPU synthetic lidar | "
                    f"ns={ns} reason={exc}",
                    file=sys.stderr,
                    flush=True,
                )
    
    # Ensure rclpy.ok() is checked after node creation
    if not rclpy.ok():
         return 1

    stop = {"requested": False}

    def _request_stop(*_):
        stop["requested"] = True

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    if args.enable_dynamic_physics:
        for ns, robot in robot_specs.items():
            _create_ros2_base_graph(
                ns,
                robot.robot_prim_path,
                robot.pose_prim_path,
                args.max_linear_speed,
                args.max_angular_speed,
                enable_articulation_control=True,
            )
    else:
        print(
            "[isaac_dual_bringup] Kinematic mode: using Python /odom and /imu publishers; "
            "skipping OmniGraph base odom/imu graph.",
            flush=True,
        )
    
    _create_ros_global_graph()

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    # Warmup simulation: RTX renderer + render products need many frames
    # to fully initialise before the lidar sensor pipeline produces output.
    for _ in range(50):
        sim_app.update()

    # ── RTX lidar diagnostic: test render product directly ────────────
    for ns, robot in robot_specs.items():
        if robot.lidar_mode != "rtx" or robot.rtx_render_product is None:
            continue
        try:
            import omni.replicator.core as rep
            rp_path = robot.rtx_render_product.path if hasattr(robot.rtx_render_product, 'path') else str(robot.rtx_render_product)
            annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
            annotator.attach([rp_path])
            # Step a few more frames then read
            for _ in range(10):
                sim_app.update()
            data = annotator.get_data()
            if data:
                keys = list(data.keys()) if isinstance(data, dict) else ["non-dict"]
                print(f"[RTX DIAG] {ns}: annotator returned keys={keys}", flush=True)
                if isinstance(data, dict):
                    for k, v in data.items():
                        import numpy as np
                        if isinstance(v, np.ndarray):
                            if v.size == 0:
                                print(
                                    f"[RTX DIAG] {ns}: {k} shape={v.shape} dtype={v.dtype} size=0 (EMPTY)",
                                    flush=True,
                                )
                            elif np.issubdtype(v.dtype, np.number):
                                finite = np.isfinite(v)
                                finite_count = int(np.count_nonzero(finite))
                                if finite_count > 0:
                                    min_v = float(np.nanmin(v))
                                    max_v = float(np.nanmax(v))
                                    print(
                                        f"[RTX DIAG] {ns}: {k} shape={v.shape} dtype={v.dtype} "
                                        f"finite={finite_count}/{v.size} min={min_v:.4f} max={max_v:.4f}",
                                        flush=True,
                                    )
                                else:
                                    print(
                                        f"[RTX DIAG] {ns}: {k} shape={v.shape} dtype={v.dtype} "
                                        f"finite=0/{v.size}",
                                        flush=True,
                                    )
                            else:
                                print(
                                    f"[RTX DIAG] {ns}: {k} shape={v.shape} dtype={v.dtype} size={v.size}",
                                    flush=True,
                                )
                        elif isinstance(v, (int, float, bool)):
                            print(f"[RTX DIAG] {ns}: {k} = {v}", flush=True)
                        elif isinstance(v, dict):
                            subkeys = list(v.keys())
                            print(f"[RTX DIAG] {ns}: {k} dict_keys={subkeys}", flush=True)
                            for sk, sv in v.items():
                                if isinstance(sv, (int, float, bool, str)):
                                    print(f"[RTX DIAG] {ns}: {k}.{sk} = {sv}", flush=True)
                                elif isinstance(sv, np.ndarray):
                                    print(
                                        f"[RTX DIAG] {ns}: {k}.{sk} shape={sv.shape} dtype={sv.dtype} size={sv.size}",
                                        flush=True,
                                    )
                                else:
                                    print(f"[RTX DIAG] {ns}: {k}.{sk} type={type(sv).__name__}", flush=True)
                        elif isinstance(v, (list, tuple)):
                            print(f"[RTX DIAG] {ns}: {k} len={len(v)} type={type(v).__name__}", flush=True)
                        else:
                            print(f"[RTX DIAG] {ns}: {k} type={type(v).__name__}", flush=True)
            else:
                print(f"[RTX DIAG] {ns}: annotator returned EMPTY/None data", flush=True)
            annotator.detach()
        except Exception as e:
            print(f"[RTX DIAG] {ns}: error: {e}", flush=True)

    target_hz = max(5.0, args.loop_hz)
    target_period = 1.0 / target_hz
    prev_wall_t = time.monotonic()
    last_pose_diag_wall_t = time.monotonic()
    mode_summary = ", ".join(f"{ns}:{robot.lidar_mode}" for ns, robot in robot_specs.items())

    print(
        "[isaac_dual_bringup] running | "
        f"world={world_file} asset={asset_used} headless={args.headless} renderer={args.renderer} "
        f"walls={len(walls)} hz={target_hz:.1f} lidar_modes={mode_summary} pc_hz={args.pointcloud_hz:.1f}",
        flush=True,
    )

    exit_code = 0
    try:
        while not stop["requested"] and sim_app.is_running() and rclpy.ok():
            tick_start = time.monotonic()
            dt = tick_start - prev_wall_t
            prev_wall_t = tick_start
            dt = max(1.0 / 240.0, min(dt, 0.25))

            # 1) handle ROS callbacks
            rclpy.spin_once(rcl_node, timeout_sec=0.0)
            
            # 2) advance logic and publish CPU lidar
            bridge.step(dt, args.enable_dynamic_physics, args.cmd_timeout_sec)

            # 3) advance Isaac frame
            # (OmniGraph automatically runs ROS2 subscriptions and publishes Odometry/IMU/TF)
            sim_app.update()
            
            if not args.enable_dynamic_physics:
                for ns, robot in robot_specs.items():
                    single._set_robot_pose(stage, robot.pose_prim_path, robot.pose, args.robot_z)

            if args.debug_pose_check:
                now_wall_t = time.monotonic()
                if now_wall_t - last_pose_diag_wall_t >= 5.0:
                    last_pose_diag_wall_t = now_wall_t
                    for ns, robot in robot_specs.items():
                        pose_world = single._get_world_translation(stage, robot.pose_prim_path)
                        robot_world = single._get_world_translation(stage, robot.robot_prim_path)
                        print(
                            "[isaac_dual_bringup] pose-check | "
                            f"ns={ns} cmd=({robot.pose.x:.2f},{robot.pose.y:.2f},{robot.pose.yaw:.2f}) "
                            f"carrier_world={pose_world} robot_world={robot_world}",
                            flush=True,
                        )

            elapsed = time.monotonic() - tick_start
            sleep_t = target_period - elapsed
            if sleep_t > 0.0:
                time.sleep(sleep_t)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[isaac_dual_bringup] ERROR: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        if rcl_node:
            rcl_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        timeline.stop()
        sim_app.close()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
