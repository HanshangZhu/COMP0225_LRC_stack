#!/usr/bin/env python3
"""Isaac Sim bringup for single Go2 exploration in t_world.

This script:
- creates a lightweight Isaac stage for the t_world corridor map
- imports a Go2 URDF
- runs a kinematic cmd_vel loop with wall-collision guards
- publishes ROS2 /clock, /odom, /imu and lidar data for go2_issac_stack
  (CPU synthetic lidar or native Isaac RTX lidar)

The ROS autonomy stack remains in ROS2 launch files; this process is the
simulator-side world + robot + stream bringup.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from isaacsim import SimulationApp
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

try:
    import numpy as np
except Exception:  # pragma: no cover - runtime fallback when numpy unavailable
    np = None


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class WallBox:
    name: str
    cx: float
    cy: float
    cz: float
    sx: float
    sy: float
    sz: float
    yaw: float


def _parse_pose(text: str | None) -> tuple[float, float, float, float, float, float]:
    if not text:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    vals = [float(v) for v in text.strip().split()]
    vals = vals + [0.0] * (6 - len(vals))
    return (vals[0], vals[1], vals[2], vals[3], vals[4], vals[5])


def parse_world_boxes(world_file: Path) -> list[WallBox]:
    tree = ET.parse(world_file)
    root = tree.getroot()
    walls: list[WallBox] = []

    for model in root.findall(".//model"):
        model_name = model.get("name", "wall")
        model_pose = _parse_pose(model.findtext("pose"))
        mx, my, mz, _mr, _mp, myaw = model_pose

        for collision in model.findall(".//collision"):
            size_txt = collision.findtext("geometry/box/size")
            if not size_txt:
                continue
            sx, sy, sz = [float(v) for v in size_txt.split()]
            cx, cy, cz, _cr, _cp, cyaw = _parse_pose(collision.findtext("pose"))
            walls.append(
                WallBox(
                    name=model_name,
                    cx=mx + cx,
                    cy=my + cy,
                    cz=mz + cz,
                    sx=sx,
                    sy=sy,
                    sz=sz,
                    yaw=myaw + cyaw,
                )
            )
    return walls


def _angle_wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _rotate_2d(x: float, y: float, yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * x - s * y, s * x + c * y)


def _point_to_local(px: float, py: float, box: WallBox) -> tuple[float, float]:
    dx = px - box.cx
    dy = py - box.cy
    return _rotate_2d(dx, dy, -box.yaw)


def _ray_box_intersection_2d(ox: float, oy: float, dx: float, dy: float, box: WallBox) -> float | None:
    # Transform ray into the box local frame.
    lox, loy = _point_to_local(ox, oy, box)
    ldx, ldy = _rotate_2d(dx, dy, -box.yaw)

    half_x = 0.5 * box.sx
    half_y = 0.5 * box.sy

    t_min = -float("inf")
    t_max = float("inf")

    def update_slab(origin: float, direction: float, slab_min: float, slab_max: float) -> tuple[float, float] | None:
        if abs(direction) < 1e-9:
            if origin < slab_min or origin > slab_max:
                return None
            return (-float("inf"), float("inf"))
        t1 = (slab_min - origin) / direction
        t2 = (slab_max - origin) / direction
        return (min(t1, t2), max(t1, t2))

    slab_x = update_slab(lox, ldx, -half_x, half_x)
    if slab_x is None:
        return None
    slab_y = update_slab(loy, ldy, -half_y, half_y)
    if slab_y is None:
        return None

    t_min = max(t_min, slab_x[0], slab_y[0])
    t_max = min(t_max, slab_x[1], slab_y[1])

    if t_max < 0.0 or t_min > t_max:
        return None
    return max(0.0, t_min)


def _circle_hits_box(px: float, py: float, radius: float, box: WallBox) -> bool:
    lx, ly = _point_to_local(px, py, box)
    half_x = 0.5 * box.sx
    half_y = 0.5 * box.sy
    dx = max(abs(lx) - half_x, 0.0)
    dy = max(abs(ly) - half_y, 0.0)
    return (dx * dx + dy * dy) <= (radius * radius)


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def _stamp_from_seconds(clock_msg_type, t_sec: float):
    sec = int(t_sec)
    nanosec = int((t_sec - sec) * 1e9)
    msg = clock_msg_type()
    msg.sec = sec
    msg.nanosec = nanosec
    return msg


def _make_base_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--world-file", required=True, help="Gazebo SDF world file with wall boxes")
    parser.add_argument("--robot-urdf", default="", help="Go2 URDF fallback path when USD asset is not used")
    parser.add_argument("--robot-usd", default="", help="Go2 USD assembly path (preferred official Unitree asset)")
    parser.add_argument(
        "--robot-lidar-prim",
        default="",
        help="optional lidar prim path; absolute (/World/...) or relative to robot prim (e.g. sensors/lidar)",
    )
    parser.add_argument("--headless", action="store_true", help="run Isaac headless")
    parser.add_argument("--loop-hz", type=float, default=60.0, help="main sim/ROS loop frequency")
    parser.add_argument(
        "--renderer",
        choices=("HydraStorm", "RayTracedLighting"),
        default="HydraStorm",
        help="viewport renderer",
    )
    parser.add_argument("--lidar-rays", type=int, default=720, help="2D lidar ray count")
    parser.add_argument("--pointcloud-hz", type=float, default=5.0, help="pointcloud publish frequency (<=0: every loop)")
    parser.add_argument(
        "--lidar-mode",
        choices=("cpu", "rtx"),
        default="cpu",
        help="lidar backend: cpu synthetic raycast or native RTX lidar",
    )
    parser.add_argument(
        "--rtx-lidar-config",
        default="Unitree_L1",
        help="fallback RTX lidar config (used only when no lidar prim exists in robot USD)",
    )
    parser.add_argument(
        "--rtx-frame-skip",
        type=int,
        default=-1,
        help="RTX lidar publish frame skip (auto from loop_hz/pointcloud_hz when < 0)",
    )
    parser.add_argument(
        "--rtx-full-scan",
        action="store_true",
        help="publish full accumulated RTX pointcloud scan",
    )
    parser.add_argument(
        "--prefer-existing-lidar",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="prefer pre-authored lidar prim from robot USD before creating fallback RTX sensor",
    )
    parser.add_argument("--lidar-range-min", type=float, default=0.20, help="lidar min range")
    parser.add_argument("--lidar-range-max", type=float, default=12.0, help="lidar max range")
    parser.add_argument("--robot-radius", type=float, default=0.28, help="collision circle radius")
    parser.add_argument("--robot-z", type=float, default=0.33, help="robot base z")
    parser.add_argument("--max-linear-speed", type=float, default=0.6, help="cmd linear clamp")
    parser.add_argument("--max-angular-speed", type=float, default=1.5, help="cmd angular clamp")
    parser.add_argument("--cmd-timeout-sec", type=float, default=0.75, help="stale cmd timeout")
    parser.add_argument(
        "--debug-pose-check",
        action="store_true",
        help="periodically print commanded vs world pose for debug",
    )
    parser.add_argument(
        "--enable-dynamic-physics",
        action="store_true",
        help="enable fully dynamic joint physics instead of teleport kinematics",
    )
    return parser


def _make_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Isaac Sim t_world bringup for go2_issac_stack",
        parents=[_make_base_arg_parser()],
    )
    parser.add_argument("--ros-namespace", default="go2_1", help="robot namespace")
    parser.add_argument("--spawn-x", type=float, default=1.0, help="spawn x in world")
    parser.add_argument("--spawn-y", type=float, default=0.0, help="spawn y in world")
    parser.add_argument("--spawn-yaw", type=float, default=0.0, help="spawn yaw in rad")
    parser.add_argument(
        "--low-level-inference-command",
        default="",
        help="optional shell command (e.g. unitree_rl_lab policy runner)",
    )
    return parser


def _create_box_mesh(stage, prim_path, sx, sy, sz, cx, cy, cz, yaw=0.0, display_color=None):
    """Create a UsdGeom.Mesh box with explicit triangulated geometry.

    8 vertices, 12 triangles.  RTX lidar needs real mesh geometry —
    procedural UsdGeom.Cube prims are NOT in the ray-tracing BVH.
    """
    from pxr import Gf, UsdGeom, UsdPhysics, Vt

    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0

    points = Vt.Vec3fArray([
        Gf.Vec3f(-hx, -hy, -hz), Gf.Vec3f( hx, -hy, -hz),
        Gf.Vec3f( hx,  hy, -hz), Gf.Vec3f(-hx,  hy, -hz),
        Gf.Vec3f(-hx, -hy,  hz), Gf.Vec3f( hx, -hy,  hz),
        Gf.Vec3f( hx,  hy,  hz), Gf.Vec3f(-hx,  hy,  hz),
    ])
    face_vertex_counts = Vt.IntArray([3] * 12)
    face_vertex_indices = Vt.IntArray([
        0, 2, 1,  0, 3, 2,   # bottom
        4, 5, 6,  4, 6, 7,   # top
        0, 1, 5,  0, 5, 4,   # front
        2, 3, 7,  2, 7, 6,   # back
        0, 4, 7,  0, 7, 3,   # left
        1, 2, 6,  1, 6, 5,   # right
    ])

    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
    mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
    mesh.CreateSubdivisionSchemeAttr("none")

    xf = UsdGeom.Xformable(mesh.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
    if abs(yaw) > 1e-6:
        xf.AddRotateZOp().Set(float(math.degrees(yaw)))

    if display_color is not None:
        mesh.CreateDisplayColorAttr([display_color])

    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    return mesh


def _setup_stage(stage, walls: Iterable[WallBox], world_name: str):
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, UsdPhysics

    world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world_prim)

    # ── OmniPBR materials (required for RTX lidar visibility) ─────────
    mat_path = "/World/Looks/WallMaterial"
    wall_mat = UsdShade.Material.Define(stage, mat_path)
    wall_shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    wall_shader.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    wall_shader.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    wall_shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    wall_shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.58, 0.37, 0.20))
    wall_shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(0.8)
    wall_shader.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    wall_mat.CreateSurfaceOutput("mdl").ConnectToSource(wall_shader.ConnectableAPI(), "out")

    gnd_path = "/World/Looks/GroundMaterial"
    gnd_mat = UsdShade.Material.Define(stage, gnd_path)
    gnd_shader = UsdShade.Shader.Define(stage, f"{gnd_path}/Shader")
    gnd_shader.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    gnd_shader.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    gnd_shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    gnd_shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.45, 0.45, 0.45))
    gnd_shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(1.0)
    gnd_mat.CreateSurfaceOutput("mdl").ConnectToSource(gnd_shader.ConnectableAPI(), "out")

    # ── Ground plane (mesh box, NOT UsdGeom.Cube) ─────────────────────
    ground = _create_box_mesh(
        stage, "/World/Ground",
        sx=80.0, sy=80.0, sz=0.04,
        cx=8.0, cy=0.0, cz=-0.02,
        display_color=Gf.Vec3f(0.45, 0.45, 0.45),
    )
    UsdShade.MaterialBindingAPI(ground.GetPrim()).Bind(gnd_mat)

    # ── Walls (mesh boxes) ────────────────────────────────────────────
    UsdGeom.Xform.Define(stage, "/World/Walls")
    for i, wall in enumerate(walls):
        prim_path = f"/World/Walls/{wall.name}_{i:03d}"
        mesh = _create_box_mesh(
            stage, prim_path,
            sx=wall.sx, sy=wall.sy, sz=wall.sz,
            cx=wall.cx, cy=wall.cy, cz=wall.cz,
            yaw=wall.yaw,
            display_color=Gf.Vec3f(0.58, 0.37, 0.20),
        )
        UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(wall_mat)

    light = UsdLux.DistantLight.Define(stage, "/World/Sun")
    light.CreateIntensityAttr(2500.0)

    dome = UsdLux.DomeLight.Define(stage, "/World/Dome")
    dome.CreateIntensityAttr(200.0)

    stage.SetMetadata("comment", f"Generated from {world_name}")
    return "/World"


def _enable_isaac_extensions() -> None:
    """Enable core Isaac extensions used by URDF/USD import + ROS RTX bridge."""
    import omni.kit.app

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    for ext in (
        "isaacsim.asset.importer.urdf",
        "isaacsim.sensors.physx",
        "isaacsim.sensors.physics",
        "isaacsim.sensors.rtx",
        "isaacsim.ros2.bridge",
        "isaacsim.core.nodes",
    ):
        try:
            ext_manager.set_extension_enabled_immediate(ext, True)
        except Exception:
            pass

    # Let extension command registration settle before command execution.
    for _ in range(3):
        omni.kit.app.get_app().update()


def _disable_dynamic_physics(stage, prim_path: str) -> None:
    """Disable articulation physics cleanly for kinematic teleport mode.

    For navigation-only (no locomotion) setups the Python bridge drives the
    robot pose directly.  Keep visual prims but disable rigid/joint simulation
    so PhysX does not attempt to instantiate joints between static bodies.
    """
    from pxr import Usd, UsdPhysics

    root = stage.GetPrimAtPath(prim_path)
    if not root.IsValid():
        return
    rb_count = 0
    joint_count = 0
    art_root_count = 0
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rb = UsdPhysics.RigidBodyAPI(prim)
            # Turn simulation off for bodies we are driving via scripted pose.
            rb.CreateRigidBodyEnabledAttr().Set(False)
            rb.CreateKinematicEnabledAttr().Set(True)
            rb_count += 1
        if prim.IsA(UsdPhysics.Joint):
            try:
                UsdPhysics.Joint(prim).CreateJointEnabledAttr().Set(False)
                joint_count += 1
            except Exception:
                pass
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            try:
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                art_root_count += 1
            except Exception:
                pass
    if rb_count or joint_count or art_root_count:
        print(
            "[isaac_bringup] disabled dynamics for kinematic mode | "
            f"rigid_bodies={rb_count} joints={joint_count} articulation_roots={art_root_count} "
            f"under {prim_path}",
            flush=True,
        )


def _import_go2_usd_to_stage(stage, ns: str, usd_path: Path, enable_dynamic_physics: bool = False) -> str:
    import omni.kit.commands

    _enable_isaac_extensions()

    target_path = f"/World/{ns}"
    
    # Check if someone already created the prim (Isaac might when parsing references)
    if stage.GetPrimAtPath(target_path).IsValid():
        return target_path

    # Try creating a reference to the USD file natively
    ok, imported_path = omni.kit.commands.execute(
        "CreateReferenceCommand",
        usd_context=omni.usd.get_context(),
        path_to=target_path,
        asset_path=str(usd_path),
        instanceable=False,
    )
    if not ok:
        raise RuntimeError(f"Failed to create USD reference for {usd_path} to {target_path}")
        
    if not enable_dynamic_physics:
        _disable_dynamic_physics(stage, target_path)

    return target_path


def _world_children_paths(stage) -> set[str]:
    world = stage.GetPrimAtPath("/World")
    if not world.IsValid():
        return set()
    return {str(child.GetPath()) for child in world.GetChildren()}


def _stage_root_children_paths(stage) -> set[str]:
    root = stage.GetPrimAtPath("/")
    if not root.IsValid():
        return set()
    return {str(child.GetPath()) for child in root.GetChildren()}


def _is_invalid_import_root(path: str) -> bool:
    return not path or path in {"/", "/World"}


def _import_go2_to_stage(
    stage,
    ns: str,
    urdf_path: Path,
    enable_dynamic_physics: bool = False,
    snapshot_before: bool = False,
) -> str:
    import omni.kit.commands

    _enable_isaac_extensions()
    before_world_children = _world_children_paths(stage) if snapshot_before else set()
    before_root_children = _stage_root_children_paths(stage) if snapshot_before else set()

    ok, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not ok:
        raise RuntimeError("URDFCreateImportConfig failed")

    import_config.merge_fixed_joints = False
    import_config.fix_base = True
    import_config.make_default_prim = False
    import_config.create_physics_scene = False
    import_config.import_inertia_tensor = True

    ok, imported_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf_path),
        import_config=import_config,
        get_articulation_root=True,
    )
    if not ok or not imported_path:
        raise RuntimeError(f"URDFParseAndImportFile failed for {urdf_path}")

    root_path = ""
    if snapshot_before:
        after_world_children = _world_children_paths(stage)
        after_root_children = _stage_root_children_paths(stage)
        new_candidates = sorted((after_world_children - before_world_children) | (after_root_children - before_root_children))
        for path in new_candidates:
            lower = path.lower()
            if "go2" in lower or "description" in lower:
                root_path = path
                break
        if not root_path and new_candidates:
            root_path = new_candidates[0]

    # URDF importer may return an articulation/root path while the visible model
    # sits under a different top-level prim (commonly /World/go2_description).
    # Always move the top-level /World child that contains the imported path.
    if not root_path:
        root_path = imported_path
    prim = stage.GetPrimAtPath(imported_path)
    while prim.IsValid():
        parent = prim.GetParent()
        if not parent.IsValid():
            break
        parent_path = str(parent.GetPath())
        if parent_path == "/World" or parent_path == "/":
            break
        prim = parent
        root_path = str(prim.GetPath())
    if _is_invalid_import_root(root_path):
        for fallback in ("/go2_description", "/World/go2_description"):
            if stage.GetPrimAtPath(fallback).IsValid():
                root_path = fallback
                break

    if _is_invalid_import_root(root_path):
        raise RuntimeError(
            f"unable to resolve imported robot root for ns={ns} "
            f"(imported_path={imported_path!r}, resolved_root={root_path!r})"
        )

    target_path = f"/World/{ns}"
    if root_path != target_path:
        try:
            ok, _ = omni.kit.commands.execute(
                "MovePrimCommand",
                path_from=root_path,
                path_to=target_path,
                keep_world_transform=False,
            )
            if ok and stage.GetPrimAtPath(target_path).IsValid():
                imported_path = target_path
            else:
                imported_path = root_path
        except Exception:
            # Keep original path if move fails.
            imported_path = root_path
    else:
        imported_path = root_path

    if _is_invalid_import_root(imported_path) or not stage.GetPrimAtPath(imported_path).IsValid():
        for fallback_path in (target_path, "/go2_description", "/World/go2_description"):
            if not _is_invalid_import_root(fallback_path) and stage.GetPrimAtPath(fallback_path).IsValid():
                imported_path = fallback_path
                break
    if _is_invalid_import_root(imported_path) or not stage.GetPrimAtPath(imported_path).IsValid():
        raise RuntimeError(f"Imported robot prim not found: {imported_path}")

    # Hide stale duplicate if importer leaves one at the default path.
    for stale_path in ("/go2_description", "/World/go2_description"):
        if stale_path == imported_path:
            continue
        stale_prim = stage.GetPrimAtPath(stale_path)
        if stale_prim.IsValid():
            from pxr import UsdGeom

            try:
                UsdGeom.Imageable(stale_prim).MakeInvisible()
                print(f"[isaac_bringup] hidden stale robot prim: {stale_path}")
            except Exception:
                pass
    if not enable_dynamic_physics:
        _disable_dynamic_physics(stage, imported_path)
    return imported_path


def _attach_robot_to_pose_carrier(stage, imported_path: str, ns: str) -> tuple[str, str, bool, bool]:
    """Attach imported robot under a parent Xform we can move reliably.

    This avoids visual stalling when the imported articulation root is not the
    right prim to animate directly in the viewport.
    """
    import omni.kit.commands
    from pxr import UsdGeom

    if _is_invalid_import_root(imported_path):
        raise RuntimeError(f"invalid imported robot path for namespace {ns}: {imported_path!r}")
    if not stage.GetPrimAtPath(imported_path).IsValid():
        raise RuntimeError(f"robot prim does not exist for namespace {ns}: {imported_path!r}")

    carrier_path = f"/World/{ns}_carrier"
    UsdGeom.Xform.Define(stage, carrier_path)

    leaf = imported_path.rsplit("/", 1)[-1] or "robot"
    target_robot_path = f"{carrier_path}/{leaf}"
    moved_robot_path = imported_path

    if imported_path != target_robot_path:
        try:
            ok, _ = omni.kit.commands.execute(
                "MovePrimCommand",
                path_from=imported_path,
                path_to=target_robot_path,
                keep_world_transform=True,
            )
            if ok:
                moved_robot_path = target_robot_path
        except Exception:
            # Keep the original path if move fails.
            pass

    attached = moved_robot_path == target_robot_path
    pose_target_path = carrier_path if attached else moved_robot_path

    # Some imported articulation roots reset xform stack and ignore parent motion.
    # In that case, drive the robot prim directly.
    uses_robot_direct_pose = False
    robot_prim = stage.GetPrimAtPath(moved_robot_path)
    if robot_prim.IsValid():
        try:
            xformable = UsdGeom.Xformable(robot_prim)
            if xformable.GetResetXformStack():
                pose_target_path = moved_robot_path
                uses_robot_direct_pose = True
        except Exception:
            pass

    return pose_target_path, moved_robot_path, attached, uses_robot_direct_pose


def _set_robot_pose(stage, prim_path: str, pose: Pose2D, z_height: float):
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    xform = UsdGeom.Xformable(prim)
    ops = xform.GetOrderedXformOps()
    translate_op = None
    orient_op = None
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
        if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            orient_op = op
    if translate_op is None:
        translate_op = xform.AddTranslateOp()
    if orient_op is None:
        orient_op = xform.AddOrientOp()

    translate_op.Set(Gf.Vec3d(pose.x, pose.y, z_height))
    orient_op.Set(Gf.Quatf(math.cos(0.5 * pose.yaw), Gf.Vec3f(0.0, 0.0, math.sin(0.5 * pose.yaw))))


def _get_world_pose(stage, prim_path: str):
    from pxr import UsdGeom
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None, None
    cache = UsdGeom.XformCache()
    mat = cache.GetLocalToWorldTransform(prim)
    vec = mat.ExtractTranslation()
    q = mat.ExtractRotation().GetQuat()
    # Pxr returns (w, x, y, z) for GetQuat() but GetImaginary returns the vector.
    im = q.GetImaginary()
    return (float(vec[0]), float(vec[1]), float(vec[2])), (float(im[0]), float(im[1]), float(im[2]), float(q.GetReal()))



def _list_world_robot_paths(stage) -> list[str]:
    world = stage.GetPrimAtPath("/World")
    if not world.IsValid():
        return []
    paths: list[str] = []
    for child in world.GetChildren():
        p = str(child.GetPath())
        if "go2" in p.lower():
            paths.append(p)
    return sorted(paths)


def _maybe_launch_low_level_process(command: str) -> subprocess.Popen | None:
    if not command.strip():
        return None
    return subprocess.Popen(["bash", "-lc", command], start_new_session=True)


def _install_custom_lidar_configs():
    """Copy custom lidar JSON profiles into Isaac Sim's built-in config dir."""
    import shutil
    import omni.kit.app
    try:
        # Use omni API to find the extension path reliably
        manager = omni.kit.app.get_app().get_extension_manager()
        if manager.is_extension_enabled("isaacsim.sensors.rtx"):
            ext_id = manager.get_enabled_extension_id("isaacsim.sensors.rtx")
            ext_path = manager.get_extension_path(ext_id)
            isaac_cfg_root = Path(ext_path) / "data" / "lidar_configs"
        else:
             # Fallback if extension not found via manager (should not happen in Isaac Sim)
             import isaacsim.sensors.rtx as _rtx_mod
             p = Path(_rtx_mod.__file__).resolve().parent
             isaac_cfg_root = None
             for _ in range(6):
                candidate = p / "data" / "lidar_configs"
                if candidate.is_dir():
                    isaac_cfg_root = candidate
                    break
                p = p.parent
             if isaac_cfg_root is None:
                 print(f"[isaac_bringup] WARNING: could not locate Isaac Sim lidar_configs dir", flush=True)
                 return
    except Exception as e:
        print(f"[isaac_bringup] WARNING: error locating Isaac Sim lidar_configs: {e}", flush=True)
        return

    # Locate config dir (handles both source/symlink and install layouts)
    base_path = Path(__file__).resolve().parent
    # 1. Source/Symlink: <src>/scripts/ -> <src>/config
    # 2. Install: <install>/lib/pkg/ -> <install>/share/pkg/config
    candidates = [
        base_path.parent / "config" / "lidar_configs",
        base_path.parent.parent / "share" / "go2_issac_stack" / "config" / "lidar_configs",
    ]

    pkg_cfg = None
    for p in candidates:
        if p.is_dir():
            pkg_cfg = p
            break
            
    if pkg_cfg is None:
        print(f"[isaac_bringup] WARNING: could not locate lidar_configs in candidates: {candidates}", flush=True)
        return

    installed = []
    for vendor_dir in pkg_cfg.iterdir():
        if not vendor_dir.is_dir():
            continue
        target_dir = isaac_cfg_root  # installing to root of lidar_configs to ensure visibility
        # target_dir = isaac_cfg_root / vendor_dir.name
        target_dir.mkdir(parents=True, exist_ok=True)
        for json_file in vendor_dir.glob("*.json"):
            dest = target_dir / json_file.name
            if not dest.exists() or json_file.read_bytes() != dest.read_bytes():
                shutil.copy2(json_file, dest)
                installed.append(json_file.name)

    if installed:
        print(f"[isaac_bringup] installed custom lidar configs: {installed}", flush=True)


def _normalize_rtx_lidar_config(config_name: str) -> str:
    # Preserve directory structure but remove extension
    path = Path(config_name)
    if path.suffix == ".json":
        return str(path.with_suffix(""))
    return str(path)


def _find_isaac_lidar_config_root() -> Path | None:
    """Locate Isaac Sim's active RTX lidar config directory."""
    import omni.kit.app

    try:
        manager = omni.kit.app.get_app().get_extension_manager()
        ext_name = "isaacsim.sensors.rtx"
        if manager.is_extension_enabled(ext_name):
            ext_id = manager.get_enabled_extension_id(ext_name)
            ext_path = Path(manager.get_extension_path(ext_id))
            cfg_root = ext_path / "data" / "lidar_configs"
            if cfg_root.is_dir():
                return cfg_root
    except Exception:
        return None
    return None


def _resolve_rtx_lidar_config(config_name: str) -> str:
    """Resolve a user config name to an installed Isaac lidar config identifier."""
    requested = _normalize_rtx_lidar_config(config_name)
    cfg_root = _find_isaac_lidar_config_root()
    if cfg_root is None:
        return requested

    rel_configs: set[str] = set()
    by_basename: dict[str, list[str]] = {}
    for json_file in cfg_root.rglob("*.json"):
        rel = json_file.relative_to(cfg_root).with_suffix("").as_posix()
        rel_configs.add(rel)
        base = Path(rel).name
        by_basename.setdefault(base, []).append(rel)

    if requested in rel_configs:
        return requested

    if "/" not in requested:
        matches = sorted(set(by_basename.get(requested, [])))
        if len(matches) == 1:
            resolved = matches[0]
            print(
                f"[isaac_bringup] resolved RTX lidar config alias {requested} -> {resolved}",
                flush=True,
            )
            return resolved
        if len(matches) > 1:
            resolved = sorted(matches, key=lambda s: (s.count("/"), len(s), s))[0]
            print(
                f"[isaac_bringup] WARNING: ambiguous RTX lidar config '{requested}' matches {matches}; "
                f"using '{resolved}'",
                flush=True,
            )
            return resolved

    fallback_candidates = (
        "Unitree/Unitree_L1",
        "Unitree_L1",
        "NVIDIA/Debug_Rotary",
        "Debug_Rotary",
    )
    for fallback in fallback_candidates:
        if fallback in rel_configs:
            print(
                f"[isaac_bringup] WARNING: RTX lidar config '{requested}' not found; "
                f"falling back to '{fallback}'",
                flush=True,
            )
            return fallback
        if "/" not in fallback:
            matches = sorted(set(by_basename.get(fallback, [])))
            if matches:
                resolved = sorted(matches, key=lambda s: (s.count("/"), len(s), s))[0]
                print(
                    f"[isaac_bringup] WARNING: RTX lidar config '{requested}' not found; "
                    f"falling back to '{resolved}'",
                    flush=True,
                )
                return resolved

    print(
        f"[isaac_bringup] WARNING: RTX lidar config '{requested}' not found and no known fallback available",
        flush=True,
    )
    return requested


def _create_ros2_control_graph(ros_namespace: str, robot_prim_path: str):
    import omni.graph.core as og
    from isaacsim.core.utils.prims import set_targets
    import omni.usd

    graph_path = f"/{ros_namespace}_ros2_control_graph"
    # Ensure graph base path exists if creating under World or similar,
    # but og.Controller creates it.
    
    # Wait, the nodeNamespace input allows us to just specify topicName as
    # "joint_states" and it'll prefix.
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("SubscribeJointState.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishJointState.inputs:topicName", "joint_states"),
                ("PublishJointState.inputs:nodeNamespace", ros_namespace),
                ("SubscribeJointState.inputs:topicName", "joint_command"),
                ("SubscribeJointState.inputs:nodeNamespace", ros_namespace),
            ]
        },
    )
    
    stage = omni.usd.get_context().get_stage()
    
    # Robustly find the articulation root
    from pxr import UsdPhysics, Usd
    target_art_path = robot_prim_path
    prim = stage.GetPrimAtPath(robot_prim_path)
    if prim.IsValid() and not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        for desc in Usd.PrimRange(prim):
            if desc.HasAPI(UsdPhysics.ArticulationRootAPI):
                target_art_path = str(desc.GetPath())
                break
                
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
    print(f"[{ros_namespace}] Created ROS2 Articulation Control Graph for {robot_prim_path}", flush=True)


def _resolve_existing_lidar_sensor_path(stage, robot_prim_path: str, lidar_prim_hint: str = "") -> str | None:
    """Find an authored lidar prim under robot, optionally from a user hint."""
    from pxr import Usd

    hint = lidar_prim_hint.strip()
    if hint:
        candidates = [hint]
        if not hint.startswith("/"):
            rel = hint.lstrip("/")
            candidates.extend(
                [
                    f"{robot_prim_path}/{rel}",
                    f"{robot_prim_path}/sensors/{rel}",
                    f"{robot_prim_path}/sensor/{rel}",
                ]
            )
        for path in candidates:
            prim = stage.GetPrimAtPath(path)
            if prim.IsValid():
                return path

    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        return None

    keywords = ("lidar", "laser", "radar", "l1", "hesai", "velodyne")
    for prim in Usd.PrimRange(robot_prim):
        path = prim.GetPath().pathString
        lname = path.lower()
        if any(k in lname for k in keywords):
            # Prefer authored sensor-like prims (Camera/Lidar APIs often use these names).
            if prim.GetTypeName() in ("Camera", "Lidar", "Xform"):
                return path
    return None


def _build_rtx_lidar_graph(
    *,
    ros_namespace: str,
    sensor_path: str,
    frame_skip: int,
    full_scan: bool,
):
    """Build an OmniGraph that publishes RTX lidar data via ROS 2.

    *sensor_path* **must** be a prim created via ``IsaacSensorCreateRtxLidar``.
    """
    import omni.graph.core as og
    import omni.replicator.core as rep

    render_product = rep.create.render_product(sensor_path, resolution=(1, 1))
    render_product_path = render_product.path

    graph_path = f"/{ros_namespace}_rtx_lidar_graph"
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("RtxPointCloudPublish", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Context.inputs:useDomainIDEnvVar", True),
                ("RtxPointCloudPublish.inputs:topicName", "lidar/points"),
                ("RtxPointCloudPublish.inputs:type", "point_cloud"),
                ("RtxPointCloudPublish.inputs:frameId", "base_link"),
                ("RtxPointCloudPublish.inputs:nodeNamespace", ros_namespace),
                ("RtxPointCloudPublish.inputs:renderProductPath", render_product_path),
                ("RtxPointCloudPublish.inputs:frameSkipCount", max(0, int(frame_skip))),
                ("RtxPointCloudPublish.inputs:fullScan", bool(full_scan)),
                ("RtxPointCloudPublish.inputs:resetSimulationTimeOnStop", True),
                ("RtxPointCloudPublish.inputs:showDebugView", False),
                ("RtxPointCloudPublish.inputs:useSystemTime", False),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RtxPointCloudPublish.inputs:execIn"),
                ("Context.outputs:context", "RtxPointCloudPublish.inputs:context"),
            ],
        },
    )
    return graph_path, render_product


def _create_rtx_graph_for_sensor(
    *,
    ros_namespace: str,
    sensor_path: str,
    frame_skip: int,
    full_scan: bool,
    lidar_config: str = "Unitree_L1",
):
    """Create a proper RTX lidar sensor near an **authored** USD prim and build the ROS graph.

    The *sensor_path* typically points to a prim in a USD asset (e.g.
    Unitree's ``base/radar``) that is **not** a valid Isaac RTX sensor.
    ``ROS2RtxLidarHelper`` only works with prims created through
    ``IsaacSensorCreateRtxLidar``, so we create one as a sibling of the
    authored prim and use *that* for the render product.
    """
    import omni.kit.commands

    from pxr import Gf, Usd, UsdGeom

    stage = omni.usd.get_context().get_stage()
    authored_prim = stage.GetPrimAtPath(sensor_path)

    # Determine the parent under which we will create the real RTX sensor.
    parent_path = str(authored_prim.GetParent().GetPath()) if authored_prim.IsValid() else sensor_path.rsplit("/", 1)[0]

    # Read the authored prim's local transform so we mount the RTX sensor
    # at exactly the same position / orientation.
    translation = Gf.Vec3d(0.0, 0.0, 0.0)
    orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
    if authored_prim.IsValid():
        xformable = UsdGeom.Xformable(authored_prim)
        if xformable:
            local_xform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            parent_prim = stage.GetPrimAtPath(parent_path)
            if parent_prim.IsValid():
                parent_xform = UsdGeom.Xformable(parent_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                local_xform = local_xform * parent_xform.GetInverse()
            translation = local_xform.ExtractTranslation()

    rtx_sensor_name = f"{ros_namespace}_rtx_lidar"
    _install_custom_lidar_configs()
    config = _resolve_rtx_lidar_config(lidar_config)

    ok, sensor_prim = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=rtx_sensor_name,
        parent=parent_path,
        config=config,
        translation=translation,
        orientation=orientation,
        visibility=False,
    )
    if not ok or sensor_prim is None:
        raise RuntimeError(
            f"IsaacSensorCreateRtxLidar failed at parent={parent_path} "
            f"(authored hint was {sensor_path})"
        )

    actual_sensor_path = sensor_prim.GetPath().pathString
    print(
        f"[isaac_bringup] created RTX lidar sensor at {actual_sensor_path} "
        f"(mounted near authored prim {sensor_path})",
        flush=True,
    )

    return _build_rtx_lidar_graph(
        ros_namespace=ros_namespace,
        sensor_path=actual_sensor_path,
        frame_skip=frame_skip,
        full_scan=full_scan,
    )


def _create_rtx_lidar_pipeline(
    *,
    ros_namespace: str,
    robot_prim_path: str,
    lidar_config: str,
    frame_skip: int,
    full_scan: bool,
):
    import omni.kit.commands

    from pxr import Gf

    # Ensure custom configs are installed in Isaac Sim's config directory.
    _install_custom_lidar_configs()
    config = _resolve_rtx_lidar_config(lidar_config)

    ok, sensor_prim = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path="velodyne",
        parent=robot_prim_path,
        config=config,
        translation=Gf.Vec3d(0.0, 0.0, 0.0),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        visibility=False,
    )
    if not ok or sensor_prim is None:
        raise RuntimeError(f"IsaacSensorCreateRtxLidar failed (config={config})")

    print(
        f"[isaac_bringup] Lidar Created with config={config} "
        f"path={sensor_prim.GetPath().pathString} type={sensor_prim.GetTypeName()}",
        flush=True,
    )
    sensor_path = sensor_prim.GetPath().pathString
    graph_path, render_product = _build_rtx_lidar_graph(
        ros_namespace=ros_namespace,
        sensor_path=sensor_path,
        frame_skip=frame_skip,
        full_scan=full_scan,
    )
    return sensor_path, graph_path, render_product, config


class IsaacKinematicBridge(Node):
    # This node is the simulator-side contract for one robot:
    # - input: `/<ns>/isaac/cmd_vel`
    # - outputs: /clock, `/<ns>/odom`, `/<ns>/imu`, `/<ns>/lidar/points`
    def __init__(self, namespace: str, walls: list[WallBox], args) -> None:
        super().__init__(f"{namespace}_isaac_kinematic_bridge")
        self.pose = Pose2D(args.spawn_x, args.spawn_y, args.spawn_yaw)
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.last_cmd_wall_t = 0.0
        self.prev_lin = 0.0
        self.sim_time = 0.0
        self.loop_count = 0
        self.last_loop_log_wall_t = time.monotonic()
        self.pointcloud_period = 0.0 if args.pointcloud_hz <= 0.0 else (1.0 / args.pointcloud_hz)
        self.next_pointcloud_t = 0.0
        rays = max(90, args.lidar_rays)
        self.lidar_angles = tuple(-math.pi + (2.0 * math.pi * i / rays) for i in range(rays))
        self.lidar_mode = args.lidar_mode
        self.rtx_sensor_path = ""
        self.rtx_graph_path = ""
        self.rtx_render_product = None
        self._walls = walls
        self._args = args
        self._wall_cache = self._build_wall_cache()
        self._lidar_angles_np = np.asarray(self.lidar_angles, dtype=np.float32) if np is not None else None

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, f"/{namespace}/odom", 10)
        self.imu_pub = self.create_publisher(Imu, f"/{namespace}/imu", 10)
        self.pc_pub = self.create_publisher(PointCloud2, f"/{namespace}/lidar/points", 10)
        self.cmd_sub = self.create_subscription(Twist, f"/{namespace}/isaac/cmd_vel", self._cmd_cb, 20)

    def _cmd_cb(self, msg: Twist):
        self.cmd_linear = float(max(-self._args.max_linear_speed, min(self._args.max_linear_speed, msg.linear.x)))
        self.cmd_angular = float(max(-self._args.max_angular_speed, min(self._args.max_angular_speed, msg.angular.z)))
        self.last_cmd_wall_t = time.monotonic()

    def _is_colliding(self, px: float, py: float) -> bool:
        for wall in self._walls:
            if _circle_hits_box(px, py, self._args.robot_radius, wall):
                return True
        return False

    def _raycast_distance(self, local_angle: float) -> float | None:
        world_angle = self.pose.yaw + local_angle
        dx = math.cos(world_angle)
        dy = math.sin(world_angle)

        best = None
        for wall in self._walls:
            t = _ray_box_intersection_2d(self.pose.x, self.pose.y, dx, dy, wall)
            if t is None:
                continue
            if t < self._args.lidar_range_min or t > self._args.lidar_range_max:
                continue
            if best is None or t < best:
                best = t
        return best

    def _build_wall_cache(self):
        if np is None or not self._walls:
            return None
        cx = np.asarray([w.cx for w in self._walls], dtype=np.float32)
        cy = np.asarray([w.cy for w in self._walls], dtype=np.float32)
        half_x = np.asarray([0.5 * w.sx for w in self._walls], dtype=np.float32)
        half_y = np.asarray([0.5 * w.sy for w in self._walls], dtype=np.float32)
        yaw = np.asarray([w.yaw for w in self._walls], dtype=np.float32)
        return {
            "cx": cx,
            "cy": cy,
            "half_x": half_x,
            "half_y": half_y,
            "cos_yaw": np.cos(yaw),
            "sin_yaw": np.sin(yaw),
        }

    def _raycast_distances_vectorized(self):
        if np is None or self._wall_cache is None or self._lidar_angles_np is None:
            return None
        cache = self._wall_cache
        angles = self.pose.yaw + self._lidar_angles_np
        dx = np.cos(angles)
        dy = np.sin(angles)

        ox = np.float32(self.pose.x)
        oy = np.float32(self.pose.y)
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
        best = np.where(
            (best >= self._args.lidar_range_min) & (best <= self._args.lidar_range_max),
            best,
            np.inf,
        )
        return best

    def _publish_clock(self):
        msg = Clock()
        msg.clock = _stamp_from_seconds(type(msg.clock), self.sim_time)
        self.clock_pub.publish(msg)

    def _publish_odom(self, linear_speed: float, angular_speed: float):
        msg = Odometry()
        msg.header.stamp = _stamp_from_seconds(type(msg.header.stamp), self.sim_time)
        msg.header.frame_id = "world"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.pose.x
        msg.pose.pose.position.y = self.pose.y
        msg.pose.pose.position.z = self._args.robot_z
        qx, qy, qz, qw = _quat_from_yaw(self.pose.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = linear_speed
        msg.twist.twist.angular.z = angular_speed
        self.odom_pub.publish(msg)

    def _publish_imu(self, dt: float, linear_speed: float, angular_speed: float):
        lin_acc = 0.0 if dt <= 1e-6 else (linear_speed - self.prev_lin) / dt
        self.prev_lin = linear_speed

        msg = Imu()
        msg.header.stamp = _stamp_from_seconds(type(msg.header.stamp), self.sim_time)
        msg.header.frame_id = "imu_link"
        qx, qy, qz, qw = _quat_from_yaw(self.pose.yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.z = angular_speed
        msg.linear_acceleration.x = lin_acc
        msg.linear_acceleration.z = 9.81
        self.imu_pub.publish(msg)

    def _publish_pointcloud(self):
        from sensor_msgs.msg import PointField

        header = Header()
        header.stamp = _stamp_from_seconds(type(header.stamp), self.sim_time)
        header.frame_id = "base_link"
        points = []
        distances = self._raycast_distances_vectorized()
        if distances is None:
            for a in self.lidar_angles:
                dist = self._raycast_distance(a)
                if dist is None:
                    continue
                points.append((dist * math.cos(a), dist * math.sin(a), 0.25, 255.0))
        elif np is not None and self._lidar_angles_np is not None:
            finite = np.isfinite(distances)
            if np.any(finite):
                d = distances[finite]
                a = self._lidar_angles_np[finite]
                x = d * np.cos(a)
                y = d * np.sin(a)
                points = list(zip(x.tolist(), y.tolist(), [0.25] * len(x), [255.0] * len(x)))

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg = point_cloud2.create_cloud(header, fields, points)
        self.pc_pub.publish(msg)

    def step(self, dt: float, dynamic_physics_enabled: bool = False, phys_x: float = 0.0, phys_y: float = 0.0, phys_yaw: float = 0.0):
        now = time.monotonic()
        cmd_stale = (self.last_cmd_wall_t <= 0.0) or ((now - self.last_cmd_wall_t) > self._args.cmd_timeout_sec)
        linear = 0.0 if cmd_stale else self.cmd_linear
        angular = 0.0 if cmd_stale else self.cmd_angular

        if dynamic_physics_enabled:
            self.pose.x = phys_x
            self.pose.y = phys_y
            self.pose.yaw = phys_yaw
        else:
            new_yaw = _angle_wrap(self.pose.yaw + angular * dt)
            step_x = linear * math.cos(self.pose.yaw) * dt
            step_y = linear * math.sin(self.pose.yaw) * dt
            new_x = self.pose.x + step_x
            new_y = self.pose.y + step_y

            # Hard-stop linear motion when the next pose intersects world walls.
            if self._is_colliding(new_x, new_y):
                new_x = self.pose.x
                new_y = self.pose.y
                linear = 0.0

            self.pose.x = new_x
            self.pose.y = new_y
            self.pose.yaw = new_yaw

        self.sim_time += dt

        self._publish_clock()
        self._publish_odom(linear_speed=linear, angular_speed=angular)
        self._publish_imu(dt=dt, linear_speed=linear, angular_speed=angular)
        if self.lidar_mode == "cpu":
            if self.pointcloud_period <= 0.0 or self.sim_time >= self.next_pointcloud_t:
                self._publish_pointcloud()
                if self.pointcloud_period > 0.0:
                    while self.next_pointcloud_t <= self.sim_time:
                        self.next_pointcloud_t += self.pointcloud_period

        self.loop_count += 1
        if now - self.last_loop_log_wall_t >= 5.0:
            self.last_loop_log_wall_t = now
            self.get_logger().info(
                f"bridge alive | pose=({self.pose.x:.2f},{self.pose.y:.2f},{self.pose.yaw:.2f}) "
                f"cmd=({self.cmd_linear:.2f},{self.cmd_angular:.2f})"
            )


def main() -> int:
    args = _make_arg_parser().parse_args()
    world_file = Path(args.world_file).expanduser().resolve()
    robot_usd = Path(args.robot_usd).expanduser().resolve() if str(args.robot_usd).strip() else None
    robot_urdf = Path(args.robot_urdf).expanduser().resolve() if str(args.robot_urdf).strip() else None

    if not world_file.exists():
        print(f"[isaac_bringup] ERROR: world file not found: {world_file}", file=sys.stderr)
        return 2
    if robot_usd is not None and not robot_usd.exists():
        print(f"[isaac_bringup] ERROR: robot usd not found: {robot_usd}", file=sys.stderr)
        return 2
    if robot_urdf is not None and not robot_urdf.exists():
        print(f"[isaac_bringup] ERROR: robot urdf not found: {robot_urdf}", file=sys.stderr)
        return 2
    if robot_usd is None and robot_urdf is None:
        print("[isaac_bringup] ERROR: provide --robot-usd or --robot-urdf", file=sys.stderr)
        return 2

    walls = parse_world_boxes(world_file)
    if not walls:
        print(f"[isaac_bringup] ERROR: no wall boxes parsed from {world_file}", file=sys.stderr)
        return 2

    # Boot Isaac Kit first; most omni/ROS bridge modules depend on this context.
    sim_app = SimulationApp(
        {
            "headless": bool(args.headless),
            "renderer": args.renderer,
            "anti_aliasing": 0,
            "hide_ui": bool(args.headless),
        }
    )

    # Import Isaac modules only after SimulationApp is created.
    import omni.timeline
    import omni.usd
    import rclpy

    stage_ctx = omni.usd.get_context()
    stage_ctx.new_stage()
    stage = stage_ctx.get_stage()
    _setup_stage(stage, walls=walls, world_name=world_file.name)

    asset_used = ""
    robot_prim_path = ""
    try:
        if robot_usd is not None:
            robot_prim_path = _import_go2_usd_to_stage(
                stage,
                ns=args.ros_namespace,
                usd_path=robot_usd,
                enable_dynamic_physics=args.enable_dynamic_physics,
            )
            asset_used = f"usd={robot_usd}"
            print(f"[isaac_bringup] Referenced Go2 USD at {robot_prim_path}")
        else:
            robot_prim_path = _import_go2_to_stage(
                stage,
                ns=args.ros_namespace,
                urdf_path=robot_urdf,
                enable_dynamic_physics=args.enable_dynamic_physics,
            )
            asset_used = f"urdf={robot_urdf}"
            print(f"[isaac_bringup] Imported Go2 URDF at {robot_prim_path}")
    except Exception as exc:
        # Keep bringup alive with a visible fallback prim so autonomy debug can proceed.
        from pxr import Gf, UsdGeom

        source = f"usd={robot_usd}" if robot_usd is not None else f"urdf={robot_urdf}"
        print(f"[isaac_bringup] WARNING: failed to import Go2 asset ({source}): {exc}", file=sys.stderr)
        robot_prim_path = f"/World/{args.ros_namespace}"
        cube = UsdGeom.Cube.Define(stage, robot_prim_path)
        cube.CreateSizeAttr(1.0)
        xf = UsdGeom.Xformable(cube.GetPrim())
        xf.AddScaleOp().Set(Gf.Vec3f(0.50, 0.28, 0.24))
        UsdGeom.Gprim(cube.GetPrim()).CreateDisplayColorAttr([Gf.Vec3f(0.15, 0.55, 0.90)])
        asset_used = "fallback_cube"

    rclpy.init(args=None)
    bridge = IsaacKinematicBridge(namespace=args.ros_namespace, walls=walls, args=args)
    pose_prim_path, robot_prim_path, attached, direct_pose = _attach_robot_to_pose_carrier(
        stage, robot_prim_path, args.ros_namespace
    )
    print(
        f"[isaac_bringup] pose prim: {pose_prim_path} | robot prim: {robot_prim_path} "
        f"| carrier_attached={attached} | direct_pose={direct_pose}",
        flush=True,
    )
    world_robot_paths = _list_world_robot_paths(stage)
    print(f"[isaac_bringup] world go2 prims: {world_robot_paths}", flush=True)
    if args.enable_dynamic_physics:
        _create_ros2_control_graph(args.ros_namespace, robot_prim_path)
    else:
        _set_robot_pose(stage, pose_prim_path, bridge.pose, args.robot_z)
    # so autonomy can still run for debugging/CI.
    if args.lidar_mode == "rtx":
        auto_skip = 0
        if args.pointcloud_hz > 0.0:
            auto_skip = max(0, int(round(max(args.loop_hz, 1.0) / args.pointcloud_hz)) - 1)
        frame_skip = args.rtx_frame_skip if args.rtx_frame_skip >= 0 else auto_skip
        try:
            sensor_path = None
            cfg_name = ""
            sensor_source = "created"

            if args.prefer_existing_lidar:
                sensor_path = _resolve_existing_lidar_sensor_path(
                    stage,
                    robot_prim_path=robot_prim_path,
                    lidar_prim_hint=args.robot_lidar_prim,
                )

            if sensor_path:
                graph_path, render_product = _create_rtx_graph_for_sensor(
                    ros_namespace=args.ros_namespace,
                    sensor_path=sensor_path,
                    frame_skip=frame_skip,
                    full_scan=args.rtx_full_scan,
                    lidar_config=args.rtx_lidar_config,
                )
                cfg_name = "authored_sensor"
                sensor_source = "authored"
            else:
                sensor_path, graph_path, render_product, cfg_name = _create_rtx_lidar_pipeline(
                    ros_namespace=args.ros_namespace,
                    robot_prim_path=robot_prim_path,
                    lidar_config=args.rtx_lidar_config,
                    frame_skip=frame_skip,
                    full_scan=args.rtx_full_scan,
                )
                sensor_source = "created"

            bridge.lidar_mode = "rtx"
            bridge.rtx_sensor_path = sensor_path
            bridge.rtx_graph_path = graph_path
            bridge.rtx_render_product = render_product
            print(
                "[isaac_bringup] RTX lidar enabled | "
                f"sensor={sensor_path} source={sensor_source} config={cfg_name} frame_skip={frame_skip} "
                f"full_scan={args.rtx_full_scan}",
                flush=True,
            )
        except Exception as exc:
            bridge.lidar_mode = "cpu"
            print(
                "[isaac_bringup] WARNING: failed to enable RTX lidar; falling back to CPU synthetic lidar | "
                f"reason={exc}",
                file=sys.stderr,
                flush=True,
            )

    low_level_proc = _maybe_launch_low_level_process(args.low_level_inference_command)
    if low_level_proc is not None:
        print(
            f"[isaac_bringup] started low-level process pid={low_level_proc.pid}",
            flush=True,
        )

    stop = {"requested": False}

    def _request_stop(*_):
        stop["requested"] = True

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    # Keep the timeline running for Isaac internal updates.
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    target_hz = max(5.0, args.loop_hz)
    target_period = 1.0 / target_hz
    prev_wall_t = time.monotonic()
    last_pose_diag_wall_t = time.monotonic()

    print(
        "[isaac_bringup] running | "
        f"ns={args.ros_namespace} world={world_file} asset={asset_used} "
        f"headless={args.headless} renderer={args.renderer} walls={len(walls)} "
        f"hz={target_hz:.1f} lidar_mode={bridge.lidar_mode} "
        f"pc_hz={args.pointcloud_hz:.1f} rays={args.lidar_rays}",
        flush=True,
    )

    exit_code = 0
    try:
        while not stop["requested"] and sim_app.is_running() and rclpy.ok():
            tick_start = time.monotonic()
            dt = tick_start - prev_wall_t
            prev_wall_t = tick_start
            dt = max(1.0 / 240.0, min(dt, 0.25))

            # One integration tick:
            # 1) consume ROS cmds
            # 2) advance kinematic state + publish ROS
            # 3) write pose into USD stage
            rclpy.spin_once(bridge, timeout_sec=0.0)
            
            phys_x, phys_y, phys_yaw = 0.0, 0.0, 0.0
            if args.enable_dynamic_physics:
                robot_world, q_rot = _get_world_pose(stage, robot_prim_path)
                if robot_world is not None:
                    phys_x, phys_y = robot_world[0], robot_world[1]
                    # Calculate yaw from quaternion (qx, qy, qz, qw)
                    qx, qy, qz, qw = q_rot
                    siny_cosp = 2.0 * (qw * qz + qx * qy)
                    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                    phys_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            bridge.step(dt, args.enable_dynamic_physics, phys_x, phys_y, phys_yaw)
            
            if not args.enable_dynamic_physics:
                _set_robot_pose(stage, pose_prim_path, bridge.pose, args.robot_z)
            sim_app.update()

            if args.debug_pose_check:
                now_wall_t = time.monotonic()
                if now_wall_t - last_pose_diag_wall_t >= 5.0:
                    last_pose_diag_wall_t = now_wall_t
                    pose_world, _ = _get_world_pose(stage, pose_prim_path)
                    robot_world, _ = _get_world_pose(stage, robot_prim_path)
                    print(
                        "[isaac_bringup] pose-check | "
                        f"cmd=({bridge.pose.x:.2f},{bridge.pose.y:.2f},{bridge.pose.yaw:.2f}) "
                        f"carrier_world={pose_world} robot_world={robot_world}",
                        flush=True,
                    )

            elapsed = time.monotonic() - tick_start
            sleep_t = target_period - elapsed
            if sleep_t > 0.0:
                time.sleep(sleep_t)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # pragma: no cover - defensive bringup guard
        print(f"[isaac_bringup] ERROR: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        try:
            if low_level_proc is not None and low_level_proc.poll() is None:
                low_level_proc.terminate()
                low_level_proc.wait(timeout=4.0)
        except Exception:
            if low_level_proc is not None and low_level_proc.poll() is None:
                low_level_proc.kill()

        try:
            bridge.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        timeline.stop()
        sim_app.close()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
