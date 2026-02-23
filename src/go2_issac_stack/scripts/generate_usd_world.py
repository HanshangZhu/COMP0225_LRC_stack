#!/usr/bin/env python3
"""Generate a native Isaac Sim USD world file from a Gazebo SDF world.

Usage:
    python3 generate_usd_world.py \
        --sdf /path/to/t_dual_corridor.world \
        --out /path/to/t_dual_corridor.usd

The output USD contains:
  - UsdGeom.Mesh box geometry for each wall (12 triangles per box)
  - OmniPBR MDL materials bound to all geometry
  - UsdPhysics.CollisionAPI on every wall + ground
  - UsdLux lights
  - Proper mesh-based ground plane

RTX lidar requires mesh geometry in the BVH — procedural UsdGeom.Cube
prims are not tessellated into the RTX ray tracing acceleration structure.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

# Allow running outside of Isaac Sim (pxr ships with USD)
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt


# ---------------------------------------------------------------------------
# Box mesh helper
# ---------------------------------------------------------------------------

def _create_box_mesh(
    stage: Usd.Stage,
    prim_path: str,
    sx: float,
    sy: float,
    sz: float,
    cx: float = 0.0,
    cy: float = 0.0,
    cz: float = 0.0,
    yaw: float = 0.0,
    display_color: Gf.Vec3f | None = None,
) -> UsdGeom.Mesh:
    """Create a UsdGeom.Mesh box with explicit triangulated geometry.

    8 vertices, 12 triangles (2 per face), with normals.
    The box is centred at (cx, cy, cz) with half-extents (sx/2, sy/2, sz/2).
    """
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0

    # 8 corners of the box in local space
    points = Vt.Vec3fArray([
        Gf.Vec3f(-hx, -hy, -hz),  # 0: left-front-bottom
        Gf.Vec3f( hx, -hy, -hz),  # 1: right-front-bottom
        Gf.Vec3f( hx,  hy, -hz),  # 2: right-back-bottom
        Gf.Vec3f(-hx,  hy, -hz),  # 3: left-back-bottom
        Gf.Vec3f(-hx, -hy,  hz),  # 4: left-front-top
        Gf.Vec3f( hx, -hy,  hz),  # 5: right-front-top
        Gf.Vec3f( hx,  hy,  hz),  # 6: right-back-top
        Gf.Vec3f(-hx,  hy,  hz),  # 7: left-back-top
    ])

    # 12 triangles (2 per face), CCW winding
    face_vertex_counts = Vt.IntArray([3] * 12)
    face_vertex_indices = Vt.IntArray([
        # bottom (z = -hz)
        0, 2, 1,  0, 3, 2,
        # top (z = +hz)
        4, 5, 6,  4, 6, 7,
        # front (y = -hy)
        0, 1, 5,  0, 5, 4,
        # back (y = +hy)
        2, 3, 7,  2, 7, 6,
        # left (x = -hx)
        0, 4, 7,  0, 7, 3,
        # right (x = +hx)
        1, 2, 6,  1, 6, 5,
    ])

    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
    mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
    mesh.CreateSubdivisionSchemeAttr("none")

    # Transform: translate + rotate
    xf = UsdGeom.Xformable(mesh.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
    if abs(yaw) > 1e-6:
        xf.AddRotateZOp().Set(float(math.degrees(yaw)))

    if display_color is not None:
        mesh.CreateDisplayColorAttr([display_color])

    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())

    return mesh


# ---------------------------------------------------------------------------
# OmniPBR material helper
# ---------------------------------------------------------------------------

def _create_omnipbr_material(
    stage: Usd.Stage,
    mat_path: str,
    diffuse: Gf.Vec3f,
    roughness: float = 0.8,
    metallic: float = 0.0,
) -> UsdShade.Material:
    """Create an OmniPBR MDL material that RTX lidar can interact with."""
    material = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    shader.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(diffuse)
    shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(metallic)
    material.CreateSurfaceOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    return material


# ---------------------------------------------------------------------------
# SDF parser (reuse the logic from isaac_t_world_bringup.py)
# ---------------------------------------------------------------------------

def _parse_sdf_boxes(sdf_path: Path):
    """Parse wall boxes from a Gazebo SDF world file.

    Returns list of dicts with keys: name, cx, cy, cz, sx, sy, sz, yaw
    """
    import xml.etree.ElementTree as ET

    tree = ET.parse(sdf_path)
    root = tree.getroot()
    boxes = []

    for model in root.iter("model"):
        name = model.get("name", "wall")
        if name in ("ground_plane",) or "spawn" in name.lower():
            continue

        # Get pose
        pose_el = model.find("pose")
        cx = cy = cz = roll = pitch = yaw = 0.0
        if pose_el is not None and pose_el.text:
            parts = pose_el.text.strip().split()
            if len(parts) >= 3:
                cx, cy, cz = float(parts[0]), float(parts[1]), float(parts[2])
            if len(parts) >= 6:
                roll, pitch, yaw = float(parts[3]), float(parts[4]), float(parts[5])

        # Get box size from collision geometry
        box_el = model.find(".//collision/geometry/box/size")
        if box_el is None or not box_el.text:
            continue
        size_parts = box_el.text.strip().split()
        if len(size_parts) < 3:
            continue
        sx, sy, sz = float(size_parts[0]), float(size_parts[1]), float(size_parts[2])

        boxes.append({
            "name": name,
            "cx": cx, "cy": cy, "cz": cz,
            "sx": sx, "sy": sy, "sz": sz,
            "yaw": yaw,
        })

    return boxes


# ---------------------------------------------------------------------------
# Main generation
# ---------------------------------------------------------------------------

def generate_world(sdf_path: Path, out_path: Path):
    boxes = _parse_sdf_boxes(sdf_path)
    if not boxes:
        print(f"ERROR: no wall boxes found in {sdf_path}", file=sys.stderr)
        return 1

    print(f"Parsed {len(boxes)} wall boxes from {sdf_path.name}")

    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # ── Materials ─────────────────────────────────────────────────────
    wall_mat = _create_omnipbr_material(
        stage, "/World/Looks/WallMaterial",
        diffuse=Gf.Vec3f(0.58, 0.37, 0.20), roughness=0.8,
    )
    ground_mat = _create_omnipbr_material(
        stage, "/World/Looks/GroundMaterial",
        diffuse=Gf.Vec3f(0.45, 0.45, 0.45), roughness=1.0,
    )

    # ── Ground plane (mesh) ───────────────────────────────────────────
    ground = _create_box_mesh(
        stage, "/World/Ground",
        sx=80.0, sy=80.0, sz=0.04,
        cx=8.0, cy=0.0, cz=-0.02,
        display_color=Gf.Vec3f(0.45, 0.45, 0.45),
    )
    UsdShade.MaterialBindingAPI(ground.GetPrim()).Bind(ground_mat)

    # ── Walls (mesh) ──────────────────────────────────────────────────
    UsdGeom.Xform.Define(stage, "/World/Walls")
    for i, box in enumerate(boxes):
        prim_path = f"/World/Walls/{box['name']}_{i:03d}"
        # Sanitize prim path (no spaces, no special chars)
        prim_path = prim_path.replace(" ", "_").replace("-", "_")
        wall = _create_box_mesh(
            stage, prim_path,
            sx=box["sx"], sy=box["sy"], sz=box["sz"],
            cx=box["cx"], cy=box["cy"], cz=box["cz"],
            yaw=box["yaw"],
            display_color=Gf.Vec3f(0.58, 0.37, 0.20),
        )
        UsdShade.MaterialBindingAPI(wall.GetPrim()).Bind(wall_mat)

    # ── Lights ────────────────────────────────────────────────────────
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(2500.0)
    sun_xf = UsdGeom.Xformable(sun.GetPrim())
    sun_xf.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 30.0, 0.0))

    dome = UsdLux.DomeLight.Define(stage, "/World/Dome")
    dome.CreateIntensityAttr(200.0)

    # ── Physics scene ─────────────────────────────────────────────────
    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)

    stage.Save()
    print(f"Saved USD world: {out_path} ({len(boxes)} walls)")
    return 0


def main():
    parser = argparse.ArgumentParser(description="Generate USD world from Gazebo SDF")
    parser.add_argument("--sdf", required=True, help="Input Gazebo SDF world file")
    parser.add_argument("--out", required=True, help="Output USD file path")
    args = parser.parse_args()

    sdf_path = Path(args.sdf).expanduser().resolve()
    out_path = Path(args.out).expanduser().resolve()

    if not sdf_path.exists():
        print(f"ERROR: SDF file not found: {sdf_path}", file=sys.stderr)
        return 1

    out_path.parent.mkdir(parents=True, exist_ok=True)
    return generate_world(sdf_path, out_path)


if __name__ == "__main__":
    raise SystemExit(main())
