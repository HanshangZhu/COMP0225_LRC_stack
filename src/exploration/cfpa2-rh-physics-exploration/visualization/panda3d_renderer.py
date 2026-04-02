"""Panda3D-based 3D renderer for VLM exploration.

Layout — single window split vertically:
  LEFT  (60%)  -> 3rd-person camera: elevated, follows & rotates with robot
  RIGHT (40%)  -> 1st-person camera: robot eye view (for VLM + human)

Visual features:
  - Green floor tiles for explored cells, dark for unexplored
  - Robot with headlight and trail markers
  - Glowing red artifact sphere with beacon pillar
  - Frontier pillars (green)
  - On-screen HUD: step, coverage, goal
"""
from __future__ import annotations

import base64
import io
import math
import os
from pathlib import Path
from typing import Any

import numpy as np

# Panda3D — must import ShowBase before most panda3d.core things
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import (
    AmbientLight,
    Camera,
    DirectionalLight,
    FrameBufferProperties,
    GeomNode,
    GraphicsOutput,
    GraphicsPipe,
    LColor,
    LPoint3,
    LVecBase3,
    LVector3,
    NodePath,
    OrthographicLens,
    PerspectiveLens,
    PointLight,
    Spotlight,
    Texture,
    TextNode,
    TransparencyAttrib,
    WindowProperties,
    loadPrcFileData,
)

from .maze_builder import CELL, WALL_H, MazeBuilder, _make_box_geom

# ── Panda3D config (must be set before ShowBase) ──────────────────────────
loadPrcFileData("", "window-title  VLM Robot Exploration — 3D")
loadPrcFileData("", "sync-video    #f")          # uncapped fps
loadPrcFileData("", "show-frame-rate-meter #f")

ROBOT_MODEL_PATH = str(
    Path(__file__).parent.parent / "robot_dog_unitree_go2" / "scene.gltf"
)

# 3rd-person chase camera (fixed, NOT scaled by grid size)
TP_HEIGHT      = 10.0    # height above robot
TP_BACK        = 8.0     # distance behind robot
TP_LOOK_Y      = 1.5     # look-at Y offset (slightly above ground)

# 1st-person camera
FP_EYE_HEIGHT  = 1.2
FP_LOOK_DIST   = 5.0     # how far ahead the camera looks

# Top-down minimap (fraction of window)
MINIMAP_SIZE   = 0.25    # 25% of window width/height
MINIMAP_MARGIN = 0.01
MINIMAP_EXTENT = 15.0    # grid cells visible in each direction from robot

# Trail
TRAIL_INTERVAL = 3     # drop a marker every N steps
TRAIL_COLOR    = LColor(0.3, 0.6, 1.0, 0.7)  # blue translucent


class Exploration3DApp(ShowBase):
    """Panda3D application for 3D exploration visualisation."""

    def __init__(
        self,
        grid: np.ndarray,
        start_pos: tuple[int, int] = (4, 4),
        robot_model_path: str = ROBOT_MODEL_PATH,
        window_w: int = 1400,
        window_h: int = 700,
    ) -> None:
        # ── Window ────────────────────────────────────────────────────────
        loadPrcFileData("", f"win-size {window_w} {window_h}")
        super().__init__()

        self.disableMouse()
        self.set_background_color(0.06, 0.06, 0.09, 1)

        # ── Scene ─────────────────────────────────────────────────────────
        self._maze = MazeBuilder(self.render)
        self._maze.build(grid)
        self._grid = grid
        self._grid_size = max(grid.shape)

        # ── Lighting ──────────────────────────────────────────────────────
        self._setup_lighting()

        # ── Robot ─────────────────────────────────────────────────────────
        self._robot_np = self._load_robot(robot_model_path)
        gx, gy = start_pos
        wx, _, wz = MazeBuilder.grid_to_world(gx, gy)
        self._robot_np.set_pos(wx, 0, wz)
        self._robot_np.set_hpr(0, 0, 0)  # upright, facing default direction
        self._robot_heading: float = 0.0

        # ── Robot headlight ──────────────────────────────────────────────
        self._setup_headlight()

        # ── Artifacts ────────────────────────────────────────────────────
        self._artifact_nodes: list[NodePath] = []
        self._artifact_found_set: set[int] = set()

        # ── Trail ────────────────────────────────────────────────────────
        self._trail_root = self.render.attach_new_node("trail")
        self._trail_step = 0

        # ── Frontier markers ─────────────────────────────────────────────
        self._frontier_root = self.render.attach_new_node("frontiers")

        # ── Split-screen cameras ─────────────────────────────────────────
        self._setup_cameras(window_w, window_h)

        # ── HUD ──────────────────────────────────────────────────────────
        self._setup_hud()

        # ── State ────────────────────────────────────────────────────────
        self._robot_grid_pos: tuple[float, float] = (float(gx), float(gy))
        self._step_idx = 0
        self._coverage = 0.0
        self._goal_text = ""
        self._vlm_reasoning = ""

    # ── Lighting ──────────────────────────────────────────────────────────

    def _setup_lighting(self) -> None:
        # Strong ambient so corridors aren't pitch black
        ambient = AmbientLight("ambient")
        ambient.set_color(LColor(0.50, 0.50, 0.55, 1))
        self.render.set_light(self.render.attach_new_node(ambient))

        # Sun — warm directional from above
        sun = DirectionalLight("sun")
        sun.set_color(LColor(1.0, 0.95, 0.85, 1))
        sun_np = self.render.attach_new_node(sun)
        sun_np.set_hpr(45, -65, 0)
        self.render.set_light(sun_np)

        # Fill — cool from opposite side
        fill = DirectionalLight("fill")
        fill.set_color(LColor(0.4, 0.42, 0.5, 1))
        fill_np = self.render.attach_new_node(fill)
        fill_np.set_hpr(-135, -30, 0)
        self.render.set_light(fill_np)

    def _setup_headlight(self) -> None:
        """Spotlight independent of robot body — oriented to match 1st-person camera.

        Attached to render (not robot) so it doesn't lag behind during turns.
        Position and direction are updated each frame in _update_cameras().
        """
        spot = Spotlight("headlight")
        spot.set_color(LColor(1.5, 1.4, 1.1, 1))
        spot.set_attenuation((0.3, 0.02, 0.003))
        spot.get_lens().set_fov(90)
        spot.get_lens().set_near_far(0.1, 30)
        self._headlight_np = self.render.attach_new_node(spot)
        self.render.set_light(self._headlight_np)

    # ── Robot model ───────────────────────────────────────────────────────

    def _load_robot(self, model_path: str) -> NodePath:
        """Load robot model inside a pivot node.

        The pivot node controls world position and heading (H only).
        The inner model gets a one-time corrective transform so it
        stands upright on the floor regardless of GLTF axis conventions.
        """
        # Pivot — only this node receives set_pos / set_h calls
        pivot = self.render.attach_new_node("robot_pivot")

        try:
            import gltf
            model_root = gltf.load_model(model_path)
            model = NodePath(model_root)
            model.reparent_to(pivot)

            # Flatten all internal GLTF transforms into vertex data so
            # we can reason about bounds in the pivot's coordinate space.
            model.flatten_medium()

            # Measure current bounds
            bounds = model.get_tight_bounds()
            if bounds:
                mn, mx = bounds
                size = max((mx - mn).length(), 1e-6)
                target_size = 0.8
                model.set_scale(target_size / size)

                # Re-measure after scaling
                bounds = model.get_tight_bounds()
                if bounds:
                    mn, mx = bounds

            # Correct orientation — the Sketchfab GLTF has baked axis
            # swaps (Z-up → Y-up). After flatten, test if the model's
            # vertical extent (Y) is smaller than its horizontal extents,
            # which means it's lying on its side.
            if bounds:
                mn, mx = bounds
                extent = mx - mn
                y_ext = extent.get_y()
                x_ext = extent.get_x()
                z_ext = extent.get_z()
                horiz = max(x_ext, z_ext)

                if y_ext < horiz * 0.5:
                    # Model is lying flat — rotate to stand up
                    # Try pitch correction (common for Z-up models)
                    model.set_p(-90)
                    model.flatten_medium()
                    bounds = model.get_tight_bounds()
                    if bounds:
                        mn, mx = bounds

            # Position model so its bottom sits at Y = 0 (floor)
            if bounds:
                mn, mx = bounds
                floor_offset = -mn.get_y()
                model.set_z(model.get_z())  # keep current
                model.set_pos(
                    model.get_x() - (mn.get_x() + mx.get_x()) / 2,  # centre X
                    model.get_y() + floor_offset,                     # feet on floor
                    model.get_z() - (mn.get_z() + mx.get_z()) / 2,  # centre Z
                )

            print(f"[3D] Robot model loaded from {model_path}")

        except Exception as e:
            print(f"[3D] Could not load GLTF robot ({e}) — using fallback geometry")
            self._add_fallback_geometry(pivot)

        # Robot glow light
        pl = PointLight("robot_glow")
        pl.set_color(LColor(0.4, 0.6, 1.0, 1))
        pl.set_attenuation((0.8, 0.1, 0.02))
        pl_np = pivot.attach_new_node(pl)
        pl_np.set_pos(0, 0.4, 0)
        self.render.set_light(pl_np)

        return pivot

    def _add_fallback_geometry(self, pivot: NodePath) -> None:
        """Bright blue box as robot placeholder — centred, bottom at Y=0."""
        gn = GeomNode("robot_body")
        gn.add_geom(_make_box_geom(0, 0, 0.7, 0.5, 0.7,
                                    color=(0.15, 0.55, 0.95, 1.0),
                                    top_color=(0.3, 0.7, 1.0, 1.0)))
        body = pivot.attach_new_node(gn)
        body.set_pos(0, 0.25, 0)  # half-height up so bottom is at Y=0

    # ── Split-screen cameras ──────────────────────────────────────────────

    def _setup_cameras(self, w: int, h: int) -> None:
        split = 0.60

        # ── 3rd-person chase camera (left panel) ──
        self.cam.node().set_lens(self.camLens)
        self.camLens.set_fov(75)
        self.camLens.set_near_far(0.1, 500)
        dr_3p = self.win.make_display_region(0, split, 0, 1)
        dr_3p.set_sort(10)
        dr_3p.set_camera(self.cam)
        dr_3p.set_clear_color_active(True)
        dr_3p.set_clear_color(LColor(0.06, 0.06, 0.09, 1))
        dr_3p.set_clear_depth_active(True)

        # ── 1st-person camera (right panel) ──
        fp_lens = PerspectiveLens()
        fp_lens.set_fov(90)
        fp_lens.set_near_far(0.05, 200)
        fp_cam_node = Camera("fp_cam", fp_lens)
        self._fp_cam = self.render.attach_new_node(fp_cam_node)

        dr_fp = self.win.make_display_region(split, 1, 0, 1)
        dr_fp.set_sort(20)
        dr_fp.set_camera(self._fp_cam)
        dr_fp.set_clear_color_active(True)
        dr_fp.set_clear_color(LColor(0.04, 0.04, 0.06, 1))
        dr_fp.set_clear_depth_active(True)

        # ── Top-down minimap (bottom-left corner overlay) ──
        td_lens = OrthographicLens()
        # Film size is set dynamically in _update_cameras based on MINIMAP_EXTENT
        td_lens.set_film_size(MINIMAP_EXTENT * CELL * 2, MINIMAP_EXTENT * CELL * 2)
        td_lens.set_near_far(-100, 500)
        td_cam_node = Camera("topdown_cam", td_lens)
        self._td_cam = self.render.attach_new_node(td_cam_node)
        # Points straight down (Y-up in Panda3D: pitch = -90)
        self._td_cam.set_hpr(0, -90, 0)
        self._td_lens = td_lens

        mm_left = MINIMAP_MARGIN
        mm_right = MINIMAP_MARGIN + MINIMAP_SIZE
        mm_bottom = MINIMAP_MARGIN
        mm_top = MINIMAP_MARGIN + MINIMAP_SIZE
        dr_td = self.win.make_display_region(mm_left, mm_right, mm_bottom, mm_top)
        dr_td.set_sort(30)
        dr_td.set_camera(self._td_cam)
        dr_td.set_clear_color_active(True)
        dr_td.set_clear_color(LColor(0.02, 0.02, 0.04, 1))
        dr_td.set_clear_depth_active(True)

        # Disable default display region
        self.win.get_display_region(0).set_active(False)
        self._dr_fp = dr_fp

        # ── Offscreen buffer for 1st-person VLM capture ──
        fp_w = int(w * (1 - split))
        fp_h = h
        fbprops = FrameBufferProperties()
        fbprops.set_rgb_color(True)
        fbprops.set_color_bits(24)
        fbprops.set_depth_bits(24)
        self._fp_buffer = self.graphics_engine.make_output(
            self.pipe,
            "fp_offscreen",
            -100,
            fbprops,
            WindowProperties.size(fp_w, fp_h),
            GraphicsPipe.BF_refuse_window | GraphicsPipe.BF_resizeable,
            self.win.get_gsg(),
            self.win,
        )
        self._fp_buffer.add_render_texture(
            Texture("fp_tex"), GraphicsOutput.RTM_copy_ram
        )
        fp_dr = self._fp_buffer.make_display_region()
        fp_dr.set_camera(self._fp_cam)
        fp_dr.set_clear_color_active(True)
        fp_dr.set_clear_color(LColor(0.04, 0.04, 0.06, 1))
        fp_dr.set_clear_depth_active(True)

    # ── HUD ──────────────────────────────────────────────────────────────

    def _setup_hud(self) -> None:
        """On-screen text overlay for the 3rd-person view."""
        self._hud_text = OnscreenText(
            text="",
            pos=(-0.95, 0.90),
            scale=0.04,
            fg=(1, 1, 1, 0.9),
            bg=(0, 0, 0, 0.5),
            align=TextNode.ALeft,
            mayChange=True,
        )
        self._hud_goal = OnscreenText(
            text="",
            pos=(-0.95, -0.92),
            scale=0.035,
            fg=(1, 0.85, 0.3, 0.9),
            bg=(0, 0, 0, 0.5),
            align=TextNode.ALeft,
            mayChange=True,
        )

    def update_hud(
        self,
        step_idx: int,
        coverage: float,
        goal: str = "",
        vlm_reasoning: str = "",
    ) -> None:
        self._step_idx = step_idx
        self._coverage = coverage
        self._goal_text = goal
        self._vlm_reasoning = vlm_reasoning

        info = f"Step: {step_idx}  |  Coverage: {coverage*100:.1f}%"
        self._hud_text.setText(info)

        goal_line = f"Goal: {goal}" if goal else ""
        if vlm_reasoning:
            short = vlm_reasoning[:90] + ("..." if len(vlm_reasoning) > 90 else "")
            goal_line += f"\nVLM: {short}"
        self._hud_goal.setText(goal_line)

    # ── Artifact ──────────────────────────────────────────────────────────

    def place_artifact(self, gx: int, gy: int) -> None:
        """Place a glowing red artifact sphere + beacon at grid position."""
        wx, _, wz = MazeBuilder.grid_to_world(gx, gy)

        # Container node
        art_root = self.render.attach_new_node(f"artifact_{gx}_{gy}")

        # Sphere
        sphere = self.loader.load_model("models/misc/sphere")
        sphere.set_scale(0.4)
        sphere.set_pos(0, 0.4, 0)
        sphere.set_color(1.0, 0.1, 0.05, 1.0)
        sphere.reparent_to(art_root)

        # Beacon pillar (thin tall box above artifact)
        beacon_gn = GeomNode("beacon")
        beacon_gn.add_geom(_make_box_geom(
            0, 0, 0.08, 6.0, 0.08,
            color=(1.0, 0.2, 0.1, 0.6),
            top_color=(1.0, 0.4, 0.2, 0.6),
        ))
        beacon_np = art_root.attach_new_node(beacon_gn)
        beacon_np.set_transparency(TransparencyAttrib.M_alpha)
        beacon_np.set_pos(0, 0, 0)

        # Glow light
        pl = PointLight(f"artifact_glow_{gx}_{gy}")
        pl.set_color(LColor(1.0, 0.15, 0.05, 1))
        pl.set_attenuation((0.3, 0.04, 0.015))
        pl_np = art_root.attach_new_node(pl)
        pl_np.set_pos(0, 1.0, 0)
        self.render.set_light(pl_np)

        art_root.set_pos(wx, 0, wz)
        self._artifact_nodes.append(art_root)

    def mark_artifact_found(self, index: int = 0) -> None:
        """Flash artifact white when discovered."""
        if index in self._artifact_found_set:
            return
        self._artifact_found_set.add(index)
        if index < len(self._artifact_nodes):
            art = self._artifact_nodes[index]
            # Turn sphere green to indicate found
            for child in art.get_children():
                if child.get_name() != "beacon":
                    child.set_color(0.2, 1.0, 0.3, 1.0)

    # ── Trail markers ────────────────────────────────────────────────────

    def _drop_trail_marker(self, wx: float, wz: float) -> None:
        """Small translucent dot on the floor showing robot path."""
        gn = GeomNode("trail_dot")
        gn.add_geom(_make_box_geom(
            0, 0, 0.15, 0.05, 0.15,
            color=(0.3, 0.6, 1.0, 0.5),
        ))
        marker = self._trail_root.attach_new_node(gn)
        marker.set_pos(wx, 0.05, wz)
        marker.set_transparency(TransparencyAttrib.M_alpha)

    # ── Frontier markers ─────────────────────────────────────────────────

    def update_frontiers(self, frontier_reps: list[tuple[int, int]]) -> None:
        """Update green frontier pillars at representative positions."""
        # Remove old
        self._frontier_root.remove_node()
        self._frontier_root = self.render.attach_new_node("frontiers")

        for gx, gy in frontier_reps:
            wx, _, wz = MazeBuilder.grid_to_world(gx, gy)
            gn = GeomNode(f"frontier_{gx}_{gy}")
            gn.add_geom(_make_box_geom(
                0, 0, 0.12, 3.5, 0.12,
                color=(0.1, 0.75, 0.2, 0.6),
                top_color=(0.3, 1.0, 0.4, 0.6),
            ))
            pillar = self._frontier_root.attach_new_node(gn)
            pillar.set_pos(wx, 0, wz)
            pillar.set_transparency(TransparencyAttrib.M_alpha)

    # ── Explored cells ───────────────────────────────────────────────────

    def mark_explored(self, cells: set[tuple[int, int]]) -> None:
        """Mark cells as explored (green floor tiles)."""
        self._maze.mark_explored(cells)

    # ── State updates (called each sim step) ─────────────────────────────

    def update_robot(self, gx: float, gy: float, heading_deg: float) -> None:
        """Move the 3D robot to new grid position and heading.

        Only heading (H) is changed on the pivot — pitch and roll are
        always 0 so the robot stays upright on the floor.
        """
        self._robot_grid_pos = (gx, gy)
        self._robot_heading = heading_deg

        wx, _, wz = MazeBuilder.grid_to_world(gx, gy)
        self._robot_np.set_pos(wx, 0, wz)
        self._robot_np.set_hpr(-heading_deg, 0, 0)  # only heading; P=0, R=0
        self._update_cameras(wx, wz, heading_deg)

        # Drop trail marker periodically
        self._trail_step += 1
        if self._trail_step % TRAIL_INTERVAL == 0:
            self._drop_trail_marker(wx, wz)

    def _update_cameras(self, wx: float, wz: float, heading_deg: float) -> None:
        rad = math.radians(heading_deg)
        fwd_x = math.cos(rad)
        fwd_z = math.sin(rad)

        # ── 3rd-person chase camera (fixed distance, NOT scaled by grid) ──
        tp_x = wx - fwd_x * TP_BACK
        tp_z = wz - fwd_z * TP_BACK
        self.cam.set_pos(tp_x, TP_HEIGHT, tp_z)
        self.cam.look_at(wx, TP_LOOK_Y, wz)

        # ── 1st-person ──
        self._fp_cam.set_pos(wx, FP_EYE_HEIGHT, wz)
        look_x = wx + fwd_x * FP_LOOK_DIST
        look_z = wz + fwd_z * FP_LOOK_DIST
        self._fp_cam.look_at(look_x, FP_EYE_HEIGHT, look_z)

        # ── Headlight — always matches 1st-person camera direction ──
        self._headlight_np.set_pos(wx, FP_EYE_HEIGHT + 0.3, wz)
        self._headlight_np.look_at(look_x, FP_EYE_HEIGHT - 0.2, look_z)

        # ── Top-down minimap — centred on robot ──
        self._td_cam.set_pos(wx, 100, wz)

    # ── VLM frame capture ─────────────────────────────────────────────────

    def capture_first_person_b64(self) -> str:
        """Render the 1st-person view to a base64 PNG string for the VLM."""
        from PIL import Image

        self.graphics_engine.render_frame()

        tex = self._fp_buffer.get_texture()
        if not tex.has_ram_image():
            self._fp_buffer.trigger_copy()
            self.graphics_engine.render_frame()

        ram = tex.get_ram_image_as("RGBA")
        w = tex.get_x_size()
        h = tex.get_y_size()
        arr = np.frombuffer(bytes(ram), dtype=np.uint8).reshape(h, w, 4)
        arr = np.flipud(arr)[:, :, :3]

        buf = io.BytesIO()
        Image.fromarray(arr).save(buf, format="PNG")
        buf.seek(0)
        return base64.b64encode(buf.read()).decode("ascii")

    # ── Step (advance one frame without blocking) ─────────────────────────

    def step(self) -> None:
        """Advance Panda3D one frame. Call once per simulation step."""
        self.task_mgr.step()
