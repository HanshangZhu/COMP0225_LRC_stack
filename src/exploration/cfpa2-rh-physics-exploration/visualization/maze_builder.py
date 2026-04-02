"""Converts a 2D occupancy grid into a 3D Panda3D scene.

Wall geometry:
  - Each OCCUPIED cell -> a box of size CELL x WALL_H x CELL
  - Floor tiles per-cell with color coding (explored / unexplored)
  - Corridor floors are lighter; walls are darker stone

Coordinate mapping  (2D grid -> 3D world):
  grid (x, y)  ->  world (x * CELL,  0,  y * CELL)
  y in grid increases downward; in Panda3D y is up.
"""
from __future__ import annotations

import numpy as np
from panda3d.core import (
    GeomNode,
    GeomTriangles,
    GeomVertexData,
    GeomVertexFormat,
    GeomVertexWriter,
    Geom,
    NodePath,
    Vec4,
    LColor,
    TransparencyAttrib,
)

CELL: float = 1.0        # world units per grid cell
WALL_H: float = 2.5      # wall height in world units

OCCUPIED = 1
FREE = 0


# ---------------------------------------------------------------------------
# Colour palette
# ---------------------------------------------------------------------------

WALL_COLOR       = (0.55, 0.50, 0.45, 1.0)   # warm stone
WALL_TOP_COLOR   = (0.62, 0.57, 0.52, 1.0)   # lighter top
FLOOR_EXPLORED   = (0.45, 0.72, 0.40, 1.0)   # vibrant green — explored
FLOOR_UNEXPLORED = (0.38, 0.36, 0.34, 1.0)   # grey-brown — visible but muted
FLOOR_WALL       = (0.38, 0.35, 0.32, 1.0)   # under-wall (not visible)


def _make_box_geom(
    cx: float, cy: float, w: float, h: float, d: float,
    color: tuple = WALL_COLOR,
    top_color: tuple | None = None,
) -> Geom:
    """Return a Geom for a box centred at (cx, 0, cy) with dims w x h x d."""
    if top_color is None:
        top_color = color

    vformat = GeomVertexFormat.get_v3n3c4()
    vdata = GeomVertexData("box", vformat, Geom.UH_static)
    vdata.set_num_rows(24)

    vertex = GeomVertexWriter(vdata, "vertex")
    normal = GeomVertexWriter(vdata, "normal")
    col_w  = GeomVertexWriter(vdata, "color")

    hw, hh, hd = w / 2, h / 2, d / 2
    x, y, z = cx, hh, cy  # y is up in Panda3D

    faces = [
        # (normal,  4 corners, face_color)
        ((0,  0,  1), [(-hw,-hh, hd),( hw,-hh, hd),( hw, hh, hd),(-hw, hh, hd)], color),
        ((0,  0, -1), [( hw,-hh,-hd),(-hw,-hh,-hd),(-hw, hh,-hd),( hw, hh,-hd)], color),
        ((-1, 0,  0), [(-hw,-hh,-hd),(-hw,-hh, hd),(-hw, hh, hd),(-hw, hh,-hd)], color),
        (( 1, 0,  0), [( hw,-hh, hd),( hw,-hh,-hd),( hw, hh,-hd),( hw, hh, hd)], color),
        ((0,  1,  0), [(-hw, hh, hd),( hw, hh, hd),( hw, hh,-hd),(-hw, hh,-hd)], top_color),
        ((0, -1,  0), [(-hw,-hh,-hd),( hw,-hh,-hd),( hw,-hh, hd),(-hw,-hh, hd)], color),
    ]

    for (nx, ny, nz), corners, fc in faces:
        for vx, vy, vz in corners:
            vertex.add_data3(x + vx, y + vy, z + vz)
            normal.add_data3(nx, ny, nz)
            col_w.add_data4(*fc)

    tris = GeomTriangles(Geom.UH_static)
    for i in range(6):
        base = i * 4
        tris.add_vertices(base, base+1, base+2)
        tris.add_vertices(base, base+2, base+3)

    geom = Geom(vdata)
    geom.add_primitive(tris)
    return geom


def _make_floor_tile(cx: float, cz: float, color: tuple) -> Geom:
    """Single floor tile quad at (cx, 0, cz) with CELL size."""
    vformat = GeomVertexFormat.get_v3n3c4()
    vdata = GeomVertexData("tile", vformat, Geom.UH_static)
    vdata.set_num_rows(4)

    vertex = GeomVertexWriter(vdata, "vertex")
    normal = GeomVertexWriter(vdata, "normal")
    col    = GeomVertexWriter(vdata, "color")

    hc = CELL / 2
    for dx, dz in [(-hc, -hc), (hc, -hc), (hc, hc), (-hc, hc)]:
        vertex.add_data3(cx + dx, 0.001, cz + dz)  # tiny Y offset to avoid z-fight
        normal.add_data3(0, 1, 0)
        col.add_data4(*color)

    tris = GeomTriangles(Geom.UH_static)
    tris.add_vertices(0, 1, 2)
    tris.add_vertices(0, 2, 3)

    geom = Geom(vdata)
    geom.add_primitive(tris)
    return geom


def _make_floor_geom(grid_w: int, grid_h: int, color: tuple) -> Geom:
    """Flat floor quad covering the full grid (base layer)."""
    vformat = GeomVertexFormat.get_v3n3c4()
    vdata = GeomVertexData("floor", vformat, Geom.UH_static)
    vdata.set_num_rows(4)

    vertex = GeomVertexWriter(vdata, "vertex")
    normal = GeomVertexWriter(vdata, "normal")
    col    = GeomVertexWriter(vdata, "color")

    w = grid_w * CELL
    d = grid_h * CELL
    for vx, vz in [(0, 0), (w, 0), (w, d), (0, d)]:
        vertex.add_data3(vx, 0.0, vz)
        normal.add_data3(0, 1, 0)
        col.add_data4(*color)

    tris = GeomTriangles(Geom.UH_static)
    tris.add_vertices(0, 1, 2)
    tris.add_vertices(0, 2, 3)

    geom = Geom(vdata)
    geom.add_primitive(tris)
    return geom


class MazeBuilder:
    """Builds and maintains the 3D maze geometry in a Panda3D scene."""

    def __init__(self, render: NodePath) -> None:
        self._render = render
        self._wall_root: NodePath = render.attach_new_node("walls")
        self._floor_node: NodePath | None = None
        self._explored_root: NodePath = render.attach_new_node("explored_tiles")
        self._grid: np.ndarray | None = None
        self._explored_set: set[tuple[int, int]] = set()

    # ------------------------------------------------------------------

    def build(self, grid: np.ndarray) -> None:
        """Build the full maze from a 2D occupancy grid (call once at start)."""
        self._grid = grid.copy()
        h, w = grid.shape

        # Base floor — dark (unexplored) colour
        gn = GeomNode("floor_base")
        gn.add_geom(_make_floor_geom(w, h, FLOOR_UNEXPLORED))
        self._floor_node = self._render.attach_new_node(gn)

        # Walls — merge into one GeomNode for performance
        wall_gn = GeomNode("walls_mesh")
        for row in range(h):
            for col in range(w):
                if grid[row, col] == OCCUPIED:
                    geom = _make_box_geom(
                        cx=col * CELL + CELL / 2,
                        cy=row * CELL + CELL / 2,
                        w=CELL, h=WALL_H, d=CELL,
                        color=WALL_COLOR,
                        top_color=WALL_TOP_COLOR,
                    )
                    wall_gn.add_geom(geom)
        self._wall_root.attach_new_node(wall_gn)

    def mark_explored(self, cells: set[tuple[int, int]]) -> None:
        """Add green explored tiles for newly-revealed free cells."""
        new_cells = cells - self._explored_set
        if not new_cells:
            return
        self._explored_set.update(new_cells)

        # Batch new tiles into one GeomNode
        gn = GeomNode(f"explored_batch_{len(self._explored_set)}")
        for (gx, gy) in new_cells:
            if self._grid is not None:
                if gy < 0 or gy >= self._grid.shape[0] or gx < 0 or gx >= self._grid.shape[1]:
                    continue
                if self._grid[gy, gx] == OCCUPIED:
                    continue
            wx = gx * CELL + CELL / 2
            wz = gy * CELL + CELL / 2
            gn.add_geom(_make_floor_tile(wx, wz, FLOOR_EXPLORED))
        self._explored_root.attach_new_node(gn)

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    @staticmethod
    def grid_to_world(gx: float, gy: float) -> tuple[float, float, float]:
        """Convert grid (x, y) cell coords to world (x, y, z) for Panda3D."""
        return (gx * CELL + CELL / 2, 0.0, gy * CELL + CELL / 2)

    @staticmethod
    def world_to_grid(wx: float, wz: float) -> tuple[float, float]:
        return (wx / CELL - 0.5, wz / CELL - 0.5)
