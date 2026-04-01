#!/usr/bin/env python3
"""Generate LRC 2025 Atlanta Confined Configuration world files.

Produces Gazebo SDF (.world) and MuJoCo MJCF (.xml) for the 4-lane
continuous L-shaped LRC 2025 confined configuration course.

Lanes: A (Ramps) -> C (Pallets/Pipes) -> D (K-Rails) -> F (Stairs/Debris)

Usage:
    python3 scripts/generate_lrc_2025_world.py          # flat
    python3 scripts/generate_lrc_2025_world.py --slope   # 15deg cross-slope
"""

import argparse
import math
import os
from dataclasses import dataclass
from typing import List, Tuple, Union

# ── Constants ──────────────────────────────────────────────────────
WALL_H = 1.2        # wall height (m)
WALL_T = 0.15       # wall thickness (m)
DOOR_W = 1.0        # doorway opening width (m)
LANE_SZ = 6.0       # lane side length (m)
HALF = LANE_SZ / 2  # 3.0

SLOPE_DEG = 15.0
SLOPE_RAD = math.radians(SLOPE_DEG)

# Zig-zag pass layout (offsets from lane center, in stacking direction)
PASS_OFF = [-1.35, 0.0, 1.35]
DIV_OFF = [-0.675, 0.675]
DIV_THICK = 0.15
GAP_LEN = 1.2

# Colors (RGBA)
C_WALL   = (0.60, 0.55, 0.45, 1.0)
C_RAMP   = (0.55, 0.45, 0.35, 1.0)
C_PALLET = (0.65, 0.50, 0.30, 1.0)
C_PIPE   = (0.40, 0.40, 0.40, 1.0)
C_CONCR  = (0.55, 0.55, 0.55, 1.0)
C_KRAIL  = (0.50, 0.50, 0.50, 1.0)
C_FLOOR  = (0.45, 0.40, 0.35, 1.0)
C_QR     = (0.0, 0.8, 0.0, 1.0)
C_HAZ    = (1.0, 0.5, 0.0, 1.0)
C_ESTOP  = (0.9, 0.9, 0.0, 1.0)
C_EBTN   = (1.0, 0.0, 0.0, 1.0)


# ── Data classes ───────────────────────────────────────────────────
@dataclass
class Box:
    name: str
    pos: Tuple[float, float, float]
    size: Tuple[float, float, float]   # FULL size (sx, sy, sz)
    euler: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # RPY radians
    color: Tuple[float, float, float, float] = C_WALL
    mu: float = 1.0

@dataclass
class Cyl:
    name: str
    pos: Tuple[float, float, float]
    radius: float
    length: float
    euler: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    color: Tuple[float, float, float, float] = C_PIPE
    mu: float = 1.0

Geom = Union[Box, Cyl]


# ── Lane configuration ────────────────────────────────────────────
# orient: 'EW' = passes run E-W, stack S->N  (Lane A vertical leg)
#         'NS' = passes run N-S, stack W->E  (Lanes C,D,F horizontal leg)
LANES = {
    'A': dict(cx=0,  cy=-7, orient='EW', terrain='ramps'),
    'C': dict(cx=0,  cy=0,  orient='NS', terrain='pallets'),
    'D': dict(cx=7,  cy=0,  orient='NS', terrain='krails'),
    'F': dict(cx=14, cy=0,  orient='NS', terrain='stairs'),
}

# Doorway positions (absolute coordinate along wall extension axis)
# south/north walls extend in X -> value is X coordinate of door center
# east/west walls extend in Y  -> value is Y coordinate of door center
# None = solid wall (no doorway)
DOORS = {
    'A': dict(south=0.0,  north=-1.5, east=None, west=None),
    'C': dict(south=-1.5, north=None, east=1.5,  west=None),
    'D': dict(south=None, north=None, east=-1.5, west=1.5),
    'F': dict(south=None, north=None, east=1.5,  west=-1.5),
}

# Divider gap side per lane:  ('min' or 'max') in the divider extension direction
# EW lanes: dividers extend in X.  'min'=west, 'max'=east
# NS lanes: dividers extend in Y.  'min'=south, 'max'=north
#
# Lane A: robot goes W,E,W -> exits west.  div0 gap west, div1 gap east
# Lane C: robot goes N,S,N -> exits at north-east.  div0 gap north, div1 gap south
# Lane D: robot goes S,N,S -> exits at south-east.  div0 gap south, div1 gap north
# Lane F: robot goes N,S,N -> exits at north-east.  div0 gap north, div1 gap south
DIV_GAPS = {
    'A': ('min', 'max'),
    'C': ('max', 'min'),
    'D': ('min', 'max'),
    'F': ('max', 'min'),
}

# Corridor connections (side walls bridging doorways between lanes)
# orient: 'V' = vertical (Y-direction), 'H' = horizontal (X-direction)
CORRIDORS = [
    dict(cx=-1.5, cy=-3.5, orient='V', w=DOOR_W, l=1.0),   # A->C
    dict(cx=3.5,  cy=1.5,  orient='H', w=DOOR_W, l=1.0),   # C->D
    dict(cx=10.5, cy=-1.5, orient='H', w=DOOR_W, l=1.0),   # D->F
]


# ── Builder helpers ────────────────────────────────────────────────

def _wall_segs(name, axis, perp, a0, a1, door_pos, wh):
    """Return Box list for a wall along `axis` at `perp`, optionally split by door."""
    segs = []
    if door_pos is not None:
        d0 = door_pos - DOOR_W / 2
        d1 = door_pos + DOOR_W / 2
        parts = []
        if d0 - a0 > 0.01:
            parts.append((f"{name}_a", a0, d0))
        if a1 - d1 > 0.01:
            parts.append((f"{name}_b", d1, a1))
    else:
        parts = [(name, a0, a1)]

    for sn, s0, s1 in parts:
        ln = s1 - s0
        mid = (s0 + s1) / 2
        if axis == 'X':
            segs.append(Box(sn, pos=(mid, perp, wh / 2),
                            size=(ln, WALL_T, wh)))
        else:
            segs.append(Box(sn, pos=(perp, mid, wh / 2),
                            size=(WALL_T, ln, wh)))
    return segs


def _slope_z(slope, orient, cx, cy, px, py):
    """Extra Z for objects sitting on a sloped floor plate."""
    if not slope:
        return 0.0
    pt = 0.10  # plate thickness
    if orient == 'EW':
        dy = py - (cy - HALF)
        return dy * math.sin(SLOPE_RAD) + pt * math.cos(SLOPE_RAD)
    else:
        dx = px - (cx - HALF)
        return dx * math.sin(SLOPE_RAD) + pt * math.cos(SLOPE_RAD)


# ── Main builder ──────────────────────────────────────────────────

def build_all(slope: bool) -> List[Geom]:
    gs: List[Geom] = []
    wh = 2.5 if slope else WALL_H

    for lid, L in LANES.items():
        cx, cy, orient = L['cx'], L['cy'], L['orient']
        doors = DOORS[lid]
        x0, x1 = cx - HALF, cx + HALF
        y0, y1 = cy - HALF, cy + HALF

        # ── outer walls ──
        for prefix, axis, perp, a_start, a_end, dkey in [
            (f"L{lid}_S", 'X', y0, x0, x1, 'south'),
            (f"L{lid}_N", 'X', y1, x0, x1, 'north'),
            (f"L{lid}_W", 'Y', x0, y0, y1, 'west'),
            (f"L{lid}_E", 'Y', x1, y0, y1, 'east'),
        ]:
            gs.extend(_wall_segs(prefix, axis, perp, a_start, a_end,
                                 doors[dkey], wh))

        # ── internal dividers ──
        for di, (d_off, gap_side) in enumerate(
                zip(DIV_OFF, DIV_GAPS[lid])):
            if orient == 'EW':
                d_perp = cy + d_off
                e_min, e_max = x0, x1
                if gap_side == 'min':
                    w_s, w_e = e_min + GAP_LEN, e_max
                else:
                    w_s, w_e = e_min, e_max - GAP_LEN
                w_len = w_e - w_s
                w_mid = (w_s + w_e) / 2
                gs.append(Box(f"L{lid}_d{di}",
                              pos=(w_mid, d_perp, wh / 2),
                              size=(w_len, DIV_THICK, wh)))
            else:  # NS
                d_perp = cx + d_off
                e_min, e_max = y0, y1
                if gap_side == 'min':
                    w_s, w_e = e_min + GAP_LEN, e_max
                else:
                    w_s, w_e = e_min, e_max - GAP_LEN
                w_len = w_e - w_s
                w_mid = (w_s + w_e) / 2
                gs.append(Box(f"L{lid}_d{di}",
                              pos=(d_perp, w_mid, wh / 2),
                              size=(DIV_THICK, w_len, wh)))

        # ── slope floor plate ──
        if slope:
            pt = 0.10
            z_c = HALF * math.sin(SLOPE_RAD) + (pt / 2) * math.cos(SLOPE_RAD)
            if orient == 'EW':
                gs.append(Box(f"L{lid}_floor", pos=(cx, cy, z_c),
                              size=(LANE_SZ, LANE_SZ, pt),
                              euler=(SLOPE_RAD, 0, 0), color=C_FLOOR))
            else:
                gs.append(Box(f"L{lid}_floor", pos=(cx, cy, z_c),
                              size=(LANE_SZ, LANE_SZ, pt),
                              euler=(0, SLOPE_RAD, 0), color=C_FLOOR))

        # ── terrain obstacles ──
        _place_terrain(gs, lid, cx, cy, orient, L['terrain'], slope)

        # ── mission task placeholders ──
        _place_tasks(gs, lid, cx, cy)

    # ── corridors ──
    for ci, c in enumerate(CORRIDORS):
        hw = c['w'] / 2
        if c['orient'] == 'V':
            gs.append(Box(f"cor{ci}_W",
                          pos=(c['cx'] - hw, c['cy'], wh / 2),
                          size=(WALL_T, c['l'], wh)))
            gs.append(Box(f"cor{ci}_E",
                          pos=(c['cx'] + hw, c['cy'], wh / 2),
                          size=(WALL_T, c['l'], wh)))
        else:
            gs.append(Box(f"cor{ci}_S",
                          pos=(c['cx'], c['cy'] - hw, wh / 2),
                          size=(c['l'], WALL_T, wh)))
            gs.append(Box(f"cor{ci}_N",
                          pos=(c['cx'], c['cy'] + hw, wh / 2),
                          size=(c['l'], WALL_T, wh)))

    # ── entry approach corridor (robot spawn at 0, -11.5) ──
    gs.append(Box("entry_W", pos=(-0.5, -10.75, wh / 2),
                  size=(WALL_T, 1.5, wh)))
    gs.append(Box("entry_E", pos=(0.5, -10.75, wh / 2),
                  size=(WALL_T, 1.5, wh)))

    return gs


# ── Terrain placement ─────────────────────────────────────────────

def _place_terrain(gs, lid, cx, cy, orient, terrain, slope):
    if terrain == 'ramps':
        _terrain_ramps(gs, lid, cx, cy, orient, slope)
    elif terrain == 'pallets':
        _terrain_pallets(gs, lid, cx, cy, orient, slope)
    elif terrain == 'krails':
        _terrain_krails(gs, lid, cx, cy, orient, slope)
    elif terrain == 'stairs':
        _terrain_stairs(gs, lid, cx, cy, orient, slope)


def _terrain_ramps(gs, lid, cx, cy, orient, slope):
    """Lane A: 60cm pitch/roll ramp tiles in checkerboard pattern."""
    ts = 0.60    # tile side
    th = 0.16    # tile height
    a15 = math.radians(15)

    for pi, p_off in enumerate(PASS_OFF):
        if orient == 'EW':
            py = cy + p_off
            for ti in range(8):
                tx = cx - 2.625 + ti * 0.75
                tz = th / 2 + _slope_z(slope, orient, cx, cy, tx, py)
                pat = (pi + ti) % 4
                if pat == 0:   e = (a15,  0,    0)
                elif pat == 1: e = (0,    a15,  0)
                elif pat == 2: e = (-a15, 0,    0)
                else:          e = (0,    -a15, 0)
                gs.append(Box(f"ramp_{pi}_{ti}", pos=(tx, py, tz),
                              size=(ts, ts, th), euler=e, color=C_RAMP))


def _terrain_pallets(gs, lid, cx, cy, orient, slope):
    """Lane C: pallet slat groups + pipe proxies."""
    for pi, p_off in enumerate(PASS_OFF):
        if orient == 'NS':
            px = cx + p_off
            for gi, y_off in enumerate([-1.5, 0.0, 1.5]):
                py = cy + y_off
                double = (gi == 1)          # middle group is double-stack
                sh = 0.40 if double else 0.20
                sz = _slope_z(slope, orient, cx, cy, px, py)

                # 5 slats per pallet group (span ~0.82m along Y)
                for si in range(5):
                    sy = py - 0.36 + si * 0.18
                    gs.append(Box(f"plt_{pi}_{gi}_{si}",
                                  pos=(px, sy, sh / 2 + sz),
                                  size=(1.0, 0.10, sh), color=C_PALLET))

                # Pipe on leading edge (runs across pass width, along X)
                gs.append(Cyl(f"pipe_{pi}_{gi}",
                              pos=(px, py - 0.50, sh + 0.05 + sz),
                              radius=0.05, length=1.0,
                              euler=(0, math.pi / 2, 0), color=C_PIPE))


def _terrain_krails(gs, lid, cx, cy, orient, slope):
    """Lane D: diagonal K-rails at 45deg yaw, incremental heights."""
    heights = [0.05, 0.10, 0.15, 0.20]
    for pi, p_off in enumerate(PASS_OFF):
        if orient == 'NS':
            px = cx + p_off
            for ri, y_off in enumerate([-1.5, 0.0, 1.5]):
                ry = cy + y_off
                h = heights[(pi * 3 + ri) % 4]
                sz = _slope_z(slope, orient, cx, cy, px, ry)
                gs.append(Box(f"kr_{pi}_{ri}",
                              pos=(px, ry, h / 2 + sz),
                              size=(1.7, 0.15, h),
                              euler=(0, 0, math.pi / 4), color=C_KRAIL))


def _terrain_stairs(gs, lid, cx, cy, orient, slope):
    """Lane F: stair steps + debris rails."""
    rise = 0.15
    run = 0.30
    ns = 5

    for pi, p_off in enumerate(PASS_OFF):
        if orient == 'NS':
            px = cx + p_off
            y0 = cy - (ns * run) / 2
            sz = _slope_z(slope, orient, cx, cy, px, cy)

            for si in range(ns):
                sh = (si + 1) * rise
                sy = y0 + si * run + run / 2
                gs.append(Box(f"step_{pi}_{si}",
                              pos=(px, sy, sh / 2 + sz),
                              size=(1.0, run, sh), color=C_CONCR))

            # Debris rail across step 2
            dz = 2 * rise + 0.02 + sz
            gs.append(Cyl(f"debris_{pi}",
                          pos=(px, y0 + 2 * run, dz),
                          radius=0.02, length=1.0,
                          euler=(0.35, math.pi / 2, 0), color=C_PIPE))


# ── Mission task placeholders ─────────────────────────────────────

def _place_tasks(gs, lid, cx, cy):
    """Place colored QR / hazmat / e-stop proxy cubes."""
    if lid == 'A':
        gs.append(Box(f"qr_{lid}",
                      pos=(cx - HALF + 0.15, cy, 0.60),
                      size=(0.02, 0.15, 0.15), color=C_QR))
        gs.append(Box(f"haz_{lid}",
                      pos=(cx + 1.5, cy + PASS_OFF[0], 0.06),
                      size=(0.12, 0.12, 0.12), color=C_HAZ))
        gs.append(Box(f"es_{lid}",
                      pos=(cx + 2.0, cy - HALF + 0.5, 0.04),
                      size=(0.08, 0.08, 0.08), color=C_ESTOP))
        gs.append(Cyl(f"esb_{lid}",
                      pos=(cx + 2.0, cy - HALF + 0.5, 0.10),
                      radius=0.03, length=0.04, color=C_EBTN))
    elif lid == 'C':
        gs.append(Box(f"qr_{lid}",
                      pos=(cx, cy + HALF - 0.15, 0.60),
                      size=(0.15, 0.02, 0.15), color=C_QR))
        gs.append(Box(f"haz_{lid}",
                      pos=(cx + PASS_OFF[0], cy + 1.5, 0.06),
                      size=(0.12, 0.12, 0.12), color=C_HAZ))
    elif lid == 'D':
        gs.append(Box(f"qr_{lid}",
                      pos=(cx, cy - HALF + 0.15, 0.60),
                      size=(0.15, 0.02, 0.15), color=C_QR))
        gs.append(Box(f"haz_{lid}",
                      pos=(cx + PASS_OFF[2], cy - 1.0, 0.06),
                      size=(0.12, 0.12, 0.12), color=C_HAZ))
        gs.append(Box(f"es_{lid}",
                      pos=(cx + PASS_OFF[1], cy + 1.0, 0.04),
                      size=(0.08, 0.08, 0.08), color=C_ESTOP))
        gs.append(Cyl(f"esb_{lid}",
                      pos=(cx + PASS_OFF[1], cy + 1.0, 0.10),
                      radius=0.03, length=0.04, color=C_EBTN))
    elif lid == 'F':
        gs.append(Box(f"qr_{lid}",
                      pos=(cx + HALF - 0.15, cy, 0.60),
                      size=(0.02, 0.15, 0.15), color=C_QR))
        gs.append(Box(f"haz_{lid}",
                      pos=(cx + PASS_OFF[0], cy - 1.5, 0.06),
                      size=(0.12, 0.12, 0.12), color=C_HAZ))


# ── SDF writer ────────────────────────────────────────────────────

def _f(v):
    """Format float for XML output."""
    return f"{v:.4f}" if isinstance(v, float) else str(v)


def write_sdf(geoms: List[Geom], path: str):
    L: List[str] = []
    a = L.append

    a('<?xml version="1.0" ?>')
    a('<sdf version="1.6">')
    a('<world name="lrc_2025_atlanta">')
    a('')
    a('  <physics type="ode">')
    a('    <max_step_size>0.002</max_step_size>')
    a('    <real_time_factor>1.0</real_time_factor>')
    a('    <real_time_update_rate>500</real_time_update_rate>')
    a('    <ode>')
    a('      <solver>')
    a('        <type>quick</type>')
    a('        <iters>25</iters>')
    a('      </solver>')
    a('    </ode>')
    a('  </physics>')
    a('')
    a('  <scene>')
    a('    <ambient>0.6 0.6 0.6 1</ambient>')
    a('    <background>0.7 0.8 0.9 1</background>')
    a('  </scene>')
    a('')
    a('  <light name="sun" type="directional">')
    a('    <cast_shadows>true</cast_shadows>')
    a('    <pose>0 0 10 0 0 0</pose>')
    a('    <diffuse>0.8 0.8 0.8 1</diffuse>')
    a('    <specular>0.2 0.2 0.2 1</specular>')
    a('    <direction>-0.5 0.1 -0.9</direction>')
    a('  </light>')
    a('')
    a('  <!-- Ground plane -->')
    a('  <model name="ground_plane">')
    a('    <static>true</static>')
    a('    <link name="link">')
    a('      <collision name="collision">')
    a('        <geometry>')
    a('          <plane><normal>0 0 1</normal><size>50 50</size></plane>')
    a('        </geometry>')
    a('      </collision>')
    a('      <visual name="visual">')
    a('        <geometry>')
    a('          <plane><normal>0 0 1</normal><size>50 50</size></plane>')
    a('        </geometry>')
    a('        <material>')
    a('          <ambient>0.8 0.8 0.8 1</ambient>')
    a('          <diffuse>0.8 0.8 0.8 1</diffuse>')
    a('        </material>')
    a('      </visual>')
    a('    </link>')
    a('  </model>')
    a('')

    for g in geoms:
        if isinstance(g, Box):
            _sdf_box(L, g)
        else:
            _sdf_cyl(L, g)

    a('')
    a('</world>')
    a('</sdf>')

    with open(path, 'w') as fh:
        fh.write('\n'.join(L) + '\n')
    print(f"  SDF:  {path}")


def _sdf_box(L, b: Box):
    x, y, z = b.pos
    sx, sy, sz = b.size
    rx, ry, rz = b.euler
    r, g, bl, al = b.color
    L.append(f'  <model name="{b.name}">')
    L.append(f'    <static>true</static>')
    L.append(f'    <pose>{_f(x)} {_f(y)} {_f(z)} {_f(rx)} {_f(ry)} {_f(rz)}</pose>')
    L.append(f'    <link name="link">')
    L.append(f'      <collision name="col">')
    L.append(f'        <geometry><box><size>{_f(sx)} {_f(sy)} {_f(sz)}</size></box></geometry>')
    if b.mu != 1.0:
        L.append(f'        <surface><friction><ode>')
        L.append(f'          <mu>{b.mu}</mu><mu2>{b.mu}</mu2>')
        L.append(f'        </ode></friction></surface>')
    L.append(f'      </collision>')
    L.append(f'      <visual name="vis">')
    L.append(f'        <geometry><box><size>{_f(sx)} {_f(sy)} {_f(sz)}</size></box></geometry>')
    L.append(f'        <material>')
    L.append(f'          <ambient>{r} {g} {bl} {al}</ambient>')
    L.append(f'          <diffuse>{r} {g} {bl} {al}</diffuse>')
    L.append(f'        </material>')
    L.append(f'      </visual>')
    L.append(f'    </link>')
    L.append(f'  </model>')


def _sdf_cyl(L, c: Cyl):
    x, y, z = c.pos
    rx, ry, rz = c.euler
    r, g, bl, al = c.color
    L.append(f'  <model name="{c.name}">')
    L.append(f'    <static>true</static>')
    L.append(f'    <pose>{_f(x)} {_f(y)} {_f(z)} {_f(rx)} {_f(ry)} {_f(rz)}</pose>')
    L.append(f'    <link name="link">')
    L.append(f'      <collision name="col">')
    L.append(f'        <geometry><cylinder><radius>{c.radius}</radius><length>{c.length}</length></cylinder></geometry>')
    L.append(f'      </collision>')
    L.append(f'      <visual name="vis">')
    L.append(f'        <geometry><cylinder><radius>{c.radius}</radius><length>{c.length}</length></cylinder></geometry>')
    L.append(f'        <material>')
    L.append(f'          <ambient>{r} {g} {bl} {al}</ambient>')
    L.append(f'          <diffuse>{r} {g} {bl} {al}</diffuse>')
    L.append(f'        </material>')
    L.append(f'      </visual>')
    L.append(f'    </link>')
    L.append(f'  </model>')


# ── MJCF writer ───────────────────────────────────────────────────

def write_mjcf(geoms: List[Geom], path: str):
    L: List[str] = []
    a = L.append

    a('<mujoco model="lrc_2025_atlanta">')
    a('  <compiler angle="radian" coordinate="local"/>')
    a('  <option timestep="0.002" gravity="0 0 -9.81"/>')
    a('')
    a('  <asset>')
    a('    <texture name="grid" type="2d" builtin="checker"')
    a('             rgb1="0.8 0.8 0.8" rgb2="0.7 0.7 0.7"')
    a('             width="512" height="512"/>')
    a('    <material name="ground_mat" texture="grid" texrepeat="10 10"/>')
    a('  </asset>')
    a('')
    a('  <worldbody>')
    a('    <light pos="0 0 10" dir="-0.5 0.1 -0.9" diffuse="0.8 0.8 0.8"/>')
    a('    <geom name="ground" type="plane" size="25 25 0.1"')
    a('          material="ground_mat"/>')
    a('')

    for g in geoms:
        if isinstance(g, Box):
            _mjcf_box(L, g)
        else:
            _mjcf_cyl(L, g)

    a('  </worldbody>')
    a('</mujoco>')

    with open(path, 'w') as fh:
        fh.write('\n'.join(L) + '\n')
    print(f"  MJCF: {path}")


def _mjcf_box(L, b: Box):
    x, y, z = b.pos
    hx, hy, hz = b.size[0] / 2, b.size[1] / 2, b.size[2] / 2
    r, g, bl, al = b.color
    parts = [f'name="{b.name}"', 'type="box"',
             f'pos="{_f(x)} {_f(y)} {_f(z)}"',
             f'size="{_f(hx)} {_f(hy)} {_f(hz)}"',
             f'rgba="{r} {g} {bl} {al}"']
    if any(v != 0 for v in b.euler):
        rx, ry, rz = b.euler
        parts.append(f'euler="{_f(rx)} {_f(ry)} {_f(rz)}"')
    if b.mu != 1.0:
        parts.append(f'friction="{b.mu} {b.mu} 0.001"')
    L.append(f'    <geom {" ".join(parts)}/>')


def _mjcf_cyl(L, c: Cyl):
    x, y, z = c.pos
    hl = c.length / 2
    r, g, bl, al = c.color
    parts = [f'name="{c.name}"', 'type="cylinder"',
             f'pos="{_f(x)} {_f(y)} {_f(z)}"',
             f'size="{c.radius} {_f(hl)}"',
             f'rgba="{r} {g} {bl} {al}"']
    if any(v != 0 for v in c.euler):
        rx, ry, rz = c.euler
        parts.append(f'euler="{_f(rx)} {_f(ry)} {_f(rz)}"')
    L.append(f'    <geom {" ".join(parts)}/>')


# ── Main ──────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="Generate LRC 2025 Atlanta Confined Configuration world files")
    ap.add_argument('--slope', action='store_true',
                    help='Add 15° cross-slope to lane floors')
    args = ap.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(script_dir)
    worlds_dir = os.path.join(repo_root, 'src', 'go2w', 'go2_gazebo_sim', 'worlds')

    suffix = '_slope' if args.slope else ''
    sdf_path = os.path.join(worlds_dir, f'lrc_terrain_maze{suffix}.world')
    mjcf_path = os.path.join(worlds_dir, f'lrc_terrain_maze{suffix}.xml')

    variant = '15° slope' if args.slope else 'flat'
    print(f"Generating LRC 2025 Atlanta Confined Configuration ({variant})...")

    geoms = build_all(args.slope)
    boxes = sum(1 for g in geoms if isinstance(g, Box))
    cyls = sum(1 for g in geoms if isinstance(g, Cyl))
    print(f"  Elements: {len(geoms)} total ({boxes} boxes, {cyls} cylinders)")

    write_sdf(geoms, sdf_path)
    write_mjcf(geoms, mjcf_path)

    print("\nLayout (top-down):")
    print("  y")
    print("  ^")
    print("  3 [  Lane C  ]--[  Lane D  ]--[  Lane F  ]")
    print("  0 [ Pallets  ]  [ K-Rails  ]  [  Stairs  ]")
    print(" -3 [  (elbow) ]  [          ]  [          ]")
    print("       |                                 -> END")
    print(" -4 [  Lane A  ]")
    print(" -7 [  Ramps   ]")
    print("-10 [          ]")
    print("       ^ ENTRY (0, -11.5)")
    print("  x: -3  0  3  4  7  10 11  14  17")
    print(f"\nRobot spawn: (0, -11.5) facing north")
    print("Done.")


if __name__ == '__main__':
    main()
