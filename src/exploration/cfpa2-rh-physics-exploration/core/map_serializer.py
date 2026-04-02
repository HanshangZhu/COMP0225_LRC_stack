from __future__ import annotations

import base64
import io
from typing import Any, Sequence

import matplotlib.figure
import matplotlib.backends.backend_agg as _agg
import numpy as np
from matplotlib.colors import ListedColormap

from .map_manager import FREE, OCCUPIED, UNKNOWN, MapManager
from .types import FrontierCandidate, GoalAssignment, RobotState


def build_map_state_json(
    map_mgr: MapManager,
    robots: Sequence[RobotState],
    frontier_candidates: Sequence[FrontierCandidate],
    step_idx: int,
    goal_prompt: str,
    artifact_mgr: Any | None = None,
) -> dict:
    """Compact JSON description of the current episode state for the VLM."""
    robot_list = [
        {
            "id": r.robot_id,
            "x": int(r.pose[0]),
            "y": int(r.pose[1]),
            "heading_deg": float(r.heading_deg),
        }
        for r in robots
    ]

    frontier_list = [
        {
            "idx": i,
            "rep_x": int(fc.representative[0]),
            "rep_y": int(fc.representative[1]),
            "cluster_size": fc.size,
        }
        for i, fc in enumerate(frontier_candidates)
    ]

    coverage = map_mgr.explored_free_ratio() if hasattr(map_mgr, "explored_free_ratio") else 0.0

    state: dict[str, Any] = {
        "grid_width": int(map_mgr.width),
        "grid_height": int(map_mgr.height),
        "step_idx": int(step_idx),
        "coverage_pct": round(float(coverage) * 100, 2),
        "robots": robot_list,
        "frontiers": frontier_list,
        "goal_prompt": goal_prompt,
    }

    if artifact_mgr is not None:
        state["artifacts_found"] = artifact_mgr.found_count
        state["artifacts_total"] = artifact_mgr.total_count

    return state


# ---------------------------------------------------------------------------
# Snapshot renderer  (dedicated hidden figure — never touches AnimationRenderer)
# ---------------------------------------------------------------------------

_SNAPSHOT_FIG: matplotlib.figure.Figure | None = None
_SNAPSHOT_AX: Any | None = None

UNKNOWN_COLOR = "#9E9E9E"
FREE_COLOR = "#FFFFFF"
OCCUPIED_COLOR = "#101010"
FRONTIER_COLOR = "#2E7D32"
ROBOT_COLOR = "#D32F2F"


def render_snapshot_png_b64(
    map_mgr: MapManager,
    robots: Sequence[RobotState],
    frontier_candidates: Sequence[FrontierCandidate],
    assignments: dict[int, GoalAssignment] | None = None,
    artifact_mgr: Any | None = None,
    figsize: tuple[float, float] = (8.0, 6.0),
) -> str:
    """Render current map state to a base64-encoded PNG string.

    Uses a private matplotlib figure so we never interfere with the
    live AnimationRenderer figure.
    """
    global _SNAPSHOT_FIG, _SNAPSHOT_AX

    if _SNAPSHOT_FIG is None:
        # Use the Agg backend explicitly for this private figure so it never
        # opens a window, regardless of the global interactive backend.
        _SNAPSHOT_FIG = matplotlib.figure.Figure(figsize=figsize)
        _agg.FigureCanvasAgg(_SNAPSHOT_FIG)
        _SNAPSHOT_AX = _SNAPSHOT_FIG.add_subplot(111)

    ax = _SNAPSHOT_AX
    assert ax is not None
    ax.clear()

    # --- map ---
    img = np.zeros_like(map_mgr.known, dtype=np.int8)
    img[map_mgr.known == FREE] = 1
    img[map_mgr.known == OCCUPIED] = 2
    cmap = ListedColormap([UNKNOWN_COLOR, FREE_COLOR, OCCUPIED_COLOR])
    ax.imshow(img, cmap=cmap, origin="upper", interpolation="nearest")

    # --- frontiers ---
    for i, fc in enumerate(frontier_candidates):
        rx, ry = fc.representative
        ax.scatter([rx], [ry], c=FRONTIER_COLOR, s=40, marker="x", linewidths=1.2)
        ax.text(rx + 0.5, ry + 0.5, str(i), color=FRONTIER_COLOR, fontsize=7)

    # --- robots ---
    for r in robots:
        ax.scatter(
            [r.pose[0]], [r.pose[1]],
            c=ROBOT_COLOR, s=60, marker="o",
            edgecolors="black", linewidths=0.4,
        )
        if r.path:
            px = [p[0] for p in r.path]
            py = [p[1] for p in r.path]
            ax.plot(px, py, linestyle="--", color=ROBOT_COLOR, linewidth=0.8, alpha=0.6)

    # --- artifacts (operator view) ---
    if artifact_mgr is not None:
        found_set = set(artifact_mgr.found_positions)
        for pos in artifact_mgr.positions:
            color = "#FF4500" if pos in found_set else "#FFD700"
            ax.scatter([pos[0]], [pos[1]], c=color, s=180, marker="*",
                       edgecolors="black", linewidths=0.4, zorder=6)

    ax.set_xlim(-0.5, map_mgr.width - 0.5)
    ax.set_ylim(map_mgr.height - 0.5, -0.5)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])

    _SNAPSHOT_FIG.tight_layout()
    buf = io.BytesIO()
    _SNAPSHOT_FIG.savefig(buf, format="png", dpi=100)
    buf.seek(0)
    return base64.b64encode(buf.read()).decode("ascii")
