from __future__ import annotations

import os
from pathlib import Path
from typing import Sequence

import imageio.v2 as imageio
import matplotlib
import numpy as np
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle, Wedge

from .map_manager import FREE, OCCUPIED, UNKNOWN, MapManager
from .types import FrontierCandidate, GoalAssignment, RobotState

if os.environ.get("MPLBACKEND") is None:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt

UNKNOWN_COLOR = "#9E9E9E"
FREE_COLOR = "#FFFFFF"
OCCUPIED_COLOR = "#101010"
FRONTIER_COLOR = "#2E7D32"

ROBOT_COLORS = {
    1: "#D32F2F",
    2: "#1976D2",
    3: "#388E3C",
    4: "#F57C00",
}

TRAJ_COLORS = {
    1: "#FFCDD2",
    2: "#BBDEFB",
    3: "#C8E6C9",
    4: "#FFE0B2",
}


class AnimationRenderer:
    def __init__(self, cfg: dict):
        acfg = cfg.get("animation", {})
        self.enable_live = bool(cfg.get("experiment", {}).get("enable_live_plot", False))
        self.save_animation = bool(cfg.get("experiment", {}).get("save_animation", True))
        self.save_gif = bool(acfg.get("save_gif", False))
        self.save_mp4 = bool(acfg.get("save_mp4", True))
        self.fps = int(acfg.get("fps", 8))
        self.plot_every_n_steps = max(1, int(acfg.get("plot_every_n_steps", 1)))
        self.show_frontier_cells = bool(acfg.get("show_frontier_cells", False))
        self.show_sensor_fov = bool(acfg.get("show_sensor_fov", True))

        fig_w, fig_h = acfg.get("figsize", [12.0, 7.2])
        self.fig, self.ax = plt.subplots(figsize=(float(fig_w), float(fig_h)))
        self.frames: list[np.ndarray] = []

        if self.enable_live:
            plt.ion()

    def should_draw(self, step_idx: int) -> bool:
        return step_idx % self.plot_every_n_steps == 0

    def _draw_map(self, map_mgr: MapManager) -> None:
        img = np.zeros_like(map_mgr.known, dtype=np.int8)
        img[map_mgr.known == FREE] = 1
        img[map_mgr.known == OCCUPIED] = 2
        cmap = ListedColormap([UNKNOWN_COLOR, FREE_COLOR, OCCUPIED_COLOR])
        self.ax.imshow(img, cmap=cmap, origin="upper", interpolation="nearest")

    def update(
        self,
        step_idx: int,
        map_mgr: MapManager,
        robots: Sequence[RobotState],
        frontier_cells: Sequence[tuple[int, int]],
        frontier_candidates: Sequence[FrontierCandidate],
        assignments: dict[int, GoalAssignment],
        coverage: float,
        planner_name: str,
        seed: int,
        sim_time: float,
        replan_count: int,
        joint_score: float | None,
        last_replan_reason: str,
        sensor_range: int,
        sensor_fov_deg: float,
        artifact_positions: list[tuple[int, int]] | None = None,
        found_positions: set[tuple[int, int]] | None = None,
        vlm_reasoning: str | None = None,
    ) -> None:
        if not self.should_draw(step_idx):
            return

        self.ax.clear()
        self._draw_map(map_mgr)

        if self.show_frontier_cells and frontier_cells:
            fx = [c[0] for c in frontier_cells]
            fy = [c[1] for c in frontier_cells]
            self.ax.scatter(fx, fy, c=FRONTIER_COLOR, s=5, marker=".", alpha=0.45)

        reps = [c.representative for c in frontier_candidates]
        if reps:
            rx = [c[0] for c in reps]
            ry = [c[1] for c in reps]
            self.ax.scatter(rx, ry, c=FRONTIER_COLOR, s=45, marker="x", linewidths=1.5)

        # --- Artifact overlay (operator view) ---
        if artifact_positions:
            _found = found_positions or set()
            for ax_, ay_ in artifact_positions:
                color = "#FF4500" if (ax_, ay_) in _found else "#FFD700"
                self.ax.scatter(
                    [ax_], [ay_], c=color, s=200, marker="*",
                    edgecolors="black", linewidths=0.5, zorder=6,
                )

        for robot in robots:
            rid = robot.robot_id
            color = ROBOT_COLORS.get(rid, "#FF9800")
            traj_color = TRAJ_COLORS.get(rid, "#FFE082")

            if len(robot.trajectory) >= 2:
                tx = [p[0] for p in robot.trajectory]
                ty = [p[1] for p in robot.trajectory]
                self.ax.plot(tx, ty, color=traj_color, linewidth=1.0, alpha=0.85)

            if robot.path:
                px = [p[0] for p in robot.path]
                py = [p[1] for p in robot.path]
                self.ax.plot(px, py, linestyle="--", color=color, linewidth=1.2, alpha=0.75)

            if self.show_sensor_fov:
                if sensor_fov_deg >= 359.0:
                    self.ax.add_patch(Circle((robot.pose[0], robot.pose[1]), radius=sensor_range, fill=False, edgecolor=color, linewidth=0.8, alpha=0.35))
                else:
                    theta = robot.heading_deg
                    self.ax.add_patch(
                        Wedge(
                            center=(robot.pose[0], robot.pose[1]),
                            r=sensor_range,
                            theta1=theta - sensor_fov_deg * 0.5,
                            theta2=theta + sensor_fov_deg * 0.5,
                            width=None,
                            fill=False,
                            edgecolor=color,
                            linewidth=0.8,
                            alpha=0.35,
                        )
                    )

            self.ax.scatter([robot.pose[0]], [robot.pose[1]], c=color, s=70, marker="o", edgecolors="black", linewidths=0.4)
            self.ax.text(robot.pose[0] + 0.4, robot.pose[1] + 0.4, f"R{rid}", color=color, fontsize=8)

        for rid, a in assignments.items():
            if not a.valid or a.target is None:
                continue
            color = ROBOT_COLORS.get(rid, "#FF9800")
            self.ax.scatter([a.target[0]], [a.target[1]], c=color, s=135, marker="*")

        score_txt = f"{joint_score:.2f}" if joint_score is not None and np.isfinite(joint_score) else "-"
        info = (
            f"planner={planner_name}\n"
            f"seed={seed}\n"
            f"step={step_idx}  t={sim_time:.1f}s\n"
            f"coverage={coverage * 100:.1f}%\n"
            f"frontier_reps={len(frontier_candidates)}\n"
            f"replans={replan_count}\n"
            f"joint_score={score_txt}\n"
            f"last_replan={last_replan_reason}"
        )
        if vlm_reasoning:
            short = vlm_reasoning[:80] + ("..." if len(vlm_reasoning) > 80 else "")
            info += f"\nvlm: {short}"
        if artifact_positions:
            n_found = len(found_positions or set())
            info += f"\nartifacts: {n_found}/{len(artifact_positions)}"
        self.ax.text(
            1.01,
            0.99,
            info,
            transform=self.ax.transAxes,
            va="top",
            ha="left",
            fontsize=8,
            bbox=dict(facecolor="white", alpha=0.85, edgecolor="none"),
        )

        self.ax.set_title("Go2W-like Multi-Robot Exploration (Planner-Level Approximation)")
        self.ax.set_xlim(-0.5, map_mgr.width - 0.5)
        self.ax.set_ylim(map_mgr.height - 0.5, -0.5)
        self.ax.set_aspect("equal")
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.fig.tight_layout(rect=(0.0, 0.0, 0.76, 1.0))

        self.fig.canvas.draw()
        if self.save_animation:
            frame = np.asarray(self.fig.canvas.buffer_rgba())[..., :3].copy()
            self.frames.append(frame)

        if self.enable_live:
            plt.pause(0.01)  # 10ms — enough for macOS to refresh the window

    def finalize(self, output_stem: str | Path) -> tuple[str | None, str | None]:
        gif_path: str | None = None
        mp4_path: str | None = None
        output_stem = Path(output_stem)
        output_stem.parent.mkdir(parents=True, exist_ok=True)

        if self.frames and self.save_animation and self.save_gif:
            gif = output_stem.with_suffix(".gif")
            imageio.mimsave(gif, self.frames, fps=self.fps)
            gif_path = str(gif)

        if self.frames and self.save_animation and self.save_mp4:
            mp4 = output_stem.with_suffix(".mp4")
            try:
                with imageio.get_writer(mp4, fps=self.fps) as w:
                    for frame in self.frames:
                        w.append_data(frame)
                mp4_path = str(mp4)
            except Exception:
                mp4_path = None

        if self.enable_live:
            plt.ioff()
        plt.close(self.fig)

        return gif_path, mp4_path
