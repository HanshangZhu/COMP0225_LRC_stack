from __future__ import annotations

import copy
import math
import shutil
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from core.animation_renderer import AnimationRenderer
from core.assignment_solver import tick_reservations, update_reservations
from core.frontier_manager import build_frontier_candidates
from core.map_manager import MapManager
from core.metrics_manager import EpisodeMetrics, save_coverage_csv, save_step_logs_csv
from core.replanning_policy import apply_hysteresis, should_replan
from core.types import GoalAssignment, PlannerInput, RobotState
from planners import build_planner
from simulators.base_simulator import BaseSimulator

from core.artifact_manager import ArtifactManager
from .map_generators import generate_map


@dataclass
class EpisodeResult:
    metrics: EpisodeMetrics
    summary: dict
    coverage_csv_path: str
    step_log_csv_path: str
    animation_gif_path: str | None
    animation_mp4_path: str | None


def _normalize_deg(angle_deg: float) -> float:
    out = angle_deg % 360.0
    if out < 0:
        out += 360.0
    return out


def _angle_diff_deg(target: float, source: float) -> float:
    return ((target - source + 180.0) % 360.0) - 180.0


def _heading_to(a: tuple[int, int], b: tuple[int, int]) -> float:
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def _build_robots(cfg: dict, map_mgr: MapManager) -> list[RobotState]:
    robots_cfg = cfg["robots"]
    num = int(robots_cfg["num_robots"])
    starts = [tuple(int(v) for v in s) for s in robots_cfg["start_positions"]]
    headings = [float(h) for h in robots_cfg.get("start_headings_deg", [0.0] * num)]

    if len(starts) < num:
        raise ValueError("Not enough start_positions")
    if len(headings) < num:
        headings = headings + [headings[-1] if headings else 0.0] * (num - len(headings))

    starts = starts[:num]
    headings = headings[:num]
    map_mgr.ensure_starts_free(starts)

    robots: list[RobotState] = []
    for i, (s, h) in enumerate(zip(starts, headings), start=1):
        if not map_mgr.in_bounds(s):
            raise ValueError(f"Robot start out of bounds: {s}")
        robots.append(RobotState(robot_id=i, pose=s, heading_deg=h))
    return robots


def _idle_assignment(robot_id: int) -> GoalAssignment:
    return GoalAssignment(robot_id=robot_id, target=None, path=[], utility=float("-inf"), valid=False, breakdown={})


def _assignment_signature(assignments: dict[int, GoalAssignment], robots: list[RobotState]) -> tuple[tuple[int, tuple[int, int] | None], ...]:
    sig: list[tuple[int, tuple[int, int] | None]] = []
    for r in sorted(robots, key=lambda rr: rr.robot_id):
        a = assignments.get(r.robot_id)
        target = None
        if a is not None and a.valid:
            target = a.target
        sig.append((int(r.robot_id), target))
    return tuple(sig)


def _probe_predictor_decisions(
    planner_name: str,
    cfg: dict,
    map_mgr: MapManager,
    robots: list[RobotState],
    frontier_candidates,
    assignments: dict[int, GoalAssignment],
    reservation_state: dict[tuple[int, int], dict[str, int]],
    step_idx: int,
    sim_time: float,
    current_output,
) -> tuple[str, dict[str, tuple], dict[str, float]]:
    predictor_cfg = cfg.get("predictor", {})
    base_predictor = str(current_output.debug.get("predictor", predictor_cfg.get("type", "path_follow")))

    signatures: dict[str, tuple] = {
        base_predictor: _assignment_signature(dict(current_output.assignments), robots),
    }
    scores: dict[str, float] = {
        base_predictor: float(current_output.joint_score),
    }

    analysis_cfg = cfg.get("analysis", {})
    probe_predictors = [str(p) for p in analysis_cfg.get("decision_probe_predictors", []) if str(p).strip()]
    if not probe_predictors:
        return base_predictor, signatures, scores

    for predictor_name in probe_predictors:
        if predictor_name == base_predictor:
            continue

        probe_cfg = copy.deepcopy(cfg)
        # Keep evaluation on RH core planner so predictor choice is not overridden by planner wrapper.
        probe_cfg["planning"]["planner_name"] = "rh_cfpa2"
        probe_cfg.setdefault("predictor", {})
        probe_cfg["predictor"]["type"] = predictor_name
        if predictor_name == "physics_residual":
            probe_cfg["predictor"].setdefault("physics_residual", {})
            probe_cfg["predictor"]["physics_residual"]["enabled"] = True

        probe_planner = build_planner(probe_cfg)
        probe_output = probe_planner.plan(
            PlannerInput(
                shared_map=map_mgr,
                robot_states=robots,
                frontier_candidates=frontier_candidates,
                current_assignments=assignments,
                reservation_state=reservation_state,
                step_idx=step_idx,
                sim_time=sim_time,
                config=probe_cfg,
            )
        )
        signatures[predictor_name] = _assignment_signature(dict(probe_output.assignments), robots)
        scores[predictor_name] = float(probe_output.joint_score)

    return base_predictor, signatures, scores


def _execute_robot_step(
    robot: RobotState,
    map_mgr: MapManager,
    cfg: dict,
    rng: np.random.Generator,
    occupied_next: set[tuple[int, int]],
    step_dt: float,
) -> tuple[bool, bool, bool]:
    """Returns: moved, conflict, congestion."""

    robot.total_steps += 1
    prev_pose = robot.pose

    if not robot.path:
        robot.velocity = (0.0, 0.0)
        robot.idle_steps += 1
        robot.steps_since_progress += 1
        robot.trajectory.append(robot.pose)
        return False, False, False

    nxt = robot.path[0]
    desired_heading = _heading_to(robot.pose, nxt)
    turn_rate = float(cfg["robots"].get("turn_rate_deg_per_step", 35.0))
    yaw_diff = _angle_diff_deg(desired_heading, robot.heading_deg)

    if bool(cfg["robots"].get("turning_pause_if_large_yaw", True)):
        pause_thr = float(cfg["robots"].get("turning_pause_threshold_deg", 30.0))
        if abs(yaw_diff) > pause_thr:
            delta = max(-turn_rate, min(turn_rate, yaw_diff))
            robot.heading_deg = _normalize_deg(robot.heading_deg + delta)
            robot.velocity = (0.0, 0.0)
            robot.idle_steps += 1
            robot.steps_since_progress += 1
            robot.trajectory.append(robot.pose)
            return False, False, True

    # Limited heading rate even when moving.
    delta = max(-turn_rate, min(turn_rate, yaw_diff))
    robot.heading_deg = _normalize_deg(robot.heading_deg + delta)

    motion_uncertainty = float(cfg["robots"].get("motion_uncertainty_prob", 0.0))
    if motion_uncertainty > 0.0 and rng.random() < motion_uncertainty:
        robot.velocity = (0.0, 0.0)
        robot.idle_steps += 1
        robot.steps_since_progress += 1
        robot.trajectory.append(robot.pose)
        return False, False, True

    if bool(cfg["robots"].get("slowdown_near_obstacle", True)):
        dist = int(cfg["robots"].get("obstacle_slowdown_distance", 1))
        slow_prob = float(cfg["robots"].get("obstacle_slowdown_prob", 0.2))
        obstacle_density = map_mgr.obstacle_count_around(robot.pose, radius=dist)
        if obstacle_density > 0 and rng.random() < slow_prob:
            robot.velocity = (0.0, 0.0)
            robot.idle_steps += 1
            robot.steps_since_progress += 1
            robot.trajectory.append(robot.pose)
            return False, False, True

    clearance = int(cfg["robots"].get("clearance_cells", 0))
    if not map_mgr.is_traversable(nxt, clearance):
        robot.path = []
        robot.velocity = (0.0, 0.0)
        robot.idle_steps += 1
        robot.steps_since_progress += 1
        robot.trajectory.append(robot.pose)
        return False, False, True

    if nxt in occupied_next:
        robot.velocity = (0.0, 0.0)
        robot.idle_steps += 1
        robot.steps_since_progress += 1
        robot.trajectory.append(robot.pose)
        return False, True, True

    occupied_next.add(nxt)
    robot.path.pop(0)
    robot.pose = nxt
    robot.total_move_steps += 1
    if nxt in robot.trajectory:
        robot.revisited_move_steps += 1

    dx = nxt[0] - prev_pose[0]
    dy = nxt[1] - prev_pose[1]
    robot.velocity = (float(dx) / max(1e-6, step_dt), float(dy) / max(1e-6, step_dt))
    robot.path_length += math.hypot(dx, dy)

    if robot.pose == prev_pose:
        robot.steps_since_progress += 1
    else:
        robot.steps_since_progress = 0

    robot.trajectory.append(robot.pose)
    return True, False, False


class GridSimulation(BaseSimulator):
    """Unified high-level planner simulation with Go2W-like approximations."""

    def run_episode(
        self,
        cfg: dict,
        planner_name: str,
        seed: int,
        output_dir: str | Path,
        animation_stem: str,
        sample_callback: callable | None = None,
    ) -> EpisodeResult:
        env_cfg = dict(cfg["environment"])
        env_cfg["random_seed"] = int(seed)

        prebuilt = cfg.get("_prebuilt_truth_map")
        if prebuilt is not None:
            truth = prebuilt
        else:
            truth = generate_map(
                map_type=str(env_cfg["map_type"]),
                width=int(env_cfg["map_width"]),
                height=int(env_cfg["map_height"]),
                obstacle_density=float(env_cfg.get("obstacle_density", 0.0)),
                seed=int(env_cfg["random_seed"]),
            )

        map_mgr = MapManager(truth)
        robots = _build_robots(cfg, map_mgr)
        cfg = dict(cfg)
        cfg["planning"] = dict(cfg["planning"])
        cfg["planning"]["planner_name"] = planner_name

        # Artifact system (only active when vlm.enabled)
        artifact_mgr: ArtifactManager | None = None
        if cfg.get("vlm", {}).get("enabled", False):
            positions = cfg["vlm"].get("_artifact_positions", [])
            artifact_mgr = ArtifactManager(positions)

        rng = np.random.default_rng(int(seed))

        sensor_range = int(cfg["robots"].get("sensor_range", 6))
        sensor_fov = float(cfg["robots"].get("sensor_fov_deg", 360.0))
        use_los = bool(cfg["robots"].get("use_line_of_sight", True))
        miss_prob = float(cfg["robots"].get("observation_miss_prob", 0.0))

        for r in robots:
            map_mgr.observe_from(r.pose, r.heading_deg, sensor_range, sensor_fov, use_los, miss_prob, rng)

        planner = build_planner(cfg)
        if hasattr(planner, "set_artifact_manager") and artifact_mgr is not None:
            planner.set_artifact_manager(artifact_mgr)
        metrics = EpisodeMetrics(
            planner_name=planner_name,
            map_name=str(env_cfg.get("map_name", env_cfg["map_type"])),
            seed=int(seed),
            rollout_horizon=int(cfg["planning"]["rollout"].get("horizon", 1)),
            predictor_type=str(cfg.get("predictor", {}).get("type", "none")),
        )

        renderer = AnimationRenderer(cfg)

        assignments: dict[int, GoalAssignment] = {r.robot_id: _idle_assignment(r.robot_id) for r in robots}
        reservation_state: dict[tuple[int, int], dict[str, int]] = {}

        old_joint_score: float | None = None
        current_joint_score: float | None = None
        prev_frontier_count = -1
        last_replan_reason = "init"
        last_planner_ms = 0.0
        last_predictor_ms = 0.0
        decision_probe_calls = 0

        max_steps = int(cfg["termination"].get("max_steps", 1000))
        coverage_threshold = float(cfg["termination"].get("coverage_threshold", 0.95))
        step_dt = float(cfg["termination"].get("step_dt", 1.0))

        success = False
        reason = "max_steps"
        step_idx = 0
        sim_time = 0.0

        while step_idx < max_steps:
            tick_reservations(reservation_state)

            for r in robots:
                map_mgr.observe_from(r.pose, r.heading_deg, sensor_range, sensor_fov, use_los, miss_prob, rng)

            frontier_cells, frontier_candidates = build_frontier_candidates(map_mgr, cfg)
            frontier_reps = set(c.representative for c in frontier_candidates)

            coverage = map_mgr.explored_free_ratio()
            metrics.log_step(
                step_idx=step_idx,
                sim_time=sim_time,
                coverage=coverage,
                frontier_cells=len(frontier_cells),
                frontier_candidates=len(frontier_candidates),
                planner_score=current_joint_score,
            )
            if metrics.step_logs:
                metrics.step_logs[-1].update(
                    {
                        "last_replan_reason": last_replan_reason,
                        "last_planner_compute_time_ms": float(last_planner_ms),
                        "last_predictor_inference_time_ms": float(last_predictor_ms),
                        "robot_poses": str({r.robot_id: r.pose for r in robots}),
                        "robot_headings_deg": str({r.robot_id: round(r.heading_deg, 2) for r in robots}),
                        "robot_velocities": str({r.robot_id: (round(r.velocity[0], 4), round(r.velocity[1], 4)) for r in robots}),
                        "robot_goals": str({rid: (a.target if a.valid else None) for rid, a in assignments.items()}),
                    }
                )

            if sample_callback is not None:
                sample_callback(
                    "step_begin",
                    {
                        "step_idx": step_idx,
                        "sim_time": sim_time,
                        "map_mgr": map_mgr,
                        "robots": robots,
                        "assignments": assignments,
                        "cfg": cfg,
                        "planner_name": planner_name,
                        "seed": seed,
                    },
                )

            if coverage >= coverage_threshold:
                success = True
                reason = "coverage_reached"
                break

            if not frontier_candidates:
                success = False
                reason = "no_frontier"
                break

            do_replan, replan_reason = should_replan(
                map_mgr=map_mgr,
                robots=robots,
                assignments=assignments,
                frontier_reps=frontier_reps,
                step_idx=step_idx,
                prev_frontier_count=prev_frontier_count,
                current_frontier_count=len(frontier_candidates),
                cfg=cfg,
            )
            if step_idx == 0:
                do_replan = True
                replan_reason = "initial"

            if do_replan:
                t0 = time.perf_counter()
                planner_output = planner.plan(
                    PlannerInput(
                        shared_map=map_mgr,
                        robot_states=robots,
                        frontier_candidates=frontier_candidates,
                        current_assignments=assignments,
                        reservation_state=reservation_state,
                        step_idx=step_idx,
                        sim_time=sim_time,
                        config=cfg,
                    )
                )
                t1 = time.perf_counter()
                plan_dt = t1 - t0
                last_planner_ms = plan_dt * 1000.0
                last_predictor_ms = 0.0

                new_assignments = dict(planner_output.assignments)
                for r in robots:
                    new_assignments.setdefault(r.robot_id, _idle_assignment(r.robot_id))

                new_score = float(planner_output.joint_score)

                critical_reason = (
                    "target_invalidated" in replan_reason
                    or "target_reached" in replan_reason
                    or "path_empty" in replan_reason
                    or "stuck" in replan_reason
                )

                if (not critical_reason) and any(a.valid for a in assignments.values()):
                    selected, kept_old = apply_hysteresis(
                        old_assignments=assignments,
                        new_assignments=new_assignments,
                        old_score=old_joint_score,
                        new_score=new_score,
                        cfg=cfg,
                    )
                    if kept_old:
                        assignments = dict(selected)
                        current_joint_score = old_joint_score
                    else:
                        assignments = dict(selected)
                        current_joint_score = new_score
                else:
                    assignments = dict(new_assignments)
                    current_joint_score = new_score

                for r in robots:
                    a = assignments.get(r.robot_id, _idle_assignment(r.robot_id))
                    r.set_plan(a.target, a.path)

                update_reservations(
                    reservation_state=reservation_state,
                    assignments=assignments,
                    ttl=int(cfg["planning"].get("reservation_ttl", 16)),
                )

                old_joint_score = current_joint_score
                last_replan_reason = replan_reason

                metrics.log_replan(replan_reason, planner_compute_time=plan_dt)
                metrics.log_assignments(assignments)
                metrics.register_predictions(step_idx, planner_output.predicted_paths)
                predictor_times = planner_output.debug.get("predictor_inference_times")
                if isinstance(predictor_times, dict):
                    metrics.log_predictor_times(predictor_times)
                    if predictor_times:
                        last_predictor_ms = (sum(float(v) for v in predictor_times.values()) / len(predictor_times)) * 1000.0

                analysis_cfg = cfg.get("analysis", {})
                probe_enabled = bool(analysis_cfg.get("enable_predictor_decision_probe", False))
                max_probe = analysis_cfg.get("decision_probe_max_per_episode")
                max_probe = None if max_probe is None else int(max_probe)
                if (
                    probe_enabled
                    and planner_name in ("rh_cfpa2", "physics_rh_cfpa2")
                    and (max_probe is None or decision_probe_calls < max_probe)
                ):
                    base_predictor, signatures, scores = _probe_predictor_decisions(
                        planner_name=planner_name,
                        cfg=cfg,
                        map_mgr=map_mgr,
                        robots=robots,
                        frontier_candidates=frontier_candidates,
                        assignments=assignments,
                        reservation_state=reservation_state,
                        step_idx=step_idx,
                        sim_time=sim_time,
                        current_output=planner_output,
                    )
                    metrics.log_decision_probe(
                        base_predictor=base_predictor,
                        decision_signatures=signatures,
                        predictor_scores=scores,
                    )
                    decision_probe_calls += 1

            # Simple near-blocking proxy: robots trying to swap positions.
            if len(robots) >= 2:
                r1, r2 = robots[0], robots[1]
                if r1.path and r2.path and r1.path[0] == r2.pose and r2.path[0] == r1.pose:
                    metrics.log_congestion()

            occupied_next: set[tuple[int, int]] = set()
            for r in robots:
                _moved, conflict, congested = _execute_robot_step(
                    robot=r,
                    map_mgr=map_mgr,
                    cfg=cfg,
                    rng=rng,
                    occupied_next=occupied_next,
                    step_dt=step_dt,
                )
                if conflict:
                    metrics.log_conflict()
                if congested:
                    metrics.log_congestion()

            newly_observed: set[tuple[int, int]] = set()
            for r in robots:
                observed = map_mgr.observe_from(r.pose, r.heading_deg, sensor_range, sensor_fov, use_los, miss_prob, rng)
                newly_observed.update(observed)

            # Artifact discovery check
            if artifact_mgr is not None:
                newly_found = artifact_mgr.check_discovery(newly_observed)
                for pos in newly_found:
                    print(f"[FOUND] Artifact at (x={pos[0]}, y={pos[1]}) — step {step_idx}")
                if artifact_mgr.all_found:
                    success = True
                    reason = "artifact_found"
                    break

            metrics.update_prediction_error(step_idx, robots)

            if sample_callback is not None:
                sample_callback(
                    "step_end",
                    {
                        "step_idx": step_idx,
                        "sim_time": sim_time,
                        "map_mgr": map_mgr,
                        "robots": robots,
                        "assignments": assignments,
                        "cfg": cfg,
                        "planner_name": planner_name,
                        "seed": seed,
                    },
                )

            renderer.update(
                step_idx=step_idx,
                map_mgr=map_mgr,
                robots=robots,
                frontier_cells=frontier_cells,
                frontier_candidates=frontier_candidates,
                assignments=assignments,
                coverage=coverage,
                planner_name=planner_name,
                seed=seed,
                sim_time=sim_time,
                replan_count=metrics.replan_count,
                joint_score=current_joint_score,
                last_replan_reason=last_replan_reason,
                sensor_range=sensor_range,
                sensor_fov_deg=sensor_fov,
                artifact_positions=artifact_mgr.positions if artifact_mgr else None,
                found_positions=set(artifact_mgr.found_positions) if artifact_mgr else None,
            )

            prev_frontier_count = len(frontier_candidates)
            step_idx += 1
            sim_time += step_dt

        metrics.finalize(robots=robots, steps=step_idx, sim_time=sim_time, success=success, reason=reason)

        if sample_callback is not None:
            sample_callback(
                "episode_end",
                {
                    "step_idx": step_idx,
                    "sim_time": sim_time,
                    "map_mgr": map_mgr,
                    "robots": robots,
                    "assignments": assignments,
                    "cfg": cfg,
                    "planner_name": planner_name,
                    "seed": seed,
                    "success": success,
                    "reason": reason,
                },
            )

        out_dir = Path(output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

        coverage_csv = out_dir / f"coverage_{animation_stem}.csv"
        step_csv = out_dir / f"steps_{animation_stem}.csv"
        save_coverage_csv(coverage_csv, metrics.coverage_curve)
        save_step_logs_csv(step_csv, metrics.step_logs)

        animation_root = out_dir / "animations" / animation_stem
        gif_path, mp4_path = renderer.finalize(animation_root)

        global_anim_dir = Path(cfg.get("experiment", {}).get("global_animation_dir", "outputs/animations"))
        global_anim_dir.mkdir(parents=True, exist_ok=True)
        if gif_path is not None:
            gif_dst = global_anim_dir / Path(gif_path).name
            if Path(gif_path).resolve() != gif_dst.resolve():
                shutil.copy2(gif_path, gif_dst)
            gif_path = str(gif_dst)
        if mp4_path is not None:
            mp4_dst = global_anim_dir / Path(mp4_path).name
            if Path(mp4_path).resolve() != mp4_dst.resolve():
                shutil.copy2(mp4_path, mp4_dst)
            mp4_path = str(mp4_dst)

        summary = metrics.to_summary_row()

        return EpisodeResult(
            metrics=metrics,
            summary=summary,
            coverage_csv_path=str(coverage_csv),
            step_log_csv_path=str(step_csv),
            animation_gif_path=gif_path,
            animation_mp4_path=mp4_path,
        )
