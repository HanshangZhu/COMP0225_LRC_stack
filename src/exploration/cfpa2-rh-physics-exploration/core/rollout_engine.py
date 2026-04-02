from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from predictors.base_predictor import BasePredictor

from .frontier_manager import is_frontier_cell
from .map_manager import FREE, OCCUPIED, UNKNOWN
from .predictor_features import local_context_for_predictor
from .types import Cell, FrontierCandidate, PredictorInput, RobotState
from .utility_service import (
    corridor_occupancy_penalty,
    narrow_passage_blocking_penalty,
    path_crossing_penalty,
    waiting_time_proxy,
)


@dataclass
class RolloutResult:
    future_score: float
    predicted_paths: dict[int, list[Cell]]
    predictor_inference_times: dict[int, float]
    breakdown: dict[str, float]


def _clamp_cell(cell: tuple[float, float], width: int, height: int) -> Cell:
    x = int(round(cell[0]))
    y = int(round(cell[1]))
    x = min(width - 1, max(0, x))
    y = min(height - 1, max(0, y))
    return (x, y)


def _future_overlap_proxy(a: Cell, b: Cell, sigma: float) -> float:
    if sigma <= 0.0:
        return 0.0
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    d2 = dx * dx + dy * dy
    return math.exp(-d2 / (2.0 * sigma * sigma))


def _normalize_angle_deg(angle_deg: float) -> float:
    out = angle_deg % 360.0
    if out < 0:
        out += 360.0
    return out


def _smallest_angle_diff_deg(a: float, b: float) -> float:
    d = abs(_normalize_angle_deg(a) - _normalize_angle_deg(b))
    return min(d, 360.0 - d)


def _heading_at_step(path_cells: list[Cell], t: int, fallback_heading_deg: float) -> float:
    if len(path_cells) < 2:
        return float(fallback_heading_deg)
    idx = min(max(1, int(t)), len(path_cells) - 1)
    a = path_cells[idx - 1]
    b = path_cells[idx]
    if a == b and idx + 1 < len(path_cells):
        b = path_cells[idx + 1]
    if a == b:
        return float(fallback_heading_deg)
    return math.degrees(math.atan2(float(b[1] - a[1]), float(b[0] - a[0])))


def _pad_path(path_cells: list[Cell], horizon: int) -> list[Cell]:
    if horizon <= 0:
        return []
    if not path_cells:
        return []
    if len(path_cells) >= horizon:
        return list(path_cells[:horizon])
    return list(path_cells) + [path_cells[-1]] * (horizon - len(path_cells))


def _bresenham_line(start: Cell, end: Cell) -> list[Cell]:
    x0, y0 = start
    x1, y1 = end
    line: list[Cell] = []

    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy

    x, y = x0, y0
    while True:
        line.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return line


def _is_visible_on_virtual_known(
    start: Cell,
    end: Cell,
    virtual_known: np.ndarray,
    unknown_blocks_los: bool,
) -> bool:
    line = _bresenham_line(start, end)
    for c in line[1:-1]:
        x, y = c
        val = int(virtual_known[y, x])
        if val == OCCUPIED:
            return False
        if unknown_blocks_los and val == UNKNOWN:
            return False
    return True


def _virtual_observed_unknown_cells(
    map_mgr,
    center: Cell,
    heading_deg: float,
    sensor_range: int,
    fov_deg: float,
    use_line_of_sight: bool,
    virtual_known: np.ndarray,
    unknown_blocks_los: bool,
) -> set[Cell]:
    if sensor_range <= 0 or not map_mgr.in_bounds(center):
        return set()

    cx, cy = center
    rr = sensor_range * sensor_range
    observed: set[Cell] = set()

    min_x = max(0, cx - sensor_range)
    max_x = min(map_mgr.width - 1, cx + sensor_range)
    min_y = max(0, cy - sensor_range)
    max_y = min(map_mgr.height - 1, cy + sensor_range)

    full_view = fov_deg >= 359.0

    for y in range(min_y, max_y + 1):
        dy = y - cy
        for x in range(min_x, max_x + 1):
            dx = x - cx
            if dx * dx + dy * dy > rr:
                continue

            if not full_view and (dx != 0 or dy != 0):
                ray_angle = math.degrees(math.atan2(float(dy), float(dx)))
                if _smallest_angle_diff_deg(ray_angle, heading_deg) > fov_deg * 0.5:
                    continue

            if use_line_of_sight and not _is_visible_on_virtual_known(center, (x, y), virtual_known, unknown_blocks_los):
                continue

            if int(virtual_known[y, x]) == UNKNOWN:
                observed.add((x, y))

    return observed


def _mark_virtual_revealed_cells(virtual_known: np.ndarray, cells: set[Cell]) -> None:
    for x, y in cells:
        if int(virtual_known[y, x]) == UNKNOWN:
            virtual_known[y, x] = FREE


def _frontier_density_proxy(
    rep: Cell,
    candidates: list[FrontierCandidate],
    map_mgr,
    virtual_known: np.ndarray,
    neighborhood: int,
    radius: int,
) -> float:
    if radius <= 0:
        return 0.0
    r2 = float(radius * radius)
    cnt = 0
    for c in candidates:
        other = c.representative
        if other == rep:
            continue
        dx = float(other[0] - rep[0])
        dy = float(other[1] - rep[1])
        if dx * dx + dy * dy > r2:
            continue
        if is_frontier_cell(map_mgr, other, neighborhood=neighborhood, grid=virtual_known):
            cnt += 1
    return float(cnt)


def _branch_opening_potential(
    map_mgr,
    rep: Cell,
    virtual_known: np.ndarray,
    inner_radius: int,
    outer_radius: int,
) -> float:
    inner = max(1, int(inner_radius))
    outer = max(inner + 1, int(outer_radius))
    inner_unknown = float(map_mgr.count_unknown_in_radius(rep, inner, grid=virtual_known))
    outer_unknown = float(map_mgr.count_unknown_in_radius(rep, outer, grid=virtual_known))
    return max(0.0, outer_unknown - inner_unknown)


def _teammate_path_proximity_penalty(rep: Cell, teammate_future_path: list[Cell], safe_distance: float) -> float:
    if safe_distance <= 0.0 or not teammate_future_path:
        return 0.0
    min_d = min(abs(rep[0] - p[0]) + abs(rep[1] - p[1]) for p in teammate_future_path)
    if float(min_d) >= safe_distance:
        return 0.0
    return (safe_distance - float(min_d)) / safe_distance


def _reservation_penalty(
    rep: Cell,
    reservation_state: dict[Cell, dict[str, int]] | None,
    robot_id: int,
) -> float:
    if not reservation_state:
        return 0.0
    entry = reservation_state.get(rep)
    if entry is None:
        return 0.0
    owner = int(entry.get("robot_id", -1))
    if owner < 0 or owner == int(robot_id):
        return 0.0
    ttl = max(1, int(entry.get("ttl", 1)))
    return 1.0 + min(1.0, float(ttl) / 10.0)


def _frontier_value_proxy(
    pose: Cell,
    robot_id: int,
    rep: Cell,
    candidates: list[FrontierCandidate],
    virtual_known: np.ndarray,
    map_mgr,
    reveal_radius: int,
    used_targets: set[Cell],
    teammate_future_path: list[Cell],
    teammate_reserved_targets: set[Cell],
    reservation_state: dict[Cell, dict[str, int]] | None,
    cfg: dict,
) -> float:
    rollout_cfg = cfg["planning"]["rollout"]
    neighborhood = int(cfg.get("frontier", {}).get("neighborhood", 8))

    if rep in used_targets:
        return float("-inf")
    if not is_frontier_cell(map_mgr, rep, neighborhood=neighborhood, grid=virtual_known):
        return float("-inf")

    ig = float(map_mgr.count_unknown_in_radius(rep, reveal_radius, grid=virtual_known))
    dist = float(abs(rep[0] - pose[0]) + abs(rep[1] - pose[1]))

    density_radius = int(rollout_cfg.get("frontier_density_radius", max(3, reveal_radius)))
    density = _frontier_density_proxy(
        rep=rep,
        candidates=candidates,
        map_mgr=map_mgr,
        virtual_known=virtual_known,
        neighborhood=neighborhood,
        radius=density_radius,
    )

    branch_inner = int(rollout_cfg.get("branch_inner_radius", max(1, reveal_radius)))
    branch_outer = int(rollout_cfg.get("branch_outer_radius", max(branch_inner + 1, 2 * reveal_radius)))
    branch = _branch_opening_potential(
        map_mgr=map_mgr,
        rep=rep,
        virtual_known=virtual_known,
        inner_radius=branch_inner,
        outer_radius=branch_outer,
    )

    teammate_safe_distance = float(rollout_cfg.get("teammate_safe_distance", 6.0))
    teammate_path_pen = _teammate_path_proximity_penalty(rep, teammate_future_path, teammate_safe_distance)
    teammate_target_pen = 1.0 if rep in teammate_reserved_targets else 0.0
    reserve_pen = _reservation_penalty(rep, reservation_state, robot_id=robot_id)

    w_ig = float(rollout_cfg.get("reassign_w_ig", 1.0))
    w_dist = float(rollout_cfg.get("reassign_w_distance", 0.35))
    w_density = float(rollout_cfg.get("reassign_w_density", 0.85))
    w_branch = float(rollout_cfg.get("reassign_w_branch", 0.45))
    w_teammate = float(rollout_cfg.get("reassign_w_teammate", 1.1))
    w_reservation = float(rollout_cfg.get("reassign_w_reservation", 1.0))

    return (
        w_ig * ig
        - w_dist * dist
        + w_density * density
        + w_branch * branch
        - w_teammate * (teammate_path_pen + teammate_target_pen)
        - w_reservation * reserve_pen
    )


def _best_virtual_frontier_target(
    pose: Cell,
    robot_id: int,
    candidates: list[FrontierCandidate],
    virtual_known: np.ndarray,
    map_mgr,
    reveal_radius: int,
    used_targets: set[Cell],
    teammate_future_path: list[Cell],
    teammate_reserved_targets: set[Cell],
    reservation_state: dict[Cell, dict[str, int]] | None,
    cfg: dict,
) -> Cell | None:
    best_target: Cell | None = None
    best_score = float("-inf")
    for c in candidates:
        rep = c.representative
        score = _frontier_value_proxy(
            pose=pose,
            robot_id=robot_id,
            rep=rep,
            candidates=candidates,
            virtual_known=virtual_known,
            map_mgr=map_mgr,
            reveal_radius=reveal_radius,
            used_targets=used_targets,
            teammate_future_path=teammate_future_path,
            teammate_reserved_targets=teammate_reserved_targets,
            reservation_state=reservation_state,
            cfg=cfg,
        )
        if score > best_score:
            best_score = score
            best_target = rep
    return best_target


def _straight_line_cells(start: Cell, goal: Cell, max_steps: int) -> list[Cell]:
    if max_steps <= 0:
        return []
    sx, sy = start
    gx, gy = goal
    out: list[Cell] = []
    cur_x, cur_y = sx, sy
    for _ in range(max_steps):
        if cur_x == gx and cur_y == gy:
            out.append((cur_x, cur_y))
            continue
        step_x = 0 if gx == cur_x else (1 if gx > cur_x else -1)
        step_y = 0 if gy == cur_y else (1 if gy > cur_y else -1)
        cur_x += step_x
        cur_y += step_y
        out.append((cur_x, cur_y))
    return out


def _rewrite_future_path_on_reach(
    path_cells: list[Cell],
    initial_goal: Cell | None,
    robot_id: int,
    candidates: list[FrontierCandidate],
    virtual_known: np.ndarray,
    map_mgr,
    reveal_radius: int,
    used_targets: set[Cell],
    teammate_future_path: list[Cell],
    teammate_reserved_targets: set[Cell],
    reservation_state: dict[Cell, dict[str, int]] | None,
    cfg: dict,
) -> tuple[list[Cell], int, set[Cell]]:
    if not path_cells or initial_goal is None:
        return path_cells, 0, set()

    rewritten = list(path_cells)
    current_goal = initial_goal
    reassign_count = 0
    selected_targets: set[Cell] = set()

    for t, pose in enumerate(list(rewritten)):
        if pose != current_goal:
            continue
        remaining = len(rewritten) - (t + 1)
        if remaining <= 0:
            continue

        teammate_suffix = teammate_future_path[t + 1 :] if teammate_future_path else []
        next_goal = _best_virtual_frontier_target(
            pose=pose,
            robot_id=robot_id,
            candidates=candidates,
            virtual_known=virtual_known,
            map_mgr=map_mgr,
            reveal_radius=reveal_radius,
            used_targets=used_targets,
            teammate_future_path=teammate_suffix,
            teammate_reserved_targets=teammate_reserved_targets,
            reservation_state=reservation_state,
            cfg=cfg,
        )
        if next_goal is None:
            continue

        used_targets.add(next_goal)
        selected_targets.add(next_goal)
        suffix = _straight_line_cells(start=pose, goal=next_goal, max_steps=remaining)
        rewritten[t + 1 :] = suffix
        current_goal = next_goal
        reassign_count += 1

    return rewritten, reassign_count, selected_targets


def rollout_pair_score(
    map_mgr,
    cfg: dict,
    robot1: RobotState,
    robot2: RobotState,
    goal1: Cell | None,
    goal2: Cell | None,
    path1: list[Cell],
    path2: list[Cell],
    candidates: list[FrontierCandidate],
    predictor: BasePredictor,
    reservation_state: dict[Cell, dict[str, int]] | None = None,
) -> RolloutResult:
    rollout_cfg = cfg["planning"]["rollout"]
    horizon = int(rollout_cfg.get("horizon", 4))
    gamma = float(rollout_cfg.get("gamma", 0.9))
    reveal_radius = int(rollout_cfg.get("virtual_reveal_radius", 4))
    gain_decay = float(rollout_cfg.get("virtual_gain_decay", 0.92))
    reassign_on_reach = bool(cfg["planning"].get("reassign_on_reach", True))

    if horizon <= 0:
        return RolloutResult(0.0, {}, {}, {"future_gain": 0.0, "future_score": 0.0})

    local_context_r1 = local_context_for_predictor(map_mgr=map_mgr, robot=robot1, teammate=robot2, cfg=cfg)
    local_context_r2 = local_context_for_predictor(map_mgr=map_mgr, robot=robot2, teammate=robot1, cfg=cfg)

    horizon_steps = int(cfg.get("predictor", {}).get("horizon_steps", horizon))
    step_dt = float(cfg["termination"].get("step_dt", 1.0))

    pred1 = predictor.predict(
        PredictorInput(
            robot_state=robot1,
            goal=goal1,
            current_path=path1,
            local_context=local_context_r1,
            horizon_steps=horizon_steps,
            step_dt=step_dt,
        )
    )
    pred2 = predictor.predict(
        PredictorInput(
            robot_state=robot2,
            goal=goal2,
            current_path=path2,
            local_context=local_context_r2,
            horizon_steps=horizon_steps,
            step_dt=step_dt,
        )
    )

    path_cells1 = [_clamp_cell((p.x, p.y), map_mgr.width, map_mgr.height) for p in pred1.trajectory[:horizon]]
    path_cells2 = [_clamp_cell((p.x, p.y), map_mgr.width, map_mgr.height) for p in pred2.trajectory[:horizon]]
    uncertainty1 = [float(v) for v in getattr(pred1, "uncertainty", [])]
    uncertainty2 = [float(v) for v in getattr(pred2, "uncertainty", [])]

    if not path_cells1:
        path_cells1 = [robot1.pose]
    if not path_cells2:
        path_cells2 = [robot2.pose]

    virtual_known = map_mgr.clone_known()

    virtual_sensor_range_cfg = rollout_cfg.get("virtual_sensor_range")
    sensor_range = int(
        cfg["robots"].get("sensor_range", 6) if virtual_sensor_range_cfg is None else virtual_sensor_range_cfg
    )
    virtual_sensor_fov_cfg = rollout_cfg.get("virtual_sensor_fov_deg")
    sensor_fov = float(
        cfg["robots"].get("sensor_fov_deg", 360.0) if virtual_sensor_fov_cfg is None else virtual_sensor_fov_cfg
    )
    use_virtual_los = bool(rollout_cfg.get("virtual_use_line_of_sight", cfg["robots"].get("use_line_of_sight", True)))
    unknown_blocks_los = bool(rollout_cfg.get("virtual_unknown_blocks_los", False))
    shared_gain_ratio = float(rollout_cfg.get("teammate_shared_gain_ratio", 0.5))
    shared_gain_ratio = max(0.0, min(1.0, shared_gain_ratio))

    penalties = cfg["planning"]["penalties"]
    sigma = penalties.get("sigma_overlap")
    if sigma is None:
        sigma = 2.0 * float(cfg["robots"].get("sensor_range", 6))
    sigma = float(sigma)
    lambda_overlap = float(penalties.get("lambda_overlap", 0.0))

    lambda_corridor = float(rollout_cfg.get("lambda_corridor_occupancy", 0.0))
    lambda_blocking = float(rollout_cfg.get("lambda_narrow_blocking", 0.0))
    lambda_crossing = float(rollout_cfg.get("lambda_path_crossing", 0.0))
    lambda_waiting = float(rollout_cfg.get("lambda_waiting_time", 0.0))
    lambda_uncertainty_risk = float(rollout_cfg.get("lambda_uncertainty_risk", 0.0))
    uncertainty_gain_discount = float(rollout_cfg.get("uncertainty_gain_discount", 0.0))
    uncertainty_clip = float(rollout_cfg.get("uncertainty_clip", 2.0))

    corridor_near_distance = float(rollout_cfg.get("corridor_near_distance", 2.5))
    blocking_window = int(rollout_cfg.get("blocking_window", 2))
    waiting_window = int(rollout_cfg.get("waiting_window", 2))

    used_targets: set[Cell] = set(c for c in (goal1, goal2) if c is not None)

    reassign_count_r1 = 0
    reassign_count_r2 = 0
    chosen_targets_r1: set[Cell] = set()
    chosen_targets_r2: set[Cell] = set()

    if reassign_on_reach:
        coupling_passes = max(1, int(rollout_cfg.get("reassign_coupling_passes", 2)))
        for _ in range(coupling_passes):
            path_cells1, c1, new_targets_1 = _rewrite_future_path_on_reach(
                path_cells=path_cells1,
                initial_goal=goal1,
                robot_id=robot1.robot_id,
                candidates=candidates,
                virtual_known=virtual_known,
                map_mgr=map_mgr,
                reveal_radius=reveal_radius,
                used_targets=used_targets,
                teammate_future_path=path_cells2,
                teammate_reserved_targets=chosen_targets_r2 | ({goal2} if goal2 is not None else set()),
                reservation_state=reservation_state,
                cfg=cfg,
            )
            chosen_targets_r1.update(new_targets_1)
            reassign_count_r1 += c1

            path_cells2, c2, new_targets_2 = _rewrite_future_path_on_reach(
                path_cells=path_cells2,
                initial_goal=goal2,
                robot_id=robot2.robot_id,
                candidates=candidates,
                virtual_known=virtual_known,
                map_mgr=map_mgr,
                reveal_radius=reveal_radius,
                used_targets=used_targets,
                teammate_future_path=path_cells1,
                teammate_reserved_targets=chosen_targets_r1 | ({goal1} if goal1 is not None else set()),
                reservation_state=reservation_state,
                cfg=cfg,
            )
            chosen_targets_r2.update(new_targets_2)
            reassign_count_r2 += c2

    path_cells1 = _pad_path(path_cells1, horizon)
    path_cells2 = _pad_path(path_cells2, horizon)

    corridor_raw = corridor_occupancy_penalty(
        path1=path_cells1,
        path2=path_cells2,
        map_mgr=map_mgr,
        known_grid=virtual_known,
        near_distance=corridor_near_distance,
    )
    blocking_raw = narrow_passage_blocking_penalty(
        path1=path_cells1,
        path2=path_cells2,
        map_mgr=map_mgr,
        known_grid=virtual_known,
        window=blocking_window,
    )
    crossing_raw = path_crossing_penalty(path_cells1, path_cells2)
    waiting_raw = waiting_time_proxy(
        path1=path_cells1,
        path2=path_cells2,
        map_mgr=map_mgr,
        known_grid=virtual_known,
        window=waiting_window,
    )

    future_congestion_penalty = (
        lambda_corridor * corridor_raw
        + lambda_blocking * blocking_raw
        + lambda_crossing * crossing_raw
        + lambda_waiting * waiting_raw
    )

    neighborhood = int(cfg.get("frontier", {}).get("neighborhood", 8))
    active_frontiers: set[Cell] = {
        c.representative
        for c in candidates
        if is_frontier_cell(map_mgr, c.representative, neighborhood=neighborhood, grid=virtual_known)
    }
    frontier_consumption_weight = float(rollout_cfg.get("frontier_consumption_weight", 0.35))
    frontier_consumption_decay = float(rollout_cfg.get("frontier_consumption_decay", 0.98))

    future_gain = 0.0
    future_overlap_penalty = 0.0
    frontier_consumption_gain = 0.0
    future_uncertainty_penalty = 0.0
    mean_uncertainty_r1 = 0.0
    mean_uncertainty_r2 = 0.0

    for t in range(horizon):
        c1 = path_cells1[t]
        c2 = path_cells2[t]

        h1 = _heading_at_step(path_cells1, t, robot1.heading_deg)
        h2 = _heading_at_step(path_cells2, t, robot2.heading_deg)

        obs1 = _virtual_observed_unknown_cells(
            map_mgr=map_mgr,
            center=c1,
            heading_deg=h1,
            sensor_range=sensor_range,
            fov_deg=sensor_fov,
            use_line_of_sight=use_virtual_los,
            virtual_known=virtual_known,
            unknown_blocks_los=unknown_blocks_los,
        )
        obs2 = _virtual_observed_unknown_cells(
            map_mgr=map_mgr,
            center=c2,
            heading_deg=h2,
            sensor_range=sensor_range,
            fov_deg=sensor_fov,
            use_line_of_sight=use_virtual_los,
            virtual_known=virtual_known,
            unknown_blocks_los=unknown_blocks_los,
        )

        shared_obs = obs1 & obs2
        g1_raw = float(len(obs1 - shared_obs)) + shared_gain_ratio * float(len(shared_obs))
        g2_raw = float(len(obs2 - shared_obs)) + shared_gain_ratio * float(len(shared_obs))

        u1 = 0.0
        u2 = 0.0
        if uncertainty1:
            u1 = float(uncertainty1[min(t, len(uncertainty1) - 1)])
        if uncertainty2:
            u2 = float(uncertainty2[min(t, len(uncertainty2) - 1)])
        if uncertainty_clip > 0.0:
            u1 = min(uncertainty_clip, max(0.0, u1))
            u2 = min(uncertainty_clip, max(0.0, u2))
        mean_uncertainty_r1 += u1
        mean_uncertainty_r2 += u2

        gain_w1 = math.exp(-uncertainty_gain_discount * u1) if uncertainty_gain_discount > 0.0 else 1.0
        gain_w2 = math.exp(-uncertainty_gain_discount * u2) if uncertainty_gain_discount > 0.0 else 1.0
        g1 = g1_raw * gain_w1
        g2 = g2_raw * gain_w2

        w = (gamma**t) * (gain_decay**t)
        future_gain += w * (g1 + g2)

        overlap = _future_overlap_proxy(c1, c2, sigma)
        future_overlap_penalty += w * lambda_overlap * overlap
        future_uncertainty_penalty += w * lambda_uncertainty_risk * (u1 + u2)

        _mark_virtual_revealed_cells(virtual_known, obs1 | obs2)

        if frontier_consumption_weight > 0.0 and active_frontiers:
            next_active: set[Cell] = set()
            consumed = 0
            for rep in active_frontiers:
                if is_frontier_cell(map_mgr, rep, neighborhood=neighborhood, grid=virtual_known):
                    next_active.add(rep)
                else:
                    consumed += 1
            if consumed > 0:
                frontier_consumption_gain += (
                    w * (frontier_consumption_decay**t) * frontier_consumption_weight * float(consumed)
                )
            active_frontiers = next_active

    if horizon > 0:
        mean_uncertainty_r1 /= float(horizon)
        mean_uncertainty_r2 /= float(horizon)

    future_score = (
        future_gain
        + frontier_consumption_gain
        - future_overlap_penalty
        - future_congestion_penalty
        - future_uncertainty_penalty
    )

    return RolloutResult(
        future_score=float(future_score),
        predicted_paths={robot1.robot_id: path_cells1, robot2.robot_id: path_cells2},
        predictor_inference_times={
            robot1.robot_id: pred1.inference_time_sec,
            robot2.robot_id: pred2.inference_time_sec,
        },
        breakdown={
            "future_gain": float(future_gain),
            "frontier_consumption_gain": float(frontier_consumption_gain),
            "future_overlap_penalty": float(future_overlap_penalty),
            "corridor_occupancy_raw": float(corridor_raw),
            "narrow_blocking_raw": float(blocking_raw),
            "path_crossing_raw": float(crossing_raw),
            "waiting_time_raw": float(waiting_raw),
            "future_corridor_occupancy_penalty": float(lambda_corridor * corridor_raw),
            "future_narrow_blocking_penalty": float(lambda_blocking * blocking_raw),
            "future_path_crossing_penalty": float(lambda_crossing * crossing_raw),
            "future_waiting_penalty": float(lambda_waiting * waiting_raw),
            "future_congestion_penalty": float(future_congestion_penalty),
            "future_uncertainty_penalty": float(future_uncertainty_penalty),
            "mean_uncertainty_r1": float(mean_uncertainty_r1),
            "mean_uncertainty_r2": float(mean_uncertainty_r2),
            "future_score": float(future_score),
            "virtual_reassign_count_r1": float(reassign_count_r1),
            "virtual_reassign_count_r2": float(reassign_count_r2),
            "virtual_reassign_count_total": float(reassign_count_r1 + reassign_count_r2),
            "shared_gain_ratio": float(shared_gain_ratio),
            "virtual_frontiers_remaining": float(len(active_frontiers)),
        },
    )
