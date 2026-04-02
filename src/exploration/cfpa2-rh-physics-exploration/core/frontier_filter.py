from __future__ import annotations

from dataclasses import dataclass

from .path_service import path_cost, plan_path
from .types import FrontierCandidate, RobotState


@dataclass
class ReachableFrontier:
    candidate: FrontierCandidate
    path: list[tuple[int, int]]
    travel_cost: float
    information_gain: float
    obstacle_count: float


def filter_reachable_frontiers(
    robot: RobotState,
    candidates: list[FrontierCandidate],
    map_mgr,
    cfg: dict,
) -> list[ReachableFrontier]:
    policy_cfg = cfg.get("planning", {}).get("active_slam_explorer", {})
    ig_radius = int(policy_cfg.get("information_gain_radius", cfg.get("frontier", {}).get("ig_radius", 6)))
    min_information_gain = float(policy_cfg.get("min_information_gain", 1.0))
    max_target_obstacles = float(policy_cfg.get("max_target_obstacles", 12.0))
    topk = int(policy_cfg.get("candidate_topk", cfg.get("planning", {}).get("topk_candidate_limit", 8)))
    clearance_radius = int(policy_cfg.get("target_obstacle_radius", 1))
    neighborhood = int(cfg.get("frontier", {}).get("neighborhood", 8))
    clearance_cells = int(cfg.get("robots", {}).get("clearance_cells", 0))

    reachable: list[ReachableFrontier] = []
    for candidate in candidates:
        rep = candidate.representative
        path = plan_path(
            map_mgr=map_mgr,
            start=robot.pose,
            goal=rep,
            cfg=cfg,
            neighborhood=neighborhood,
            clearance_cells=clearance_cells,
        )
        if path is None:
            continue

        ig = float(map_mgr.count_unknown_in_radius(rep, ig_radius))
        if ig < min_information_gain:
            continue

        obstacle_count = float(map_mgr.obstacle_count_around(rep, radius=clearance_radius))
        if obstacle_count > max_target_obstacles:
            continue

        reachable.append(
            ReachableFrontier(
                candidate=candidate,
                path=path,
                travel_cost=float(path_cost(path)),
                information_gain=ig,
                obstacle_count=obstacle_count,
            )
        )

    reachable.sort(
        key=lambda item: (
            -(item.information_gain / max(1.0, item.travel_cost)),
            item.travel_cost,
            -item.candidate.size,
        )
    )
    return reachable[: max(1, topk)] if reachable else []
