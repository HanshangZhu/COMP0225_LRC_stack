from __future__ import annotations

from .base_planner import BasePlanner
from .active_slam_explorer_planner import ActiveSLAMExplorerPlanner
from .cfpa2_planner import CFPA2Planner
from .hybrid_explorer_planner import HybridExplorerPlanner
from .macro_frontier_explorer_planner import MacroFrontierExplorerPlanner
from .physics_rh_cfpa2_planner import PhysicsRHCFPA2Planner
from .rh_cfpa2_planner import RHCFPA2Planner
from .single_robot_frontier_planner import SingleRobotFrontierPlanner
from .vlm_planner import VLMPlanner


def build_planner(cfg: dict) -> BasePlanner:
    planner_name = str(cfg.get("planning", {}).get("planner_name", "cfpa2"))
    if planner_name == "cfpa2":
        return CFPA2Planner()
    if planner_name == "active_slam_explorer":
        return ActiveSLAMExplorerPlanner()
    if planner_name == "macro_frontier_explorer":
        return MacroFrontierExplorerPlanner()
    if planner_name == "rh_cfpa2":
        return RHCFPA2Planner(cfg)
    if planner_name == "physics_rh_cfpa2":
        return PhysicsRHCFPA2Planner(cfg)
    if planner_name == "hybrid_explorer":
        return HybridExplorerPlanner(cfg)
    if planner_name == "single_robot_frontier":
        return SingleRobotFrontierPlanner()
    if planner_name == "vlm_frontier":
        return VLMPlanner(cfg)
    raise ValueError(f"Unsupported planner_name: {planner_name}")


__all__ = [
    "BasePlanner",
    "ActiveSLAMExplorerPlanner",
    "CFPA2Planner",
    "HybridExplorerPlanner",
    "MacroFrontierExplorerPlanner",
    "RHCFPA2Planner",
    "PhysicsRHCFPA2Planner",
    "SingleRobotFrontierPlanner",
    "VLMPlanner",
    "build_planner",
]
