from __future__ import annotations

from core.config import deep_merge
from predictors import build_predictor

from .rh_cfpa2_planner import RHCFPA2Planner


class PhysicsRHCFPA2Planner(RHCFPA2Planner):
    name = "physics_rh_cfpa2"

    def __init__(self, cfg: dict):
        physics_override = {
            "predictor": {
                "type": "physics_residual",
                "physics_residual": {
                    "enabled": True,
                },
            }
        }
        merged = deep_merge(cfg, physics_override)
        super().__init__(merged)
        self.predictor = build_predictor(merged)
