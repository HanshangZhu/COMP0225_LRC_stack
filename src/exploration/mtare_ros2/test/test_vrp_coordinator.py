from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def _load_coordinator_module():
    scripts_dir = Path(__file__).resolve().parents[1] / "scripts"
    sys.path.insert(0, str(scripts_dir))
    module_path = scripts_dir / "mtare_coordinator.py"
    spec = importlib.util.spec_from_file_location("mtare_coordinator", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load module from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_select_first_route_goals_returns_assignments():
    mod = _load_coordinator_module()
    namespaces = ["robot_a", "robot_b"]
    routes = {0: [0, 1], 1: [2]}
    cells = [
        (0.2, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (8.0, 0.0, 0.0),
    ]
    robot_xy = {"robot_a": (0.0, 0.0), "robot_b": (10.0, 0.0)}

    goals = mod.select_first_route_goals(
        namespaces=namespaces,
        routes=routes,
        exploring_cells=cells,
        robot_xy=robot_xy,
        min_assign_distance=0.5,
    )

    assert goals["robot_a"] == (2.0, 0.0)
    assert goals["robot_b"] == (8.0, 0.0)


def test_select_first_route_goals_handles_missing_routes():
    mod = _load_coordinator_module()
    namespaces = ["robot_a", "robot_b"]
    routes = {0: [0]}
    cells = [(1.0, 1.0, 0.0)]
    robot_xy = {"robot_a": (0.0, 0.0), "robot_b": (2.0, 2.0)}

    goals = mod.select_first_route_goals(
        namespaces=namespaces,
        routes=routes,
        exploring_cells=cells,
        robot_xy=robot_xy,
        min_assign_distance=0.1,
    )

    assert goals == {"robot_a": (1.0, 1.0)}
