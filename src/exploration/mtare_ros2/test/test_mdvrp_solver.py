from __future__ import annotations

import importlib.util
from pathlib import Path


def _load_module():
    module_path = Path(__file__).resolve().parents[1] / "scripts" / "mdvrp_solver.py"
    spec = importlib.util.spec_from_file_location("mdvrp_solver", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load module from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_build_mdvrp_distance_matrix_shape():
    mod = _load_module()
    cells = [(0.0, 0.0, 0.0), (3.0, 4.0, 0.0)]
    robots = [(1.0, 1.0, 0.0)]
    matrix = mod.build_mdvrp_distance_matrix(cells, robots, scale=100.0)
    assert len(matrix) == 3
    assert all(len(row) == 3 for row in matrix)
    assert matrix[0][0] == 0
    assert matrix[1][1] == 0
    assert matrix[2][2] == 0
    assert matrix[0][1] == matrix[1][0]


def test_solve_mdvrp_balanced_non_overlapping_routes():
    mod = _load_module()

    cells = [
        (0.0, 0.0, 0.0),
        (0.0, 2.0, 0.0),
        (0.0, 4.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 2.0, 0.0),
        (10.0, 4.0, 0.0),
    ]
    robots = [(0.0, 1.0, 0.0), (10.0, 1.0, 0.0)]
    matrix = mod.build_mdvrp_distance_matrix(cells, robots, scale=100.0)
    routes = mod.solve_mdvrp(cells, robots, matrix, time_limit_sec=1.0)

    assert set(routes.keys()) == {0, 1}
    assigned = [cell for route in routes.values() for cell in route]
    assert len(assigned) == len(cells)
    assert len(set(assigned)) == len(cells)
    assert all(0 <= cell < len(cells) for cell in assigned)
    assert abs(len(routes[0]) - len(routes[1])) <= 1
