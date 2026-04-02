from __future__ import annotations

import numpy as np

FREE = 0
OCCUPIED = 1


def _rng(seed: int) -> np.random.Generator:
    return np.random.default_rng(seed)


def _add_boundaries(grid: np.ndarray) -> None:
    grid[0, :] = OCCUPIED
    grid[-1, :] = OCCUPIED
    grid[:, 0] = OCCUPIED
    grid[:, -1] = OCCUPIED


def _carve_corridor(grid: np.ndarray, p0: tuple[int, int], p1: tuple[int, int], width: int = 2) -> None:
    x0, y0 = p0
    x1, y1 = p1
    if x0 == x1:
        x_min = max(1, x0 - width)
        x_max = min(grid.shape[1] - 2, x0 + width)
        y_min = min(y0, y1)
        y_max = max(y0, y1)
        grid[y_min : y_max + 1, x_min : x_max + 1] = FREE
    elif y0 == y1:
        y_min = max(1, y0 - width)
        y_max = min(grid.shape[0] - 2, y0 + width)
        x_min = min(x0, x1)
        x_max = max(x0, x1)
        grid[y_min : y_max + 1, x_min : x_max + 1] = FREE


def generate_corridor_maze(width: int, height: int, obstacle_density: float, seed: int) -> np.ndarray:
    rng = _rng(seed)
    grid = np.full((height, width), OCCUPIED, dtype=np.int8)
    _add_boundaries(grid)

    x_nodes = list(range(4, width - 4, max(8, width // 10)))
    y_nodes = list(range(4, height - 4, max(8, height // 8)))

    # Carve a connected corridor graph with bottlenecks and T-junction style turns.
    for y in y_nodes:
        _carve_corridor(grid, (x_nodes[0], y), (x_nodes[-1], y), width=1)

    for x in x_nodes:
        _carve_corridor(grid, (x, y_nodes[0]), (x, y_nodes[-1]), width=1)

    # Create dead-ends and side branches.
    for _ in range(max(6, len(x_nodes) + len(y_nodes))):
        x = int(rng.choice(x_nodes))
        y = int(rng.choice(y_nodes))
        length = int(rng.integers(4, 12))
        direction = int(rng.integers(0, 4))
        if direction == 0:
            _carve_corridor(grid, (x, y), (min(width - 2, x + length), y), width=1)
        elif direction == 1:
            _carve_corridor(grid, (x, y), (max(1, x - length), y), width=1)
        elif direction == 2:
            _carve_corridor(grid, (x, y), (x, min(height - 2, y + length)), width=1)
        else:
            _carve_corridor(grid, (x, y), (x, max(1, y - length)), width=1)

    # Add a few random internal blockers for bottleneck-heavy behavior.
    free_cells = np.argwhere(grid == FREE)
    n_blockers = int(obstacle_density * 0.15 * len(free_cells))
    if n_blockers > 0:
        picks = free_cells[rng.choice(len(free_cells), size=n_blockers, replace=False)]
        for y, x in picks:
            if x in (1, width - 2) or y in (1, height - 2):
                continue
            if rng.random() < 0.6:
                grid[y, x] = OCCUPIED

    # Guarantee a known-free launch zone.
    grid[2:10, 2:14] = FREE
    _add_boundaries(grid)
    return grid


def generate_bottleneck_rooms(width: int, height: int, obstacle_density: float, seed: int) -> np.ndarray:
    rng = _rng(seed)
    grid = np.full((height, width), FREE, dtype=np.int8)
    _add_boundaries(grid)

    room_w = max(10, width // 5)
    room_h = max(10, height // 4)

    v_walls = list(range(room_w, width - 1, room_w))
    h_walls = list(range(room_h, height - 1, room_h))

    for x in v_walls:
        grid[:, x] = OCCUPIED
    for y in h_walls:
        grid[y, :] = OCCUPIED

    # Narrow doors create realistic bottlenecks.
    for x in v_walls:
        for y0, y1 in zip([0] + h_walls, h_walls + [height - 1]):
            if y1 - y0 < 4:
                continue
            door_y = int(rng.integers(y0 + 2, y1 - 1))
            grid[door_y - 1 : door_y + 1, x] = FREE

    for y in h_walls:
        for x0, x1 in zip([0] + v_walls, v_walls + [width - 1]):
            if x1 - x0 < 4:
                continue
            door_x = int(rng.integers(x0 + 2, x1 - 1))
            grid[y, door_x - 1 : door_x + 1] = FREE

    interior = np.argwhere(grid[1:-1, 1:-1] == FREE)
    n_noise = int(obstacle_density * 0.10 * len(interior))
    if n_noise > 0:
        picks = interior[rng.choice(len(interior), size=n_noise, replace=False)]
        for y, x in picks:
            grid[y + 1, x + 1] = OCCUPIED

    grid[2:10, 2:16] = FREE
    _add_boundaries(grid)
    return grid


def generate_branching_deadend(width: int, height: int, obstacle_density: float, seed: int) -> np.ndarray:
    rng = _rng(seed)
    grid = np.full((height, width), OCCUPIED, dtype=np.int8)
    _add_boundaries(grid)

    # Depth-first maze backbone.
    if width % 2 == 0:
        width -= 1
    if height % 2 == 0:
        height -= 1

    local = np.full((height, width), OCCUPIED, dtype=np.int8)

    def neighbors(cx: int, cy: int):
        for dx, dy in ((2, 0), (-2, 0), (0, 2), (0, -2)):
            nx, ny = cx + dx, cy + dy
            if 1 <= nx < width - 1 and 1 <= ny < height - 1:
                yield nx, ny, dx, dy

    start = (1, 1)
    stack = [start]
    visited = {start}
    local[1, 1] = FREE
    while stack:
        cx, cy = stack[-1]
        cand = [(nx, ny, dx, dy) for nx, ny, dx, dy in neighbors(cx, cy) if (nx, ny) not in visited]
        if not cand:
            stack.pop()
            continue
        nx, ny, dx, dy = cand[int(rng.integers(0, len(cand)))]
        local[cy + dy // 2, cx + dx // 2] = FREE
        local[ny, nx] = FREE
        visited.add((nx, ny))
        stack.append((nx, ny))

    # Add sparse loops and dead-end blockers.
    walls = np.argwhere(local == OCCUPIED)
    n_open = max(1, int(0.02 * len(walls)))
    for y, x in walls[rng.choice(len(walls), size=n_open, replace=False)]:
        if 0 < x < width - 1 and 0 < y < height - 1:
            local[y, x] = FREE

    free_cells = np.argwhere(local == FREE)
    n_close = int(obstacle_density * 0.06 * len(free_cells))
    if n_close > 0:
        for y, x in free_cells[rng.choice(len(free_cells), size=n_close, replace=False)]:
            if x > 2 and y > 2:
                local[y, x] = OCCUPIED

    grid[: local.shape[0], : local.shape[1]] = local
    grid[2:10, 2:14] = FREE
    _add_boundaries(grid)
    return grid


def generate_narrow_t_branches(width: int, height: int, obstacle_density: float, seed: int) -> np.ndarray:
    """Generate a narrow T-shaped corridor map with side branches."""
    rng = _rng(seed)
    grid = np.full((height, width), OCCUPIED, dtype=np.int8)
    _add_boundaries(grid)

    cx = width // 2
    y_bottom = height - 6
    y_t = max(12, height // 3)

    # Main vertical trunk of the T.
    _carve_corridor(grid, (cx, y_bottom), (cx, y_t), width=0)
    # Top horizontal cap of the T.
    x_left = max(4, width // 8)
    x_right = min(width - 5, width - width // 8)
    _carve_corridor(grid, (x_left, y_t), (x_right, y_t), width=0)

    # Slightly wider launch bay for two robots near trunk bottom.
    bay_y0 = max(2, y_bottom - 3)
    bay_y1 = min(height - 3, y_bottom + 1)
    bay_x0 = max(2, cx - 3)
    bay_x1 = min(width - 3, cx + 3)
    grid[bay_y0 : bay_y1 + 1, bay_x0 : bay_x1 + 1] = FREE

    # Side branches along the trunk, alternating left/right with random length.
    branch_rows = list(range(y_t + 5, y_bottom - 4, 6))
    for i, by in enumerate(branch_rows):
        length = int(rng.integers(8, max(9, width // 5)))
        if i % 2 == 0:
            _carve_corridor(grid, (cx, by), (max(2, cx - length), by), width=0)
        else:
            _carve_corridor(grid, (cx, by), (min(width - 3, cx + length), by), width=0)

        # Add short dead-end twigs to make side corridors less regular.
        twig_len = int(rng.integers(3, 8))
        if rng.random() < 0.5:
            bx = max(2, cx - length) if i % 2 == 0 else min(width - 3, cx + length)
            _carve_corridor(grid, (bx, by), (bx, min(height - 3, by + twig_len)), width=0)
        else:
            bx = max(2, cx - length) if i % 2 == 0 else min(width - 3, cx + length)
            _carve_corridor(grid, (bx, by), (bx, max(2, by - twig_len)), width=0)

    # Branches dropping from the T top bar.
    top_branch_x = list(range(x_left + 6, x_right - 5, max(7, width // 14)))
    for i, bx in enumerate(top_branch_x):
        length = int(rng.integers(6, max(7, height // 4)))
        end_y = min(height - 4, y_t + length)
        _carve_corridor(grid, (bx, y_t), (bx, end_y), width=0)

        # Small horizontal fork near branch end.
        if i % 2 == 0 and end_y + 1 < height - 2:
            fork = int(rng.integers(3, 8))
            _carve_corridor(grid, (bx, end_y), (max(2, bx - fork), end_y), width=0)
        elif end_y + 1 < height - 2:
            fork = int(rng.integers(3, 8))
            _carve_corridor(grid, (bx, end_y), (min(width - 3, bx + fork), end_y), width=0)

    # Optional sparse blockers along long corridors to mimic clutter.
    free_cells = np.argwhere(grid == FREE)
    n_blockers = int(obstacle_density * 0.05 * len(free_cells))
    if n_blockers > 0:
        picks = free_cells[rng.choice(len(free_cells), size=n_blockers, replace=False)]
        for y, x in picks:
            # Keep trunk and top bar passable.
            if x == cx or y == y_t:
                continue
            if x in (0, width - 1) or y in (0, height - 1):
                continue
            if rng.random() < 0.35:
                grid[y, x] = OCCUPIED

    _add_boundaries(grid)
    return grid


def generate_narrow_t_dense_branches(width: int, height: int, obstacle_density: float, seed: int) -> np.ndarray:
    """Generate a denser T-corridor maze with frequent short branches and dead-ends."""
    rng = _rng(seed)
    grid = generate_narrow_t_branches(width, height, obstacle_density=0.0, seed=seed)

    cx = width // 2
    y_bottom = height - 6
    y_t = max(12, height // 3)
    branch_rows = list(range(y_t + 4, y_bottom - 4, 4))

    for by in branch_rows:
        left_len = int(rng.integers(8, max(10, width // 4)))
        right_len = int(rng.integers(8, max(10, width // 4)))
        _carve_corridor(grid, (cx, by), (max(2, cx - left_len), by), width=0)
        _carve_corridor(grid, (cx, by), (min(width - 3, cx + right_len), by), width=0)

        if rng.random() < 0.8:
            x_left = max(2, cx - left_len)
            twig = int(rng.integers(3, 7))
            _carve_corridor(grid, (x_left, by), (x_left, max(2, by - twig)), width=0)
        if rng.random() < 0.8:
            x_right = min(width - 3, cx + right_len)
            twig = int(rng.integers(3, 7))
            _carve_corridor(grid, (x_right, by), (x_right, min(height - 3, by + twig)), width=0)

    cap_rows = [y_t + 6, y_t + 12]
    for idx, y_loop in enumerate(cap_rows):
        if y_loop >= height - 6:
            continue
        left_x = max(4, width // 7)
        right_x = min(width - 5, width - width // 7)
        _carve_corridor(grid, (left_x, y_loop), (right_x, y_loop), width=0)
        if idx == 0:
            _carve_corridor(grid, (left_x, y_t), (left_x, y_loop), width=0)
            _carve_corridor(grid, (right_x, y_t), (right_x, y_loop), width=0)

    free_cells = np.argwhere(grid == FREE)
    n_blockers = int(obstacle_density * 0.03 * len(free_cells))
    if n_blockers > 0:
        picks = free_cells[rng.choice(len(free_cells), size=n_blockers, replace=False)]
        for y, x in picks:
            if x == cx or y in (y_t, y_t + 6, y_t + 12):
                continue
            if rng.random() < 0.25:
                grid[y, x] = OCCUPIED

    _add_boundaries(grid)
    return grid


def generate_narrow_t_asymmetric_branches(width: int, height: int, obstacle_density: float, seed: int) -> np.ndarray:
    """Generate an asymmetric T-corridor map with uneven side-branch density."""
    rng = _rng(seed)
    grid = np.full((height, width), OCCUPIED, dtype=np.int8)
    _add_boundaries(grid)

    cx = width // 2
    y_bottom = height - 6
    y_t = max(13, height // 3)

    _carve_corridor(grid, (cx, y_bottom), (cx, y_t), width=0)

    x_left = max(4, width // 6)
    x_right = min(width - 5, width - width // 10)
    _carve_corridor(grid, (x_left, y_t), (x_right, y_t), width=0)

    bay_y0 = max(2, y_bottom - 3)
    bay_y1 = min(height - 3, y_bottom + 1)
    grid[bay_y0 : bay_y1 + 1, cx - 3 : cx + 4] = FREE

    left_rows = list(range(y_t + 4, y_bottom - 4, 5))
    right_rows = list(range(y_t + 6, y_bottom - 4, 8))

    for by in left_rows:
        left_len = int(rng.integers(max(10, width // 7), max(14, width // 3)))
        end_x = max(2, cx - left_len)
        _carve_corridor(grid, (cx, by), (end_x, by), width=0)
        if rng.random() < 0.85:
            twig = int(rng.integers(3, 8))
            _carve_corridor(grid, (end_x, by), (end_x, max(2, by - twig)), width=0)

    for by in right_rows:
        right_len = int(rng.integers(6, max(8, width // 5)))
        end_x = min(width - 3, cx + right_len)
        _carve_corridor(grid, (cx, by), (end_x, by), width=0)
        if rng.random() < 0.55:
            twig = int(rng.integers(3, 7))
            _carve_corridor(grid, (end_x, by), (end_x, min(height - 3, by + twig)), width=0)

    top_branch_x = list(range(x_left + 5, x_right - 4, max(8, width // 12)))
    for i, bx in enumerate(top_branch_x):
        drop = int(rng.integers(5, max(7, height // 4)))
        end_y = min(height - 4, y_t + drop)
        _carve_corridor(grid, (bx, y_t), (bx, end_y), width=0)
        fork = int(rng.integers(3, 7))
        if i % 2 == 0:
            _carve_corridor(grid, (bx, end_y), (max(2, bx - fork), end_y), width=0)
        else:
            _carve_corridor(grid, (bx, end_y), (min(width - 3, bx + fork), end_y), width=0)

    free_cells = np.argwhere(grid == FREE)
    n_blockers = int(obstacle_density * 0.04 * len(free_cells))
    if n_blockers > 0:
        picks = free_cells[rng.choice(len(free_cells), size=n_blockers, replace=False)]
        for y, x in picks:
            if x == cx or y == y_t:
                continue
            if rng.random() < 0.25:
                grid[y, x] = OCCUPIED

    _add_boundaries(grid)
    return grid


def generate_narrow_t_loop_branches(width: int, height: int, obstacle_density: float, seed: int) -> np.ndarray:
    """Generate a T-corridor map with a narrow upper loop and branch pockets."""
    rng = _rng(seed)
    grid = generate_narrow_t_branches(width, height, obstacle_density=0.0, seed=seed)

    cx = width // 2
    y_t = max(12, height // 3)
    loop_y = min(height - 8, y_t + max(8, height // 7))
    left_x = max(4, width // 7)
    right_x = min(width - 5, width - width // 7)

    _carve_corridor(grid, (left_x, loop_y), (right_x, loop_y), width=0)
    _carve_corridor(grid, (left_x, y_t), (left_x, loop_y), width=0)
    _carve_corridor(grid, (right_x, y_t), (right_x, loop_y), width=0)

    connector_xs = [max(6, cx - width // 8), min(width - 7, cx + width // 8)]
    for bx in connector_xs:
        drop = int(rng.integers(5, max(7, height // 5)))
        end_y = min(height - 4, loop_y + drop)
        _carve_corridor(grid, (bx, loop_y), (bx, end_y), width=0)
        branch = int(rng.integers(4, 8))
        if bx < cx:
            _carve_corridor(grid, (bx, end_y), (max(2, bx - branch), end_y), width=0)
        else:
            _carve_corridor(grid, (bx, end_y), (min(width - 3, bx + branch), end_y), width=0)

    lower_rows = list(range(loop_y + 5, height - 10, 6))
    for i, by in enumerate(lower_rows):
        span = int(rng.integers(7, max(8, width // 5)))
        if i % 2 == 0:
            _carve_corridor(grid, (cx, by), (max(2, cx - span), by), width=0)
        else:
            _carve_corridor(grid, (cx, by), (min(width - 3, cx + span), by), width=0)

    free_cells = np.argwhere(grid == FREE)
    n_blockers = int(obstacle_density * 0.03 * len(free_cells))
    if n_blockers > 0:
        picks = free_cells[rng.choice(len(free_cells), size=n_blockers, replace=False)]
        for y, x in picks:
            if x in (cx, left_x, right_x) or y in (y_t, loop_y):
                continue
            if rng.random() < 0.22:
                grid[y, x] = OCCUPIED

    _add_boundaries(grid)
    return grid


def generate_sharp_turn_corridor(width: int, height: int, obstacle_density: float, seed: int) -> np.ndarray:
    """Generate zig-zag narrow corridors with repeated sharp turns."""
    rng = _rng(seed)
    grid = np.full((height, width), OCCUPIED, dtype=np.int8)
    _add_boundaries(grid)

    x_left = max(2, width // 5)
    x_right = min(width - 3, width - width // 5)
    y = height - 5
    direction_right = True

    while y > 4:
        x0 = x_left if direction_right else x_right
        x1 = x_right if direction_right else x_left
        _carve_corridor(grid, (x0, y), (x1, y), width=0)
        y_next = max(3, y - int(rng.integers(4, 8)))
        _carve_corridor(grid, (x1, y), (x1, y_next), width=0)
        y = y_next
        direction_right = not direction_right

    # Add short side pockets near corners to create ambiguous turning choices.
    for by in range(6, height - 6, 7):
        if rng.random() < 0.5:
            _carve_corridor(grid, (x_left, by), (max(2, x_left - int(rng.integers(3, 6))), by), width=0)
        else:
            _carve_corridor(grid, (x_right, by), (min(width - 3, x_right + int(rng.integers(3, 6))), by), width=0)

    # Sparse clutter for obstacle-near slowdowns.
    free_cells = np.argwhere(grid == FREE)
    n_blockers = int(obstacle_density * 0.06 * len(free_cells))
    if n_blockers > 0:
        picks = free_cells[rng.choice(len(free_cells), size=n_blockers, replace=False)]
        for y0, x0 in picks:
            if (x0 in (x_left, x_right)) or y0 <= 2 or y0 >= height - 3:
                continue
            if rng.random() < 0.4:
                grid[y0, x0] = OCCUPIED

    # Launch area.
    grid[height - 8 : height - 3, x_left - 1 : x_left + 4] = FREE
    _add_boundaries(grid)
    return grid


def generate_interaction_cross(width: int, height: int, obstacle_density: float, seed: int) -> np.ndarray:
    """Generate crossing-heavy map with central intersection and narrow connectors."""
    rng = _rng(seed)
    grid = np.full((height, width), OCCUPIED, dtype=np.int8)
    _add_boundaries(grid)

    cx = width // 2
    cy = height // 2

    # Main cross structure.
    _carve_corridor(grid, (2, cy), (width - 3, cy), width=0)
    _carve_corridor(grid, (cx, 2), (cx, height - 3), width=0)

    # Diagonal-like branch approximations via stepped segments.
    for k in range(4, min(width, height) // 3, 4):
        _carve_corridor(grid, (cx - k, cy - k), (cx - k, cy - k + 2), width=0)
        _carve_corridor(grid, (cx + k, cy + k), (cx + k - 2, cy + k), width=0)

    # Add interaction loops around center.
    loop_r = max(4, min(width, height) // 8)
    _carve_corridor(grid, (cx - loop_r, cy - loop_r), (cx + loop_r, cy - loop_r), width=0)
    _carve_corridor(grid, (cx + loop_r, cy - loop_r), (cx + loop_r, cy + loop_r), width=0)
    _carve_corridor(grid, (cx + loop_r, cy + loop_r), (cx - loop_r, cy + loop_r), width=0)
    _carve_corridor(grid, (cx - loop_r, cy + loop_r), (cx - loop_r, cy - loop_r), width=0)

    # Narrow side branches and dead-ends.
    for bx in range(6, width - 6, 8):
        length = int(rng.integers(4, 9))
        _carve_corridor(grid, (bx, cy), (bx, max(2, cy - length)), width=0)
        if rng.random() < 0.5:
            _carve_corridor(grid, (bx, cy), (bx, min(height - 3, cy + length)), width=0)

    # Light clutter, keep center traversable.
    free_cells = np.argwhere(grid == FREE)
    n_blockers = int(obstacle_density * 0.04 * len(free_cells))
    if n_blockers > 0:
        picks = free_cells[rng.choice(len(free_cells), size=n_blockers, replace=False)]
        for y0, x0 in picks:
            if abs(x0 - cx) <= 1 and abs(y0 - cy) <= 1:
                continue
            if rng.random() < 0.35:
                grid[y0, x0] = OCCUPIED

    # Dual launch bays at opposite sides for crossing interactions.
    grid[cy - 2 : cy + 3, 2:7] = FREE
    grid[cy - 2 : cy + 3, width - 7 : width - 2] = FREE
    _add_boundaries(grid)
    return grid


def find_dead_end_cells(
    grid: np.ndarray,
    count: int,
    rng: np.random.Generator,
    exclusion_zone: set[tuple[int, int]] | None = None,
    min_distance_from_edge: int = 2,
) -> list[tuple[int, int]]:
    """Return up to `count` free cells that are corridor dead-ends (exactly 1 free neighbour).

    Dead-ends are semantically interesting artifact locations — they sit at the tip
    of a branch and force the robot to explore the full corridor to find them.
    """
    exclusion_zone = exclusion_zone or set()
    dead_ends: list[tuple[int, int]] = []
    h, w = grid.shape
    for y in range(min_distance_from_edge, h - min_distance_from_edge):
        for x in range(min_distance_from_edge, w - min_distance_from_edge):
            if grid[y, x] != FREE:
                continue
            if (x, y) in exclusion_zone:
                continue
            free_neighbours = sum(
                1
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
                if 0 <= y + dy < h and 0 <= x + dx < w and grid[y + dy, x + dx] == FREE
            )
            if free_neighbours == 1:
                dead_ends.append((x, y))

    if not dead_ends:
        # Fallback: sample any free interior cell if no dead-ends found
        free_cells = [
            (x, y)
            for y in range(min_distance_from_edge, h - min_distance_from_edge)
            for x in range(min_distance_from_edge, w - min_distance_from_edge)
            if grid[y, x] == FREE and (x, y) not in exclusion_zone
        ]
        dead_ends = free_cells

    indices = rng.permutation(len(dead_ends)).tolist()
    return [dead_ends[i] for i in indices[:count]]


def generate_map_with_artifacts(
    map_type: str,
    width: int,
    height: int,
    obstacle_density: float,
    seed: int,
    artifact_count: int = 1,
    exclusion_zone: set[tuple[int, int]] | None = None,
) -> tuple[np.ndarray, list[tuple[int, int]]]:
    """Generate a map and choose artifact positions at corridor dead-ends.

    Returns
    -------
    (grid, artifact_positions)
        grid              : occupancy grid (same as generate_map)
        artifact_positions: list of (x, y) cells where artifacts are placed
    """
    grid = generate_map(map_type, width, height, obstacle_density, seed)
    rng = np.random.default_rng(seed + 99991)
    artifact_positions = find_dead_end_cells(
        grid, artifact_count, rng, exclusion_zone=exclusion_zone
    )
    return grid, artifact_positions


def generate_map(map_type: str, width: int, height: int, obstacle_density: float, seed: int) -> np.ndarray:
    if map_type == "corridor_maze":
        return generate_corridor_maze(width, height, obstacle_density, seed)
    if map_type == "bottleneck_rooms":
        return generate_bottleneck_rooms(width, height, obstacle_density, seed)
    if map_type == "branching_deadend":
        return generate_branching_deadend(width, height, obstacle_density, seed)
    if map_type == "narrow_t_branches":
        return generate_narrow_t_branches(width, height, obstacle_density, seed)
    if map_type == "narrow_t_dense_branches":
        return generate_narrow_t_dense_branches(width, height, obstacle_density, seed)
    if map_type == "narrow_t_asymmetric_branches":
        return generate_narrow_t_asymmetric_branches(width, height, obstacle_density, seed)
    if map_type == "narrow_t_loop_branches":
        return generate_narrow_t_loop_branches(width, height, obstacle_density, seed)
    if map_type == "sharp_turn_corridor":
        return generate_sharp_turn_corridor(width, height, obstacle_density, seed)
    if map_type == "interaction_cross":
        return generate_interaction_cross(width, height, obstacle_density, seed)
    if map_type == "open":
        grid = np.full((height, width), FREE, dtype=np.int8)
        _add_boundaries(grid)
        return grid
    raise ValueError(f"Unsupported map_type: {map_type}")
