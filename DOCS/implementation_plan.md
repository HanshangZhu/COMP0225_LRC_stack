# Coverage Metric, Ghost Fix, C++ Acceleration, and Parameter Optimization

## Phase 1: Ghost Particles Fix

**Problem:** Noisy scan hits → false occupied cells in corridors → planner routes in messy circles.

**Fix:** Increase `miss_decrement` 1→2 in [simple_scan_mapper_single_go2w.yaml](file:///home/hz/COMP0225_LRC_stack/config/nav/simple_scan_mapper_single_go2w.yaml). Single-scan noise decays in ~2 sweeps; real walls persist. YAML-only change, no rebuild.

---

## Phase 2: C++ Acceleration of CFPA2 Hot Paths

##### [NEW] [cfpa2_grid_ops.cpp](file:///home/hz/COMP0225_LRC_stack/src/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_grid_ops.cpp)

Three `extern "C"` functions (same ctypes pattern as [astar_grid.cpp](file:///home/hz/COMP0225_LRC_stack/src/go2w_control/scripts/reactive_nav_core/astar_grid.cpp)):

| Function | What it replaces | Complexity | Expected Speedup |
|---|---|---|---|
| [extract_frontiers()](file:///home/hz/COMP0225_LRC_stack/src/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_coordinator_node.py#633-705) | [_extract_frontiers](file:///home/hz/COMP0225_LRC_stack/src/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_coordinator_node.py#633-705) (Python grid scan + BFS) | O(W×H) | 50-100× |
| [distance_transform()](file:///home/hz/COMP0225_LRC_stack/src/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_coordinator_node.py#706-758) | [_distance_transform](file:///home/hz/COMP0225_LRC_stack/src/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_coordinator_node.py#706-758) (Python BFS → dict) | O(W×H) | 30-60× |
| `batch_info_gain()` | [_frontier_information_gain](file:///home/hz/COMP0225_LRC_stack/src/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_coordinator_node.py#1529-1546) (per-frontier) | O(R²×N) | 30-50× |

##### [MODIFY] [cfpa2_coordinator_node.py](file:///home/hz/COMP0225_LRC_stack/src/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_coordinator_node.py)

Replace Python implementations with ctypes calls; fallback to Python if [.so](file:///home/hz/COMP0225_LRC_stack/src/go2w_control/scripts/reactive_nav_core/astar_grid.so) missing.

**Target:** p95 tick time from ~50ms → <15ms.

---

## Phase 3: Ground-Truth Visibility Coverage

**Approach:** Alignment-free coverage using ground-truth positions (p3d odom) + raycasting.

```
coverage = seen_gt_free_cells / total_gt_free_cells
```

- Robot's GT position each tick → raycast within sensor_range → mark GT-free cells as "seen"
- Independent of SLAM quality — measures physical sensor illumination
- Cannot game it with SLAM-drift "fake openspace"

##### [NEW] [gt_coverage.cpp](file:///home/hz/COMP0225_LRC_stack/src/go2w_control/scripts/reactive_nav_core/gt_coverage.cpp)

C++ raycast + grid tracking. Functions: [init(walls)](file:///home/hz/COMP0225_LRC_stack/src/go2w_control/scripts/reactive_nav_core/coordinator.py#14-20), [update(robot_x, robot_y, range)](file:///home/hz/COMP0225_LRC_stack/src/go2w_control/scripts/reactive_nav_core/dstar_lite.py#258-268), `query() → (seen, total)`.

##### [MODIFY] [exploration_metrics_logger.py](file:///home/hz/COMP0225_LRC_stack/src/go2w_control/scripts/exploration_metrics_logger.py)

Log `gt_coverage_pct` each tick. Trigger "complete" at ≥98%.

---

## Phase 4: Utility Parameter Optimization

**Goal:** Find the parameter set that achieves **≥98% GT coverage** in **shortest wall time** with **zero wall collisions** (no wheel/chassis contact), under a **110s timeout**.

### Parameter Space

| Parameter | Range | Current |
|---|---|---|
| `cfpa2_w_ig` | 0.3 – 2.0 | 0.5 |
| `cfpa2_w_c` | 0.3 – 3.0 | 0.8 |
| `cfpa2_w_momentum` | 0.5 – 4.0 | 2.5 |
| `cfpa2_min_utility` | -3.0 – 0.0 | -1.0 |
| `planner_inflation_radius` | 0.35 – 0.55 | 0.40 |
| `planner_safety_clearance` | 0.35 – 0.60 | 0.50 |
| `obstacle_stop_dist` | 0.25 – 0.45 | 0.35 |

### Search Strategy

1. **Baseline run** (3×) with current params → establish baseline time + coverage
2. **Coarse grid sweep** (12 configs × 1 run each) — vary one param at a time from baseline
3. **Top-3 configs** from coarse sweep → **3 runs each** to confirm
4. **Winner** → **5 final validation runs** to confirm reliability

Total: ≥ **20 runs** (3 + 12 + 9 + 5 = 29 runs minimum)

### Automated Test Harness

##### [NEW] [sweep_test.sh](file:///home/hz/COMP0225_LRC_stack/sweep_test.sh)

```bash
# For each config: launch headless with 110s timeout, grep PASS/FAIL + coverage + time
# Output: CSV of (config_id, coverage_pct, time_sec, wall_hits, backtracks, pass/fail)
```

### Pass Criteria (per run)

- `gt_coverage ≥ 98%` OR `fronts=0` (map fully explored)
- `wall_hits = 0` (no wheel/chassis contact)
- `time ≤ 110s` (sim time)
- Robot NOT stuck (displacement > 30m)

### Output

Final [demo.sh](file:///home/hz/COMP0225_LRC_stack/demo.sh) updated with winning parameter set + documented rationale.

---

## Execution Order

1. **Phase 1** — Ghost fix (5 min, YAML change)
2. **Phase 2** — C++ acceleration (needed before Phase 4's 20+ runs)
3. **Phase 3** — GT coverage metric (needed for Phase 4's pass criteria)
4. **Phase 4** — Parameter sweep (20+ automated runs)
