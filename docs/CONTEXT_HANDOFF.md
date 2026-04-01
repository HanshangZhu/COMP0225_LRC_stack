# Context Handoff: Go2W Gazebo Exploration Stack

**Project:** `COMP0225_LRC_stack` — Multi-robot autonomous exploration with Unitree Go2W in Gazebo.
**Date:** 2026-03-08
**Map:** `3.world` (corridors)
**Robots:** `robot_a`, `robot_b` (namespaced), Go2W variant with wheels + legs.

---

## Session Summary (2026-03-08): Fast-LIO Pipeline + Mapper Robustness + QoS Fix

### Problem 1: Wall Duplication During Turns (RESOLVED ✅)

**Symptom:** Occupancy grid showed doubled/smeared walls when robot turned.

**Root Causes:**
1. `max_clear_distance: 2.0` limited ALL free rays to 2m — old wall cells beyond 2m never cleared during turns.
2. Raw Gazebo scan had no motion compensation.
3. Symmetric scoring (hit:miss = 5:3) made walls too easy to erase.

**Fixes:**
- **Fast-LIO pipeline activated** — `cloud_registered_body` (motion-undistorted) feeds `pointcloud_to_laserscan`. Static TF `imu → body` (identity) on namespaced `/robot/tf_static` connects Fast-LIO's body frame to URDF chain. `scan_bodyframe_pub_en: true` in `pointlio_gazebo.yaml`.
- **`max_clear_distance`** — hit rays clear all the way to wall; only no-hit rays capped.
- **Stop-at-occupied raytrace** — `raytrace_free()` stops at cells above `occupied_score_threshold_`.
- **Asymmetric log-odds** — hit:miss = 3:1. `occupied_threshold: 6`, `free_threshold: -3`, clamp [-10, 20].

### Problem 2: Map QoS Mismatch — Global Planner Never Ran (RESOLVED ✅)

**Symptom:** Paths curved backwards, went through walls. Log: `incompatible QoS ... Last incompatible policy: DURABILITY`.

**Root Cause:** Mapper published with VOLATILE, default_nav subscribed with TRANSIENT_LOCAL → silent message drop → global A* never ran → fell back to local scan-based planner (tiny grid, no memory).

**Fix:** Mapper publisher changed to `TRANSIENT_LOCAL` durability.

### Problem 3: Fast-LIO "No Effective Points" (RESOLVED ✅)

**Root Cause:** Aggressive voxel filters (0.5m) collapsed indoor point clouds. Low iterations (3) prevented EKF convergence.

**Fix:** `filter_size_surf/map: 0.15`, `max_iteration: 4`, `point_filter_num: 1`, `scan_bodyframe_pub_en: true`, tightened bias covariances.

### Problem 4: Fast-LIO Topic Namespace (RESOLVED ✅)

**Fix:** Added remapping `"/cloud_registered_body" → "/{ns}/cloud_registered_body"` — Fast-LIO uses absolute topic names that bypass namespace prefixing.

### Improvement: Path Shortcut Logic (IMPLEMENTED ✅)

**Fix:** O(n) closest-point pruning in `default_nav_core/planner.py`. If robot is near waypoint #5, skip #1-#4 instead of backtracking.

---

## Key Files Modified (2026-03-08)

| File | Change |
|---|---|
| `go2_nav_algorithms/src/simple_scan_mapper_cpp.cpp` | Stop-at-occupied raytrace, max_clear_distance fix, TRANSIENT_LOCAL QoS |
| `go2_gazebo_sim/config/nav/simple_scan_mapper_single_go2w.yaml` | Asymmetric log-odds scoring (3:1), tighter clamping |
| `go2_gazebo_sim/config/slam/pointlio_gazebo.yaml` | Finer voxel filters, more iterations, body cloud enabled |
| `go2_gazebo_sim/launch/single_go2w_gazebo_cfpa2.launch.py` | Fast-LIO cloud rewiring, static TF imu→body, topic remapping |
| `go2w_control/scripts/default_nav_core/planner.py` | Path shortcut logic (closest-point pruning) |

---

## Known Issues / TODO

1. ~~**Wall duplication**~~ — Fixed via Fast-LIO + stop-at-occupied + asymmetric scoring.
2. ~~**Map QoS mismatch**~~ — Fixed: TRANSIENT_LOCAL durability.
3. ~~**Fast-LIO "No Effective Points"**~~ — Fixed via voxel filter tuning.
4. **frontier_replan feedback** — CFPA2 doesn't subscribe to `/robot/frontier_replan`.
5. **external_stop rotation** — Controller returns `(0,0)` on external_stop; robot can't turn away.
6. **Dual-robot config** — May need same scoring rebalance.
7. **Local planner gap detection** — Scan-based planner can plan through sub-robot-width wall gaps (inflation can't close unseen gaps). Mitigated now that global planner receives the map.

---

## Previous Session (2026-03-07)

### Map Flickering / Starburst (RESOLVED ✅)
`last_scan_.reset()` after `publish_map()`. Each scan painted exactly once.

### No-Path Standstill / Goal Thrashing (IMPLEMENTED 🔧)
D* Lite incremental planner, goal dedup in `goal_cb`, `AsyncGridPlanner` with persistent D* Lite instance.

---

## Debugging Methodology

1. **Search git history** for prior art
2. **Diagnostic logging** with timestamps
3. **Measure topic rates** — `ros2 topic hz`
4. **Check QoS** — silent drops with `incompatible QoS` warnings
5. **First-principles tick tracing** — what happens each timer/scan/odom tick
6. **Full data pipeline trace** — source → frame → transform → consumer

---

## Archived Plan: Coverage Metric, Ghost Fix, C++ Acceleration, and Parameter Optimization

> This was a past implementation plan. Included here for reference.

### Phase 1: Ghost Particles Fix (COMPLETED ✅)
Increased `miss_decrement` 1→2 in `simple_scan_mapper_single_go2w.yaml`. Single-scan noise decays in ~2 sweeps; real walls persist.

### Phase 2: C++ Acceleration of CFPA2 Hot Paths
Three `extern "C"` functions (ctypes pattern): `extract_frontiers()`, `distance_transform()`, `batch_info_gain()`. Target: p95 tick time from ~50ms → <15ms.

### Phase 3: Ground-Truth Visibility Coverage
Alignment-free coverage using GT positions + raycasting. `coverage = seen_gt_free_cells / total_gt_free_cells`. C++ raycast in `gt_coverage.cpp`.

### Phase 4: Utility Parameter Optimization
Automated sweep of CFPA2 utility weights, inflation radius, safety clearance, obstacle stop distance. 20+ headless runs with 110s timeout, ≥98% GT coverage target.
