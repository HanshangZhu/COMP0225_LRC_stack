# Context Handoff: Go2W Gazebo Exploration Stack

**Project:** `COMP0225_LRC_stack` — Multi-robot autonomous exploration with Unitree Go2W in Gazebo.
**Date:** 2026-03-07
**Map:** `3.world` (corridors)
**Robots:** `robot_a`, `robot_b` (namespaced), Go2W variant with wheels + legs.

---

## Session Summary (2026-03-07): Map Quality + Path Planning

### Problem 1: Map Flickering / Starburst (RESOLVED ✅)

**Symptom:** Occupancy grid map showed doubled/rotated corridor structures. Map flickered between correct and corrupted views.

**Root Cause (found via diagnostics):** `simple_scan_mapper_cpp` never cleared `last_scan_` after processing. The update timer (4Hz) re-painted the SAME scan with progressively newer odom poses on each tick. As the robot moved between ticks, identical scans were projected at different yaw angles → starburst/doubled walls.

**Fix:** Added `last_scan_.reset()` after `publish_map()` in `simple_scan_mapper_cpp.cpp` (line ~350). Each scan is now painted exactly once.

**Additional fixes applied:**
- Added `max_scan_odom_dt` parameter + enforcement to reject scan-odom pairs with >100ms timestamp gap (the parameter was being passed from launch but never checked in C++).
- `tf_timeout_ms` default increased from 50ms to 1000ms.
- `p3d` plugin update_rate set to 50Hz (in xacro, but runtime shows ~10Hz — may not have taken effect in all spawns).

### Problem 2: Occupancy Grid Scoring (RESOLVED ✅)

**Symptom:** After fixing starburst, walls were too faint (not enough hits) or had scatter noise.

**Fix:** Rebalanced scoring in `simple_scan_mapper_single_go2w.yaml`:
- `hit_increment: 5` (was 3) — strong evidence per hit
- `occupied_score_threshold: 12` — requires 3 hits to confirm wall (filters noise)
- `miss_decrement: 2` (was 5) — walls persist longer
- `score_max: 20` (was 8) — well-observed walls accumulate strong evidence

Also: `min_height` in `pointcloud_to_laserscan` raised from `-0.05` to `0.05` to filter robot leg / ground hits.

### Problem 3: Path Through Walls (IN PROGRESS 🔧)

**Symptom:** `reactive_nav` planned straight-line paths through walls because it only used scan-based local grid (couldn't see walls outside current LiDAR FOV).

**Fix:** Added `AsyncGridPlanner` — A* on the global occupancy grid:
- New module: `src/go2w_control/scripts/reactive_nav_core/grid_planner.py`
- Uses numpy + scipy for fast inflation (~8ms on 400×400 grid)
- Runs in background thread (never blocks 15Hz control loop)
- `reactive_nav.py` subscribes to `/{ns}/map`, polls async planner each tick
- Falls back to scan-based local planner if map unavailable
- Re-plans every 2s or on goal change

**Status:** Implemented, needs runtime verification.

---

## Key Files Modified This Session

| File | Change |
|---|---|
| `src/go2_nav_algorithms/src/simple_scan_mapper_cpp.cpp` | `last_scan_.reset()`, `max_scan_odom_dt` gate, diagnostics logging |
| `src/go2_gazebo_sim/config/nav/simple_scan_mapper_single_go2w.yaml` | Rebalanced scoring for single-paint |
| `src/go2_gazebo_sim/launch/single_go2w_gazebo_cfpa2.launch.py` | `min_height: 0.05`, removed `occupied_score_threshold` override |
| `src/go2_gazebo_sim/urdf/go2w/go2w_description_3d_lidar.xacro` | `p3d` update_rate 10→50Hz |
| `src/go2w_perception/scripts/pointcloud_adapter.py` | Timestamp span 100ms→10ms |
| `src/go2w_control/scripts/reactive_nav.py` | Map subscription + AsyncGridPlanner integration |
| `src/go2w_control/scripts/reactive_nav_core/grid_planner.py` | **NEW** — numpy-accelerated A* on occupancy grid |

---

## Known Issues / TODO

1. **p3d 50Hz rate may not be active** — `ros2 topic hz` showed ~8Hz. May need model re-spawn or Gazebo restart.
2. **Grid planner needs runtime test** — A* paths should curve around walls in RViz planned_path display.
3. **Dual-robot mapper config** — `geometric_frontier_dual.yaml` may need same scoring rebalance as single-robot config.
4. **MAPPER_DIAG logging** — diagnostic `RCLCPP_INFO_THROTTLE` left in mapper; remove when stable.
5. **D* Lite** — Current A* replans from scratch every 2s. D* Lite would incrementally update when map changes. Worth implementing if A* latency becomes a problem.

---

## Debugging Methodology That Worked

1. **Search git history for prior art** (`CODEX_CONTEXT_REPO_SUMMARY.md` documented the identical Isaac stack fix)
2. **Add diagnostic logging with timestamps** (MAPPER_DIAG revealed scan-odom timing)
3. **Measure actual topic rates** (`ros2 topic hz` revealed p3d wasn't at 50Hz)
4. **Benchmark before optimizing** (timed pure-Python A* → confirmed numpy needed)
5. **First-principles "what actually happens each tick"** (traced the timer→scan→odom→TF→paint pipeline step by step to find the re-paint bug)
