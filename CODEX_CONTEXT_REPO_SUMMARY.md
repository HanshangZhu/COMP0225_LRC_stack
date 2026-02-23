# Codex Context: CMU Exploration Workspace

_Last updated: 2026-02-22_

## Current focus
- Active work is Isaac-first in `src/go2_issac_stack` (dual Go2 autonomy in Isaac Sim).
- Gazebo in `src/go2_gazebo_sim` remains a reference baseline.

## What was implemented (this pass)

### 1) Performance-oriented launcher profiles
- Added profile-based launcher script in `run_exploration_issac.sh`.
- Profiles:
  - `perf`: headless, RTX lidar, no FAST-LIO, dynamic physics off, coordinator/monitor/coverage off.
  - `balanced`: similar to `perf`, but coordinator + status monitor on.
  - `debug`: heavier fidelity mode (CPU lidar high rays, RViz on, dynamic on, CHAMP on).
- Shell startup hardened by using `set -eo pipefail` (removed `-u` issue with env sourcing).

### 2) Launch refactor + runtime knobs (helper decomposition)
Refactored `src/go2_issac_stack/launch/two_go2_isaac_coordinated_autonomy.launch.py` into smaller builders:
- `_build_slam_nodes`
- `_build_core_bridge_nodes`
- `_build_exploration_nodes`

Added launch arguments:
- `planning_scan_max_height`
- `planning_scan_range_max`
- `mapper_update_rate`
- `frontier_update_rate`
- `enable_goal_assigner`
- `enable_status_monitor`
- `enable_coverage_visualizer`
- `enable_champ_stack`

Applied conditions so optional nodes can be disabled cleanly for perf runs.

### 3) CHAMP stack condition support
- Updated `src/go2_issac_stack/scripts/isaac_robot_stack.py` so `build_isaac_robot_stack(...)` accepts an optional `condition` and applies it to all stack nodes.

### 4) Isaac bringup helper decomposition + lidar optimization
In `src/go2_issac_stack/scripts/isaac_t_world_dual_bringup.py`:
- Broke large bringup flow into smaller helpers:
  - `_parse_robot_specs`
  - `_import_robot_prim`
  - `_create_robot_runtime`
  - `_configure_robot_rtx_lidar`
- Added vectorized CPU raycasting path (NumPy) + fallback to scalar path when unavailable.
- Improved CPU pointcloud scheduling catch-up logic.

In `src/go2_issac_stack/scripts/isaac_t_world_bringup.py`:
- Added matching vectorized CPU raycasting path.

### 5) Runtime crash fix
- Removed invalid `SensorMaterialAPI` usage in stage setup (`isaac_t_world_bringup.py`).
- This resolved the previous `NameError: SensorMaterialAPI` startup failure.

### 6) Major kinematic-mode error flood fix
- In dual bringup graph creation, articulation/joint-control OmniGraph nodes are now created only when dynamic physics is enabled.
- Kinematic mode now publishes odom/imu without binding articulation nodes.

Impact:
- Eliminated large repeated PhysX articulation mismatch error spam in kinematic mode.

## Validation performed

### Static checks
- `python3 -m py_compile` passed for updated Isaac scripts and launch file.
- `ros2 launch ... --show-args` confirmed new launch arguments are wired.

### Runtime test command
- `timeout 120s ./run_exploration_issac.sh perf > /tmp/isaac_perf_run3.log 2>&1`
- Exit: `124` (timeout), indicating the stack remained running throughout the window.

### Log monitoring used
- `grep -in "[Error]" /tmp/isaac_perf_run3.log`
- `grep -inE "isaac_topic_router|qos_bridge|simple_scan_mapper|simple_frontier_explorer|reactive_nav|odom_tf_broadcaster|isaac_dual_bringup" /tmp/isaac_perf_run3.log`

### Observed behavior
- Core nodes start and remain active for both robots:
  - `isaac_topic_router`
  - `qos_bridge`
  - `simple_scan_mapper_cpp`
  - `simple_frontier_explorer`
  - `reactive_nav`
  - `odom_tf_broadcaster`
- Continuous odom + pointcloud relay counters increase for both namespaces.

## Error-state summary
- `SensorMaterialAPI` crash: fixed.
- Kinematic articulation mismatch spam:
  - before: very high repeated errors
  - after: eliminated in perf kinematic run
- Remaining errors are startup-only PhysX static-joint creation messages from imported static configuration (limited count), not continuous runtime floods.

## Suggested operational defaults
- For throughput benchmarking: `./run_exploration_issac.sh perf`
- For coordinated behavior checks: `./run_exploration_issac.sh balanced`
- For full visual + dynamic debugging: `./run_exploration_issac.sh debug`

## RTX lidar debugging (2026-02-22) — full diagnosis

### Symptom
Occupancy grid in RViz showed a uniform clear circle of free space around each robot — no walls detected despite walls being visible in the Isaac Sim viewport.

### Root cause 1 — Wall cubes had no USD material (rays pass through)
Isaac Sim RTX lidar casts rays via the **rendering pipeline**, not the physics pipeline.
Wall cubes only had `UsdPhysics.CollisionAPI` applied. Without a `UsdShade.Material` binding, USD geometry uses the default transparent shading model and RTX rays pass through it.

**Fix** (`isaac_t_world_bringup.py`, `_setup_stage`):
A shared `UsdPreviewSurface` material (`roughness=1.0`, `opacity=1.0`) is defined at `/World/WallMaterial` and bound to every wall cube via `UsdShade.MaterialBindingAPI.Bind()`.

### Root cause 2 — `frameId` lie: points are in sensor frame, not base_link
`_build_rtx_lidar_graph` sets `frameId: "base_link"` in the OmniGraph publisher.
This writes `base_link` into the message **header only** — the actual XYZ coordinates are still in the lidar sensor's local frame (origin at sensor mount, not robot base).
`pointcloud_to_laserscan` then uses those coordinates as if they were already in `base_link`, producing mis-located points that don't correspond to actual wall positions.

**Fix (interim)**: `perf` and `balanced` profiles switched to `isaac_lidar_mode:=cpu` which uses the proven vectorised Python raycaster that publishes correct `base_link`-frame coordinates directly.
**RTX fix (proper, TODO)**: Mount the lidar prim at the exact Go2 `base_link` origin, or use a correct TF chain (sensor frame → base_link) and pass the real sensor `frameId` so `pointcloud_to_laserscan` can TF-transform correctly.

### Root cause 3 — `planning_scan_max_height` too tight for ±45° beams
The Unitree L1 fires 18 beams across ±45° elevation.
At 1 m range, the steepest beam hits a wall at z = sin(45°) × 1 m ≈ **0.71 m** in the sensor frame.
The default `planning_scan_max_height = 0.60 m` clipped all upper-beam wall hits before they reached the mapper.

**Fix** (`two_go2_isaac_coordinated_autonomy.launch.py`): default raised `0.60 → 2.0 m`.

### Incidental — IMU topic name changed
`_create_ros2_base_graph` OmniGraph now publishes to `imu` instead of `utlidar/imu`.
`isaac_topic_router` `input_imu_topic` is `/{ns}/imu` — matches. ✓
Any node subscribing to `/{ns}/utlidar/imu` directly would need updating.

### Applied fixes summary

| Fix | File | Status |
|---|---|---|
| Wall `UsdPreviewSurface` material binding | `isaac_t_world_bringup.py` | Applied + rebuilt ✅ |
| `perf` + `balanced` → `cpu` lidar, 720 rays | `run_exploration_issac.sh` | Applied ✅ |
| `planning_scan_max_height` `0.60 → 2.0 m` | `two_go2_...launch.py` | Applied + rebuilt ✅ |
| RTX frame/TF proper fix | — | **TODO** |

