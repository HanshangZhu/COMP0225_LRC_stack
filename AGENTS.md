# AGENTS.md — Working Instructions for AI Coding Assistants

This file describes how to work effectively on this codebase (`COMP0225_LRC_stack`) with the maintainer.

---

## Communication Style

- **Be fast and direct.** The maintainer asks short questions and expects immediate, precise answers. Don't over-explain when they ask "what is X" — give the answer, not a lecture.
- **Show your work but don't narrate it.** Run the commands, make the changes, report what happened. Don't ask for permission unless the action is destructive.
- **When told "still not working"**, don't repeat the same fix. Go deeper. The maintainer will tell you when something works.
- **Respect the maintainer's hypotheses.** When they say "I AM certain it's due to X", treat that as strong signal. Validate or disprove with evidence, not conjecture.

---

## Debugging Philosophy

### First Principles Thinking

When debugging this stack, always trace the **full data pipeline** step by step:

1. **What generates the data?** (Gazebo plugin, sensor, etc.)
2. **What timestamp domain is it in?** (sim time vs wall time)
3. **What transforms it?** (pointcloud_adapter, pointcloud_to_laserscan, etc.)
4. **What consumes it?** (mapper, planner, etc.)
5. **What does the consumer actually do with it each tick?** (This is where bugs hide — re-processing, stale data, wrong frame)

### The Debugging Process That Works

1. **Check prior art first.** Search `CODEX_CONTEXT_REPO_SUMMARY.md` and git history. The maintainer has likely encountered a variant of this problem before (Isaac vs Gazebo stacks share patterns).
2. **Add diagnostic logging with timestamps.** Print actual values (yaw, dt, timestamps) — don't guess. Use `RCLCPP_INFO_THROTTLE` or Python logger with rate limiting.
3. **Measure, don't assume.** Use `ros2 topic hz`, `ros2 topic echo`, and grep logs. The p3d rate was assumed to be 50Hz but was actually 8Hz.
4. **Benchmark before optimizing.** Time the actual code path. If it's 220ms in Python, know that before deciding to use numpy.
5. **Trace what happens each timer tick.** The starburst bug was found by asking: "what if the timer fires twice with the same scan but different odom?"

### Common Root Causes in This Stack

| Symptom | Likely Cause | Where to Look |
|---|---|---|
| Map flickering / doubled walls | Same scan painted multiple times with different poses | `simple_scan_mapper_cpp.cpp` update timer vs scan arrival rate |
| Map starburst / rotated structures | TF timestamp mismatch or stale TF fallback | TF lookup code, `ExtrapolationException` handlers |
| "No Effective Points!" in Fast-LIO | Wrong timestamp span in pointcloud_adapter | `pointcloud_adapter.py` time_offset calculation |
| Robot walks through walls | Planner doesn't use occupancy grid | `default_nav_core/planner.py` — plans on scan grid, not map |
| Scattered occupied cells in map | Ground/leg hits passing height filter | `pointcloud_to_laserscan` `min_height` parameter |

---

## Architecture Overview

```
Gazebo LiDAR → registered_scan → [qos_bridge] → registered_scan_reliable
                                                        │
                                  ┌─────────────────────┤
                                  ▼                     ▼
                          pointcloud_adapter     pointcloud_to_laserscan
                          (for Fast-LIO)         → scan_3d (LaserScan)
                                                        │
                                  ┌─────────────────────┤
                                  ▼                     ▼
                        simple_scan_mapper_cpp    default_nav
                        → /{ns}/map (OccGrid)     ← /{ns}/map (A* global)
                                                  ← scan_3d (local avoid)
                                                  → cmd_vel_stamped

Gazebo p3d → odom/ground_truth → slam_odom_relay → odom/nav
                                                  → TF: world → base_link
```

### Key Nodes

- **`simple_scan_mapper_cpp`**: Builds 2D occupancy grid from laser scan + TF. Owns its own TF broadcaster (world → base_link from odom). Each scan painted ONCE.
- **`default_nav`**: Layered navigation — A* global planner on map + scan-based local obstacle avoidance. Background thread for A*.
- **`pointcloud_adapter`**: Adapts Gazebo point clouds for Fast-LIO (adds per-point timestamps, ring field). Only used when `use_fast_lio:=true`.
- **`slam_odom_relay`**: Passes through or aligns odometry. Does NOT broadcast TF.

---

## Build & Run

```bash
# Activate environment
micromamba activate cmu_env

# Build specific packages (fast iteration)
colcon build --symlink-install --packages-select go2_nav_algorithms go2w_control go2_gazebo_sim

# Single robot demo
./demo.sh

# Dual robot with CFPA2 coordination
./run_cfpa2_go2w_gazebo.sh

# Kill stale processes before re-launch
killall -9 gzserver gzclient rviz2
```

---

## Config Files That Matter

| Config | Purpose |
|---|---|
| `config/nav/simple_scan_mapper_single_go2w.yaml` | Grid scoring (hit_increment, thresholds) |
| `config/nav/geometric_frontier_single.yaml` | Frontier detection params |
| `config/nav/default_nav_single_go2w.yaml` | Local nav speeds, tolerances |
| `config/control/go2w_hybrid_motion.yaml` | Wheel/leg blending |
| `urdf/go2w/go2w_description_3d_lidar.xacro` | Sensor configs (LiDAR, p3d rates) |

---

## Rules for This Codebase

1. **Always use `use_sim_time: true`** for ALL nodes in Gazebo. Mixed time domains = map corruption.
2. **Never use stale TF fallback.** If TF lookup fails, DROP the scan. Don't use `tf2::TimePointZero`.
3. **Each scan must be painted exactly once.** Clear `last_scan_` after processing in any mapper.
4. **Test with `ros2 topic hz`** after changing sensor rates. Xacro changes require model re-spawn.
5. **Dual-robot TF must be namespaced.** Remap `/tf` → `/{ns}/tf` for all nodes.
6. **Symlink-install for fast iteration.** YAML/Python changes take effect immediately; C++ needs rebuild.
