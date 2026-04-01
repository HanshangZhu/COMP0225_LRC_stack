# Architecture

System design and data pipeline for the Go2W autonomous exploration stack.

## Data Pipeline

```
Gazebo LiDAR (1024×16 gpu_ray @ 10Hz)
  → registered_scan → [qos_bridge] → registered_scan_reliable
                                            │
                          ┌─────────────────┤
                          ▼                 ▼
                  pointcloud_adapter    (use_fast_lio=false only)
                  → velodyne_points     pointcloud_to_laserscan
                          ↓             → scan_3d (LaserScan)
                     Fast-LIO (SLAM)
                          │
            ┌─────────────┼─────────────────┐
            ▼             ▼                 ▼
      /Odometry    cloud_registered_body  (use_fast_lio=true)
      → slam_odom_relay   → pointcloud_to_laserscan
      → odom/nav + TF       → scan_3d (undistorted)
                                            │
                          ┌─────────────────┤
                          ▼                 ▼
                simple_scan_mapper_cpp   default_nav (12Hz)
                → /{ns}/map (OccGrid)    ← /{ns}/map (A* global planner)
                  (TRANSIENT_LOCAL QoS)  ← scan_3d (local obstacle avoidance)
                                         → cmd_vel_stamped

Gazebo p3d (50Hz) → odom/ground_truth → slam_odom_relay (GT bootstrap)
```

## Key Nodes

| Node | Language | Role |
|---|---|---|
| `simple_scan_mapper_cpp` | C++ | Builds 2D occupancy grid from LaserScan + TF (log-odds, stop-at-occupied raytrace) |
| `default_nav` | Python | Layered navigation: A* on global map + scan-based local avoidance |
| `cfpa2_coordinator_node` | C++ | Joint frontier assignment with overlap penalties, space-time A* |
| `go2w_hybrid_cmd_router` | Python | Routes cmd_vel between leg controller and wheel velocity controller |
| `wall_collision_checker` | Python | Sensor-level obstacle gating (emergency stop) |
| `qos_bridge` | Python | Re-publishes sensor data with RELIABLE QoS |
| `slam_odom_relay` | Python | Odometry normalization + TF broadcast |

## Locomotion

The Go2W is a hybrid platform: 12 leg joints (effort-controlled via CHAMP) + 4 wheel joints (velocity-controlled via `forward_command_controller`). `go2w_hybrid_cmd_router` arbitrates between wheeled and legged modes at runtime.

## CFPA2 Algorithm

Per tick, the coordinator builds a planning map (shared if available, else local merge), extracts frontier cells, and computes per-robot reachable distance maps via BFS.

**Single-robot utility:**
```
U_i(g) = w_ig * IG(g) - w_c * dist_i(g) - w_sw * switch_penalty_i(g)
```

**Joint 2-robot assignment:** maximize `U_a(g_a) + U_b(g_b) - λ_overlap * overlap(g_a, g_b)` with `g_a ≠ g_b`.

Optional space-time A* converts the final goal into a safer short-horizon waypoint when robots may conflict.

### Key Hyperparameters

- **Utility weights:** `w_ig=1.2`, `w_c=0.6`, `w_sw=0.2`, `λ_overlap=6.0`, `σ_overlap=1.5m`
- **Stuck recovery:** lock 8s, min motion 0.20m, blacklist 15s
- **Space-time A*:** horizon 5.0s, dt=0.40s, safety radius 0.55m, max expansions 12000
- **Shared map:** topic `/disco_slam/global_map`, wait 8.0s, local patch radius 2.5m

### Safeguards

- Goal blacklist + repeated-near-goal blacklist; too-close goal rejection
- Stuck recovery auto-blacklists current goal and forces alternative
- Close-proximity arbitration forces lower-momentum robot to stop
- Switch hysteresis reduces goal thrashing

## Perception/Navigation Pipeline (per robot)

```
/registered_scan → qos_bridge → /registered_scan_reliable
                               → pointcloud_to_laserscan → /scan_3d
/scan_3d + /odom/nav → simple_scan_mapper_cpp → /map
frontier node → waypoint topic → default_nav
default_nav → /cmd_vel_stamped → twist_bridge → /cmd_vel
```

**SLAM/odom source:**
- `use_fast_lio:=false` — GT odom relay to `/odom/nav`
- `use_fast_lio:=true` — Fast-LIO pipeline, odom from `/Odometry`

## Dual-Robot Launch Flow

1. `gzserver`
2. Optional `gzclient` (`gui:=true`)
3. RViz (`rviz:=true`)
4. Robot A stack → wait for controllers
5. Robot B stack → wait for controllers
6. Backend coordinator
7. Global observability nodes

## Backend Modes

| Backend | Description |
|---|---|
| `none` | No coordinator; frontiers feed local waypoint directly |
| `coordinated` | Runs `multi_robot_goal_assigner.py` |
| `cfpa2` | Centralized utility-based joint assignment |
| `mtare_ros2` | M-TARE coordinator |
| `tare_ros2_exact` | Exact-split with FAR planners + behavior executive |

## Performance Notes

Main bottlenecks:

1. **ODE physics** at 500 Hz × 25 solver iterations (single-threaded `gzserver`)
2. **GPU ray LiDAR** — 16,384 rays per frame per robot
3. **Process count** — dual-robot pipeline spawns ~50 ROS 2 processes
4. **Rendering** — `gzclient` + `rviz` consume 1.5–2.5 CPU cores

Run headless (`gui:=false rviz:=false`) for experiments.

## Package Ownership

| Package | Owns |
|---|---|
| `go2_gazebo_sim` | Launch orchestration, spawn wrappers, observability |
| `go2_nav_algorithms` | Map/frontier/goal-assigner pipeline |
| `cfpa2_collaborative_autonomy` | CFPA2 coordinator |
| `go2w_control` | Reactive nav, hybrid cmd router, collision checker |
| `go2w_perception` | QoS bridge, odom relay, pointcloud adapter |
| `mtare_ros2` | M-TARE coordinator + exact backend |
| `autonomy_stack_go2` | FAR planner / graph stack (submodule) |
| `fast_lio` | LIO odometry backend (submodule) |
| `unitree-go2-ros2` | CHAMP controller + Go2 description (submodule) |
