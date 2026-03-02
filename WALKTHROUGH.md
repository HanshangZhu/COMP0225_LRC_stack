# Workspace Walkthrough

This document explains the current runtime wiring for the Gazebo-first stack and where to inspect code when behavior changes.

## 1. Canonical Launch Model

Primary launch file:

- `src/go2_gazebo_sim/launch/dual_go2_modular.launch.py`

Compatibility wrappers still exist and forward into the canonical launch:

- `two_go2_t_world_cfpa2.launch.py`
- `two_go2_t_world_mtare_ros2.launch.py`
- other legacy wrappers under `src/go2_gazebo_sim/launch/`

The wrapper used by `run_cfpa2_gazebo.sh` is:

- `two_go2_t_world_cfpa2.launch.py`

It forwards to `dual_go2_modular.launch.py` with:

- `profile:=mtare_ros2`
- default `mtare_algorithm_mode:=cfpa2`
- default `planner_backend:=cfpa2`

## 2. Top-Level Script Behavior

### `run_cfpa2_gazebo.sh`

- Activates `cmu_env` (conda or micromamba path)
- Sources ROS2 Humble and workspace `install/setup.bash`
- Launches:

```bash
ros2 launch go2_gazebo_sim two_go2_t_world_cfpa2.launch.py
```

### `run_mtare_gazebo.sh`

- Activates `cmu_env`
- Sources ROS2/workspace
- Optionally kills stale Gazebo processes
- Picks free `GAZEBO_MASTER_URI` port from `11345-11350`
- Launches:

```bash
ros2 launch go2_gazebo_sim dual_go2_modular.launch.py \
  profile:=mtare_ros2 \
  planner_backend:=tare_ros2_exact \
  ...
```

Important: this script defaults to `planner_backend:=tare_ros2_exact`. For the most robust default in this snapshot, pass:

```bash
./run_mtare_gazebo.sh planner_backend:=mtare_ros2
```

### `run_mtare_ros1_bridge_container.sh`

- Builds/starts container image `cmu-mtare-ros1-bridge:foxy-noetic`
- Mounts `src/mtare_ros1_ws`
- Runs `roscore`, ROS1 `tare_planner` launch, and ROS1<->ROS2 `dynamic_bridge`

### `run_exploration_issac.sh`

- Starts Isaac-only profiles (`autonomy_baseline`, `sensor_realism`, `debug`)
- Kept for reference; Gazebo is the main development path

## 3. `dual_go2_modular` Execution Flow

### 3.1 Profile and backend resolution

`planner_backend` accepted values:

- `auto`
- `none`
- `coordinated`
- `go2_nav_algorithms`
- `cfpa2`
- `mtare_ros2`
- `ros1_mtare`
- `far_ros2`
- `tare_ros2_exact`

`auto` resolves as:

- profile `coordinated` -> backend `coordinated`
- profile `mtare_ros2` -> backend `mtare_ros2`
- otherwise -> backend `none`

### 3.2 Startup timeline

The launch brings up, in order:

1. Optional stale-process cleanup (`cleanup_stale`)
2. `gzserver`
3. optional `gzclient` (`gui:=true`)
4. RViz (`rviz:=true`)
5. robot A stack
6. wait until robot A controllers are loaded
7. robot B stack
8. wait until robot B controllers are loaded
9. backend coordinator/assigner actions
10. global observability nodes

This controller-gated ordering is intentional to avoid race conditions in spawn/control startup.

### 3.3 Robot spawn/control pipeline

Main files:

- `src/go2_gazebo_sim/launch/modules/assets.py`
- `src/go2_gazebo_sim/config/ros_control/ros_control_robot_a.yaml`
- `src/go2_gazebo_sim/config/ros_control/ros_control_robot_b.yaml`

For each robot namespace (`robot_a`, `robot_b`), `build_dual_robot_stack(...)` includes:

- namespaced `robot_state_publisher`
- CHAMP controller/state-estimation nodes
- `spawn_entity_direct.py`
- `initial_pose_guard.py`
- controller spawners:
  - `<ns>_joint_states_controller`
  - `<ns>_joint_group_effort_controller`
- `stand_up_slowly.py`

`build_namespaced_robot_description(...)` rewrites plugin namespaces/remaps and injects the namespace-specific ros2_control YAML path.

### 3.4 Perception/navigation pipeline (per robot)

Standard ROS2 path:

- `/registered_scan` -> `qos_bridge.py` -> `/registered_scan_reliable`
- `/registered_scan_reliable` -> `pointcloud_to_laserscan` -> `/scan_3d`
- `/scan_3d` + `/odom/nav` -> `simple_scan_mapper_cpp` -> `/map`
- frontier node -> waypoint topic (`/way_point` or `/way_point_raw` depending on profile/backend)
- `reactive_nav.py` consumes waypoint + scan + odom and publishes `/cmd_vel_stamped`
- `twist_bridge.py` converts `/cmd_vel_stamped` -> `/cmd_vel`

SLAM/odom source:

- `use_fast_lio:=false`: GT odom relay to `/odom/nav`
- `use_fast_lio:=true`: FAST-LIO pipeline and odom relay from `/Odometry`

### 3.5 Backend logic

- `none`: no global coordinator; frontiers feed local waypoint directly
- `coordinated` / `go2_nav_algorithms`: runs `multi_robot_goal_assigner.py`
- `mtare_ros2`: runs `mtare_coordinator.py` (`algorithm_mode` configurable)
- `cfpa2`: same coordinator executable, forced `algorithm_mode=cfpa2`
- `tare_ros2_exact`: exact-split flow with:
  - `mtare_topic_bridge.py`
  - `mtare_behavior_executive_cpp`
  - `mtare_coordinator.py output_mode:=exact_split`
  - FAR planners (+ optional `graph_decoder` shared bus)

`tare_ros2_exact` has stricter package requirements than `mtare_ros2`.

### 3.6 RViz logic

RViz profile selection in `dual_go2_modular`:

- `profile:=mtare_ros2` -> `src/go2_gazebo_sim/rviz/dual_mtare_shared.rviz`
- other profiles -> both:
  - `src/go2_gazebo_sim/rviz/dual_robot_a.rviz`
  - `src/go2_gazebo_sim/rviz/dual_robot_b.rviz`

If RViz behavior differs from Isaac-era expectations, this profile-driven RViz switch is one of the first places to compare.

## 4. Package Ownership Map

- `go2_gazebo_sim`: launch orchestration, spawn wrappers, local control helpers, observability
- `go2_nav_algorithms`: map/frontier/goal-assigner pipeline components
- `mtare_ros2`: MTARE coordinator + exact backend bridge/executive
- `autonomy_stack_go2` (submodule): FAR planner / graph stack dependencies
- `fast_lio` (submodule): optional LIO odometry backend
- `unitree-go2-ros2` (submodule): Go2 CHAMP description/controller resources
- `mtare_ros1_ws/src/mtare_planner` (submodule): ROS1 planner for bridge mode

## 5. Common Failure Modes

1. `ValueError: '...launch.py' is not a valid package name`

Cause: using `ros2 launch` with an absolute file path in package slot.
Fix: use package+file form, for example:

```bash
ros2 launch go2_gazebo_sim two_go2_t_world_cfpa2.launch.py
```

2. `FileNotFoundError` under `install/.../launch/...`

Cause: stale/missing install tree after code changes.
Fix: rebuild and re-source workspace:

```bash
colcon build --symlink-install --cmake-clean-cache
eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash
source install/setup.bash
```

3. controller_manager: `The 'type' param was not defined`

Cause: ros2_control YAML mismatch/stale install or namespacing mismatch.
Primary files to verify:

- `src/go2_gazebo_sim/config/ros_control/ros_control_robot_a.yaml`
- `src/go2_gazebo_sim/config/ros_control/ros_control_robot_b.yaml`
- `src/go2_gazebo_sim/launch/modules/assets.py`

4. `catkin` missing during `colcon build`

Cause: colcon trying to build ROS1 workspace.
Fix:

```bash
touch src/mtare_ros1_ws/COLCON_IGNORE
```

## 6. Minimal Debug Checklist

```bash
eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon list | rg 'go2_gazebo_sim|go2_nav_algorithms|mtare_ros2'
./run_cfpa2_gazebo.sh gui:=false rviz:=false
ros2 topic list | rg 'robot_a|robot_b|way_point|nav_status|coordinator_map'
ros2 topic echo /robot_a/nav_status --once
```


# Working Algorithms

## CFPA2 


run_cfpa2_gazebo.sh is wiring a CFPA2 coordinator backend (not just mtare mode): see run_cfpa2_gazebo.sh (line 40), dual_go2_modular.launch.py (line 1052).

### Underlying CFPA2 algorithm

Per tick, build a planning map (shared map if available, else fallback), extract frontier cells, and compute per-robot reachable distance maps via BFS: cfpa2_coordinator_node.py (line 1992), cfpa2_coordinator_node.py (line 510), cfpa2_coordinator_node.py (line 580).
Single-robot utility for goal g:
U_i(g) = w_ig*IG(g) - w_c*dist_i(g) - w_sw*switch_penalty_i(g)
at cfpa2_coordinator_node.py (line 1278).
Joint 2-robot assignment: maximize
U_a(g_a) + U_b(g_b) - lambda_overlap*overlap(g_a,g_b) with g_a != g_b: cfpa2_coordinator_node.py (line 2165).
Optional space-time A* converts final goal into a safer short-horizon waypoint when robots may conflict: cfpa2_coordinator_node.py (line 1422), cfpa2_coordinator_node.py (line 1549).
Hyperparameters (effective defaults from your script)

Utility weights: cfpa2_w_ig=1.2, cfpa2_w_c=0.6, cfpa2_w_sw=0.2, cfpa2_lambda_overlap=6.0, cfpa2_sigma_overlap_m=1.5.
Frontier/assignment: cfpa2_frontier_min_cluster_area_m2=0.1, switch_hysteresis=0.08, mtare_exploration_gain_radius_cells=4, min_assign_distance=0.30 (from yaml), switch_min_dist=0.80 (from yaml).
Stuck/close-range: cfpa2_stuck_lock_sec=8, cfpa2_stuck_min_motion_m=0.20, cfpa2_stuck_blacklist_sec=15, cfpa2_close_stop_radius_m=0.35, cfpa2_close_stop_speed_epsilon=0.02.
Space-time A*: enabled, horizon 5.0s, dt=0.40s, safety radius 0.55m, lookahead 0.90m, max expansions 12000, assumed/max speeds 0.25/0.60 m/s.
Shared map: use_shared_map=true, topic /disco_slam/global_map, wait 8.0s, local patch radius 2.5m (yaml).
Safeguards

Goal blacklist + repeated-near-goal blacklist; too-close goal rejection; reachability checks: cfpa2_coordinator_node.py (line 658), cfpa2_coordinator_node.py (line 688).
Stuck recovery auto-blacklists current goal and forces alternative: cfpa2_coordinator_node.py (line 1619).
Close-proximity arbitration forces lower-momentum robot to stop: cfpa2_coordinator_node.py (line 1690).
Switch hysteresis reduces goal thrashing: cfpa2_coordinator_node.py (line 775).
Shared map logic

Launch starts shared_map_fuser.py when use_shared_map=true, merging /robot_a/map + /robot_b/map into /disco_slam/global_map: dual_go2_modular.launch.py (line 644), shared_map_fuser.py (line 1).
Coordinator behavior:
Wait for shared map up to shared_map_wait_sec.
If missing after timeout, fail-open to merged local maps.
When shared map arrives later, switch back automatically.
Even in shared mode, patch near each robot with local map data (shared_map_local_patch_radius_m) to reduce staleness.
See cfpa2_coordinator_node.py (line 2006), map_merge_utils.py (line 164).

### Is it auction model based?

Not in the strict sense.

In your run_cfpa2_gazebo.sh path (planner_backend:=cfpa2), the coordinator does centralized utility-based assignment, not a distributed auction protocol:

It computes per-robot utilities for each frontier.
Then it picks the best joint pair (goal_a, goal_b) maximizing a global score with overlap penalty.
That is closer to a centralized combinatorial scorer than a classic auction model (no explicit bid messages, auctioneer rounds, or winner determination protocol).


### so bacically each frontier for each robot has a score, and goal is jointly maximum score

Yes, exactly.

For cfpa2 mode with two robots:

each robot scores each reachable frontier (IG - cost - switch penalty),
then the coordinator evaluates frontier pairs (a,b) with a != b,
and selects the pair with maximum joint score after overlap penalty.

# Launch script nodegraphs: 

I’m going to do one live snapshot run (headless) and capture ros2 node list so the graph includes dynamically created Gazebo/plugin nodes and controller-manager nodes that static parsing can miss.

I have enough live data now (node list + node info + topics). I’m stopping the launch session and then I’ll provide the full graph in a readable grouped format (runtime core + startup/transient nodes + disabled branches).

From run_cfpa2_gazebo.sh, the effective launch is dual_go2_modular.launch.py with profile:=mtare_ros2 and planner_backend:=cfpa2 (plus all module gates enabled).


robot_b pipeline (mirrorsrobot_a)
robot_a pipeline
/robot_a/registered_scan
/robot_a/registered_scan_reliable
/robot_a/scan_3d
/robot_a/scan_3d
/robot_a/odom/ground_truth
/robot_a/odom/nav
/robot_a/odom/nav
/robot_a/way_point_coord
/robot_a/way_point_coord
/robot_a/cmd_vel_stamped
/robot_a/cmd_vel
/robot_a/joy
/robot_a/map
/robot_b/registered_scan
/robot_b/registered_scan_reliable
/robot_b/scan_3d
/robot_b/scan_3d
/robot_b/odom/ground_truth
/robot_b/odom/nav
/robot_b/odom/nav
/robot_b/way_point_coord
/robot_b/way_point_coord
/robot_b/cmd_vel_stamped
/robot_b/cmd_vel
/robot_b/joy
/robot_b/map
/disco_slam/global_map
/robot_a/map
/robot_b/map
/robot_a/odom/nav
/robot_b/odom/nav
/mtare/coordinator_map
/mtare/robot_markers
/robot_a/mtare_goal_marker +/robot_b/mtare_goal_marker
/robot_a/map
/robot_b/map
/robot_a/odom/nav
/robot_b/odom/nav
/dual_robot/coverage_markers
/robot_a/nav_status
/robot_b/nav_status
/robot_a/map
/robot_b/map
/robot_a/odom/nav
/robot_b/odom/nav
/robot_a/gazebo_ros_3d_lidar_robot_a
/robot_a/qos_bridge
/robot_a/pointcloud_to_laserscan
/robot_a/simple_scan_mapper_cpp
/robot_a/reactive_nav
/robot_a/p3d_base_controller_robot_a
/robot_a/gt_odom_relay
/cfpa2_coordinator
/robot_a/autonomy_enabler
/robot_a/twist_bridge
/robot_a/quadruped_controller_node
/shared_map_fuser
/robot_b/gazebo_ros_3d_lidar_robot_b
/robot_b/qos_bridge
/robot_b/pointcloud_to_laserscan
/robot_b/simple_scan_mapper_cpp
/robot_b/reactive_nav
/robot_b/p3d_base_controller_robot_b
/robot_b/gt_odom_relay
/robot_b/autonomy_enabler
/robot_b/twist_bridge
/robot_b/quadruped_controller_node
RViz/consumers
/dual_map_coverage_visualizer
/robot_status_monitor
Live node snapshot (headless run, same backend/flags) included these runtime nodes:

Core orchestration: /cfpa2_coordinator, /shared_map_fuser, /robot_status_monitor, /dual_map_coverage_visualizer, /gazebo, /gazebo/gazebo_ros_state
robot_a: robot_state_publisher, quadruped_controller_node, state_estimation_node, base_to_footprint_ekf, footprint_to_odom_ekf, controller_manager, robot_a_joint_states_controller, robot_a_joint_group_effort_controller, contacts_sensor, gazebo_ros2_control_robot_a, gazebo_ros_3d_lidar_robot_a, gazebo_ros_front_laserscan_robot_a, imu_plugin_robot_a, p3d_base_controller_robot_a, qos_bridge, pointcloud_to_laserscan, simple_scan_mapper_cpp, gt_odom_relay, autonomy_enabler, reactive_nav, twist_bridge, stand_up_slowly
robot_b: same set with _b
Script default also adds RViz node (rviz2_mtare_shared) because rviz:=true in the script.
Script default also starts gzclient process because gui:=true (not a ROS node).
Not launched in this script profile:

mtare_coordinator.py path (because backend is cfpa2, not mtare_ros2)
simple_frontier_explorer.py nodes (because enable_frontier_aux:=false and not tare_ros2_exact)
fast_lio / pointcloud_adapter (because use_fast_lio:=false)