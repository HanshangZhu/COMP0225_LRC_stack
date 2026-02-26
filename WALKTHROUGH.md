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
source setup_cmu_env.bash
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
source setup_cmu_env.bash
colcon list | rg 'go2_gazebo_sim|go2_nav_algorithms|mtare_ros2'
./run_cfpa2_gazebo.sh gui:=false rviz:=false
ros2 topic list | rg 'robot_a|robot_b|way_point|nav_status|coordinator_map'
ros2 topic echo /robot_a/nav_status --once
```
