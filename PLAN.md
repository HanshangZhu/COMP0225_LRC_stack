# Go2W Gazebo Size Inflation And Hybrid Wheel/Leg Motion

## ✅ Implementation Status: COMPLETE (verified 2026-03-06)

| Section | Status | Key Files |
|---------|--------|-----------|
| 1. Go2W obstacle clearance | ✅ Done | `config/nav/reactive_nav_{single,dual,real}_go2w.yaml` |
| 2. Wheel joint ros2_control | ✅ Done | `urdf/go2w/go2w_description_3d_lidar.xacro`, `config/ros_control/ros_control_go2w_robot{,_a,_b}.yaml` |
| 3. Hybrid motion router | ✅ Done | `scripts/control/go2w_hybrid_cmd_router.py`, `config/control/go2w_hybrid_motion.yaml` |
| 4. Launch wiring | ✅ Done | `launch/dual_go2w_modular.launch.py`, `launch/single_go2w_gazebo_cfpa2.launch.py`, `launch/single_go2w_real_cfpa2.launch.py` |

**Static checks passed:** `py_compile` for all launch files; syntax verified for hybrid router.
**Key design:** Go2W variant auto-detected via `generate_fixed_variant_launch_description(robot_variant="go2w")`, which sets `cmd_vel_input_topic="cmd_vel_legged"` and activates wheel controller spawning + hybrid router node. Go2 launches remain unchanged. Real-robot launch uses inflated clearance only, no Gazebo hybrid controller.

## Summary
Make Go2W obstacle clearance explicitly larger than Go2 in Go2W-only configs, and add a Gazebo-only hybrid motion path that uses CHAMP legged motion for turning/recovery and powered wheel joints for straight-line speed. Apply the behavior consistently to both Go2W Gazebo launches (`single` and `dual`) while leaving Go2 launches unchanged and leaving real-robot locomotion unchanged for now.

## Key Changes
### 1. Split and tune Go2W obstacle clearance separately from Go2
- Keep all size-related tuning in Go2W-only YAMLs; do not touch any Go2 config.
- Add a dedicated single-Gazebo Go2W nav profile `reactive_nav_single_go2w.yaml` instead of reusing the real-robot YAML from the single-robot sim launch.
- Set the Go2W footprint-related defaults to the same inflated values across:
  - `reactive_nav_single_go2w.yaml`
  - `reactive_nav_dual_go2w.yaml`
  - `reactive_nav_real_go2w.yaml`
- Chosen size-clearance defaults:
  - `obstacle_slow_dist: 0.70`
  - `obstacle_stop_dist: 0.32`
  - `planner_inflation_radius: 0.30`
- In the dual Go2W config only, also inflate teammate clearance:
  - `teammate_avoid_slow_radius_m: 1.05`
  - `teammate_avoid_stop_radius_m: 0.55`
- Do not change Go2W speed/turn tuning in this task except where required by the hybrid controller.

### 2. Expose and control the wheel joints in Gazebo
- Extend the Go2W Gazebo URDF `ros2_control` block so `FL_foot_joint`, `FR_foot_joint`, `RL_foot_joint`, and `RR_foot_joint` have:
  - `command_interface: velocity`
  - `state_interfaces: position, velocity`
- Keep the 12 leg joints on the existing effort-controlled joint trajectory path for CHAMP.
- Add wheel velocity controllers using `forward_command_controller/ForwardCommandController` on the velocity interface.
- Add or update Go2W ros2_control configs so each namespace has a matching wheel controller:
  - `robot_wheel_velocity_controller` for the single-robot launch
  - `robot_a_wheel_velocity_controller`
  - `robot_b_wheel_velocity_controller`
- Add a dedicated single-robot Go2W ros_control YAML instead of reusing the `robot_a` file in the single-robot launch.

### 3. Add a Go2W Gazebo hybrid motion router
- Add a new Go2W-only node in `go2_gazebo_sim`, named `go2w_hybrid_cmd_router.py`.
- Inputs:
  - `/<ns>/cmd_vel` from the existing `twist_bridge`
- Outputs:
  - `/<ns>/cmd_vel_legged` as `geometry_msgs/Twist` for CHAMP
  - `/<ns>/<wheel_controller_name>/commands` as `std_msgs/Float64MultiArray` for wheel joint velocities
  - `/<ns>/mobility_mode` as `std_msgs/String` with `wheel|legged|idle`
- Conservative automatic switching policy:
  - `wheel` mode only when all are true:
    - `abs(linear.x) >= 0.18`
    - `abs(linear.y) <= 0.05`
    - `abs(angular.z) <= 0.20`
    - `abs(angular.z) / max(abs(linear.x), 0.05) <= 0.45`
  - otherwise use `legged` mode
- Add hysteresis to prevent chattering:
  - `wheel_mode_hold_sec: 0.6`
  - `legged_mode_hold_sec: 0.6`
- Wheel-mode behavior:
  - send zero Twist to CHAMP
  - send equal wheel angular velocities for all four wheel joints
  - conversion: `wheel_omega = clamp(linear.x / wheel_radius_m, +/- wheel_max_angular_speed)`
- Legged-mode behavior:
  - forward the full requested Twist to CHAMP
  - publish zero wheel commands
- Idle behavior:
  - zero both outputs
- Chosen defaults for the new Go2W-only hybrid config:
  - `wheel_radius_m: 0.09`
  - `wheel_max_angular_speed: 8.5`
  - `wheel_joint_signs: [1.0, 1.0, 1.0, 1.0]`
- Put these parameters in a new Go2W-only config file, `config/control/go2w_hybrid_motion.yaml`.

### 4. Wire both Go2W Gazebo launches to the hybrid path
- Update the shared Gazebo robot-stack builder so the quadruped controller input topic is configurable instead of hardwired through the current direct `cmd_vel` path.
- For Go2 launches, preserve the current behavior exactly.
- For both Go2W Gazebo launches:
  - keep `reactive_nav -> twist_bridge`
  - route `twist_bridge` output to `/<ns>/cmd_vel`
  - insert `go2w_hybrid_cmd_router`
  - remap CHAMP so it consumes `/<ns>/cmd_vel_legged`
  - spawn and connect the namespace-matched wheel controller
- Apply this to:
  - `dual_go2w_modular.launch.py`
  - `single_go2w_gazebo_cfpa2.launch.py`
- Do not change the single real-robot Go2W launch beyond picking up the inflated real Go2W nav config.

## Public Interfaces
- New node: `go2w_hybrid_cmd_router.py`
- New config: `config/control/go2w_hybrid_motion.yaml`
- New sim config: `config/nav/reactive_nav_single_go2w.yaml`
- New status topic: `/<ns>/mobility_mode`
- New internal CHAMP input topic: `/<ns>/cmd_vel_legged`
- New wheel controller command topic: `/<ns>/<wheel_controller_name>/commands`
- No public-interface change for any Go2 launch or real-robot `cmd_vel` API

## Test Plan
- Static checks:
  - `python3 -m py_compile` for the new router and updated launches
  - `ros2 launch go2_gazebo_sim single_go2w_gazebo_cfpa2.launch.py --show-args`
  - `ros2 launch go2_gazebo_sim dual_go2w_modular.launch.py --show-args`
- Controller validation:
  - confirm each Go2W launch brings up the new wheel controller in `controller_manager`
  - confirm wheel joints expose velocity command/state interfaces
- Hybrid routing validation:
  - low-curvature forward command switches `/<ns>/mobility_mode` to `wheel`
  - sharp-turn or in-place-turn command switches it to `legged`
  - idle command zeros both actuator paths
- Motion validation:
  - in wheel mode, wheel joint velocities change and straight-line speed exceeds current legged-only Go2W behavior
  - in legged mode, CHAMP still performs the turns and recovery maneuvers
  - Go2W single and dual launches behave the same for the same commanded twist
- Safety/regression:
  - inflated Go2W obstacle tuning causes earlier slowdown/stop than before
  - dual Go2W robots maintain larger teammate clearance
  - Go2 launches remain byte-for-byte behaviorally unchanged
  - real Go2W launch only picks up the new inflated clearance values, not the Gazebo hybrid wheel controller

## Assumptions
- Hybrid wheel/leg control is Gazebo-only in this change; the real robot stays on the stock Go2W driver’s `cmd_vel` behavior.
- Wheel mode is for straight or nearly straight motion in both forward and reverse; turning remains legged.
- The initial wheel radius default is `0.09 m` and initial joint signs are all positive; acceptance testing must verify forward sign and adjust only if the first smoke test shows inversion.
