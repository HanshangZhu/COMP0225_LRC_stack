# go2_issac_stack Runtime Contracts

This package migrates the Gazebo-oriented exploration pipeline to an Isaac Sim/Lab topic contract while preserving the autonomy nodes from `go2_gazebo_sim`.

## Robot namespaces

- `go2_1`
- `go2_2`

## Isaac-side expected input topics (per robot)

- `/<ns>/lidar/points` (`sensor_msgs/PointCloud2`)
- `/<ns>/imu` (`sensor_msgs/Imu`)
- `/<ns>/odom` (`nav_msgs/Odometry`)
- `/<ns>/isaac/cmd_vel` (`geometry_msgs/Twist`) command output produced by this stack
- `/clock` (`rosgraph_msgs/Clock`) when `use_sim_time:=true`

## Compatibility topics produced for autonomy (per robot)

- `/<ns>/registered_scan`
- `/<ns>/registered_scan_reliable`
- `/<ns>/imu/data`
- `/<ns>/odom/ground_truth`
- `/<ns>/odom/nav`
- `/<ns>/scan_3d`
- `/<ns>/way_point_coord`
- `/<ns>/nav_status`

## Launch target

- `ros2 launch go2_issac_stack two_go2_isaac_coordinated_autonomy.launch.py`
- `ros2 launch go2_issac_stack single_go2_t_world_frontier.launch.py`
- `bash src/go2_issac_stack/scripts/run_single_t_world_readiness.sh` (build/source/launch/log automation)

## Single-robot readiness gate

The single-robot launch can run `readiness_gate.py` (enabled by default), which validates:

- odom, imu, pointcloud, cmd, map, and nav-status streams are active and non-stale
- bounded-map coverage (derived from closed-wall occupancy bounds) meets target ratio
- health remains stable for a continuous window

Exit code:

- `0` pass
- `1` fail (timeout or stale/freeze behavior)

Default coverage target:

- `required_coverage_ratio = 0.80`

## Optional Isaac process launch

Set `start_isaac_sim:=true` and provide `isaac_sim_command:=<shell command>` if you want this launch file to start Isaac Sim directly.

`single_go2_t_world_frontier.launch.py` now defaults `isaac_sim_command` to
`isaac_t_world_bringup.py`, which provides a full single-robot t_world bringup.
