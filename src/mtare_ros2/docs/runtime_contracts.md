# mtare_ros2 Runtime Contracts

This package owns M-TARE backend coordination logic for dual Go2 Isaac runs.

## Backends

- `mtare_ros2`: Pure ROS2 M-TARE coordination (overlap penalty + pursuit)
- `ros1_mtare`: ROS1 TARE + dynamic bridge + ROS2 adapter
- `far_ros2`: FAR planner backend (local vendored package `go2_far_planner`)
- `tare_ros2_exact`: exact-split backend using BE arbitration and shared graph bus
- `none`: no global planner backend

## Core Topics (per robot namespace `<ns>`)

Inputs consumed by M-TARE coordinator:
- `/<ns>/map` (`nav_msgs/OccupancyGrid`)
- `/<ns>/odom/nav` (`nav_msgs/Odometry`)

Outputs produced by M-TARE coordinator:
- `/<ns>/way_point_coord` (`geometry_msgs/PointStamped`)
- `/<ns>/mtare_goal_marker` (`visualization_msgs/Marker`)
- `/mtare/coordinator_map` (`nav_msgs/OccupancyGrid`) coordinator-view map:
  - shared-map mode: shared map with near-robot local-map patching (`shared_map_local_patch_radius_m`)
  - fallback mode: merged per-robot local maps when shared map is unavailable
- `/mtare/robot_markers` (`visualization_msgs/MarkerArray`) robot pose/label/trajectory/goal overlays

When `output_mode:=exact_split`:
- local assignments publish to `/<ns>/way_point_tare`
- relocation/pursuit assignments publish to `/<ns>/goal_point`
- `/<ns>/way_point_coord` is published by `mtare_behavior_executive_cpp` (single writer)

## Coordinator Performance Gate

`mtare_coordinator.py` reports periodic `PERF coordinator` summaries with:
- tick latency window stats (`p50`, `p95`, `mean`, `max`, budget overrun count)
- process CPU percentage over the same summary interval

Default warning thresholds:
- `perf_tick_warn_p95_ms=150.0`
- `perf_cpu_warn_pct=15.0`

These thresholds define the trigger for considering a C++ coordinator parity port for
`tare_ros2_exact`.

## Map-Merge Boundary

- Occupancy-grid merge mechanics are isolated in `scripts/map_merge_utils.py`.
- `scripts/mtare_coordinator.py` consumes those helpers and keeps assignment logic
  (`_merge_targets`) separate from map-merging implementation.

## ROS1 bridge adapter topics (`ros1_mtare` backend)

- Bridges ROS2 autonomy feeds to ROS1 planner feeds (`state_estimation`, scans, terrain maps)
- Bridges ROS1 `/way_point` back to ROS2 `/<ns>/way_point_coord`
- Publishes `/<ns>/mtare_goal_marker`

## Environment switching model

`run_mtare_planner.sh` remains the operator entrypoint. Switch behavior using:
- `--planner-backend mtare_ros2`
- `--planner-backend ros1_mtare`
- `--planner-backend far_ros2`
- `--planner-backend tare_ros2_exact`

Docker ROS1 runtime is only required for `ros1_mtare`.
