# `tare_ros2_exact` Backend Contract

## Overview

`tare_ros2_exact` is an opt-in backend that keeps existing backends unchanged while splitting
global-coordinator outputs into local TARE and relocation channels.

## Dataflow

```text
mtare_coordinator.py (output_mode=exact_split)
  -> /<ns>/way_point_tare   (local assignment)
  -> /<ns>/goal_point       (relocation / pursuit)

go2_far_planner
  <- /<ns>/goal_point
  -> /<ns>/way_point_far
  <-> /mtare/robot_vgraph, /mtare/decoded_vgraph (shared graph bus)

graph_decoder
  <- /mtare/robot_vgraph
  -> /mtare/decoded_vgraph

mtare_behavior_executive_cpp
  <- /<ns>/way_point_tare
  <- /<ns>/way_point_far
  <- /<ns>/goal_point
  <- /<ns>/odom/nav
  -> /<ns>/way_point_coord   (single writer)
  -> /<ns>/planner_mode

default_nav
  <- /<ns>/way_point_coord
```

## State Machine

- `LOCAL_TARE`
- `FAR_TRANSIT`
- `RECOVERY_HOLD`

Guard flags are present but default `false`:

- `enable_hysteresis_guard`
- `enable_switch_lock_guard`
- `enable_stale_guard`

## Dependency Preflight

When `planner_backend:=tare_ros2_exact`:

- Required: `go2_far_planner`, `go2_tare_planner_ros2`
- Shared graph: `graph_decoder`, `visibility_graph_msg`
- If `require_shared_graph:=true`, missing shared-graph packages fail launch immediately.

## Coordinator Port Trigger

Python coordinator remains default unless measured thresholds are exceeded:

- tick latency p95 > `150 ms` (`perf_tick_warn_p95_ms`)
- or process CPU > `15%` of one core (`perf_cpu_warn_pct`)

`mtare_coordinator.py` emits `PERF coordinator` summaries so this decision is data-driven.
