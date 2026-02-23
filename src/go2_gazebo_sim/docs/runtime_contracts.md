# Runtime Contracts (go2_gazebo_sim)

This document defines ownership boundaries between frontier generation, assignment, and local motion control.

## Node Responsibilities

- `geometric_frontier.py`
  - Owns frontier extraction from occupancy evidence.
  - Publishes frontier goal candidates to `frontier_goal_topic` (e.g. `/way_point` or `/way_point_raw`).
  - Publishes visualization markers.
  - Does not command velocity directly.

- `multi_robot_goal_assigner.py`
  - Owns coordinated goal assignment across robots (coordinated launch).
  - Consumes per-robot frontiers/maps and publishes assigned goals to `/<ns>/way_point_coord`.
  - Does not command velocity directly.

- `reactive_nav.py` + `reactive_nav_core/*`
  - Owns local motion command synthesis.
  - Subscribes to `/way_point`, `/scan`, and odometry.
  - Publishes `/cmd_vel_stamped` and optional `/frontier_replan` trigger.

- `wall_collision_checker.py`
  - Owns near-field safety stop assertion using front-sector laser constraints.
  - Publishes binary stop signal on `/stop`.

## Topic Ownership Rules

- Goal source of truth for navigation:
  - independent mode: `/<ns>/way_point`
  - coordinated mode: `/<ns>/way_point_coord` (remapped into `/way_point` in `reactive_nav.py`)
- Velocity source of truth: `/cmd_vel_stamped` from `reactive_nav.py`.
- Safety stop source of truth: `/stop` from `wall_collision_checker.py` (pointlio test mode).
- Frontier replan trigger source: `/frontier_replan` from `reactive_nav.py`.

## Sequencing Expectations

1. Bridge nodes and scan producers start first.
2. Frontier (and assigner for coordinated mode) start next.
3. `autonomy_enabler.py` and `reactive_nav.py` start after startup delay.
4. `reactive_nav.py` waits for settle gate before motion.

## Debug Checklist

- If robot spins in place:
  - verify `/stop` toggling and `wall_collision_checker` thresholds.
  - verify `/way_point` receives updates from frontier/assigner.
- If frontiers exist but no movement:
  - verify `autonomy_enabler.py` has enabled `/joy`.
  - verify `reactive_nav.py` publishes non-zero `/cmd_vel_stamped`.
- If goals oscillate:
  - inspect hysteresis/reselect values in geometric frontier YAML profiles.
