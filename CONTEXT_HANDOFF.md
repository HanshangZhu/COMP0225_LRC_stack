# Context Handoff: "Goal Reached" Stagnation Debugging

**Project Status:** Debugging multi-robot autonomous exploration (`cmu_exploration_ws`) for two Go2 robots in Gazebo (`go2_gazebo_sim`).
**Map:** `3.world` (corridors).
**Robots:** `robot_a`, `robot_b` (namespaced).

## Current Critical Bug: "Goal Reached" Stagnation
**Symptoms:** Robots successfully reach a frontier but then **stop moving completely**.
**Log Analysis:** CSV logs (`/tmp/reactive_nav_log_*.csv`) show the robot entering `goal_reached` state (`dist_to_goal < 0.65m`) but **refusing to request a new goal** (`replan` column stays 0).

**Hypothesis:**
1.  `reactive_nav.py` logic flaw: stops publishing `replan` signal after entering "reached" state.
2.  `geometric_frontier` returning the same goal repeatedly (needs blacklist/force explore).

## Key Files & Tunings
*   `src/go2_gazebo_sim/scripts/reactive_nav.py`: Added CSVLogger.
*   `src/go2_gazebo_sim/config/nav/reactive_nav_dual.yaml`:
    *   `goal_tolerance`: **0.65**
    *   `obstacle_slow_dist`: **0.45**
*   `src/go2_gazebo_sim/config/nav/geometric_frontier_dual.yaml`:
    *   `obstacle_clearance_cells`: **6** (0.6m)
    *   `update_rate`: **2.0 Hz**
*   `src/go2_gazebo_sim/launch/_stack_components.py`: `pointcloud_to_laserscan` min_height **0.3m**.

## Resolved Issues
*   **Deadlock:** Tuned `reactive_nav` wall panic distance (0.18).
*   **Phantom Obstacles:** Filtered floor noise (min_height 0.3m).
*   **Controller Crash:** Staggered launches by 10s.

## Action Plan for Next Session

### 1. Check Replan Logic
Inspect `reactive_nav_core.py` (or `control_loop` in `reactive_nav.py`).
**Task:** Ensure `request_replan` is pulsed **continuously** (or at 1Hz) while in `goal_reached` state until a new goal is received (dist > tolerance).

### 2. Verify Frontier Updates
Subscribe to `/robot_a/frontier_goal`.
**Task:** See if the goal actually changes when replan is requested. If not, `geometric_frontier` needs a "blacklist" or "force explore" logic.

### 3. Log Analysis
Run: `grep ",1$" /tmp/reactive_nav_log_*.csv`
**Check:** Did any replan requests happen? If none, the trigger is broken.

### 4. Force Replan (Manual Test)
Command:
```bash
ros2 topic pub --once /robot_a/frontier_replan std_msgs/msg/Empty "{}"
```
**Check:** If robot moves, the issue is definitely the trigger in `reactive_nav`.
