# CMU-Exploration (Flagged) Walkthrough <!-- title -->
<!-- This repo walkthrough is tailored to the current Gazebo stack. --> <!-- note -->
<!-- All descriptions focus on the simulation stack under development. --> <!-- scope -->
 <!-- spacer -->
## Repository Flag <!-- section -->
This repository is a flagged variant of the CMU-Exploration stack. <!-- status -->
The original CMU-Exploration code is preserved and not modified. <!-- constraint -->
All additions and changes target the Gazebo simulation layer. <!-- intent -->
The goal is rapid iteration on autonomy behavior in simulation. <!-- goal -->
 <!-- spacer -->
## Current Focus: Gazebo Simulation (Single Robot) <!-- section -->
The Gazebo stack is actively being developed and stabilized. <!-- status -->
Primary work involves exploration and navigation behavior for a single Unitree Go2. <!-- focus -->
The simulation is the testbed for autonomy and frontier logic. <!-- usage -->
Unity/Isaac stacks are not the primary target here. <!-- scope -->
This walkthrough documents the Gazebo stack only. <!-- scope -->
 <!-- spacer -->
## Gazebo Stack: High-Level Flow <!-- section -->
1) Launch environment, singe robot, and bridges via `full_autonomy.launch.py`. <!-- step -->
2) Build a 2D occupancy grid from the front laser. <!-- step -->
3) Detect geometric frontiers and publish goals. <!-- step -->
4) Drive toward the goal using reactive obstacle avoidance. <!-- step -->
5) Visualize goals and frontiers in RViz/Gazebo. <!-- step -->
 <!-- spacer -->
## Gazebo Stack: Key Components <!-- section -->
### 1) Simulation Core <!-- subsection -->
- Gazebo Classic world + robot spawn. <!-- component -->
- `go2_gazebo_sim` package provides world and launch files. <!-- component -->
- `start_exploration.sh` calls `full_autonomy.launch.py`. <!-- entrypoint -->
- `gazebo_ros` plugins integrate Gazebo with ROS 2. <!-- bridge -->
 <!-- spacer -->
### 2) Robot Model + Control <!-- subsection -->
- Unitree Go2 robot description (URDF/Xacro). <!-- model -->
- CHAMP-based quadruped controller stack. <!-- control -->
- `gazebo_ros2_control` drives joint controllers. <!-- control -->
- `/cmd_vel` is the target velocity interface. <!-- io -->
 <!-- spacer -->
### 3) Sensors (Gazebo) <!-- subsection -->
- Front 2D laser (`front_laser`) publishes `/scan`. <!-- sensor -->
- LaserScan is the primary obstacle signal. <!-- sensor -->
- No 3D lidar or depth camera in this stack. <!-- limitation -->
 <!-- spacer -->
### 4) Exploration + Goal Generation <!-- subsection -->
- `geometric_frontier.py` builds an occupancy grid. <!-- node -->
- Frontiers are free cells adjacent to unknown space. <!-- algorithm -->
- Clustering selects candidate frontier regions. <!-- algorithm -->
- Goal selection uses largest-cluster heuristic. <!-- policy -->
- Published goal topic: `/way_point`. <!-- io -->
 <!-- spacer -->
### 5) Navigation + Obstacle Avoidance <!-- subsection -->
- `reactive_nav.py` drives toward the current goal. <!-- node -->
- Uses live `/scan` for reactive obstacle avoidance. <!-- algorithm -->
- No global path planner is used here. <!-- decision -->
- Motion is goal-as-you-move, not preplanned. <!-- behavior -->
 <!-- spacer -->
### 6) Safety + Recovery <!-- subsection -->
- `wall_collision_checker.py` monitors `/scan` for near-wall stops. <!-- safety -->
- `frontier_recovery.py` publishes recovery goals if stopped. <!-- recovery -->
- `motion_monitor.py` reports movement and stop state. <!-- observability -->
 <!-- spacer -->
### 7) Visualization <!-- subsection -->
- RViz markers for frontier regions and goals. <!-- viz -->
- Gazebo sphere marker for the current frontier goal. <!-- viz -->
 <!-- spacer -->
## Gazebo Stack: Tech Stack (Detailed) <!-- section -->
### Runtime + Build <!-- subsection -->
- ROS 2 Humble (`rclpy`, `rclcpp`). <!-- runtime -->
- `colcon build --symlink-install`. <!-- build -->
- `micromamba` environment (`cmu_env`). <!-- env -->
- FastDDS configuration to disable SHM when needed. <!-- middleware -->
 <!-- spacer -->
### Core ROS 2 Packages <!-- subsection -->
- `sensor_msgs`: LaserScan / PointCloud2. <!-- msgs -->
- `nav_msgs`: Odometry / OccupancyGrid. <!-- msgs -->
- `geometry_msgs`: TwistStamped / PointStamped. <!-- msgs -->
- `visualization_msgs`: Marker / MarkerArray. <!-- msgs -->
- `tf2_ros`: transforms for frames. <!-- tf -->
 <!-- spacer -->
### Gazebo Integration <!-- subsection -->
- `gazebo_ros` and `gazebo_ros2_control`. <!-- gazebo -->
- `gazebo_msgs` for spawn/delete entities. <!-- gazebo -->
- Gazebo world: `l_corridor.world`. <!-- world -->
 <!-- spacer -->
### Autonomy Stack Adapters (Gazebo) <!-- subsection -->
- `qos_bridge.py`: BestEffort to Reliable scan bridge. <!-- bridge -->
- `twist_bridge.py`: `/cmd_vel_stamped` → `/cmd_vel`. <!-- bridge -->
- `goalpoint_to_waypoint.py`: RViz goal → `/way_point`. <!-- bridge -->
 <!-- spacer -->
### Custom Exploration Nodes <!-- subsection -->
- `geometric_frontier.py`: frontier detection + goals. <!-- node -->
- `reactive_nav.py`: goal-driven reactive navigation. <!-- node -->
- `gazebo_frontier_visual.py`: Gazebo goal marker. <!-- node -->
 <!-- spacer -->
## How to Run (Gazebo) <!-- section -->
1) Ensure the `cmu_env` environment is available. <!-- step -->
2) Run: `./start_exploration.sh`. <!-- step -->
3) Use RViz to view frontiers and goals. <!-- step -->
4) Observe robot motion in Gazebo. <!-- step -->
 <!-- spacer -->
## Design Notes <!-- section -->
- This stack is optimized for fast ROS 2 iteration. <!-- note -->
- 2D lidar drives exploration and obstacle logic. <!-- note -->
- CMU 3D terrain planner is not used in this stack. <!-- note -->
- The system favors robustness over optimal paths. <!-- note -->
 <!-- spacer -->
## Directory Map (Gazebo) <!-- section -->
- `src/go2_gazebo_sim/launch/` — Gazebo launch files (`full_autonomy.launch.py`). <!-- map -->
- `src/go2_gazebo_sim/worlds/` — Gazebo world definitions. <!-- map -->
- `src/go2_gazebo_sim/scripts/` — Gazebo-specific nodes. <!-- map -->
- `start_exploration.sh` — top-level launcher. <!-- map -->
