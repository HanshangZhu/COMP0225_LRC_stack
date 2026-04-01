# ROS 2 Package Reference

All packages live under `src/`. Grouped by function.

---

## Core Navigation & Exploration

| Package | Path | Description |
|---|---|---|
| **go2_nav_algorithms** | `src/go2_nav_algorithms/` | Custom navigation: `simple_scan_mapper_cpp` (2D occupancy grid from LaserScan + TF), `default_nav` (A* global + local obstacle avoidance), `geometric_frontier_explorer` |
| **cfpa2_collaborative_autonomy** | `src/cfpa2_collaborative_autonomy/` | Burgard-style CFPA2 multi-robot coordinator. Assigns frontiers to robots using utility = info-gain − cost + momentum |
| **go2_tare_planner_ros2** | `src/go2_tare_planner_ros2/` | TARE exploration planner (single robot), ROS 2 port. Contains both generated and upstream variants |
| **mtare_ros2** | `src/mtare_ros2/` | M-TARE multi-robot exploration coordinator (ROS 2 native, alternative to CFPA2) |

---

## SLAM Backends

| Package | Path | Description |
|---|---|---|
| **fast_lio** *(submodule)* | `src/fast_lio/` | Fast-LIO2 LiDAR-inertial SLAM. Tight IMU coupling, good for motion undistortion |
| **point_lio_unilidar** | `src/autonomy_stack_go2/.../point_lio_unilidar/` | Point-LIO (LiDAR-only SLAM). Faster convergence, no IMU init wait |
| **sc_pgo** *(submodule)* | `src/sc_pgo/` | Scan Context + Pose Graph Optimization. Loop closure addon for Fast-LIO / Point-LIO |
| **BALM** *(submodule)* | `src/BALM/` | Bundle Adjustment for LiDAR Mapping. Offline refinement tool |
| *Cartographer* | *(system install)* | Google Cartographer 3D. Installed via apt, configured in `src/go2_real_bringup/config/cartographer/` |

---

## Simulation

| Package | Path | Description |
|---|---|---|
| **go2_gazebo_sim** | `src/go2_gazebo_sim/` | Main simulation launcher. Launch files for single/dual Go2W in Gazebo Classic with all navigation nodes |
| **go2w_spawn** | `src/go2w_spawn/` | Gazebo model spawner for Go2W |
| **go2w_control** | `src/go2w_control/` | Wheel-leg hybrid motion controller for simulation (velocity blending, Gazebo plugin interface) |
| **vehicle_simulator** | `src/autonomy_stack_go2/.../vehicle_simulator/` | CMU's differential-drive vehicle simulator (used in Isaac mode) |

---

## Robot Description (URDF / Meshes)

| Package | Path | Description |
|---|---|---|
| **go2_description** | `src/unitree-go2-ros2/robots/descriptions/go2_description/` | Go2 (legged) URDF, meshes, Gazebo configs. Has added Velodyne 3D LiDAR + IMU sensors |
| **go2_config** | `src/unitree-go2-ros2/robots/configs/go2_config/` | CHAMP walking gait parameters for Go2 |
| **go2w_description** | `src/unitree_go2w_ros2/src/go2w_description/` | Go2W (wheeled-legged) URDF + meshes including wheel DAEs and Mid-360 LiDAR |
| **champ_description** | `src/unitree-go2-ros2/champ/champ_description/` | Base CHAMP quadruped URDF components (legs, accessories) |

---

## CHAMP Quadruped Controller (Simulation)

| Package | Path | Description |
|---|---|---|
| **champ_base** | `src/unitree-go2-ros2/champ/champ_base/` | Core quadruped controller: state estimation + leg IK |
| **champ_bringup** | `src/unitree-go2-ros2/champ/champ_bringup/` | Bringup launch files for CHAMP |
| **champ_gazebo** | `src/unitree-go2-ros2/champ/champ_gazebo/` | Gazebo plugin: foot contact sensor, odometry, IMU |
| **champ_msgs** | `src/unitree-go2-ros2/champ/champ_msgs/` | Custom ROS messages for CHAMP (joint states, contacts) |
| **champ_teleop** | `src/unitree-go2-ros2/champ_teleop/` | Keyboard/joystick teleop for CHAMP |

---

## Real Robot Drivers & Bringup

| Package | Path | Description |
|---|---|---|
| **go2w_driver** | `src/unitree_go2w_ros2/src/go2w_driver/` | C++ Ethernet/DDS driver for real Go2W. Publishes joint states, IMU, accepts velocity commands |
| **go2_robot_sdk** *(submodule)* | `src/go2_ros2_sdk/go2_robot_sdk/` | Unitree WebRTC SDK for WiFi connectivity |
| **go2_real_bringup** | `src/go2_real_bringup/` | Real-robot launch files + Cartographer configs for field deployment |
| **go2w_bringup** | `src/unitree_go2w_ros2/src/go2w_bringup/` | Go2W Gazebo launch (low-level / high-level control modes) |
| **go2w_mock** | `src/unitree_go2w_ros2/src/go2w_mock/` | Mock node for testing without real hardware |

---

## Perception & Sensor Processing

| Package | Path | Description |
|---|---|---|
| **go2w_perception** | `src/go2w_perception/` | `slam_odom_relay` (odom passthrough + TF), `carto_odom_bridge` (Cartographer TF → nav odom), `pointcloud_to_laserscan` wrapper |
| **transform_sensors** | `src/autonomy_stack_go2/.../transform_sensors/` | `transform_everything`: rotates UTLidar cloud/IMU from sensor frame → body frame (15.1° pitch + axis flip + self-hit filter) |
| **sensor_scan_generation** | `src/autonomy_stack_go2/.../sensor_scan_generation/` | CMU's scan generation from registered point clouds |
| **terrain_analysis** | `src/autonomy_stack_go2/.../terrain_analysis/` | Ground segmentation and traversability analysis |
| **terrain_analysis_ext** | `src/autonomy_stack_go2/.../terrain_analysis_ext/` | Extended terrain analysis with elevation mapping |
| **lidar_processor** | `src/go2_ros2_sdk/lidar_processor/` | Python LiDAR processing utilities (SDK) |
| **lidar_processor_cpp** | `src/go2_ros2_sdk/lidar_processor_cpp/` | C++ LiDAR processing (SDK, faster) |

---

## Observability & Monitoring

| Package | Path | Description |
|---|---|---|
| **go2w_observability** | `src/go2w_observability/` | Runtime monitoring: CPU/memory, topic rates, node health checks |

---

## Utilities & RViz Plugins

| Package | Path | Description |
|---|---|---|
| **goalpoint_rviz_plugin** | `src/autonomy_stack_go2/.../goalpoint_rviz_plugin/` | RViz click-to-goal plugin |
| **teleop_rviz_plugin** | `src/autonomy_stack_go2/.../teleop_rviz_plugin/` | RViz teleop panel |
| **waypoint_rviz_plugin** | `src/autonomy_stack_go2/.../waypoint_rviz_plugin/` | RViz waypoint placing tool |
| **visualization_tools** | `src/autonomy_stack_go2/.../visualization_tools/` | Marker/path visualization helpers |
| **calibrate_imu** | `src/autonomy_stack_go2/.../calibrate_imu/` | IMU bias calibration node (keep robot still, record biases) |

---

## Message Packages

| Package | Path | Description |
|---|---|---|
| **go2_interfaces** | `src/unitree_go2w_ros2/src/go2_interfaces/` | Go2W service definitions (mode, pose, gait, volume) |
| **unitree_api** | `src/unitree_go2w_ros2/src/unitree_api/` | Unitree sport API message types (Request/Response) |
| **unitree_go** | `src/unitree_go2w_ros2/src/unitree_go/` | Unitree Go2 message types (LowState, MotorCmd, IMU, BMS) |
| **visibility_graph_msg** | `src/autonomy_stack_go2/.../visibility_graph_msg/` | Messages for route planner visibility graph |

---

## Route Planner (CMU Autonomy, optional)

| Package | Path | Description |
|---|---|---|
| **far_planner** | `src/autonomy_stack_go2/.../far_planner/` | FAR Planner: fast and robust autonomous navigation |
| **boundary_handler** | `src/autonomy_stack_go2/.../boundary_handler/` | Exploration boundary management |
| **graph_decoder** | `src/autonomy_stack_go2/.../graph_decoder/` | Topological graph loading/saving |
| **local_planner** | `src/autonomy_stack_go2/.../local_planner/` | CMU's local planner (collision avoidance in 3D terrain) |

---

## Stale / Deprecated

| Package | Path | Description |
|---|---|---|
| **stale/** | `src/stale/` | Deprecated code kept for reference. Not built (has COLCON_IGNORE) |
| **mtare_ros1_ws** | `src/mtare_ros1_ws/` | ROS 1 M-TARE workspace. Runs in Docker container, bridged via `ros1_bridge`. Has COLCON_IGNORE |
| **librealsense** | `src/librealsense/` | RealSense camera driver (not currently used) |
