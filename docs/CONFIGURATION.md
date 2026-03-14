# Configuration Reference

All configuration files, Gazebo worlds, SLAM backends, and validation commands.

## Configuration Files

| Config | Purpose |
|---|---|
| `src/go2_gazebo_sim/config/nav/simple_scan_mapper_single_go2w.yaml` | Mapper scoring (hit/miss/thresholds) |
| `src/go2_nav_algorithms/config/nav/geometric_frontier_single.yaml` | Frontier detection parameters |
| `src/go2_nav_algorithms/config/gbplanner2_single.yaml` | GBPlanner2 single-robot params |
| `src/go2_nav_algorithms/config/gbplanner2_dual.yaml` | GBPlanner2 dual-robot params |
| `src/go2w_control/config/reactive_nav_single_go2w.yaml` | Navigation speeds, planner params (sim) |
| `src/go2w_control/config/reactive_nav_real_go2w.yaml` | Navigation params (real robot — slower, wider margins) |
| `src/go2w_control/config/go2w_hybrid_motion.yaml` | Wheel/leg mode blending |
| `src/go2_gazebo_sim/config/ros_control/ros_control_go2w_robot.yaml` | ros2_control joints + controller_manager rate |
| `src/go2_gazebo_sim/urdf/go2w/go2w_description_3d_lidar.xacro` | Full robot URDF with sensors |
| `src/go2_real_bringup/config/cartographer/go2w_3d_mapping_tuned.lua` | Cartographer 3D SLAM config (real robot) |
| `config/fastdds_no_shm.xml` | FastDDS shared memory disable |

## Gazebo Worlds

| World | Description |
|---|---|
| `3.world` | Multi-corridor with T-junction and L-branch (default) |
| `t_dual_corridor.world` | T-shaped dual corridor |
| `l_corridor.world` | L-shaped corridor |
| `empty_test.world` | Empty world for debugging |

## SLAM Backends

| Backend | Status | When to use |
|---|---|---|
| **Fast-LIO** | **Default in sim** | `use_fast_lio:=true` — LiDAR-inertial SLAM with motion undistortion |
| Ground-truth odometry | Fallback | `use_fast_lio:=false` — uses Gazebo `p3d` plugin (no motion compensation) |
| Point-LIO | Real robot only | Launched via `go2w_bringup` on the real Go2W |
| Cartographer | Real robot (tuning) | 3D SLAM, config: `go2w_3d_mapping_tuned.lua` |

Fast-LIO provides motion-undistorted point clouds (`cloud_registered_body`) that eliminate wall duplication during turns. The undistorted cloud feeds `pointcloud_to_laserscan` via a static TF `imu → body` (identity). Key config: `pointlio_gazebo.yaml`.

## Validation

After launch, verify nodes are running:

```bash
ros2 node list | grep -E "cfpa2|reactive_nav|simple_scan_mapper"
ros2 topic hz /robot/scan_3d        # expect ~10 Hz
ros2 topic hz /robot/odom/nav       # expect ~50 Hz
ros2 topic hz /robot/map            # expect ~4 Hz
```

## Common Failure Modes

| Symptom | Likely Cause | Fix |
|---|---|---|
| `ValueError: '...launch.py' is not a valid package name` | Using absolute path in `ros2 launch` | Use package+file form: `ros2 launch go2_gazebo_sim ...` |
| `FileNotFoundError` under `install/` | Stale install tree | Rebuild: `colcon build --symlink-install --cmake-clean-cache` |
| Controller `type` param not defined | ros2_control YAML mismatch | Check `config/ros_control/` YAMLs |
| `catkin` missing during build | Colcon building ROS1 workspace | `touch src/mtare_ros1_ws/COLCON_IGNORE` |
| Map flickering / doubled walls | Same scan painted multiple times | Check `simple_scan_mapper_cpp` timer vs scan rate |
| "No Effective Points!" in Fast-LIO | Wrong voxel filter or timestamp span | Check `pointlio_gazebo.yaml` filter sizes |
| Map starburst | TF timestamp mismatch | Check TF lookup code, avoid `tf2::TimePointZero` |
