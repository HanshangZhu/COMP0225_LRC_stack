---
description: Replay recorded bag data through Cartographer 3D SLAM for offline tuning
---

# Cartographer 3D SLAM Tuning (Offline Replay)

// turbo-all

## Prerequisites

```bash
micromamba activate cmu_env
source /opt/ros/humble/setup.bash
source ~/COMP0225_LRC_stack/install/setup.bash
```

## Available Bag Data

| Bag | Duration | IMU msgs | Cloud msgs | Path |
|-----|----------|----------|------------|------|
| tuning_bag_2 | ~74s | 18,551 | 1,089 (~15Hz) | `cartographer_tuning_bag_2/` |
| tuning_bag_3 | ~93s | 13,003 | 763 (~8Hz) | `cartographer_tuning_bag_3/` |

Both contain raw `/utlidar/imu` and `/utlidar/cloud` (BEST_EFFORT QoS).

## Replay Pipeline

```
rosbag2 → /utlidar/{cloud,imu}
  → transform_everything (15.1° pitch + axis flip + IMU bias correction)
  → /utlidar/transformed_{cloud,raw_imu}
  → Cartographer 3D (local SLAM + global loop closure)
  → /map (OccupancyGrid) + TF (map→odom→body)
  → RViz
```

## Steps

### 1. Kill any stale processes

```bash
pkill -9 -f cartographer; pkill -9 -f transform_everything; killall -9 rviz2 2>/dev/null; ros2 daemon stop
```

### 2. Start transform_everything

```bash
ros2 run transform_sensors transform_everything &
```

Wait 2 seconds for it to initialize.

### 3. Start Cartographer

```bash
CARTO_CFG=~/COMP0225_LRC_stack/src/go2_real_bringup/config/cartographer
ros2 run cartographer_ros cartographer_node \
  -configuration_directory "$CARTO_CFG" \
  -configuration_basename go2w_3d_mapping.lua \
  --ros-args \
  -r points2:=/utlidar/transformed_cloud \
  -r imu:=/utlidar/transformed_raw_imu \
  -p use_sim_time:=false &
```

Wait 3 seconds for Cartographer to initialize.

### 4. Start occupancy grid publisher

```bash
ros2 run cartographer_ros cartographer_occupancy_grid_node \
  --ros-args \
  -p use_sim_time:=false \
  -p resolution:=0.05 \
  -p publish_period_sec:=1.0 &
```

### 5. Start RViz

```bash
rviz2 -d ~/COMP0225_LRC_stack/src/go2_real_bringup/config/cartographer/cartographer.rviz &
```

### 6. Play the bag

```bash
# Bag 2 (74s, denser data)
ros2 bag play ~/COMP0225_LRC_stack/cartographer_tuning_bag_2 \
  --rate 1.0 \
  --topics /utlidar/cloud /utlidar/imu

# Or Bag 3 (93s, sparser)
ros2 bag play ~/COMP0225_LRC_stack/cartographer_tuning_bag_3 \
  --rate 1.0 \
  --topics /utlidar/cloud /utlidar/imu
```

Use `--rate 0.5` for slower playback if Cartographer can't keep up.

### 7. Evaluate results

Watch for:
- **Pose graph constraints**: Cartographer logs `N computations resulted in M additional constraints`
- **Occupation grid quality**: Does the map look correct in RViz?
- **IMU integration**: Check `imu_based_pose_extrapolator` iteration counts (should be ~3 avg)
- **Loop closure**: `Score histogram` should show matches above `min_score` threshold

## Tuning Parameters

The Lua config is at: `src/go2_real_bringup/config/cartographer/go2w_3d_mapping.lua`

### Key parameters to tune

| Parameter | Current | Purpose |
|-----------|---------|---------|
| `num_accumulated_range_data` | 1 | Clouds per scan match (1 = every cloud) |
| `submaps.num_range_data` | 60 | Clouds per submap |
| `min_range` | 0.1 | Min range filter |
| `max_range` | 21.0 | Max range filter |
| `ceres_scan_matcher.translation_weight` | 8.0 | Higher = trust scan match more |
| `ceres_scan_matcher.rotation_weight` | 100 | Higher = trust rotation match more |
| `imu_acceleration_weight` | 0.1 | IMU accel trust (low = less drift from gravity noise) |
| `imu_rotation_weight` | 5.0 | IMU gyro trust |
| `optimize_every_n_nodes` | 60 | Global optimization frequency |
| `constraint_builder.min_score` | 0.45 | Loop closure match threshold |
| `motion_filter.max_distance_meters` | 0.1 | Skip submap insert if moved < this |
| `motion_filter.max_angle_radians` | 5° | Skip submap insert if rotated < this |

### Common tuning strategies

- **Map too noisy / walls thick**: Increase `ceres_scan_matcher.translation_weight`, decrease `max_range`
- **Drift / map splitting**: Lower `constraint_builder.min_score`, increase `optimize_every_n_nodes`
- **Too slow**: Increase `num_accumulated_range_data` to 2-3, reduce `max_range`
- **Loop closure not triggering**: Lower `min_score` and `global_localization_min_score`

## IMU Calibration

Calibration file: `~/COMP0225_LRC_stack/imu_calib_data.yaml`

Current values:
```yaml
acc_bias_x: -1.176904
acc_bias_y: 2.082775
acc_bias_z: -0.328919
ang_bias_x: 0.003033
ang_bias_y: 0.005737
ang_bias_z: 0.001789
ang_z2x_proj: 0.1087
ang_z2y_proj: -0.185
```

These are applied by `transform_everything` before feeding to Cartographer.
