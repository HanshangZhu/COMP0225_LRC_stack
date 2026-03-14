# Cartographer IMU Calibration Session Summary

## Current Status
✅ **Map quality is good** — room walls visible, correct geometry  
⚠️ **Odom oscillates/drifts when robot is stationary**

## What Worked
| Change | Result |
|---|---|
| Original pipeline: negate+pitch → bias subtract → `frame_id='body'` → `tracking_frame='body'` | **Map looks correct**, same as yesterday |
| IMU rate 250 Hz, points2 14.7 Hz | ✅ Healthy data rates |
| Old calibration biases (`acc_bias_x: -1.18, acc_bias_y: +2.08, acc_bias_z: -0.33`) | ✅ Matches original working config |
| `use_online_correlative_scan_matching = false` | ✅ Essential — correlative matcher kills CPU in 3D |

## What Failed
| Attempt | Why it Failed |
|---|---|
| **Gravity quaternion** (rotate IMU by calibrated quaternion) | Changed IMU noise profile; Cartographer's internal gravity estimator got confused |
| **Online correlative scan matcher** (`= true`) | Way too expensive for 3D — dropped IMU rate from 250→16 Hz |
| **Textbook IMU-frame approach** (`tracking_frame='utlidar_imu'`, raw IMU) | Static TF `body→utlidar_imu` rotation was wrong (quaternion convention mismatch); frame not found by Cartographer |
| **Multicast disable** in CycloneDDS | Wasn't the cause — DDS errors were from physical cable disconnect |

## Remaining Issue: Static Odom Oscillation

The robot's estimated pose drifts/oscillates even when stationary. This is a known issue with Cartographer 3D + noisy IMU.

### Root Cause
Cartographer's `imu_based_pose_extrapolator` uses accelerometer data to estimate gravity and position. Between scan matches (every ~68ms at 14.7 Hz), the pose is extrapolated purely from IMU. Accelerometer noise causes small position oscillations that accumulate between scans.

### Recommended Fix (from TiNredmc's tuning guide)
> "If you feels like the Z axis is drifting or jumping? It has something to do with the IMU, try decreasing `imu_acceleration_weight` and `imu_rotation_weight`."

**Try in [go2w_3d_mapping.lua](file:///home/hz/COMP0225_LRC_stack/src/go2_real_bringup/config/cartographer/go2w_3d_mapping.lua):**
```lua
-- Current (default-ish):
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_acceleration_weight = 1e0
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_rotation_weight = 5e0

-- Try reducing accel weight to suppress oscillation:
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_acceleration_weight = 1e-1
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_rotation_weight = 3e0
```

### Alternative: TiNredmc's Full Approach
Their reference implementation uses:
- **URDF + `robot_state_publisher`** for static TFs (not code)
- **Raw IMU** directly to Cartographer (`tracking_frame = "unilidar_imu"`)
- **No `transform_everything`** for IMU — only for point cloud
- This is the textbook-correct approach but requires a proper URDF

## Files Modified (Final State)

| File | Status |
|---|---|
| [transform_everything.py](file:///home/hz/COMP0225_LRC_stack/src/autonomy_stack_go2/src/utilities/transform_sensors/transform_sensors/transform_everything.py) | ✅ Reverted to original (negate+pitch+bias, `frame_id='body'`) |
| [go2w_3d_mapping.lua](file:///home/hz/COMP0225_LRC_stack/src/go2_real_bringup/config/cartographer/go2w_3d_mapping.lua) | ✅ Reverted (`tracking_frame='body'`, correlative matcher off) |
| [imu_calib_data.yaml](file:///home/hz/COMP0225_LRC_stack/imu_calib_data.yaml) | ✅ Restored yesterday's working biases |
| [calibrate_imu.py](file:///home/hz/COMP0225_LRC_stack/calibrate_imu.py) | Modified (gyro-only mode added, original still works) |
| [calibrate_imu.sh](file:///home/hz/COMP0225_LRC_stack/calibrate_imu.sh) | Minor: added pkill at top |
| [go2w_ethernet_start.sh](file:///home/hz/COMP0225_LRC_stack/go2w_ethernet_start.sh) | Minor: added pkill at top |
