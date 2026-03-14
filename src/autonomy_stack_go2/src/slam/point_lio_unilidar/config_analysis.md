# Point LIO Unilidar Configuration Analysis

## Overview
Analysis of the configuration files in `src/autonomy_stack_go2/src/slam/point_lio_unilidar`.

### 1. Sensor Configuration (`config/unilidar.yaml`)
- **Lidar Topic**: `/unilidar/cloud`
- **IMU Topic**: `/unilidar/imu`
- **Lidar Type**: 5 (Unitree specific)
- **Scan Lines**: 18
- **Blind Spot**: 0.5m
- **IMU Integration**:
  - `imu_en`: true
  - `imu_time_inte`: 0.004 (250 Hz)
  - `extrinsic_est_en`: false (Fixed extrinsics)

### 2. Extrinsics
- **Translation (IMU to Lidar)**: `[0.007698, 0.014655, -0.00667]`
- **Rotation**: Identity (Aligned)
- **Gravity Alignment**: `true` with gravity vector `[0.0, 0.0, -9.810]`

### 3. Launch Settings (`launch/mapping_unilidar.launch`)
- **IMU Input**: `use_imu_as_input` set to `0` (False).
- **Propagation**: `prop_at_freq_of_imu` set to `1` (True).
- **Downsampling**: `space_down_sample` set to `1` (True).
- **Filter Size**: 0.1m for both surf and map.
