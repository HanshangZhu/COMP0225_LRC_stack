# Isaac Sim Lidar Debugging Journey

This document chronicles the debugging process to resolve the "Eraser Circle" issue where the Lidar failed to detect walls, causing the map to be cleared.

## 1. The Issue: "Eraser Circle"
**Symptom**: The robot's local map showed a large empty circle (radius ~1m) around the robot. Walls that should have been visible were being "erased" from the map.
**Implication**: The Lidar was returning `inf` (no return) for all rays, or only detecting the ground/self at close range. The SLAM/Mapper interpreted these "misses" as "free space", effectively clearing any existing map data.

## 2. Hypothesis 1: Configuration & Reflectance
**Analysis**: The simulated walls in Isaac Sim often lack physical material properties (textures), resulting in a simulated reflectivity of `0.0`.
**Finding**: The default Lidar configuration (`Unitree_L1.json`) had `minReflectance: 0.1`. This meant valid geometric hits on the walls were being discarded as "too dark/noise".
**Attempted Fix**: 
- Set `minReflectance: 0.0` to accept all hits.
- Set `nearRangeM: 0.4` to avoid self-collisions with the robot body.
**Result**: **Failure**. The behavior persisted unmodified. We suspected the configuration file was being cached or ignored by Isaac Sim.

## 3. Hypothesis 2: Forced Re-Initialization
**Analysis**: Isaac Sim attempts to attach to existing sensor prims on the robot USD. If the robot USD already had a broken/misconfigured "radar", our config changes would be ignored.
**Attempted Fix**:
- Modified `isaac_t_world_dual_bringup.py` to set `prefer_existing_lidar=False`.
- Forced the creation of a BRAND NEW Lidar sensor using a custom config (`Unitree_L1_Custom.json`).
**Result**: **Failure**. Even with a fresh sensor and correct config, the RTX Lidar failed to see the walls. This pointed to a fundamental issue with RTX sensor rendering or physics material compatibility in this specific Gym environment.

## 4. The Breakthrough: Switching to CPU Lidar
**Strategy**: Bypass the complex RTX (Graphics/Physics) rendering pipeline entirely and use a simplified CPU-based geometric raycaster.
**Action**: Changed `isaac_lidar_mode` from `rtx` to `cpu` in `run_exploration_issac.sh`.
**Result**: **Success!** The walls instantly became visible in the scan.
**Conclusion**: The geometry was correct, but the RTX Lidar sensor was failing to interact with it properly.

## 5. Refining the Solution: "Leakage" & Performance
**New Issue**: While walls were visible, they were "leaky". The scanner showed free space *behind* the walls.
**Cause**: The CPU Lidar default resolution was low (360 rays = 1° spacing). At distance, rays were slipping through gaps or mesh seams, returning `inf` (which cleans the map behind the wall).
**Fix 1 (Resolution)**: Increased `isaac_lidar_rays` to **3600** (0.1° resolution).
**Result**: Solid walls. Leakage eliminated.
**New Issue**: Performance dropped to **0.5Hz**. Casting 3600 rays x 2 robots in a Python `for` loop was too slow.

## 6. Final Optimization: Vectorization
**Fix 2 (Performance)**: Rewrote the raycaster in `isaac_t_world_dual_bringup.py` using `numpy`.
- Replaced the O(N) loop with vectorized broadcasting.
- Computes intersection of 3600 rays against 20+ walls simultaneously.
**Final Result**:
- **High Fidelity**: Solid walls, no leakage.
- **High Performance**: >60Hz real-time simulation.

## 7. Debugging Methodology & Iteration History
This section details how we identified the root cause through systematic isolation.

### Phase 1: Verification
1.  **Symptom**: "Eraser Circle" on map.
2.  **Check 1: Topic Frequency**:
    - `ros2 topic hz /go2_1/scan_3d` -> **10Hz**.
    - **Conclusion**: The sensor driver is running and publishing. The data itself is the problem.
3.  **Check 2: Visual Inspection**:
    - Opened `rviz`.
    - Observed: Scan points only appeared at ~1m radius (ground/self-collision) or were completely `inf`.
    - **Hypothesis**: The sensor is blind to the walls.

### Phase 2: Configuration Isolation
4.  **Hypothesis**: Is the config file being loaded?
    - Added print statements to `isaac_t_world_bringup.py`: `print(f"Lidar Config: {lidar_config}")`.
    - **Result**: Config was `Unitree_L1`.
5.  **Hypothesis**: Is the config just cached/stale?
    - Created `Unitree_L1_Custom.json` (copy).
    - Forced load in launch file.
    - **Result**: No change. The config was loading, but the behavior persisted.
6.  **Hypothesis**: Is the robot blocking the view (Self-Collision)?
    - Modified `isaac_t_world_dual_bringup.py`: Added `translate(0, 0, 0.5)` to raise Lidar above the robot body.
    - **Result**: "Circle persisted".
    - **Conclusion**: Even with clear line-of-sight, the walls were invisible. The issue was **Material Interaction**.

### Phase 3: The "Reflectance" Trap
7.  **Analysis**:
    - Checked `Unitree_L1.json`. Found `minReflectance: 0.1`.
    - **Theory**: Simulated walls often have `0.0` reflectivity (default grey material).
    - **Experiment**: Changed `minReflectance` to `0.0`.
    - **Result**: **Failure**. This strongly suggested the RTX rendering pipeline itself was fundamentally incompatible with the simple geometry in this environment.

### Phase 4: Pivot to CPU Lidar
8.  **Strategy**: Abandon RTX (Graphics/Physics hybrid) for CPU (Pure Physics Raycast).
9.  **Action**: Set `isaac_lidar_mode:=cpu` in launch script.
10. **Result**: **Success!** Visual walls appeared in Rviz immediately.
    - **Cons**: "Leakage" (rays passing through solid walls).

### Phase 5: Solving Leakage & Performance
11. **Diagnosis**: Leakage was due to low angular resolution (360 rays = 1.0°). At 5m distance, the gap between rays was ~8cm, allowing them to slip past wall edges or through seams.
12. **Fix**: Increased rays to **3600** (0.1°).
    - **Result**: Walls became solid. Leakage vanished.
    - **New Problem**: Simulation slowed to **0.5Hz**.
13. **Optimization**:
    - **Profile**: Python loop over 3600 rays x 20 walls = 72,000 raycasts per frame per robot.
    - **Solution**: Vectorized the raycaster using `numpy`.
    - **Code**: `_vectorized_raycast` computes all 3600 intersections in a single matrix operation.
    - **Final Result**: >60Hz performance with high-fidelity scans.

