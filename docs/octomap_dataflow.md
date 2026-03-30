# OctoMap Occupancy Grid — Full Data Flow

End-to-end pipeline from Gazebo lidar to `/robot/map` OccupancyGrid
when `map_backend=octomap`.

---

## Pipeline Overview

```
┌─────────────────┐     ┌─────────────┐     ┌──────────────────────────┐     ┌─────────────────┐     ┌──────────────┐
│  Gazebo gpu_ray  │────▶│  QoS Bridge │────▶│ PointCloud Frame Bridge  │────▶│  OctoMap Server  │────▶│  /robot/map   │
│  (livox_mid360)  │     │             │     │                          │     │                  │     │ OccupancyGrid │
└─────────────────┘     └─────────────┘     └──────────────────────────┘     └─────────────────┘     └──────────────┘
        │                                             ▲
        │                                             │ TF: map → base_link
        │                                   ┌─────────┴─────────┐
        └──────────────────────────────────▶│  Cartographer 3D   │
                                            │  (SLAM / TF only)  │
                                            └────────────────────┘
```

---

## Stage 1: Gazebo GPU Ray Sensor

**File:** `src/go2_gazebo_sim/urdf/go2w/go2w_description_3d_lidar.xacro` (lines 792–829)

**What it does:** Casts 1024×16 rays in Gazebo physics, produces a PointCloud2
at 10 Hz. All rays are instantaneous (single sim timestep — no intra-scan
motion distortion in simulation).

| IO | Detail |
|----|--------|
| **Input** | Gazebo world geometry (ray intersection) |
| **Output topic** | `/{ns}/registered_scan` |
| **Output msg** | `sensor_msgs/PointCloud2` |
| **Output frame** | `livox_mid360` |
| **Output QoS** | Best Effort |

**Sensor parameters:**
- Horizontal: 1024 samples, ±180° (full 360°)
- Vertical: 16 samples, ±15°
- Range: 0.2 m – 20.0 m

**Mount geometry** (`src/go2_gazebo_sim/urdf/go2w/livox_mid360.xacro`):
- Offset from IMU: `x=0.187 m, z=0.080 m`
- Pitch: **13° downward** (0.2269 rad)
- Effective z-height above base_link: ~10–15 cm

**Corruption risk:** None in simulation (instantaneous capture).

---

## Stage 2: QoS Bridge

**File:** `src/go2w_perception/scripts/qos_bridge.py`
**Launch:** `src/go2_gazebo_sim/launch/single_go2w_gazebo_cfpa2.launch.py` (lines 396–409)

**What it does:** Pure pass-through — re-publishes the same message with
Reliable QoS so downstream Reliable subscribers can receive it.
Zero filtering, zero transformation.

| IO | Detail |
|----|--------|
| **Input topic** | `/{ns}/registered_scan` |
| **Input msg** | `sensor_msgs/PointCloud2` |
| **Input frame** | `livox_mid360` |
| **Input QoS** | Best Effort, depth=5 |
| **Output topic** | `/{ns}/registered_scan_reliable` |
| **Output msg** | `sensor_msgs/PointCloud2` (identical bytes) |
| **Output frame** | `livox_mid360` (unchanged) |
| **Output QoS** | Reliable, depth=5 |

**Corruption risk:** None (identity operation).

---

## Stage 3: Cartographer 3D SLAM (TF provider)

**File:** `src/vlm_explorer/config/cartographer_sim_3d.lua`
**Launch:** `src/vlm_explorer/launch/single_vlm_gazebo_far.launch.py` (lines 221–240)

**What it does:** Full 3D SLAM — scan-matches incoming clouds against a 3D
submap, runs pose graph optimization, and publishes the `map → odom → base_link`
TF tree. Does **not** produce an occupancy grid in this configuration.

| IO | Detail |
|----|--------|
| **Input topic** | `/{ns}/registered_scan_reliable` (remapped to `points2`) |
| **Input msg** | `sensor_msgs/PointCloud2` |
| **Input frame** | `livox_mid360` |
| **Input 2** | `/{ns}/imu/data` (`sensor_msgs/Imu`, frame `imu`) |
| **Output** | TF: `map → odom → base_link` on `/{ns}/tf` |
| **Output rate** | ~200 Hz (pose_publish_period = 5 ms) |

**Key config:**
- `tracking_frame`: `imu`
- `published_frame`: `base_link`
- `num_point_clouds`: 1
- `lookup_transform_timeout_sec`: 1.0 s

**Corruption risk:** Pose graph loop closures can cause discrete jumps in
`map → odom`. Any cloud inserted during a jump gets placed at a suddenly-shifted
pose.

---

## Stage 4: PointCloud Frame Bridge

**File:** `src/go2w_perception/scripts/pointcloud_frame_bridge.py`
**Launch (FAR backend):** `src/go2_gazebo_sim/launch/single_go2w_gazebo_cfpa2.launch.py` (lines 950–969)
**Launch (non-FAR + octomap):** `src/vlm_explorer/launch/single_vlm_gazebo_far.launch.py` (lines 294–312)

**What it does:** Buffers incoming sensor-frame clouds, waits for the
corresponding TF to become available (up to 0.3 s), then transforms the
entire cloud to the `map` frame as a rigid body. **No motion compensation /
deskewing** — the cloud is transformed with the single pose at `header.stamp`.

| IO | Detail |
|----|--------|
| **Input topic** | `/{ns}/registered_scan_reliable` |
| **Input msg** | `sensor_msgs/PointCloud2` |
| **Input frame** | `livox_mid360` |
| **Input QoS** | Best Effort (sensor data profile) |
| **Output topic** | `/{ns}/registered_scan_map` (FAR) or `/{ns}/registered_scan_map_octo` (non-FAR) |
| **Output msg** | `sensor_msgs/PointCloud2` (same point count, transformed xyz) |
| **Output frame** | `map` |
| **Output QoS** | Reliable, depth=10 |

**Timing parameters:**
| Parameter | Value | Meaning |
|-----------|-------|---------|
| `tf_timeout_sec` | 0.1 s | Max time to wait for a single TF lookup |
| `transform_wait_sec` | 0.3 s | Max time to hold a cloud waiting for TF |
| `max_cloud_age_sec` | 2.0 s | Drop clouds older than this |

**TF chain used:**
```
map → odom → base_link → base → imu → livox_mid360_base → livox_mid360
      └─ Cartographer ─┘   └──────── URDF static TF ────────────────┘
```

**Corruption risk:**
- **TF latency smear:** If Cartographer's TF for time `t` arrives after the
  0.3 s deadline, the cloud is dropped (safe) or transformed with a slightly
  stale pose (causes spatial shift).
- **No motion compensation:** Robot movement during the 100 ms scan window
  is not corrected. In sim this is ~zero (instantaneous rays), but on real
  hardware this would cause streaking.

---

## Stage 5: OctoMap Server

**File:** `src/go2_real_bringup/config/cartographer/octomap_mapping.yaml`
**Launch:** `src/vlm_explorer/launch/single_vlm_gazebo_far.launch.py` (lines 315–328)

**What it does:** Maintains a 3D octree of occupancy probabilities. For each
incoming cloud, casts rays from sensor origin through each point, updating
voxels with Bayesian log-odds. Projects the 3D octree to a 2D OccupancyGrid.

| IO | Detail |
|----|--------|
| **Input topic** | `/{ns}/registered_scan_map` (or `_octo`) |
| **Input msg** | `sensor_msgs/PointCloud2` |
| **Input frame** | `map` |
| **TF lookup** | `map → base_link` at `header.stamp` (for sensor origin) |
| **Output topic** | `/{ns}/map` (remapped from `projected_map`) |
| **Output msg** | `nav_msgs/OccupancyGrid` (0=free, 100=occupied, -1=unknown) |
| **Output frame** | `map` |

### Internal processing steps

#### 5a. Height filtering (input cloud)
Discard points outside the z-band **before** inserting into the octree:
```
point_cloud_min_z: -0.05 m   (exclude ground returns)
point_cloud_max_z:  0.80 m   (exclude ceiling)
```

#### 5b. Ground plane filtering
```
filter_ground_plane: true
ground_filter.distance: 0.04 m
ground_filter.angle: 0.15 rad
ground_filter.plane_distance: 0.07 m
```
RANSAC-fits a ground plane and removes inlier points. Helps with the 13°
downward-pitched lidar that sees the floor at close range.

#### 5c. Sensor origin & raycasting
OctoMap looks up `base_link` in `map` frame at the cloud's `header.stamp`
to find the sensor origin. Then for each remaining point:
- **Miss ray:** Every voxel from sensor origin to the point gets a free update
- **Hit voxel:** The endpoint voxel gets an occupied update

Log-odds update rule per voxel:
```
l(v) ← l(v) + log(p / (1-p))

For a hit:  Δl = log(0.65 / 0.35) = +0.619
For a miss: Δl = log(0.30 / 0.70) = -0.847

Clamped to: [log(0.12/0.88), log(0.97/0.03)] = [-1.99, +3.48]
```

**Key ratio:** One miss (−0.85) more than cancels one hit (+0.62). This
means free-space carving is now slightly **stronger** than accumulation —
transient occupied hits will clear within 1–2 sweeps.

#### 5d. 3D → 2D projection
```
occupancy_min_z: -0.05 m
occupancy_max_z:  0.60 m
use_height_map: true
```
For each (x, y) column in the octree, within the z-band [-0.05, 0.60]:
- `use_height_map: true` → cell = **max occupied probability** in the column
- If max P(occ) > 0.5 → cell = 100 (occupied)
- If max P(occ) ≤ 0.5 and column was observed → cell = 0 (free)
- If never observed → cell = -1 (unknown)

#### 5e. Speckle filtering
```
filter_speckles: true
```
Removes isolated occupied voxels with no occupied neighbors (single-voxel
noise from stray lidar returns).

### Corruption sources (THIS IS THE CURRENT PROBLEM)

1. **`use_height_map: true` + max-projection:** Same mathematical problem as
   Cartographer 3D. If ANY voxel in a column at any z in [-0.05, 0.60] is
   occupied, the 2D cell is occupied. A single noisy hit at z=0.3 that hasn't
   been cleared yet makes the whole column black.

2. **Sensor origin TF race:** Even though the cloud is pre-transformed to
   `map` frame, OctoMap still does a **separate TF lookup** for the sensor
   origin (needed for raycasting). If this TF is stale/wrong, rays originate
   from the wrong position → free-space carving misses the right cells and
   occupied cells accumulate at wrong locations.

3. **Incremental 2D projection:** `incremental_2D_projection: true` means
   the 2D grid is only updated for voxels that changed. If a voxel was
   previously occupied and is now cleared in 3D, the 2D cell might not
   update if the projection logic doesn't re-check the full column.

---

## Summary: Where corruption enters

```
Gazebo sensor ──── clean (instantaneous rays)
      │
QoS bridge ─────── clean (identity)
      │
Cartographer ───── TF can jump on loop closure
      │
Frame bridge ───── single-pose transform (no deskew)
      │                                           ┌──────────────────────────┐
      └──▶ OctoMap ──▶ 3D octree ──▶ 2D projection │ ◀── CORRUPTION HERE    │
                ▲                                   │  • max-projection       │
                │                                   │  • sensor origin TF lag │
                │                                   │  • incremental update   │
           TF lookup for                            │    stale columns        │
           sensor origin                            └──────────────────────────┘
           (separate from
            cloud transform)
```

The cloud points themselves are correctly placed in the map frame by the time
they reach OctoMap. The corruption comes from OctoMap's **raycasting** using a
potentially misaligned sensor origin, and from the **max-projection** 2D
conversion amplifying any residual noise into occupied cells.
