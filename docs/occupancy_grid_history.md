# Occupancy Grid: Evolution and Backend Comparison

## The Smearing Problem

The VLM exploration stack needs a 2D OccupancyGrid (`/robot/map`) with proper **free / occupied / unknown** semantics for:
- CFPA2 frontier extraction (needs explicit free-to-unknown boundaries)
- reactive_nav grid planning (needs free-space corridors to plan A* paths)

Getting a clean 2D map from a 3D LiDAR + SLAM stack turned out to be non-trivial.

---

## Backend History

### 1. Cartographer 3D Occupancy Grid (original, abandoned)

The first approach used Cartographer's 3D trajectory builder and its built-in `cartographer_occupancy_grid_node` to project the 3D occupancy grid to 2D.

**Problem:** The projected 2D map was almost entirely "occupied + unknown" with nearly zero explicit free cells. Cartographer's 3D builder does not do proper free-space carving in 2D projection — it marks cells that were hit as occupied, but doesn't trace free-space rays through the 2D plane. CFPA2 could not find frontiers and reactive_nav could not plan paths.

### 2. OctoMap (experimental, had radial spoke artifacts)

OctoMap (`octomap_server`) builds a 3D occupancy tree and projects to 2D with free-space carving.

**Problem:** OctoMap uses `cloud->header.frame_id` as **both** the point transform frame **and** the sensor ray origin for raycasting. When the cloud was in the `base_link` frame, OctoMap cast rays from (0,0,0) in `base_link` — the robot's body center, not the LiDAR mount point. This created radial spoke artifacts in the 2D projection: phantom occupied rings radiating from the robot center.

**Fix:** `pointcloud_frame_bridge.py` re-expresses the cloud in the physical `livox_mid360` sensor frame before OctoMap ingestion, so rays originate from the actual LiDAR position. An additional `pointcloud_octomap_filter.py` rejects ground returns and high-yaw-rate scans in sensor frame before insertion. This path (`map_backend=octomap`) works but adds ~3 extra nodes and is sensitive to TF timing.

### 3. simple_scan_mapper_cpp (first working default)

A custom ray-tracing mapper that works entirely in 2D:
1. Subscribes to `scan_3d` (LaserScan, 2D-projected from the 3D cloud by `pointcloud_to_laserscan`)
2. Looks up the robot pose via TF (from Cartographer or Fast-LIO)
3. Ray-traces each laser beam: marks endpoint as occupied, traces free cells along the ray

**Why it worked:** By operating directly on the 2D LaserScan, it sidesteps all 3D-to-2D projection issues. Every ray either hits something (occupied) or reaches max range (free along path, unknown beyond). The resulting map always has clean free/occupied/unknown semantics.

**Limitations:**
- Sensitive to scan-odom timing synchronization — drops scans when `dt > max_scan_odom_dt` (0.1s), which is common under CPU load, leaving temporal gaps in the map
- No scan matching or drift correction — accumulated pose error from upstream SLAM appears directly as map distortion
- No submap architecture — the single global map accumulates noise and cannot self-correct
- Occupancy updates are binary (one hit = occupied) with no probabilistic persistence — transient noise becomes permanent map clutter
- The 2D projection in `pointcloud_to_laserscan` loses vertical structure, so short obstacles below the projection band disappear

### 4. carto_2d (current default for VLM demo)

Runs Cartographer's **2D trajectory builder** directly on the raw 3D PointCloud2. Cartographer filters the 3D cloud to a configurable Z-band (`min_z`/`max_z` in the tracking frame) and uses the resulting 2D slice for both scan matching and map building.

**Why it is more robust:**
- **Joint optimization:** The map and pose estimates are jointly optimized by Cartographer's Ceres-based scan matcher. Pose drift is corrected before rays are inserted into the map, so the map stays self-consistent even over long exploration runs.
- **Submap architecture with loop closure:** The map is built from overlapping submaps. When the robot revisits an area, Cartographer's pose graph optimizer (loop closure) retroactively corrects accumulated drift across the entire trajectory. simple_scan_mapper has no mechanism for this.
- **Probabilistic occupancy:** Each cell's occupancy is maintained as a log-odds probability, updated with configurable hit/miss weights. A single noisy observation doesn't permanently corrupt the map — it takes multiple consistent observations to change cell state. (The downstream `probability_grid_binarizer` then discretizes probabilities into {free, occupied, unknown} for the planner.)
- **No scan-odom timing sensitivity:** Cartographer handles its own time synchronization between IMU, point cloud, and its internal pose estimate. There is no external `max_scan_odom_dt` threshold that drops data.
- **Direct 3D input:** Takes the raw PointCloud2 without a lossy intermediate `pointcloud_to_laserscan` step. The Z-band filter (`min_z=0.05, max_z=0.60`) is applied in the IMU tracking frame, giving precise control over which height band contributes to the 2D map.
- **Proper free-space carving:** `insert_free_space=true` with configurable `hit_probability` and `miss_probability` means every ray explicitly carves free space along its path. The asymmetric tuning (hit=0.70, miss=0.40) ensures obstacles persist even when later rays pass through from the far side (~2 misses needed to cancel 1 hit).

**Known issue:** Ground returns from the pitched LiDAR can appear as obstacle rings near the robot when the robot's pitch oscillates during locomotion. Addressed by raising `min_z` to exclude the ground-return band.

---

## Current Configuration (VLM demo)

| Setting | Value |
|---------|-------|
| SLAM | Cartographer 3D (`cartographer_sim_3d.lua`) for pose/TF |
| Map backend | `carto_2d` — Cartographer 2D builder on PointCloud2 |
| Z-band | `min_z=0.05, max_z=0.60` (IMU frame) |
| Hit/miss | `hit=0.70, miss=0.40` (asymmetric, obstacles persist) |
| Binarizer | `free<=25, occupied>=50, min_component=2 cells` |
| Resolution | 5 cm |

The `scan` backend remains available as a lightweight fallback (`map_backend:=scan` on the launch command line).
