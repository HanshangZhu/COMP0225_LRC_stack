# Open Problems

## 1) RTX lidar "circle" / sparse scan behavior persists
- Status: unresolved
- Symptom:
  - Lidar data is present, but exploration still behaves like low-information circular scans and map occupancy remains mostly free/unknown.
  - Frontiers repeat and robots do not progress to meaningful motion goals.
- What is known:
  - `Unitree_L1` often returned empty RTX annotator fields (`numChannels=0`, `numReturnsPerScan=0`) in prior runs.
  - `NVIDIA/Debug_Rotary` returns non-empty data, but it is effectively single-ring behavior and likely too sparse for robust mapping/navigation.
  - No `Velodyne VLP-16` profile exists in this Isaac installation.
  - Available Velodyne profile is `Velodyne/Velodyne_VLS128`.
- Immediate next checks:
  - Run headless A/B tests with `Velodyne/Velodyne_VLS128` and one Ouster multi-beam profile.
  - Capture:
    - `/go2_*/lidar/points` width/fields/rate
    - RTX DIAG (`numChannels`, `numReturnsPerScan`, array sizes)
    - mapper occupancy growth (`occ` should become non-zero over time)
  - If multi-beam profiles still fail, pivot to CPU lidar for autonomy while keeping RTX as an R&D branch.

## 2) Joint loading errors (static joint creation) in kinematic mode
- Status: mitigated in local changes, needs regression verification
- Prior symptom:
  - `PhysicsUSD: CreateJoint - cannot create a joint between static bodies`
  - `ArticulationRootAPI definition on a kinematic rigid body is not allowed`
- Local fix direction:
  - Disable rigid-body simulation + disable joints + strip articulation root APIs for kinematic mode.
- Verification still required:
  - Confirm no joint/articulation errors across multiple launches and both robots.

## 3) Navigation stack remains in `no_goal` / startup loops
- Status: unresolved
- Symptom:
  - `reactive_nav`: `mode=no_goal` with zero cmd output.
  - `simple_frontier_explorer`: repeated stale costmap fallback.
- Working hypothesis:
  - Upstream sensing quality (point cloud structure/density) is insufficient, so costmap/frontier pipeline never stabilizes.
- Next checks:
  - Confirm costmap topic freshness and transform consistency.
  - Add short-term metrics logging for goal selection decisions.

## 4) Branching / reproducibility hygiene
- Status: open
- Need:
  - Keep Isaac debugging work isolated in a dedicated branch (`issac`) and avoid mixing with unrelated Gazebo/nav refactors.
