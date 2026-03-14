-- Cartographer 3D config for Unitree Go2 UTLidar — TUNED
-- Based on trajectory ground-truth analysis (14.4m loop, 4.2×3.8m room)
-- Key changes from original:
--   1. Smaller submaps (40 range data) for more pose graph constraints
--   2. More frequent optimization (every 10 nodes)
--   3. Finer voxel/resolution for preserving thin wall features
--   4. Stronger loop closure propagation
--   5. Z-filtering notes for upstream pipeline

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "body",
  published_frame = "body",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  publish_tracked_pose = true,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.num_background_threads = 4
MAP_BUILDER.use_trajectory_builder_3d = true

-- ==================== Local SLAM ====================
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

-- Smaller submaps → more nodes → more constraints in pose graph
-- Original: 60. In a small room, many small submaps beats few large ones.
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 40

TRAJECTORY_BUILDER_3D.min_range = 0.1
TRAJECTORY_BUILDER_3D.max_range = 21.0

-- Finer voxel filter — preserve the few left-wall points
-- Original Carto default is 0.025. Keep small to not discard wall pts.
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.02
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 2.0

-- Higher resolution submaps for thin wall features
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.08      -- was 0.10 default
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.35       -- was 0.45 default

-- Online correlative scan matching — expensive but prevents local minima
-- This is the brute-force matcher that prevents translation drift.
-- Disable for real-time, enable for offline bag replay tuning.
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false
-- If enabled for offline tuning:
-- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.10
-- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20)

-- Constant-velocity extrapolator (NOT imu_based — avoids 14km gravity drift)
TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = false
TRAJECTORY_BUILDER_3D.pose_extrapolator.constant_velocity.imu_gravity_time_constant = 10.0
TRAJECTORY_BUILDER_3D.pose_extrapolator.constant_velocity.pose_queue_duration = 0.001

-- Ceres local scan matcher
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 4
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 3e-1  -- lower: trust scan match more for position
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 5e0      -- moderate: gyro is decent
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50

-- Motion filter
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.08  -- slightly tighter
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(3)
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 5.0

-- ==================== Global SLAM (loop closure) ====================

-- More frequent optimization — costs CPU but small room can handle it
-- Original: 60. With num_range_data=40, this fires every submap.
POSE_GRAPH.optimize_every_n_nodes = 10

-- Constraint builder
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e3
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1.1e3
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5  -- was 0.4, sample more for constraints

-- Lower score thresholds to find more inter-submap constraints
-- The return leg has poor scan quality, needs gentler acceptance
POSE_GRAPH.constraint_builder.min_score = 0.40          -- was 0.45
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55  -- was 0.60

-- Wider search window (room is ~4.5m)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 4.0  -- was 3.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 0.5
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 50

-- Pose graph optimization
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50  -- was 20, more iters for convergence
-- Let loop closure override local SLAM estimates more aggressively
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5   -- default ~1e5, keep
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.acceleration_weight = 1e-2
POSE_GRAPH.optimization_problem.rotation_weight = 1e-2

-- Search for global constraints more frequently
POSE_GRAPH.global_constraint_search_after_n_seconds = 2  -- was 3

-- Logging
POSE_GRAPH.log_residual_histograms = false
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.optimization_problem.log_solver_summary = false

return options
