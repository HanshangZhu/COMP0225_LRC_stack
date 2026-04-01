-- Cartographer 2D config for real Go2W UTLidar.
--
-- Runs the 2D trajectory builder on 3D PointCloud2 input, giving proper
-- free-space carving in the occupancy grid. Use this when the navigation
-- planner consumes the Cartographer-published map directly (carto_2d mode).
--
-- Adapted from cartographer_sim_2d.lua with real-robot frame conventions:
--   tracking_frame = "body"  (transform_everything publishes IMU in body)
--   published_frame = "body"

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
  publish_frame_projected_to_2d = true,
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

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0
-- Match max_range so rays up to 8 m carve free space instead of being dropped.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0

-- Keep only the wall-height band; exclude ground returns.
-- Real robot: transform_everything corrects the 15.1deg pitch, so the cloud
-- is roughly level in body frame. body is ~0.25m above floor.
-- min_z=0.15 rejects residual ground hits; max_z=0.80 catches walls/obstacles.
TRAJECTORY_BUILDER_2D.min_z = 0.15
TRAJECTORY_BUILDER_2D.max_z = 0.80

-- Asymmetric free-space carving: hits stickier than misses so partially-
-- observed obstacles are not erased by miss rays from the far side.
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.60
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.45

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(3.)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 2.0

-- Global SLAM (loop closure) — same tuning as 3D config
POSE_GRAPH.optimize_every_n_nodes = 60
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4
POSE_GRAPH.global_constraint_search_after_n_seconds = 5.
POSE_GRAPH.optimization_problem.log_solver_summary = false

return options
