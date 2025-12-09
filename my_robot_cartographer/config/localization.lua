include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  
  use_odometry = true, -- On garde le topic pour le timestamp, mais on va mettre son poids a 0
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- === LIDAR + IMU STRATEGY (NO ODOMETRY TRUST) ===

-- 1. IMU IS CRITICAL HERE
-- Because we ignore wheels, IMU gives the rotation hint.
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- 2. INSTANT SCAN MATCHING
-- Don't wait for multiple scans. Snap immediately.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 12.0

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(45.)

-- 3. WEIGHTS: KILL THE ODOMETRY
-- We keep 'use_odometry = true' so Cartographer receives the messages (needed for timing),
-- BUT we set weights to 0 so they don't move the map.

-- Lidar Weight (Infinite)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1e6
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1e6

-- Odometry Weight (ZERO)
-- This creates the "Abandon Odometry" effect you asked for.
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0

-- 4. MOTION FILTER
-- Only calculate if we really moved (seen by Lidar/IMU)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

POSE_GRAPH.optimize_every_n_nodes = 5
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.global_sampling_ratio = 0.003

return options