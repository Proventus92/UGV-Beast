include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link", -- Le repère physique du robot
  published_frame = "base_link", -- Cartographer publie la position du robot
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,

  use_odometry = true,
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


TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 1. Recherche Locale : On autorise une très grande erreur d'angle initiale
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(45.) -- 45 degrés de tolérance !

-- 2. Poids du Scan Matcher (Le Lidar est ROI ABSOLU)
-- On met un poids démesuré sur la rotation du Lidar.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1e4 -- 10,000 ! (C'était 400 avant)

-- 3. Poids de l'Odométrie (On la méprise pour la rotation)
-- On dit à l'optimiseur global : "Ne te base pas sur l'angle des roues".
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e-2 -- 0.01 (Quasiment nul)

-- 4. Filtre d'arrêt (Pour ne pas dériver à l'arrêt)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.02

return options
