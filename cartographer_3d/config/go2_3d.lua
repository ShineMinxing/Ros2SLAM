include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_imu",
  published_frame = "odom",

  odom_frame = "odom",
  provide_odom_frame = false,

  publish_frame_projected_to_2d = false,

  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,

  lookup_transform_timeout_sec  = 1.0,
  submap_publish_period_sec     = 0.3,
  pose_publish_period_sec       = 0.02,
  trajectory_publish_period_sec = 0.2,

  rangefinder_sampling_ratio      = 1.0,
  odometry_sampling_ratio         = 1.0,
  fixed_frame_pose_sampling_ratio = 0.0,
  imu_sampling_ratio              = 1.0,
  landmarks_sampling_ratio        = 0.0,
}

MAP_BUILDER.use_trajectory_builder_3d = true

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 5
TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 30.0
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.25

TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.0
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length  = 4.0
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points  = 200

TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight    = 10.0

POSE_GRAPH.optimization_problem.odometry_translation_weight = 10000.0
POSE_GRAPH.optimization_problem.odometry_rotation_weight    = 10000.0
POSE_GRAPH.optimization_problem.acceleration_weight         = 1.0
POSE_GRAPH.optimization_problem.rotation_weight             = 1.0

POSE_GRAPH.optimize_every_n_nodes = 100
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

return options
