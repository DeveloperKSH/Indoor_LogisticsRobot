include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  publish_frame_projected_to_2d = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
}

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7

TRAJECTORY_BUILDER_3D.min_range = 0.1
TRAJECTORY_BUILDER_3D.max_range = 100.0
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05

TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter = {
  max_length = 0.9,
  min_num_points = 100,
  max_range = 50.,
}

TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter = {
  max_length = 2,
  min_num_points = 200,
  max_range = 50.,
}

TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 5.

POSE_GRAPH.optimize_every_n_nodes = 20
MAP_BUILDER.num_background_threads = 6

POSE_GRAPH.global_sampling_ratio = 0.003

POSE_GRAPH.constraint_builder.sampling_ratio = 0.2 --0.4
POSE_GRAPH.constraint_builder.max_constraint_distance = 5

--MAP_BUILDER.pose_graph.constraint_builder.max_constraint_distance = 1.
--MAP_BUILDER.pose_graph.constraint_builder.sampling_ratio = 0.05
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55 -- 0.7

POSE_GRAPH.constraint_builder.min_score = 0.55 -- 0.62

POSE_GRAPH.optimization_problem.huber_scale = 1e2

POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e3
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e3

POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3

POSE_GRAPH.global_constraint_search_after_n_seconds = 250
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.5 --0.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(10.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7
POSE_GRAPH.max_num_final_iterations = 200

POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 40 --10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 10.   --4
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.

--POSE_GRAPH.constraint_builder.loop_closure_translation_weight =1.1e4 --1.1e4
--POSE_GRAPH.constraint_builder.loop_closure_rotation_weight =1e5 --1e5

return options
