-- Cartographer 2D SLAM config for MAPPING (albomb, real car only).
--
-- Based on ForzaETH race_stack (stack_master/config/NUC2/slam/f110_2d.lua).
--
-- Architecture note ("early fusion"):
--   VESC wheel odom + VESC IMU  ->  robot_localization EKF (ekf.yaml, publish_tf: false)
--                               ->  /early_fusion/odom  ->  cartographer `odom` input
--   Cartographer consumes LiDAR + the fused odom and OWNS the TF (publish_to_tf = true).
--
--   The IMU is therefore fused in the EKF, NOT fed to cartographer directly
--   (use_imu_data = false below). Feeding it here would force
--   tracking_frame = "ego_racecar/imu", because cartographer hard-CHECKs that the
--   IMU frame is colocated with the tracking frame (translation < 1e-5), and our
--   IMU sits at (0.07, 0, 0.09) from base_link in albomb_description.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "ego_racecar/base_link",
  -- published_frame = base_link means cartographer emits map->base_link directly.
  -- This is NOT REP-105 compliant (no odom frame appears in TF), but it keeps the
  -- drifty VESC odometry out of the TF path between the localizer and the car.
  -- For REP-105, set provide_odom_frame = true instead of changing published_frame.
  published_frame = "ego_racecar/base_link",
  odom_frame = "odom",
  provide_odom_frame = false,

  use_odometry = true,   -- /early_fusion/odom from the EKF
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

  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.5,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,

  publish_to_tf = true,
  publish_tracked_pose = true,
  publish_frame_projected_to_2d = true,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.use_trajectory_builder_3d = false
MAP_BUILDER.num_background_threads = 3.0

-- IMU goes through the EKF, not through cartographer. See header note.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 25.0
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100

-- Loosen the scan matcher so it trusts the scan more than the motion prior.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight =
  0.2 * TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight =
  0.2 * TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight

POSE_GRAPH.optimize_every_n_nodes = 20
-- Odometry is a front-end initial-guess hint only; it must never act as a
-- back-end constraint. The VESC is sensorless (ERPM from the back-EMF flux
-- observer, with a hardcoded 0.05 m/s deadband) and its yaw is dead-reckoned
-- from the servo command, so wheel slip and low-speed dropout would poison the
-- pose graph. ForzaETH and UNIST independently zero these same two weights.
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0

return options
