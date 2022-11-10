-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

--https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true, --True for the cartographer to provide the ODOM in this case the use_odometry is false
  use_odometry = false,
  use_laser_scan = true,
  use_multi_echo_laser_scan = false,
  --use_landmarks = true,
  --use_nav_sat = false, -- to use GPS
  --num_laser_scans = 1, --sensor_msgs/LaserScan topics
  --num_multi_echo_laser_scans = 0, --sensor_msgs/MultiEchoLaserScan topics
  num_point_clouds = 0, --sensor_msgs/PointCloud2 topics
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
}

--###2D SLAM
MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
--TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 100
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 20.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

SPARSE_POSE_GRAPH.optimization_problem.huber_scale = 1e2
SPARSE_POSE_GRAPH.optimize_every_n_scans = 35
SPARSE_POSE_GRAPH.constraint_builder.min_score = 0.65




--###3D SLAM
--TRAJECTORY_BUILDER_3D.scans_per_accumulation = 160
--TRAJECTORY_BUILDER_3D.min_range = 0.5
--TRAJECTORY_BUILDER_3D.max_range = 20.
--TRAJECTORY_BUILDER_3D.submaps.num_range_data = 40.
--TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 100

--MAP_BUILDER.use_trajectory_builder_3d = true
--MAP_BUILDER.num_background_threads = 7
--SPARSE_POSE_GRAPH.optimization_problem.huber_scale = 5e2
--SPARSE_POSE_GRAPH.optimize_every_n_scans = 320
--SPARSE_POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
--SPARSE_POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
	-- Reuse the coarser 3D voxel filter to speed up the computation of loop closure
	-- constraints.
--SPARSE_POSE_GRAPH.constraint_builder.adaptive_voxel_filter = TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter
--SPARSE_POSE_GRAPH.constraint_builder.min_score = 0.62
--SPARSE_POSE_GRAPH.constraint_builder.log_matches = true

return options
