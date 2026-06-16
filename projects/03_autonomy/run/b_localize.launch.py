"""Option B — tightly-coupled prior-map localization (FAST-LIO locate_in_prior_map).

icp_node bootstraps the initial pose (raw scan vs prior map → /icp_result); fastlio_mapping
then runs the full iEKF localizing IN the prior map (IMU + scan-to-prior-map in one filter);
transform_publisher emits map→odom. Validate offline on the corridor2.0 bag.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node

PRIOR = "/home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/results/prior_map_loopclosed.pcd"
CFG = "/home/nus-ai/ros2_ws/src/Fast-LIO2-Localization/FAST_LIO/config/helios16p_localize.yaml"


def generate_launch_description():
    return LaunchDescription([
        Node(package="icp_relocalization", executable="transform_publisher", name="transform_publisher",
             output="screen",
             parameters=[{"map_frame_id": "map", "odom_frame_id": "odom", "use_sim_time": True}]),
        Node(package="icp_relocalization", executable="icp_node", name="icp_node", output="screen",
             parameters=[{
                 "initial_x": 0.0, "initial_y": 0.0, "initial_z": 0.0, "initial_a": 0.0,
                 "map_path": PRIOR, "map_frame_id": "map",
                 "map_voxel_leaf_size": 0.4, "cloud_voxel_leaf_size": 0.2,
                 "max_correspondence_distance": 1.0, "fitness_score_thre": 0.3,
                 "converged_count_thre": 5, "pcl_type": "pointcloud", "use_sim_time": True,
             }],
             remappings=[("/pointcloud2", "/rslidar_points")]),   # raw scan for the bootstrap ICP
        Node(package="fast_lio", executable="fastlio_mapping", name="laserMapping", output="screen",
             parameters=[CFG, {"use_sim_time": True}],
             remappings=[("/Odometry", "/lio_loc/odometry")]),
    ])
