"""Offline-validate prior-map relocalization (icp_relocalization) on top of spark-fast-lio.

icp_node ICPs FAST-LIO's world-frame registered cloud (/lio/cloud_registered, frame odom_lio)
against the prior map PCD and publishes map->odom_lio. transform_publisher broadcasts that TF.
Params corrected per code review (max_correspondence_distance raised, fitness_score_thre>0,
feed /lio/cloud_registered NOT a body-frame cloud, odom_frame_id=odom_lio).

NOTE: icp_node's generic path is ONE-SHOT (shuts down after first converged relocalization).
This launch is the FIRST GATE: confirm ICP locks onto the corridor map with the right frame.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node

PRIOR_MAP = "/home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/results/prior_map_loopclosed.pcd"


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="icp_relocalization", executable="icp_node", name="icp_node",
            output="screen",
            parameters=[{
                "initial_x": 0.0, "initial_y": 0.0, "initial_z": 0.0, "initial_a": 0.0,
                "map_path": PRIOR_MAP,
                "map_frame_id": "map",
                "map_voxel_leaf_size": 0.4,
                "cloud_voxel_leaf_size": 0.15,
                "max_correspondence_distance": 1.0,     # raised from 0.1 (was starving ICP)
                "fitness_score_thre": 0.25,             # MUST be > 0 (cpp default 0.0 never fires)
                "converged_count_thre": 5,              # one-shot: lock quickly
                "solver_max_iter": 75,
                "RANSAC_outlier_rejection_threshold": 1.0,
                "pcl_type": "pointcloud",               # non-livox (no-op with USE_LIVOX off, set anyway)
                "use_sim_time": True,
            }],
            remappings=[("/pointcloud2", "/lio/cloud_registered")],   # FAST-LIO WORLD-frame cloud
        ),
        Node(
            package="icp_relocalization", executable="transform_publisher", name="transform_publisher",
            output="screen",
            parameters=[{
                "map_frame_id": "map",
                "odom_frame_id": "odom_lio",            # == frame_id of /lio/cloud_registered
                "use_sim_time": True,
            }],
        ),
    ])
