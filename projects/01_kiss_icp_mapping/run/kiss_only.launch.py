"""KISS-ICP odometry/mapping ONLY (no LiDAR driver, no rviz).

Use when the LiDAR driver is already running. Publishes /kiss/odometry and the
debug clouds (/kiss/local_map, /kiss/frame, /kiss/keypoints) for Foxglove.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    kiss_config = os.path.join(get_package_share_directory("kiss_icp"), "config", "config.yaml")
    return LaunchDescription([
        Node(
            package="kiss_icp",
            executable="kiss_icp_node",
            name="kiss_icp_node",
            output="screen",
            remappings=[("pointcloud_topic", "/rslidar_points")],
            parameters=[
                {
                    "base_frame": "",
                    "lidar_odom_frame": "odom_lidar",
                    "publish_odom_tf": True,
                    "invert_odom_tf": True,
                    "publish_debug_clouds": True,   # /kiss/local_map etc. for Foxglove
                    "use_sim_time": False,          # LIVE sensor
                    "position_covariance": 0.1,
                    "orientation_covariance": 0.1,
                },
                kiss_config,
            ],
        ),
    ])
