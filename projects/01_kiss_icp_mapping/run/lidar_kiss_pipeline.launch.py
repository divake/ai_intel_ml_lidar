"""Data-engine launch: LiDAR driver + KISS-ICP odometry/mapping, NO GUI.

Decoupled from any rviz so a viewer crash / stray Ctrl+C in another terminal
can never kill the LiDAR stream or the recording. View with watch_map.sh.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    kiss_config = os.path.join(get_package_share_directory("kiss_icp"), "config", "config.yaml")
    return LaunchDescription([
        # --- LiDAR driver (no rviz) ---
        Node(
            namespace="rslidar_sdk",
            package="rslidar_sdk",
            executable="rslidar_sdk_node",
            output="screen",
            parameters=[{"config_path": ""}],
        ),
        # --- KISS-ICP odometry + map (no rviz); debug clouds ON for the viewer ---
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
                    "publish_debug_clouds": True,   # publish /kiss/local_map etc. without bundled rviz
                    "use_sim_time": False,          # LIVE sensor (not bag playback)
                    "position_covariance": 0.1,
                    "orientation_covariance": 0.1,
                },
                kiss_config,
            ],
        ),
    ])
