"""Phase A: offline loop-closure mapping of the corridor2.0 bag.
Runs pointcloud_to_laserscan (3D /rslidar_points -> 2D /scan) + slam_toolbox
(async, mapping mode, loop closure). Play the bag separately with --clock.
No robot motion. View in Foxglove (map + /scan) if desired.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node

CONFIG = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                      "..", "config", "slam_toolbox_mapping.yaml")


def generate_launch_description():
    return LaunchDescription([
        # 3D LiDAR -> 2D laser scan (horizontal slice around the sensor plane)
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            remappings=[("cloud_in", "/rslidar_points"), ("scan", "/scan")],
            parameters=[{
                "use_sim_time": True,
                "target_frame": "rslidar",
                "transform_tolerance": 0.1,
                "min_height": -0.5,
                "max_height": 1.5,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0087,   # 0.5 deg
                "scan_time": 0.1,
                "range_min": 0.5,
                "range_max": 30.0,
                "use_inf": True,
            }],
            output="screen",
        ),
        # 2D SLAM with loop closure
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[os.path.normpath(CONFIG)],
            output="screen",
        ),
    ])
