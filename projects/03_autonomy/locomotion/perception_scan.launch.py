"""Perception for L1 corridor cruise: LiDAR -> 2D /scan. No TF / localization needed.

Brings up:
  * rslidar_sdk  -> /rslidar_points  (frame_id: rslidar)
  * pointcloud_to_laserscan -> /scan (flattened to a horizontal band, floor excluded)

The scan stays in the LiDAR's own frame (target_frame == cloud frame == rslidar), so
corridor_cruise reads left/right/front distances directly off it — no map, no TF tree.
Band (in rslidar frame): min_height -0.20 (~0.19 m above floor, EXCLUDES floor returns)
to max_height 1.0 (keeps the wall band, drops ceiling). range_min 0.5 ignores the mount.

    ros2 launch perception_scan.launch.py     # (from this dir, with ROS + ws sourced)
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rslidar_start = os.path.join(
        get_package_share_directory("rslidar_sdk"), "launch", "start.py")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(rslidar_start)),
        Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
             name="pointcloud_to_laserscan",
             remappings=[("cloud_in", "/rslidar_points"), ("scan", "/scan")],
             parameters=[{"use_sim_time": False, "target_frame": "rslidar",
                          "transform_tolerance": 0.1, "min_height": -0.20, "max_height": 1.0,
                          "angle_min": -3.14159, "angle_max": 3.14159, "angle_increment": 0.0087,
                          "scan_time": 0.1, "range_min": 0.5, "range_max": 30.0, "use_inf": True}],
             output="screen"),
    ])
