"""Phase B localization: map_server + AMCL + pointcloud_to_laserscan, brought to
ACTIVE by a nav2 lifecycle_manager (autostart). For the OFFLINE test, replay the
corridor2.0 bag with --clock alongside this. AMCL localizes /scan vs the grid and
publishes map -> odom_lio.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node

CONFIG = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                          "..", "config", "localization.yaml"))


def generate_launch_description():
    return LaunchDescription([
        Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
             name="pointcloud_to_laserscan",
             remappings=[("cloud_in", "/rslidar_points"), ("scan", "/scan")],
             parameters=[{"use_sim_time": True, "target_frame": "rslidar",
                          "transform_tolerance": 0.1, "min_height": -0.5, "max_height": 1.5,
                          "angle_min": -3.14159, "angle_max": 3.14159, "angle_increment": 0.0087,
                          "scan_time": 0.1, "range_min": 0.5, "range_max": 30.0, "use_inf": True}],
             output="screen"),
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             parameters=[CONFIG], output="screen"),
        Node(package="nav2_amcl", executable="amcl", name="amcl",
             parameters=[CONFIG], output="screen"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization",
             parameters=[{"use_sim_time": True, "autostart": True,
                          "node_names": ["map_server", "amcl"]}],
             output="screen"),
    ])
