"""Phase C LIVE autonomy stack (teach-and-repeat). NO map_server/planner — we
follow the taught path; the local costmap + collision_monitor handle real obstacles.
Assumes base (scout_cmd) + FAST-LIO2 are ALREADY running (odom_lio->rslidar TF live).
cmd_vel chain: controller(cmd_vel_nav) -> velocity_smoother(cmd_vel_smoothed)
               -> collision_monitor(cmd_vel) -> Scout.
Bringing this up does NOT move the robot — motion only when follow_taught_path.py runs.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node

CFG = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                       "..", "config", "nav2_live.yaml"))


def generate_launch_description():
    return LaunchDescription([
        Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
             name="pointcloud_to_laserscan",
             remappings=[("cloud_in", "/rslidar_points"), ("scan", "/scan")],
             parameters=[{"use_sim_time": False, "target_frame": "rslidar",
                          "transform_tolerance": 0.1, "min_height": -0.5, "max_height": 1.5,
                          "angle_min": -3.14159, "angle_max": 3.14159, "angle_increment": 0.0087,
                          "scan_time": 0.1, "range_min": 0.5, "range_max": 30.0, "use_inf": True}],
             output="screen"),
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             parameters=[CFG], remappings=[("cmd_vel", "cmd_vel_nav")], output="screen"),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother",
             parameters=[CFG], remappings=[("cmd_vel", "cmd_vel_nav")], output="screen"),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor",
             parameters=[CFG], output="screen"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_nav",
             parameters=[{"use_sim_time": False, "autostart": True,
                          "node_names": ["controller_server", "velocity_smoother", "collision_monitor"]}],
             output="screen"),
    ])
