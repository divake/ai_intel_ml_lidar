"""Phase C LIVE autonomy stack (teach-and-repeat) — MODELED robot (project 03).

Frame tree this launch stands up (single connected tree):
    odom_lio --(FAST-LIO2)--> rslidar --(static mount)--> base_link --(URDF)--> wheels/footprint

Prereqs already running:
  * Scout base with publish_odom:=FALSE  (so base_link has ONE parent — see below)
  * project-02 FAST-LIO2 pipeline (publishes odom_lio->rslidar TF + /lio/odometry)

  ⚠ Relaunch the Scout base WITHOUT wheel-odom TF, or base_link gets two parents:
      ros2 launch scout_cmd scout_mini.launch.py publish_odom:=false

No map_server/global planner — we follow the taught path; local costmap + MPPI +
collision_monitor handle real obstacles. cmd_vel chain:
    controller(cmd_vel_nav) -> velocity_smoother(cmd_vel_smoothed) -> collision_monitor(cmd_vel) -> Scout
Bringing this up does NOT move the robot — motion only when follow_taught_path.py runs.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
CFG = os.path.normpath(os.path.join(HERE, "..", "config", "nav2_live.yaml"))
URDF = os.path.normpath(os.path.join(HERE, "..", "config", "scout_mini.urdf"))

with open(URDF, "r") as f:
    ROBOT_DESCRIPTION = f.read()

# rslidar -> base_link : robot body sits below the LiDAR. MEASURE-AND-REFINE.
# z=-0.30 (LiDAR ~0.30 m above body center), yaw≈0 (calibration: mount aligned).
# The lateral (y) term is what matters most for corridor centering — keep ~0 if centered.
MOUNT_RSLIDAR_TO_BASE = ["0", "0", "-0.30", "0", "0", "0", "rslidar", "base_link"]


def generate_launch_description():
    return LaunchDescription([
        # --- Robot self-model: base_link -> base_footprint + 4 wheels ---
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher",
             parameters=[{"use_sim_time": False, "robot_description": ROBOT_DESCRIPTION}],
             output="screen"),
        # --- Bridge the LiDAR pose down to the robot body (connects the two trees) ---
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="rslidar_to_base_link",
             arguments=MOUNT_RSLIDAR_TO_BASE, output="screen"),

        # --- 3D cloud -> 2D scan for the costmap ---
        Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
             name="pointcloud_to_laserscan",
             remappings=[("cloud_in", "/rslidar_points"), ("scan", "/scan")],
             parameters=[{"use_sim_time": False, "target_frame": "rslidar",
                          # rslidar sits ~0.39 m above the floor. Band is in the rslidar
                          # frame: min_height -0.20 (~0.19 m above floor) EXCLUDES floor
                          # returns (the fake-obstacle ring that boxed MPPI in); max 1.0
                          # keeps the wall band, drops ceiling/lights.
                          "transform_tolerance": 0.1, "min_height": -0.20, "max_height": 1.0,
                          "angle_min": -3.14159, "angle_max": 3.14159, "angle_increment": 0.0087,
                          "scan_time": 0.1, "range_min": 0.5, "range_max": 30.0, "use_inf": True}],
             output="screen"),

        # --- Nav2 control: MPPI + costmap + smoother + collision monitor ---
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
