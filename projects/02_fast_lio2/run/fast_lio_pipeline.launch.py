"""Project 02 data engine: LiDAR driver + D455 (IMU) + spark-fast-lio, NO GUI.

Same decoupled philosophy as project 01: no viewer in this launch, so a crashed
viewer or stray Ctrl+C elsewhere can never kill the sensors or the recording.
View via Foxglove (start_foxglove.sh).

Notes (hard-won, do not "simplify"):
- The D455 must run with color+depth ENABLED — an IMU-only config makes the
  driver fail with `device busy` errors on this rig (verified 2026-06-12).
- spark_fast_lio subscribes to literal topics `lidar` and `imu` -> remapped here.
- The IMU subscriber is SensorDataQoS (best_effort), matching the camera. OK.
- /rslidar_points is now XYZIRT (ring + per-point timestamp -> real deskew);
  this is a compile-time option of rslidar_sdk (see project README).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

CONFIG = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                      "..", "config", "helios16p_d455.yaml")


def generate_launch_description():
    return LaunchDescription([
        # --- LiDAR driver (XYZIRT build) ---
        Node(
            namespace="rslidar_sdk",
            package="rslidar_sdk",
            executable="rslidar_sdk_node",
            output="screen",
            parameters=[{"config_path": ""}],
        ),
        # --- D455: IMU @200 Hz (color+depth must stay enabled, see header) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch", "rs_launch.py")),
            launch_arguments={
                "enable_color": "true",
                "enable_depth": "true",
                "enable_gyro": "true",
                "enable_accel": "true",
                "unite_imu_method": "2",
                "accel_fps": "200",
                "gyro_fps": "200",
            }.items(),
        ),
        # --- spark-fast-lio (LiDAR-inertial odometry) ---
        Node(
            package="spark_fast_lio",
            executable="spark_lio_mapping",
            name="lio_mapping",
            output="screen",
            remappings=[
                ("lidar", "/rslidar_points"),
                ("imu", "/camera/camera/imu"),
                ("odometry", "/lio/odometry"),
                ("path", "/lio/path"),
                ("cloud_registered", "/lio/cloud_registered"),
            ],
            parameters=[
                {
                    "common.lidar_frame": "rslidar",
                    "common.imu_frame": "camera_imu_optical_frame",
                    "common.map_frame": "odom_lio",
                    "common.base_frame": "rslidar",
                    "common.visualization_frame": "lidar",  # literal selector: lidar|base|imu (NOT a frame name)
                    # DISABLED on purpose (see FINDINGS #5/#9): when enabled, the node
                    # GATES mapIncremental + /lio/cloud_registered behind ~2 s of motion
                    # (spark_fast_lio.cpp:1129) -> a parked robot builds no map and the
                    # growing-map view stays empty until you drive, and the START of the
                    # trajectory (pre-alignment) is poorly tracked. The only thing it buys
                    # is rotating world +z to true-up, which is cosmetic (our mount is
                    # level) and does NOT affect the loop-error metric or the EKF's own
                    # gravity estimate (flat floor is retained). Off = deterministic from t=0.
                    "gravity_alignment.enable_gravity_alignment": False,
                },
                os.path.normpath(CONFIG),
            ],
        ),
    ])
