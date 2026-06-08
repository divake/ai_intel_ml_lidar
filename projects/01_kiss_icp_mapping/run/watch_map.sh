#!/bin/bash
# VIEWER: live map in rviz on the VNC desktop (display :2).
# Safe to Ctrl+C / close anytime — it does NOT affect the LiDAR or recording.
#
# NOTE: ~/.bashrc forces DISPLAY=:0 (physical monitor); we override to :2 (VNC)
# and force software OpenGL (the VNC display has no GPU).
export DISPLAY=:2
export LIBGL_ALWAYS_SOFTWARE=1
export XDG_RUNTIME_DIR=/run/user/1000
source /opt/ros/jazzy/setup.bash
source /home/nus-ai/ros2_ws/install/setup.bash
RVIZ=/home/nus-ai/ros2_ws/install/kiss_icp/share/kiss_icp/rviz/kiss_icp.rviz
echo "Opening rviz on the VNC desktop (:2). Fixed Frame odom_lidar, shows /kiss/local_map."
exec rviz2 -d "$RVIZ"
