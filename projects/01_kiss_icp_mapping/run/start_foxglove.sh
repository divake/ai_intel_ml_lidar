#!/bin/bash
# VIEWER BRIDGE: serve ROS topics to Foxglove (renders on YOUR laptop's GPU).
# Connect from laptop: Foxglove -> Open connection -> Foxglove WebSocket ->
#   ws://100.120.151.19:8765
# Then add a 3D panel, enable /kiss/local_map + /kiss/odometry, Fixed Frame = odom_lidar.
#
# This is just a data bridge — safe to Ctrl+C; it does NOT touch the LiDAR or recording.
source /opt/ros/jazzy/setup.bash
source /home/nus-ai/ros2_ws/install/setup.bash
echo "Foxglove bridge on ws://0.0.0.0:8765  (laptop connects to ws://100.120.151.19:8765)"
exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 address:=0.0.0.0
