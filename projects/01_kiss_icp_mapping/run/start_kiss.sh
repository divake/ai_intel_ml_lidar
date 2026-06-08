#!/bin/bash
# Start KISS-ICP only (LiDAR driver must already be running). No rviz.
# Publishes /kiss/odometry + /kiss/local_map for Foxglove. Leave this running.
set -e
source /opt/ros/jazzy/setup.bash
source /home/nus-ai/ros2_ws/install/setup.bash
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Starting KISS-ICP (no GUI). Watch /kiss/local_map in Foxglove (frame odom_lidar)."
exec ros2 launch "$DIR/kiss_only.launch.py"
