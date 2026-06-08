#!/bin/bash
# DATA ENGINE: LiDAR + KISS-ICP (no GUI). Run in one VNC terminal and leave it.
# Ctrl+C here stops the pipeline (intentional). The viewer/recorder are separate.
set -e
source /opt/ros/jazzy/setup.bash
source /home/nus-ai/ros2_ws/install/setup.bash
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Starting LiDAR + KISS-ICP data engine (no GUI). Leave this terminal running."
exec ros2 launch "$DIR/lidar_kiss_pipeline.launch.py"
