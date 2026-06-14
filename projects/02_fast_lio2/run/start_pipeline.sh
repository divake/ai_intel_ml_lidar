#!/bin/bash
# DATA ENGINE (project 02): LiDAR + D455 IMU + spark-fast-lio (no GUI).
# Run in one terminal and leave it. Ctrl+C here stops the pipeline (intentional).
# The viewer/recorder are separate (project-01 lesson: decouple or a viewer
# crash kills the sensors).
set -e
source /opt/ros/jazzy/setup.bash
source /home/nus-ai/ros2_ws/install/setup.bash
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Starting LiDAR + D455-IMU + FAST-LIO2 data engine (no GUI). Leave this running."
exec ros2 launch "$DIR/fast_lio_pipeline.launch.py"
