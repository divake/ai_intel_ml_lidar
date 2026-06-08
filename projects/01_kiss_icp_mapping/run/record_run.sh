#!/bin/bash
# Record a mapping run while driving. Usage: bash record_run.sh <run_name>
# Ctrl-C to stop. Saves into results/<run_name>/  (raw points + KISS-ICP poses + TF)
set -e
NAME=${1:-gallery_run1}
source /opt/ros/jazzy/setup.bash
source /home/nus-ai/ros2_ws/install/setup.bash
RESULTS=/home/nus-ai/divek_nus/ml_lidar/projects/01_kiss_icp_mapping/results
cd "$RESULTS"
echo "Recording '$NAME' ... drive the robot now. Ctrl-C to stop."
# Record only what we need to rebuild everything offline (drop /kiss/local_map: redundant + bloats).
# Raw LiDAR is ~280 MB/min — plenty of disk (749 GB free).
exec ros2 bag record -o "$NAME" /rslidar_points /kiss/odometry /tf /tf_static
