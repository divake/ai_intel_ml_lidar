#!/bin/bash
# Record a FAST-LIO2 mapping run while driving. Usage: bash record_run.sh <run_name>
# Ctrl-C to stop. Saves into results/<run_name>/
#
# Records the RAW inputs (XYZIRT points + IMU) so any algorithm can be replayed
# offline against the exact same drive — including KISS-ICP for the
# apples-to-apples baseline comparison. Also records FAST-LIO2's live outputs.
# (/lio/cloud_registered is NOT recorded — rebuildable offline, bloats the bag.)
set -e
NAME=${1:-corridor2}
source /opt/ros/jazzy/setup.bash
source /home/nus-ai/ros2_ws/install/setup.bash
RESULTS=/home/nus-ai/divek_nus/ml_lidar/projects/02_fast_lio2/results
cd "$RESULTS"
echo "Recording '$NAME' ... drive the robot now. Ctrl-C to stop."
# ~280 MB/min LiDAR (XYZIRT is 2x XYZI) + ~1 MB/min IMU.
exec ros2 bag record -o "$NAME" \
  /rslidar_points \
  /camera/camera/imu \
  /lio/odometry \
  /lio/path \
  /tf /tf_static
