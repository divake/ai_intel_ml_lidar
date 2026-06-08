#!/bin/bash
# Export a recorded run into an ML-ready dataset.
# Usage: bash export_run.sh <run_name> [voxel_m]
set -e
NAME=${1:-gallery_run1}
VOXEL=${2:-0.05}
source /opt/ros/jazzy/setup.bash
source /home/nus-ai/ros2_ws/install/setup.bash
PROJ=/home/nus-ai/divek_nus/ml_lidar/projects/01_kiss_icp_mapping
/usr/bin/python3 "$PROJ/run/export_dataset.py" \
  "$PROJ/results/$NAME" "$PROJ/results/${NAME}_dataset" --voxel "$VOXEL"
echo "Dataset -> $PROJ/results/${NAME}_dataset"
