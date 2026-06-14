#!/bin/bash
export XDG_RUNTIME_DIR=/tmp/rt; mkdir -p /tmp/rt
OUT=/home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/results/kiss_slam
mkdir -p "$OUT"; cd "$OUT"
BAG=/home/nus-ai/divek_nus/ml_lidar/projects/02_fast_lio2/results/corridor2.0
/home/nus-ai/kiss_venv/bin/kiss_slam_pipeline "$BAG" \
    --dataloader rosbag --topic /rslidar_points --refuse-scans 2>&1
echo "=== EXIT $? ==="
echo "=== outputs under $OUT ==="
find "$OUT" -maxdepth 4 \( -name '*.ply' -o -name '*.txt' -o -name '*poses*' -o -name '*.kitti' -o -name '*.tum' \) 2>/dev/null
