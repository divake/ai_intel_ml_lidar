#!/bin/bash
# Project 03 Phase A: build a LOOP-CLOSED 2D map from the corridor2.0 bag, offline.
# Stops the live pipeline (recording done), runs pointcloud_to_laserscan +
# slam_toolbox while replaying the bag, then saves the occupancy grid + pose graph.
# NO robot motion. Re-runnable.
set -u
source /opt/ros/jazzy/setup.bash
source /home/nus-ai/ros2_ws/install/setup.bash
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG=/home/nus-ai/divek_nus/ml_lidar/projects/02_fast_lio2/results/corridor2.0
OUT=/home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/results
LOG=/tmp/slam_offline.log
mkdir -p "$OUT"; : > "$LOG"

echo "[1/5] stopping live pipeline (offline work now) ..."
pkill -f rslidar_sdk_node 2>/dev/null; pkill -f spark_lio_mapping 2>/dev/null
pkill -f realsense2_camera_node 2>/dev/null
sleep 3
pgrep -f 'rslidar_sdk_node|spark_lio_mapping' >/dev/null && echo "  WARN: pipeline still up" || echo "  pipeline stopped."

echo "[2/5] launching slam_toolbox + pointcloud_to_laserscan ..."
ros2 launch "$DIR/slam_offline.launch.py" >>"$LOG" 2>&1 &
SLAM=$!
sleep 7

echo "[3/5] replaying corridor2.0 bag (--clock --rate 3) ..."
ros2 bag play "$BAG" --clock --rate 3 >>"$LOG" 2>&1
echo "  bag finished."
sleep 4

echo "[4/5] saving occupancy grid -> $OUT/loopclosed_map.{pgm,yaml} ..."
ros2 run nav2_map_server map_saver_cli -f "$OUT/loopclosed_map" \
    --ros-args -p save_map_timeout:=30.0 >>"$LOG" 2>&1
echo "[4b] serializing pose graph (for later 3D re-render) ..."
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$OUT/loopclosed_posegraph'}" >>"$LOG" 2>&1 || echo "  (serialize skipped)"

echo "[5/5] cleanup ..."
sleep 2; kill "$SLAM" 2>/dev/null; pkill -f slam_toolbox 2>/dev/null
pkill -f pointcloud_to_laserscan 2>/dev/null
ls -la "$OUT"/loopclosed_map.* 2>/dev/null && echo "MAP SAVED OK" || echo "MAP SAVE FAILED (see $LOG)"
echo "DONE"
