#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/nus-ai/ros2_ws/install/setup.bash
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)
DIR=/home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/run
BAG=/home/nus-ai/divek_nus/ml_lidar/projects/02_fast_lio2/results/corridor2.0
LOG=/tmp/loc_test.log; : > "$LOG"
pkill -9 -f '[r]slidar_sdk_node' 2>/dev/null; pkill -9 -f '[s]park_lio_mapping' 2>/dev/null

echo "[1] launching localization stack ..." | tee -a "$LOG"
ros2 launch "$DIR/localization.launch.py" >>"$LOG" 2>&1 &
sleep 12
echo "[2] lifecycle states:" >>"$LOG"
ros2 lifecycle get /map_server >>"$LOG" 2>&1; ros2 lifecycle get /amcl >>"$LOG" 2>&1
echo "[3] recording /amcl_pose + replaying bag (rate 3) ..." | tee -a "$LOG"
/usr/bin/python3 /tmp/rec_amcl.py 300 >>"$LOG" 2>&1 &
REC=$!
sleep 2
ros2 bag play "$BAG" --clock --rate 3 >>"$LOG" 2>&1
echo "[4] bag done." >>"$LOG"
sleep 3
kill "$REC" 2>/dev/null
pkill -9 -f '[a]mcl' 2>/dev/null; pkill -9 -f 'nav2_map_server/[m]ap_server' 2>/dev/null
pkill -9 -f '[l]ifecycle_manager' 2>/dev/null; pkill -9 -f '[p]ointcloud_to_laserscan' 2>/dev/null
echo "amcl samples: $(wc -l < /tmp/amcl_track.csv 2>/dev/null)" >>"$LOG"
echo "DONE" >>"$LOG"
