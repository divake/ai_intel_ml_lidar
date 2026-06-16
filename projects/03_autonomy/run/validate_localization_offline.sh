#!/bin/bash
source /home/nus-ai/ros2_ws/install/setup.bash
# consumers
ros2 run spark_fast_lio spark_lio_mapping --ros-args \
  --params-file /home/nus-ai/divek_nus/ml_lidar/projects/02_fast_lio2/config/helios16p_d455.yaml \
  -r lidar:=/rslidar_points -r imu:=/camera/camera/imu \
  -r odometry:=/lio/odometry -r path:=/lio/path -r cloud_registered:=/lio/cloud_registered \
  -p use_sim_time:=true -p common.lidar_frame:=rslidar -p common.imu_frame:=camera_imu_optical_frame \
  -p common.map_frame:=odom_lio -p common.base_frame:=rslidar -p common.visualization_frame:=lidar \
  -p gravity_alignment.enable_gravity_alignment:=false \
  > /tmp/sparklio_cont.log 2>&1 &
SPID=$!
ros2 launch /home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/run/icp_loc.launch.py > /tmp/icp_cont.log 2>&1 &
LPID=$!
sleep 7
# play FULL bag at 2x (sensors only)
ros2 bag play /home/nus-ai/divek_nus/ml_lidar/projects/02_fast_lio2/results/corridor2.0 \
  --clock --storage mcap --rate 2.0 --topics /rslidar_points /camera/camera/imu /tf_static \
  > /tmp/bag_cont.log 2>&1
sleep 3
kill -9 $SPID $LPID 2>/dev/null
pkill -9 -f spark_lio_mapping 2>/dev/null; pkill -9 -f icp_node 2>/dev/null; pkill -9 -f transform_publisher 2>/dev/null
# analyze fitness trend
echo "=== CONTINUOUS LOCALIZATION RESULT ==="
python3 - <<'PY'
import re
vals=[]
for line in open('/tmp/icp_cont.log'):
    m=re.search(r'fitness score: ([\d.]+)', line)
    if m: vals.append(float(m.group(1)))
if not vals:
    print("NO fitness values — check log"); raise SystemExit
import statistics as st
n=len(vals); lost=sum(1 for v in vals if v>0.25)
print(f"frames ICP'd: {n}")
print(f"fitness: min {min(vals):.3f}  median {st.median(vals):.3f}  mean {sum(vals)/n:.3f}  max {max(vals):.3f}")
print(f"frames with fitness>0.25 (lost lock): {lost} ({100*lost/n:.1f}%)")
# trend: split into 10 segments, report median per segment (does it degrade mid-corridor?)
seg=max(1,n//10)
print("per-segment median fitness (start->end of loop):")
print("  "+"  ".join(f"{st.median(vals[i:i+seg]):.3f}" for i in range(0,n,seg)))
PY
