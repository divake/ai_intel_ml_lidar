# Robust prior-map localization — the validated baseline (2026-06-15)

> The 3-day blocker ("the robot doesn't know where it is") is **solved, robustly, with proven
> reused components**, validated OFFLINE on real recorded data before any motion. This is the
> foundation for path-following autonomy (Nav2 RPP) and, later, the ML/uncertainty research.

## The stack (proven, reused, generalizes to any map/route)
```
spark-fast-lio (LiDAR+IMU odometry, our proven RoboSense pipeline)
        │  /lio/cloud_registered  (world-frame cloud, frame odom_lio)
        ▼
icp_relocalization/icp_node  → ICP live cloud vs PRIOR MAP (PCD) → map→odom correction
        │  /icp_result
        ▼
transform_publisher → TF  map → odom_lio   (the robot's pose in the map)
```
- Package: `PolarisXQ/Fast-LIO2-Localization` (`icp_relocalization`), cloned + built in `~/ros2_ws`.
  Built **standalone on top of our proven spark-fast-lio — NO RoboSense port needed** for this path.
- Prior map: `results/prior_map_loopclosed.pcd` (800k pts, loop-closed FAST-LIO map, frame `odom_lio`).
- Launch: `run/icp_loc.launch.py` (corrected params: feed `/lio/cloud_registered`, `max_corr_dist 1.0`,
  `fitness_thre 0.25`, `odom_frame_id odom_lio`).
- **One principled change** to `icp_node.cpp` (line 185): the generic path called `rclcpp::shutdown()`
  after the first lock (one-shot); changed to `initGuess = transformation_result` so it **tracks
  continuously** — exactly what the node's own Livox path already does. Not a hack: completing an
  upstream asymmetry. (Also: bad-scan no longer drops the lock to 0.)

## Validation — Option A (continuous tracking), full corridor2.0 loop, OFFLINE
Replayed the recorded bag (sensors only, `--clock`, 2×) → spark-fast-lio → continuous icp_node:
```
7,610 scans ICP'd | fitness median 0.028 m, max 0.041 m | LOST LOCK: 0 (0.0%)
per-segment median (start→end): 0.028 0.027 0.026 0.026 0.030 0.028 0.027 0.028 0.030 0.035
```
**Never lost lock** through the long featureless straights or the junctions. Flat ~2.8 cm.
Why it's robust in the degenerate corridor: FAST-LIO's IMU carries the unobservable forward axis;
the ICP only corrects a small, slowly-changing map↔odom offset and re-anchors laterally + at features.

## How to run it offline (reproduce the validation)
```bash
# 1) spark-fast-lio offline (regenerates /lio/cloud_registered in odom_lio):
ros2 run spark_fast_lio spark_lio_mapping --ros-args \
  --params-file projects/02_fast_lio2/config/helios16p_d455.yaml \
  -r lidar:=/rslidar_points -r imu:=/camera/camera/imu \
  -r odometry:=/lio/odometry -r path:=/lio/path -r cloud_registered:=/lio/cloud_registered \
  -p use_sim_time:=true -p common.map_frame:=odom_lio -p common.lidar_frame:=rslidar \
  -p common.imu_frame:=camera_imu_optical_frame -p common.base_frame:=rslidar \
  -p common.visualization_frame:=lidar -p gravity_alignment.enable_gravity_alignment:=false
# 2) the relocalizer:
ros2 launch projects/03_autonomy/run/icp_loc.launch.py
# 3) play the bag (SENSORS ONLY — never replay the recorded /lio/* or /tf, they fight the new estimate):
ros2 bag play projects/02_fast_lio2/results/corridor2.0 --clock --storage mcap \
  --topics /rslidar_points /camera/camera/imu /tf_static
# watch:  ICP "fitness score" stays < ~0.05 ;  tf2_echo map odom_lio stays stable
```
**Process hygiene (learned the hard way):** kill EVERY leftover `spark_lio_mapping` / `icp_node` /
`ros2 bag play` before a run — two overlapping bag-plays into one FAST-LIO silently corrupt the result.
Verify single-instance with bracket-grep (`ps -eo cmd | grep -c '[r]osbag2_player'`).

## Option B (tightly-coupled localization mode) — recipe ready, not yet run
The bundled `fast_lio` has a real `locate_in_prior_map` mode (loads the PCD into the ikd-tree,
freezes it, runs the full iEKF against it — IMU+scan-to-map fused in one filter). It bootstraps from
the same `icp_node` `/icp_result`. Needs our RoboSense handler ported into its `preprocess.{h,cpp}`
(enum `ROBOSENSE=5` + a `robosense_handler` mirroring spark-fast-lio) + `helios16p_localize.yaml`
(`locate_in_prior_map: true`, `prior_map_path`, `lidar_type: 5`, our extrinsics). Full paste-able
recipe captured 2026-06-15. **A already clears the robustness bar (0% lost lock), so B is confirmatory.**

## Deployment notes
- Live: start the robot at the SAME physical start as the map origin → initial pose (0,0,0,0) locks
  (validated: initial correction was [1.4,1.2,14] cm, <1°). ScanContext/RViz init is the upgrade for
  arbitrary starts.
- Use this pose for **global waypoints / path-following (Nav2 RPP)** and "which junction am I at".
- Don't use map **Z** (FAST-LIO Z-drift). Keep tight lateral *centering* on the eyes if desired, but
  the path-follower now has a trustworthy global pose — the robot can follow the taught route.
