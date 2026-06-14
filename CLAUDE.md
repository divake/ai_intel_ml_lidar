# ml_lidar — project instructions (keep this file SMALL)

LiDAR mapping & 3D ML on an Intel NUC 14 Pro. Repo: github.com/divake/ai_intel_ml_lidar

## Pythons (critical)
- **ROS / rclpy / rosbag**: use `/usr/bin/python3` (the conda `python3` is 3.13 and breaks rclpy).
- **Rendering (open3d/plotly/matplotlib)**: use `/home/nus-ai/miniconda3/envs/intel_ai/bin/python`.

## Sensor / data
- RoboSense Helios 16P → `/rslidar_points` (PointCloud2 x,y,z,intensity), organized **height=1800 × width=16**, point_step=16.
  Beam/ring = `index % 16` (column axis). No per-point timestamp.
- KISS-ICP gives `/kiss/odometry` (frame `odom_lidar`); LiDAR-only, drifts (~1%), no loop closure.
- Pipeline scripts: `projects/01_kiss_icp_mapping/run/` (start_pipeline / record_run / export_run / render_run / dataset_loader).
- **Since 2026-06-12** rslidar_sdk is built with `POINT_TYPE XYZIRT` → `/rslidar_points` has native `ring`+`timestamp` (point_step 32, not 16). Project 02 = FAST-LIO2 (spark-fast-lio, patched `lidar_type: 5` for RoboSense) + D455 IMU (`/camera/camera/imu`, ~190 Hz, best_effort QoS); scripts in `projects/02_fast_lio2/run/`.

## Gotchas
- `git push` fails (stale gh helper) — push with `TOKEN=$(/usr/bin/gh auth token)` inline.
- rviz fails over VNC (OGRE/GLX) — use Foxglove (renders on laptop GPU).
- Big hardware/accel reference lives in `/home/nus-ai/divek_nus/CLAUDE.md` — read it only if you actually need hardware details.
