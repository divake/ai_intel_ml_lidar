# Project 02 — FAST-LIO2: LiDAR-Inertial Odometry (LiDAR + D455 IMU)

> **Goal:** fuse the RealSense D455's 200 Hz IMU with the Helios 16P LiDAR using
> **FAST-LIO2** and beat project 01's LiDAR-only baseline on the SAME corridor
> loop: **4.0 m loop-closure error over 392 m (~1.0 % drift), 3.4 m Z-sag**.
> Success = visibly lower loop error + flat floor, with nothing degraded.
>
> **Builds on:** [`01_kiss_icp_mapping`](../01_kiss_icp_mapping) — same decoupled
> pipeline pattern, same record→offline-export workflow, all its fixes inherited.
> **Status:** ✅ built, configured, **stationary smoke test passed** (2026-06-12).
> Next: drive the corridor1 loop → compare. See [`FINDINGS.md`](./FINDINGS.md).

---

## What's new vs project 01

| | Project 01 (KISS-ICP) | Project 02 (FAST-LIO2) |
|---|---|---|
| Sensors | LiDAR only | LiDAR + D455 IMU (200 Hz, the rig's only inertial source) |
| Point format | XYZI (ring reconstructed by hand, no deskew) | **XYZIRT** — native ring + per-point timestamps → real motion deskew |
| Odometry | frame-to-frame ICP | tightly-coupled iterated-EKF LiDAR-inertial |
| Expected drift | ~1.0 % | ~0.1–0.3 % (literature) — measure it! |
| Z behavior | drifts (3.4 m sag) | gravity-pinned roll/pitch → flat floor |

### The XYZIRT change (fixes two project-01 annoyances at the source)
`rslidar_sdk`'s point format is a **compile-time** option:
`set(POINT_TYPE XYZIRT)` in `~/ros2_ws/src/rslidar_sdk/CMakeLists.txt` (was XYZI),
then `colcon build --packages-select rslidar_sdk`. Done 2026-06-12.
`/rslidar_points` now carries `x,y,z,intensity` + **`ring` (uint16)** +
**`timestamp` (float64, absolute s)** per point. KISS-ICP also benefits
(deskew turns on). ⚠️ point_step doubled → project 01's `export_dataset.py`
assumes XYZI; update it before exporting NEW bags (old bags/datasets untouched).

### spark-fast-lio + our RoboSense patch
Using [MIT-SPARK/spark-fast-lio](https://github.com/MIT-SPARK/spark-fast-lio)
(ROS 2 Jazzy support; builds clean). FAST-LIO has no RoboSense input, so we
added one (`lidar_type: 5`):
- `include/preprocess.h` — `rslidar_ros::Point` (XYZIRT) + `RS16 = 5` enum
- `src/preprocess.cpp` — `robosense_handler()`: absolute `timestamp` → relative
  ms offsets, skips the NaN padding points (`dense_points: false` keeps them)

Build: `cd ~/ros2_ws && colcon build --packages-select spark_fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release`

### Extrinsics (LiDAR ↔ IMU)
Camera mounted directly **below** the LiDAR on the same rigid plate (~10 cm,
same heading). IMU frame = `camera_imu_optical_frame` (x right, y down,
z forward — verified: gravity reads +9.6 on y at rest). FAST-LIO convention is
**LiDAR pose w.r.t. IMU**:
```
extrinsic_T = [0, -0.10, 0]                  # LiDAR ~10 cm "up" = -y in optical frame
extrinsic_R = [0,-1,0, 0,0,-1, 1,0,0]        # x_fwd→z, y_left→-x, z_up→-y
extrinsic_est_en = true                      # online refinement absorbs tape-measure error
```

---

## Run it (3 terminals, same pattern as project 01)

```bash
# T1 — data engine: LiDAR + camera/IMU + FAST-LIO2 (no GUI). Leave running.
bash run/start_pipeline.sh

# T2 — viewer bridge (laptop: Foxglove → ws://100.120.151.19:8765,
#       3D panel: /lio/cloud_registered + /lio/path, Fixed Frame = odom_lio)
bash run/start_foxglove.sh

# T3 — record while driving (Ctrl-C to stop)
bash run/record_run.sh corridor2
```

Outputs: `/lio/odometry`, `/lio/path`, `/lio/cloud_registered` (frame `odom_lio`).
Recorded: raw XYZIRT points + IMU + LIO odometry (+TF). The registered cloud is
rebuilt offline — never recorded (project-01 lesson).

## Baseline comparison protocol (don't break the baseline!)
1. Drive the **same corridor loop** as `corridor1`, ending exactly at the start.
2. Loop error = ‖last pose − first pose‖ from `/lio/odometry` → compare 4.0 m.
3. Z-sag: height difference start→end → compare 3.4 m.
4. Because the bag has raw XYZIRT + IMU, **KISS-ICP can be replayed offline on
   the very same drive** for a true apples-to-apples table.

## Gotchas inherited from project 01 (do not re-learn these)
- ROS python = `/usr/bin/python3` (conda's 3.13 breaks rclpy).
- Camera topics are best_effort QoS — reliable subscribers hear silence.
- Exactly ONE realsense node (duplicates fake a hardware error). The D455 needs
  color+depth enabled — IMU-only config fails `device busy` (found 2026-06-12).
- rviz over VNC crashes (OGRE/GLX) → Foxglove on the laptop.
- Viewer decoupled from data engine, always.
- USB: D455 must be on USB 3.x (blue port, own cable) or the IMU dies.
