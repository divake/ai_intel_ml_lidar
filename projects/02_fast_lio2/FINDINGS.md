# Project 02 (FAST-LIO2) — Findings & Fixes

> Issue→fix log, companion to [`README.md`](./README.md). Same format as
> [project 01's](../01_kiss_icp_mapping/FINDINGS.md). Last updated: 2026-06-12.

## Setup verified (2026-06-12)
- D455 IMU: ~187 Hz united stream, gravity 9.6 m/s² on +y at rest (optical frame
  confirmed), raw 6-axis (no orientation — exactly what FAST-LIO2 needs).
- rslidar_sdk rebuilt with `POINT_TYPE XYZIRT`: `/rslidar_points` now has native
  `ring` (uint16 @16) + `timestamp` (float64 @18); per-point stamps span the full
  99.5 ms scan, header = first point. Deskew data is real.
- spark-fast-lio builds clean on Jazzy (1 min). Our RoboSense patch
  (`lidar_type: 5`) compiles and runs.
- **Stationary smoke test passed:** pose wobble ~cm, creep converges
  0.72 → ~0.1 cm/s within 2 min as IMU biases settle.

## Issues & fixes

### 1. LIO diverged 500 m while stationary
> pose (188, 275, −508) at rest, /lio/odometry at inflated rate, camera spamming
> `xioctl(VIDIOC_QBUF) failed … No such device`

**Cause:** TWO copies of the whole pipeline running — duplicate rslidar drivers
double-publishing `/rslidar_points` and duplicate camera nodes fighting USB
(project-01 gotcha #2, new costume). The first launch's children survived a
`pkill` whose "verify 0 remain" grep was silently broken (bracket-pattern
matched nothing → looked clean).
**Fix:** kill by PID list from `ps`, then verify with a plain grep + `wc -l`
that prints the count. With ONE pipeline: stable immediately.
**Lesson:** when output looks insane, count your nodes first
(`ros2 node list | sort | uniq -c`) before debugging algorithms.

### 2. `Invalid visualization frame has been given` → node dies
**Cause:** `common.visualization_frame` is a literal selector — `"lidar"`,
`"base"`, or `"imu"` — NOT a TF frame name. We passed `"rslidar"`.
**Fix:** set `"lidar"` in the launch.

### 3. Startup warnings that are NOT problems (don't chase)
- `curvature (36.5) should be close to 100` + `No point, skip this scan!` —
  only the first 1–2 scans: the driver starts publishing mid-rotation, so the
  first cloud is partial. Steady-state scans span the full 99.5 ms.
- `IMU loopback / Lidar loopback, clearing buffers` once at startup — one stale
  message; buffers clear and it recovers.
- `Looking up transform from rslidar -> rslidar … Extrinsics detected [0,0,0]` —
  that's the base↔lidar visualization transform (identity by design), NOT the
  IMU↔LiDAR extrinsic (which comes from `mapping.extrinsic_T/R` in our yaml).

### 4. Init transient: pose settles ~3.4 m from zero at rest
The startup buffer-clears happen mid IMU-init, so the filter walks a few meters
before locking. Harmless for our metric (loop error = ‖end − start‖ of the
*recorded trajectory*), but: **start the pipeline, wait ~60 s for convergence,
THEN start recording/driving.** (A restart of the LIO node after the drivers
are warm would also avoid it entirely.)

### 5. Whole map tilted in Foxglove
> "the entire plane felt like tilted on the left side"

**Cause:** `gravity_alignment.enable_gravity_alignment` was `False` (copied from
the MIT example launch), so the world frame = the sensor's initial orientation —
the map tilts by exactly the LiDAR's mounting angle. Odometry itself unaffected.
**Fix:** set it `True` (the package default): the node collects accel samples at
rest and rotates the world so +z = true up. Also makes the Z-sag metric honest.
⚠️ The alignment only triggers **after ~2 s of continuous MOTION** (it logs
"Waiting for motion to perform gravity alignment"). A freshly started, parked
robot shows the tilted world until you drive — so **wiggle the robot a few
meters BEFORE starting a recording**; the snap-to-level is a one-time frame
rotation and must not land mid-bag.

### 6. Watching the camera while the pipeline runs
`realsense-viewer` can NOT open the camera while the ROS node holds it (known
exclusive-access gotcha). No need: the pipeline already publishes
`/camera/camera/color/image_raw/compressed` — add an **Image panel in Foxglove**
on that topic and drive with live video next to the 3D map. (Image topics do NOT
render inside the 3D panel's topic list — needs its own panel.)

### 7. "Growing map" view (the /kiss/local_map experience)
FAST-LIO2 publishes only the current scan in world frame (`/lio/cloud_registered`),
no accumulated-map topic. **Foxglove decay time** substitutes: 3D panel →
`/lio/cloud_registered` → Decay time = 99999 → scans persist and the map grows
as you drive. Viewer-only, no effect on the bag. `dense_publish_en: false` keeps
the accumulated point count browser-survivable; the dense map is rebuilt offline.

### 8. After a reboot: LIO diverges to ~800 km (UDP buffer reset) ⚠️ HIGH-VALUE
> Day after it worked, fresh boot: `/lio/odometry` exploding ~14 km / 8 s while
> parked, the `/lio/path` (yellow line) flailing, `/lio/cloud_registered`
> invisible (flung 800 km off-origin). Raw `/rslidar_points` still looked fine.

**NOT a duplicate pipeline (issue #1) and NOT the LiDAR web config.** Decisive
diagnostics:
- Per-scan data perfect: 27.6 k finite pts, per-point timestamp **span 0.0996 s**
  (clean 10 Hz single return), monotonic, header = first point. Sensor config
  untouched.
- IMU healthy: 199 Hz, |accel| ≈ 9.6.
- **But receive rate was 4.5 Hz with gaps up to 0.9 s** — whole scans dropped.
- `cat /proc/net/snmp | grep Udp` → **`RcvbufErrors` climbing** (6571).
- `sysctl net.core.rmem_max` → **212992** (208 KB, the Linux default).

**Cause:** RoboSense streams high-rate UDP (MSOP); the 208 KB default receive
buffer overflows under CPU load (spark_lio eats ~2 cores) → whole scans lost →
FAST-LIO must dead-reckon the IMU across 0.9 s gaps → iterated-EKF runs away.
The larger buffer had been set at runtime in a prior session with `sysctl -w`
but **never persisted, so the reboot reset it.**
**Fix (persisted 2026-06-13):** `/etc/sysctl.d/10-rslidar-udp.conf` sets
`net.core.rmem_max = net.core.rmem_default = 26214400` (26 MB). A socket only
picks up the bigger buffer when it's (re)opened, so **restart the data engine
after applying** (`sysctl -w` alone doesn't fix the already-running driver).
**Triage drill when LIO looks insane:** ① count nodes (issue #1) → ② check the
LiDAR *receive rate* and `RcvbufErrors`, not just whether points render. A
complete-but-infrequent cloud means transport loss, not a bad sensor.

### 9. `/lio/cloud_registered` silent + no growing map (gravity-alignment GATE)
> After the UDP fix the pose was stable, odometry healthy at 199 Hz, but
> `/lio/cloud_registered` published **0 messages** and the map never grew.

**NOT a fault — a gate.** `enable_gravity_alignment: True` makes the node
**early-return before the publish/mapping block** (`spark_fast_lio.cpp:1129`)
until `is_gravity_aligned_` flips true, which needs **~2 s of continuous motion**
(`num_moving_frames_thr_=20` + 20 gravity samples, lines 1089-1119). While parked
it therefore skips `mapIncremental()`, `publishPath()`, and
`publishFrameWorld()`. The 199 Hz odometry is the *separate* IMU-propagation
publisher (line 492), which is NOT gated — so odometry looks fine while the whole
LiDAR-update publish path is silently off. Deeper consequence beyond issue #5's
cosmetic tilt: **the start of a drive (pre-alignment) is poorly tracked and
unmapped.**
**Decision (2026-06-13):** set `enable_gravity_alignment: False` in the launch.
Pipeline then builds the map + publishes the registered cloud from t=0,
deterministic. Trade-off is only world-frame leveling (cosmetic; our mount is
level) — the loop-error metric is frame-invariant and the EKF's gravity estimate
(flat floor) is unaffected. Supersedes issue #5's "wiggle to trigger" advice.
**Diagnostic that nailed it:** `ros2 topic info <topic> -v` (publisher exists but
0 msgs) + reading which publish calls sit before/after the gate. Note: node
`RCLCPP` logs go to the T1 terminal, NOT `~/.ros/log/*/launch.log` (only 544 B).

## Procedure for the corridor comparison run
1. `start_pipeline.sh`, wait 60 s (watch creep settle in Foxglove).
2. `record_run.sh corridor2`, drive the SAME loop as corridor1, end exactly at
   the start marker, Ctrl-C.
3. Offline: loop error + Z-sag from `/lio/odometry`; replay KISS-ICP on the same
   bag (it has raw XYZIRT + IMU) for the apples-to-apples table vs the 4.0 m /
   3.4 m baseline.
