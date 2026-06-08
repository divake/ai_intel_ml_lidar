# Project 01 (KISS-ICP) — Findings & Fixes

> Project-specific log of what we discovered and every problem we hit + how we solved it.
> Companion to [`README.md`](./README.md) (how to run) and the golden [`../../TROUBLESHOOTING.md`](../../TROUBLESHOOTING.md) (rig-wide).
> Format: **quoted symptom → cause → fix**. Last updated: 2026-06-07.

---

## Project structure
```
01_kiss_icp_mapping/
├── README.md            # goal + how to run + data workflow
├── FINDINGS.md          # this file (issues, fixes, discoveries)
├── config/              # (reserved) tuned rviz/params
├── run/
│   ├── start_pipeline.sh        # DATA ENGINE: LiDAR + KISS-ICP, NO GUI (robust)
│   ├── lidar_kiss_pipeline.launch.py  # the launch start_pipeline.sh runs
│   ├── start_foxglove.sh        # VIEWER BRIDGE: serve topics to Foxglove (laptop GPU)
│   ├── record_run.sh <name>     # RECORDER: raw points + odom + tf -> results/<name>/
│   ├── export_run.sh <name>     # bag -> ML dataset -> results/<name>_dataset/
│   ├── export_dataset.py        # the exporter (run with /usr/bin/python3)
│   ├── run_kiss_icp.sh          # (legacy) KISS-ICP + bundled rviz — rviz fails over VNC
│   └── watch_map.sh             # (legacy) rviz on :2 — DOES NOT WORK over VNC, use Foxglove
└── results/             # rosbags + exported datasets
```

**Recommended runtime layout — 3 terminals + laptop:**
1. `bash run/start_pipeline.sh` — LiDAR + KISS-ICP (no GUI). Leave running.
2. `bash run/start_foxglove.sh` — Foxglove bridge. Leave running.
3. `bash run/record_run.sh <name>` — record while driving. Ctrl-C to stop.
4. On laptop: Foxglove → `ws://100.120.151.19:8765` → 3D panel → `/kiss/local_map`, frame `odom_lidar`.

---

## What works (verified end-to-end, 2026-06-07)
- LiDAR `/rslidar_points` ~10 Hz, fields **x, y, z, intensity** (float32), organized **16×1800** (16 = beam/ring).
- KISS-ICP: `/kiss/odometry` (frame `odom_lidar`), `/kiss/local_map`, `/kiss/frame`, `/kiss/keypoints`. TF `rslidar → odom_lidar`.
- Recording: rosbag (.mcap) of points + odom + tf.
- ML export: per-frame `.npy` (N×5 x,y,z,intensity,ring) + `poses_tum.txt` + `map.ply` + `meta.json`. (Dry run: 87 frames, 2.37 M pts.)

---

## Issues & fixes

### 1. rviz2 will not render over VNC
> "rviz::RenderSystem: error creating render window: RenderingAPIException: Invalid parentWindowHandle (wrong server or screen) … Unable to create the rendering window after 100 tries → Aborted (core dumped)"

**Cause:** rviz2's 3D engine (OGRE) needs a GLX/OpenGL context that the **TigerVNC virtual display can't provide**. Known rviz-over-VNC incompatibility. (Other GL apps like `realsense-viewer` use a different path and DO work over VNC — so VNC itself is fine.)
**Fix:** **Use Foxglove instead** — it renders the 3D on the **laptop's GPU** and only streams data from the NUC, sidestepping NUC GL entirely.
```bash
bash run/start_foxglove.sh            # on NUC
# laptop: Foxglove -> Foxglove WebSocket -> ws://100.120.151.19:8765 -> 3D panel -> /kiss/local_map
```
Do **not** use `watch_map.sh`/`run_kiss_icp.sh` rviz over VNC.

### 2. GUI in VNC opens on the wrong display
> "Local session - Display set to NUC: :0"  (then the GLX error above)

**Cause:** `~/.bashrc` (lines ~147–151) forces `export DISPLAY=:0` (physical monitor), so VNC terminals get flipped from `:2` to `:0`.
**Fix:** for any GUI launched in a VNC terminal, `export DISPLAY=:2` first (the viewer scripts already do this). Mostly moot now that we use Foxglove (browser, no X needed).

### 3. LiDAR driver dies when its terminal is touched
> Driver gone, `/rslidar_points` Publisher count 0 after an accidental Ctrl+C.

**Cause:** `rslidar_sdk/start.py` bundles an **rviz node** in the same launch; Ctrl+C (or the launch teardown) kills the driver with it.
**Fix:** run the driver **without rviz**. `start_pipeline.sh` (via `lidar_kiss_pipeline.launch.py`) runs only the driver node + KISS-ICP — no GUI to crash or kill. View separately via Foxglove.

### 4. KISS-ICP map cloud not published
> Only `/kiss/odometry` appears; `/kiss/local_map` and `/kiss/frame` missing.

**Cause:** in `odometry.launch.py`, `publish_debug_clouds` is hard-wired to the `visualize` flag. Headless (`visualize:=false`) → no debug clouds; and `publish_debug_clouds:=true` passed on the CLI is ignored.
**Fix:** our `lidar_kiss_pipeline.launch.py` sets `publish_debug_clouds: True` on the node directly (no rviz), so the map publishes for Foxglove.

### 5. KISS-ICP for LIVE sensor needs `use_sim_time:=false`
**Cause:** the stock launch defaults `use_sim_time=true` (meant for bag playback with `--clock`). For a live sensor there's no `/clock`.
**Fix:** our pipeline launch sets `use_sim_time: False`. (Use `true` only when replaying a bag with `--clock`.)

### 6. Scan deskew auto-disabled
> "Field 't', 'timestamp', 'time_stamp', or 'time' does not exist. Disabling scan deskewing"

**Cause:** the rslidar PointCloud2 has only x,y,z,intensity — **no per-point timestamp**.
**Fix:** acceptable for slow indoor driving. To enable later: turn on per-point timestamps in the rslidar driver config, then `deskew:=true`.

### 7. `python3` breaks rclpy (exporter/scripts)
> "ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'"

**Cause:** `python3` on PATH is conda 3.13; ROS rclpy is built for system 3.12.
**Fix:** run ROS Python with **`/usr/bin/python3`** (export scripts already do). See golden TROUBLESHOOTING §4.

### 8. Rosbag grows fast / large
> "test_drive_0.mcap … almost 1.7 GB"

**Cause:** raw 3D LiDAR ≈ **280 MB/min** (28,800 pts × 16 B × 10 Hz). Recording `/kiss/local_map` too made it worse (it grows every frame).
**Fix:** normal — 749 GB free is hours of headroom. `record_run.sh` now records only `/rslidar_points /kiss/odometry /tf /tf_static` (drops redundant `/kiss/local_map`; the map is rebuilt offline by the exporter).

---

## Meta-lessons
- **Separate the viewer from the data pipeline.** A stray Ctrl+C or a crashed viewer must never kill the LiDAR or the recording.
- **Render remotely on the laptop (Foxglove), not on the NUC** — avoids all headless-GL pain and is faster.
- **Record raw + odometry only**; derive maps/datasets offline (reproducible, lean bags).

---

## Run log

### `lab_room1` — first clean lab-room map (2026-06-07)
First real end-to-end deliverable. Recorded with `record_run.sh lab_room1`, exported with `export_run.sh lab_room1`.

| Metric | Value |
|---|---|
| Bag | `results/lab_room1/` (1.1 GiB, mcap) |
| Duration | 260 s, 2602 frames @ 10 Hz |
| Topics | `/rslidar_points`, `/kiss/odometry`, `/tf` (clean, no gaps) |
| Raw points | 71.76 M |
| Path driven | 73.3 m (within a ~5.5 m-radius room) |
| Loop closure error (end vs start) | 0.69 m — good for LiDAR-only |
| Voxel map (5 cm) | 407,465 pts |
| Map extent | X 15.9 m, Y 28.6 m, Z 3.8 m (Y/X reach = LiDAR seeing out the doorway, not driven area) |

**Dataset:** `results/lab_room1_dataset/` (89 MB)
- `frames/000000.npy … 002601.npy` — per-frame `N×5` (x,y,z,intensity,ring)
- `poses_tum.txt` (2602 poses, TUM), `timestamps.txt`, `map.ply`, `meta.json`
- `map_topdown.png` — top-down preview (height-colored + white robot path), generated headlessly with conda python + matplotlib/open3d (no Foxglove needed). Room shows as a dense box; doorway shows as a thin streak.

**Observations:** old `test_drive` (2.5 GB, had dead-LiDAR gap + redundant `/kiss/local_map`) deleted. Next target: engineering-building corridor (same pipeline, longer drive) for the team demo.
