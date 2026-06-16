# DATASET — ml_lidar LiDAR + IMU corridor data (handoff card)

**Read this first if you are an AI/engineer on the compute server.** It tells you what
was collected, how, what each file means, how to load it, and the traps to avoid. The data
was recorded on an Intel NUC 14 Pro (a small edge box); the heavy ML is meant to run on the
server. Everything here is the *ground truth of the collection*, written so you do not have
to reverse-engineer the bags.

Last verified against the files on disk: **2026-06-16** (every number below was read off the
actual bag/PCD/npy, not remembered).

---

## 0. Why this data exists (the research goal)

The north star is **uncertainty quantification in LiDAR / point-cloud machine learning** —
models that not only predict (registration, odometry, segmentation, place recognition,
deskew, completion) but also say *how sure they are*. To study that honestly you need real
sensor data with its real failure modes (self-similar corridors, sparse 16-beam returns,
estimator drift), plus a trustworthy reference to measure against. SLAM and autonomy on the
robot were the **rig** that produced and validated this data — they are infrastructure, not
the goal.

The NUC can record and run real-time SLAM but is too small for heavy training. So: **collect
on the NUC, compute on the server.** This card is the bridge.

---

## 1. TL;DR — what to use

| Want | Use | Why |
|---|---|---|
| **Best raw data** (deskewable, has IMU) | `corridor2.0` bag (project 02) | 10 Hz LiDAR **XYZIRT** (native ring + per-point timestamp) + ~200 Hz IMU + FAST-LIO poses |
| **Ready-to-load frames today** | `corridor1_dataset/` (project 01) | already exported to `.npy` per-frame + a `LidarDataset` loader; **but older XYZI format, no IMU** |
| **Global map / geometry reference** | `prior_map_loopclosed.pcd` | 801,837-pt loop-closed FAST-LIO map of the building loop |
| **The driven route** | `teach_path_fastlio.csv` | 628-pose, 251.6 m figure-8 trajectory in the map frame |

If you only take one thing: **`corridor2.0` is the crown jewel** (modern sensor format +
IMU). It is still a rosbag, so see §6 for how to read it without ROS.

---

## 2. The sensor rig

- **LiDAR:** RoboSense **Helios 16P**, 16 beams, spinning, 10 Hz, 360° FOV. Topic
  `/rslidar_points` (`sensor_msgs/PointCloud2`), frame `rslidar`. Organized
  **height=1800 × width=16** (1800 azimuth columns × 16 rings).
- **IMU:** Intel **RealSense D455** built-in IMU, ~200 Hz. Topic `/camera/camera/imu`
  (`sensor_msgs/Imu`), frame `camera_imu_optical_frame`, **best_effort** QoS.
- **Mounting:** camera sits on the same rigid plate **~10 cm directly below** the LiDAR,
  same heading. Robot: Agilex Scout Mini (skid-steer), ~0.58 m wide, LiDAR at center.
- **Clocks:** no hardware time-sync; **both sensors are stamped with the NUC system clock**
  (`time_sync_en: false`). The per-point LiDAR `timestamp` is an absolute float64 second.

### LiDAR ↔ IMU extrinsic (from `projects/02_fast_lio2/config/helios16p_d455.yaml`)

`extrinsic_T` (LiDAR position in the IMU frame) and `extrinsic_R` (LiDAR orientation in the
IMU frame), i.e. **T_imu_lidar**:

```
extrinsic_T = [ 0.0, -0.10, 0.0 ]        # ~10 cm offset
extrinsic_R = [ 0, -1,  0,
                0,  0, -1,
                1,  0,  0 ]
```

⚠️ This is a **tape-measure guess refined online** by FAST-LIO (`extrinsic_est_en: true`).
It is good enough for odometry but is **not a metrology-grade calibration**. If your ML task
is extrinsic-sensitive, treat it as an initial guess and refine.

### Coordinate frames / conventions

- **LiDAR / map / poses:** ROS **REP-103** — **x forward, y left, z up**, metres, right-handed.
- **IMU frame** `camera_imu_optical_frame`: **x right, y down, z forward** (optical convention).
  At rest, gravity reads on **+y** (measured `accel ≈ (0.5, -9.5, 0.8)` → ~9.8 along y). Do
  not assume the IMU shares the LiDAR axes — apply the extrinsic above.
- **intensity:** 0–255-ish, **uncalibrated** sensor reflectivity.
- **ring:** beam id **0..15** (`index % 16` along the width axis for organized clouds).
- **TF tree (live):** `map → odom_lio → rslidar → base_link → wheels`.

---

## 3. The datasets in detail

### A. `corridor2.0` — PRIMARY (project 02, FAST-LIO, XYZIRT) ⭐

Path: `projects/02_fast_lio2/results/corridor2.0/` (mcap, ros_distro **jazzy**, ~5.6 GB).

| Topic | Type | Count | Rate | Notes |
|---|---|---|---|---|
| `/rslidar_points` | PointCloud2 | **7,619** | **10.0 Hz** | organized 1800×16, **XYZIRT**, `is_dense:false` |
| `/camera/camera/imu` | Imu | 151,910 | ~199 Hz | best_effort, frame `camera_imu_optical_frame` |
| `/lio/odometry` | Odometry | 159,515 | ~209 Hz | **FAST-LIO 6-DoF pose** (frame `odom_lio`) |
| `/lio/path` | Path | 762 | ~1 Hz | accumulated path |
| `/tf` | TFMessage | 159,515 | — | `odom_lio → rslidar` |
| `/tf_static` | TFMessage | 1 | — | static extrinsics |

- **Duration:** 761.9 s (**12.7 min**), one continuous figure-8 loop of the building.
- **PointCloud2 layout** (`point_step = 26`, `row_step = 416`):

  | field | offset | type | meaning |
  |---|---|---|---|
  | x | 0 | float32 | metres, REP-103 |
  | y | 4 | float32 | |
  | z | 8 | float32 | |
  | intensity | 12 | float32 | uncalibrated |
  | ring | 16 | uint16 | beam 0..15 |
  | timestamp | 18 | float64 | absolute seconds (per point) → **deskew** |

- **The 10 Hz is clean here.** (A live ~8 Hz delivery problem existed during *autonomy
  driving*; it is **NOT** in this recorded bag — the bag is a true 10 Hz. Ignore it for
  offline ML.)
- **Per-scan poses:** the authoritative trajectory is `/lio/odometry` inside the bag
  (FAST-LIO). For each LiDAR scan, take the nearest-in-time odometry sample.

### B. `corridor1` + `corridor1_dataset` — READY-TO-LOAD (project 01, KISS-ICP, XYZI)

- **Bag:** `projects/01_kiss_icp_mapping/results/corridor1/` — 1118 s (**18.6 min**),
  **11,185** `/rslidar_points`, `/tf`, `/kiss/odometry` (11,093). ~4.9 GB.
- **Exported frames:** `projects/01_kiss_icp_mapping/results/corridor1_dataset/` (~6.1 GB):
  - `frames/000000.npy … 011184.npy` — **11,185** per-frame point clouds, each
    `(N, 5) float32 = [x, y, z, intensity, ring]` in the **sensor frame** (N varies,
    ~27k points/frame).
  - `poses_tum.txt` — TUM-format poses (`t x y z qx qy qz qw`), **KISS-ICP estimate**.
  - `timestamps.txt`, `map.ply` (137 MB, accumulated map), `meta.json` (fields, extents,
    voxel size 0.05 m, 318 M raw points).
  - `dataset_loader.py` (in `projects/01_kiss_icp_mapping/run/`) — `LidarDataset` class,
    numpy-only, KITTI/Waymo-style. See §6.
- ⚠️ **Older sensor build:** XYZI + ring but **no per-point timestamp** (can't deskew the
  same way) and **no IMU** in this bag. Poses are LiDAR-only KISS-ICP.

### C. Derived / reference artifacts (project 03)

- **`projects/03_autonomy/results/prior_map_loopclosed.pcd`** — 801,837 points, `x y z`
  float32 binary. **Loop-closed FAST-LIO map** of the corridor2.0 loop. Extent:
  **x ∈ [−65.6, 48.2] (114 m), y ∈ [−18.5, 33.9] (52 m), z ∈ [−4.0, 4.3] (8.2 m)**.
  This is the global geometry reference for localization / map-relative tasks.
- **`projects/03_autonomy/results/teach_path_fastlio.csv`** — 628 poses `# x y yaw`, in the
  `odom_lio`/map frame, **251.6 m** total, loop closure gap 1.25 m. The RC-taught route.
- **`projects/03_autonomy/results/kiss_slam/poses_loopclosed.npy`** — `(7619, 4, 4)` float64,
  one 4×4 homogeneous pose per corridor2.0 scan. ⚠️ **Provenance to verify** (lives under
  `kiss_slam/`, but KISS reported **0 loop closures**; the map's closure was done on
  FAST-LIO). Cross-check it against `/lio/odometry` and the PCD before trusting the name.
- **`slam_output/…`** — KISS-SLAM run dumps (g2o, KITTI/TUM poses, trajectory.png,
  `result_metrics.log`). Reproducible; mostly for inspection.

### D. Small extras

- **`lab_room1`** (project 01) — 260 s (4.3 min), 2,602 scans, a *room* not a corridor.
  Old XYZI. Useful as a second, different-geometry scene.

---

## 4. How the data was created (pipeline)

1. **Record** on the NUC: `projects/02_fast_lio2/run/record_run.sh` runs the RoboSense
   driver (`rslidar_sdk`, built `POINT_TYPE XYZIRT` → native ring+timestamp), the D455 IMU,
   and `spark-fast-lio`, then `ros2 bag record` of the topics in §3A.
2. **Odometry / SLAM:** **spark-fast-lio** (tightly-coupled LiDAR-inertial), config
   `helios16p_d455.yaml`, **`lidar_type: 5`** (a patched RoboSense handler that consumes
   XYZIRT and converts the absolute float64 `timestamp` to per-scan ms offsets, skipping NaN
   padding). `blind: 0.5` → FAST-LIO ignores returns < 0.5 m (robot body). `scan_line: 16`,
   `scan_rate: 10`.
3. **Map + loop closure:** the dense map is rebuilt **offline from the bag** (live
   `pcd_save_en: false`), then loop-closed → `prior_map_loopclosed.pcd`.
4. **Teach-and-repeat route:** the loop was driven once by RC; FAST-LIO poses were saved as
   `teach_path_fastlio.csv` (the route the robot later followed autonomously).
5. **Project-01 export (corridor1):** `export_dataset.py` replayed the bag through KISS-ICP
   and dumped per-frame `.npy` + TUM poses + map → the `corridor1_dataset/` loader format.

Pythons on the NUC (FYI, not needed on the server): ROS tools use `/usr/bin/python3`
(the conda python is 3.13 and breaks rclpy); numpy/render use the conda `intel_ai` env.

---

## 5. ⚠️ Things to take care of (read before training)

1. **The data is NOT in git.** `*.pcd, *.mcap, *.ply, *.npy, **/results/` are gitignored,
   so a `git clone` brings **code only**. **Copy the data separately** (rsync the
   `projects/*/results/` trees). Total ≈ **18 GB**: corridor2.0 5.6 G, corridor1 4.9 G,
   corridor1_dataset 6.1 G, lab_room1 1.2 G, map+poses ~17 M.
2. **Organized cloud has invalid points.** corridor2.0 is `is_dense:false`, 1800×16 with
   NaN/zero padding where a beam got no return. **Mask NaNs / zero-range** before use; don't
   assume 28,800 valid points per scan.
3. **Poses are estimates, not survey ground truth.** FAST-LIO (corridor2.0) drifts ~1 %;
   KISS-ICP (corridor1) is LiDAR-only and drifts more. The loop-closed PCD is the best
   *global* reference. For **uncertainty** work this is a feature (you're studying the
   estimator) — just never label them "GT".
4. **z is the least-constrained axis.** The map's 8.2 m z-span on a single floor is mostly
   vertical drift + ceiling/floor + outliers, not real height. Be skeptical of z.
5. **ring/timestamp exist only in corridor2.0** (XYZIRT). corridor1/lab_room1 have ring but
   **no per-point timestamp** → deskewing those needs scan-rate assumptions.
6. **IMU axes ≠ LiDAR axes.** IMU is x-right/y-down/z-forward, gravity on +y; LiDAR is REP-103.
   Apply the §2 extrinsic. The extrinsic is tape-measure + online-refined, not exact.
7. **No hardware time-sync.** Both use the NUC clock; align LiDAR↔IMU↔odometry by timestamp,
   and expect small jitter. Per-point LiDAR `timestamp` is absolute seconds (float64).
8. **`blind: 0.5` is a FAST-LIO setting, not applied to the raw bag.** The raw
   `/rslidar_points` in the bag still contains < 0.5 m returns; FAST-LIO drops them
   internally. If you want parity with FAST-LIO, replicate the 0.5 m cut.
9. **corridor1 frame↔pose count mismatch:** 11,185 frames vs 11,093 odom. Verify alignment
   (`poses_tum.txt` row count vs frame index) before pairing — the first frames may lack a
   pose. Don't blindly trust `pose(i)` for every `i`.
10. **`poses_loopclosed.npy` naming** — verify provenance (§3C) before using as labels.

---

## 6. How to load it on the server

### corridor1_dataset (no ROS needed — numpy only)

```python
# projects/01_kiss_icp_mapping/run/dataset_loader.py
from dataset_loader import LidarDataset
ds  = LidarDataset("projects/01_kiss_icp_mapping/results/corridor1_dataset")
pts = ds.frame(0)        # (N,5) float32: x,y,z,intensity,ring  (sensor frame)
T   = ds.pose(0)         # (4,4) world<-sensor (KISS-ICP estimate)
wpts= ds.frame_world(0)  # xyz mapped into the map frame
```
(No `splits.json` ships yet → `ds.split(...)` raises; make splits with
`run/make_splits.py` or your own — keep them **contiguous/leak-free**, corridors are
temporally correlated so random splits leak.)

### corridor2.0 (a rosbag — two options)

**Option 1 — read the mcap without ROS** (`pip install mcap mcap-ros2-support`, or parse the
raw bytes with the §3A field offsets):

```python
import numpy as np
from mcap_ros2.reader import read_ros2_messages

# Parse the 26-byte XYZIRT record by explicit field offsets (robust to the
# uint16 `ring` sitting at an odd offset — do NOT use ndarray.view on slices).
PT = np.dtype({
    "names":   ["x", "y", "z", "intensity", "ring", "timestamp"],
    "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f8"],
    "offsets": [0, 4, 8, 12, 16, 18],
    "itemsize": 26,
})
PC = "projects/02_fast_lio2/results/corridor2.0/corridor2.0_0.mcap"
for m in read_ros2_messages(PC, topics=["/rslidar_points"]):
    a    = np.frombuffer(m.ros_msg.data, dtype=PT)            # (N,) structured
    xyz  = np.stack([a["x"], a["y"], a["z"]], axis=1)         # (N,3) float32
    ring, ts = a["ring"], a["timestamp"]                      # 0..15 ; abs seconds
    valid = np.isfinite(xyz).all(1) & (np.abs(xyz).sum(1) > 0)  # drop NaN/zero padding
    xyz, ring, ts = xyz[valid], ring[valid], ts[valid]
    break
```

**Option 2 — export to `.npy` on the NUC first** (recommended if the server has no ROS):
adapt `projects/01_kiss_icp_mapping/run/export_dataset.py` for the XYZIRT layout and produce
a `corridor2_dataset/` in the same `LidarDataset` format, then transfer that. This also lets
you bake in deskew (using the per-point `timestamp`) once, on the NUC.

The IMU and `/lio/odometry` come out of the same mcap with the same reader (different topic);
pair each scan with the nearest odometry by timestamp.

---

## 7. What this data supports (grounded, not exhaustive)

- **LiDAR odometry / registration with uncertainty** — corridor2.0 gives scans + IMU +
  FAST-LIO poses; the self-similar corridor is a real degeneracy testbed (the along-corridor
  axis is poorly observable — exactly where a model *should* report high uncertainty).
- **Deskew / motion-compensation learning** — per-point timestamps + IMU are present.
- **Place recognition / loop closure** — the figure-8 revisits itself; the loop-closed PCD
  is the reference. (Classical KISS found 0 closures here → a genuine hard case.)
- **Map-relative localization** — scan ↔ `prior_map_loopclosed.pcd`.
- **Segmentation / completion / generative** — 16-beam sparsity is a strong sparse-input case.

Single building, single rig, two scenes (corridor + room) — great for a **single-sequence /
self-supervised** study, thin for cross-environment generalization. If you need diversity,
record a second environment on the NUC (~20–25 min) before scaling up training.

---

## 8. File inventory (what to copy to the server)

```
projects/02_fast_lio2/results/corridor2.0/            5.6G   ⭐ primary bag (XYZIRT + IMU + FAST-LIO)
projects/01_kiss_icp_mapping/results/corridor1/       4.9G   secondary bag (XYZI, KISS)
projects/01_kiss_icp_mapping/results/corridor1_dataset/ 6.1G ready-to-load .npy frames + loader format
projects/01_kiss_icp_mapping/results/lab_room1/       1.2G   small room scene (XYZI)
projects/03_autonomy/results/prior_map_loopclosed.pcd 9.2M   loop-closed global map (801,837 pts)
projects/03_autonomy/results/teach_path_fastlio.csv    15K   251.6 m route (628 poses)
projects/03_autonomy/results/kiss_slam/                8.0M   per-scan pose npy + SLAM dumps (verify)
projects/01_kiss_icp_mapping/run/dataset_loader.py            the numpy loader (in git)
```

Code travels with `git clone`; the `results/` data does **not** (gitignored) — rsync it.
