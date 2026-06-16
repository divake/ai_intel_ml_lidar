# RESUME HERE — proj03 autonomy (last updated 2026-06-16)

Open this first when you come back. Where we are, what works, what's left, how to reproduce.

---

## TL;DR — autonomous path-following WORKS; one localization blocker remains

The robot **autonomously followed the RC-taught route**: it drove the **entire first straight
(~36 m, cross-track 0.1–0.2 m) and cleanly took the left corner** — no spin, no wall contact —
steering on a robust prior-map pose. It then failed in the **narrow corridor *after* the
corner**, where the localization **collapsed** (the pose teleported 16–31 m off the map) and it
drove into the right wall.

**This is a success.** The control stack (localization → path-following → wall-safety) is
proven on the hard part (long featureless straight + a corner). The remaining work is one
well-understood thing: make the localization survive the post-corner corridor.

---

## The architecture (what actually drives the robot)

```
RoboSense Helios 16P  ──/rslidar_points──┐
D455 IMU ──/camera/camera/imu──┐         │
                               ▼         ▼
                 spark-fast-lio (LiDAR+IMU odometry)
                               │  /lio/cloud_registered (frame odom_lio)
                               ▼
              icp_relocalization/icp_node  (ICP vs prior-map PCD)
                               │  /icp_result
                               ▼
              transform_publisher → TF  map → odom_lio   ← the robot's pose in the map
                               │
                               ▼
   run/rpp_safe.py:  RPP path-follower on TF(map→rslidar)  +  gentle LiDAR wall-repel
                               │  /cmd_vel
                               ▼
   locomotion/robot_drive.py (L0: clamps, ramps, dead-man, hard-stop) → Scout base (CAN)
```

The map is in the **route** loop (where to go) AND a trustworthy global pose; the **eyes**
(LiDAR) are a reactive wall-safety on top. One parent per TF frame:
`map → odom_lio → rslidar → base_link → wheels`.

---

## What WORKS ✅ (proven, keep)

| Piece | File / package | Evidence |
|---|---|---|
| **Prior-map localization** | `~/ros2_ws` `icp_relocalization` on `spark-fast-lio` | offline: 0% lost lock, ~2.8 cm over the full loop; live: re-locked at start to ICP fitness **0.030 m**, tracked the whole first straight + corner |
| **Loop-closed map** | `results/prior_map_loopclosed.pcd` (800k pts) | verified; the localization reference |
| **Path-follower** | `run/rpp_safe.py` (RPP + gentle wall-repel) | live: drove 36 m straight + left corner, cross-track 0.1–0.2 m, **no spin** |
| **L0 locomotion** | `locomotion/robot_drive.py` | the one reusable motion box (skid-steer /cmd_vel, ramps, dead-man) — unchanged all along |
| **Eyes-only baseline** | `locomotion/corridor_center.py` (GOLDEN) | proven eyes-only centering; the wall-repel law in `rpp_safe.py` is derived from it |
| **Taught route** | `results/teach_path_fastlio.csv` (628 poses, 251 m) | the RC-driven figure-8 loop, in the map frame |

`run/path_publisher.py` (publishes the route as `/plan`) and `sim/pp_sim.py` (RPP control
geometry, validated in sim) are also working/useful.

---

## What went RIGHT (the wins — this is the result)

1. **Robust localization, validated then proven live.** The 3-day "robot doesn't know where it
   is" blocker is solved on the hard part: it never lost lock through the long featureless
   straight, and it re-locks cleanly at the start (fitness 0.030 m). Architecture reused from
   proven components — no fragile new SLAM.
2. **The corner — the original failure — now works.** Run 1 *diverged* at the bottom-right
   corner. The fix (`best_effort` QoS on `/rslidar_points`, so the follower can never
   back-pressure FAST-LIO) made it round the left corner cleanly, cross-track 0.1–0.2 m.
3. **The controller is solved and simple.** Always follow the route (RPP); add a *gentle,
   continuous* wall-repel nudge only when a wall is closer than the path normally runs — while
   still driving forward, so it **arcs** off a wall and **cannot spin**. No mode switching.
4. **The autonomy is 100% local.** A full run continued correctly with the NUC's internet
   dropped — the network only carries the Foxglove view, never control.
5. **Safety logic that works:** clean zero-stream stop on exit (no base-latched velocity), and
   a "localization-lost" stop that halts rather than driving blind.

---

## What went WRONG + the remaining blocker

**Solved this session (fixes that worked):**
- Corner divergence → `best_effort` LiDAR QoS (back-pressure was starving FAST-LIO in the turn).
- Robot SPUN at a wall → a separate "GOLDEN mode" used crawl + hard turn = a pivot-in-place that
  spun and wrecked localization. **Fix: one mode only**, gentle continuous repel that keeps
  moving forward (arcs, never pivots).
- Premature stops → a stall watchdog halted on transient localization freezes. **Removed.**
- Base latched the last velocity on crash → clean STOPPING state streams zeros before exit.
- Localization stale after a physical carry → **restart FAST-LIO + ICP fresh** re-locks at the
  start (fitness 0.030). Do this before every session.

**THE remaining blocker (next time):**
- **Localization collapse in the long narrow corridor *after* the corner.** Heading north up a
  long, self-similar corridor is the worst case for the along-corridor aperture ambiguity; with
  the LiDAR delivering only ~8 Hz (not 10), the ICP found a wrong match and the pose **teleported
  16→31 m**. On that garbage pose the follower steered into the right wall. The lost-stop fired
  but a moment too late (after contact).

**The fix (well-understood, do this first next time):**
1. **ICP jump-reject gate** in `icp_node.cpp`: reject any single-scan correction that moves the
   pose more than ~1 m (real motion is < 2 cm/scan). Dead-reckon on FAST-LIO through the
   ambiguous stretch and re-anchor when features return. This makes the teleport *impossible* —
   the robust, textbook way to handle the degeneracy. (Same file already patched for continuous
   tracking, line ~185.)
2. **Faster lost-stop** in `rpp_safe.py`: lower the cross-track / consecutive-tick threshold so
   it halts within a few cm of a collapse, not after contact.
3. **Optional:** chase the 8 Hz LiDAR rate (it is *not* CPU — load ~1.3/22 cores; likely
   sensor/network delivery). A clean 10 Hz would reduce ICP starvation.

---

## How to reproduce a live run (exact, in order)

```bash
# 0) CAN up (every reboot); Scout base; RC physically OFF/disconnected, hand on e-stop.
sudo ip link set can0 up type can bitrate 500000
ros2 launch scout_cmd scout_mini.launch.py publish_odom:=false
#    (LiDAR /rslidar_points + D455 /camera/camera/imu must be live.)

# 1) FAST-LIO (LiDAR+IMU odometry, frame odom_lio):
ros2 run spark_fast_lio spark_lio_mapping --ros-args \
  --params-file projects/02_fast_lio2/config/helios16p_d455.yaml \
  -r lidar:=/rslidar_points -r imu:=/camera/camera/imu \
  -r odometry:=/lio/odometry -r path:=/lio/path -r cloud_registered:=/lio/cloud_registered \
  -p use_sim_time:=false -p common.map_frame:=odom_lio -p common.lidar_frame:=rslidar \
  -p common.imu_frame:=camera_imu_optical_frame -p common.base_frame:=rslidar \
  -p common.visualization_frame:=lidar -p gravity_alignment.enable_gravity_alignment:=false

# 2) Relocalizer (ICP vs prior map, initial guess 0,0 ⇒ re-lock at the start):
ros2 run icp_relocalization icp_node --ros-args -p use_sim_time:=false \
  -p map_path:=projects/03_autonomy/results/prior_map_loopclosed.pcd \
  -p map_frame_id:=map -p map_voxel_leaf_size:=0.4 -p cloud_voxel_leaf_size:=0.15 \
  -p max_correspondence_distance:=1.0 -p fitness_score_thre:=0.25 -p converged_count_thre:=5 \
  -p initial_x:=0.0 -p initial_y:=0.0 -p initial_z:=0.0 -p initial_a:=0.0 \
  -p pcl_type:=pointcloud -r /pointcloud2:=/lio/cloud_registered
ros2 run icp_relocalization transform_publisher --ros-args -p use_sim_time:=false \
  -p map_frame_id:=map -p odom_frame_id:=odom_lio
#    Park the robot at the map start; verify: tf2_echo map rslidar ≈ (0,0); ICP fitness < 0.05.

# 3) The follower (dry-run first, then live; hand on the e-stop):
cd projects/03_autonomy/run
/usr/bin/python3 rpp_safe.py --dry-run                 # prints clearances, NO motion
/usr/bin/python3 rpp_safe.py --cruise 0.12 --max-time 600
```
All ROS with `/usr/bin/python3` (conda python is 3.13, breaks rclpy). Per-cycle log →
`/tmp/rpp_safe.csv`. Snapshot a run over the map: `run/snap_drive.py --csv /tmp/rpp_safe.csv`
(conda python — matplotlib).

**Scout base gotcha:** it only actuates in CAN-command mode. Status frame `0x211` byte1 must be
`01` (CAN) and byte0 `00` (no e-stop/fault); the motion frame `0x111` carries linear mm/s.
Disconnecting the RC can drop the base to standby — confirm `0x211` before blaming software.

---

## Dead ends (do NOT resurrect)

- **"GOLDEN mode" switch with crawl + hard steer-away** → pivots in place → spins → wrecks
  localization. (Replaced by the gentle continuous repel in `rpp_safe.py`.)
- **Stall watchdog** ("stop if not advancing") → fires on transient localization freezes, halts
  a healthy robot. Removed.
- **Option B tightly-coupled `locate_in_prior_map`** → `No Effective Points` in the symmetric
  corridor. Closed; Option A (ICP on the registered cloud) wins. See `LOCALIZATION.md`.
- **Map-frame position steering** (`path_follow.py`) and **position-only centering**
  (`corridor_cruise.py`) → fishtail / walls. Superseded.
- **Sparse fixed-beam wall sensing** → misses thin protrusions. `rpp_safe.py` uses dense
  per-sector minima.

---

## Detailed docs (depth, all current)

- `LOCALIZATION.md` — the localization stack, Option A vs B, offline validation.
- `PATHFOLLOW.md` — the path-following steps (RPP), sim validation.
- `SLAM_AND_DATA_VERIFIED.md` — the map / loop closure / recorded data.
- `ROBOT_CONTROL_LEARNINGS.md` — the eyes-only control law (corridor_center.py) in depth.
- `sim/pp_sim.py`, `sim/junction_sim.py` — RPP + junction-routing simulations.

## Data collection (for later research on the clouds)
```bash
ros2 bag record /rslidar_points /camera/camera/imu /lio/odometry /tf /tf_static /cmd_vel
```
One supervised drive ⇒ aligned point clouds + IMU + trajectory.
