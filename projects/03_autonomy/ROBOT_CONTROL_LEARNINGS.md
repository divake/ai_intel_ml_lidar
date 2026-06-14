# Robot Control — Technical Learnings (the "don't re-research this" file)

> Everything we learn about **actually controlling this robot** lives here: which
> file, which parameter, which frame, why it matters. When a new idea comes up, read
> this first instead of re-deriving. Append freely; date the entries.

---

## 0. The machine (Agilex Scout Mini)

| Property | Value | Source / note |
|---|---|---|
| Drive type | 4-wheel **skid-steer** (differential) | can rotate in place; `motion_model: DiffDrive` for MPPI |
| Command topic | `/cmd_vel` (`geometry_msgs/Twist`) | only `linear.x` + `angular.z` are used |
| Dead-man | stops if `/cmd_vel` stream stops | Ctrl-C the publisher = instant stop |
| Body L×W×H | 0.612 × 0.580 × 0.245 m | footprint = body rectangle |
| Wheelbase (X) | 0.452 m → wheel x = ±0.226 | front/rear axle separation |
| Track (Y) | 0.490 m → wheel y = ±0.245 | left/right wheel separation |
| Wheel radius | 0.0875 m (Ø175 mm) | base_link sits at axle height |
| Base driver | `scout_cmd` pkg, `scout_mini.launch.py` (CAN `can0`) | params below |

### scout_cmd node params (file: `~/ros2_ws/src/scout_cmd/launch/scout_mini.launch.py`)
- `publish_odom` (default `true`) → publishes `/odom` **and** the `odom→base_link` TF.
  - **CRITICAL:** set `false` when FAST-LIO2 is the localizer, or you get a TF fight
    (two parents of `base_link`). `publish_odom:=false`.
- `odom_frame` (`odom`), `base_frame` (`base_link`).
- Wheel odometry heading is ~28° inaccurate over a run — **never trust it for nav**.

---

## 1. The TF tree (the thing that made or broke everything)

**The #1 lesson of project 03:** Nav2 navigates `base_link` (the robot body) inside
`global_frame`. If `base_link` isn't correctly connected to the localization frame,
nothing downstream can work — the robot literally doesn't know where its body is.

### What was WRONG (patched setup, June 14)
Two disconnected trees:
```
odom      → base_link   (Scout wheels)
odom_lio  → rslidar     (FAST-LIO2)
```
`base_link` and `rslidar` never linked → Nav2 was forced to use **rslidar (the LiDAR)
as the robot base**. No footprint, no body → can't center in a corridor.

### What's RIGHT (modeled setup)
One connected tree, FAST-LIO2 as the single pose source:
```
odom_lio ─(FAST-LIO2)→ rslidar ─(static mount)→ base_link ─(URDF)→ base_footprint, wheels
```
- FAST-LIO2 keeps publishing `odom_lio→rslidar` (DON'T touch project 02 — it's validated).
- A **static transform `rslidar→base_link`** bridges LiDAR pose down to the robot body.
- `robot_state_publisher` + the URDF publishes `base_link→{footprint, 4 wheels}`.
- Scout wheel-odom TF is **disabled** (`publish_odom:=false`) so `base_link` has ONE parent.

### Static mount `rslidar → base_link` (MEASURE-AND-REFINE)
- It is the LiDAR's pose expressed so that base_link sits *below* it.
- Args: `x y z yaw pitch roll rslidar base_link`.
- Current estimate: `0 0 -0.30 0 0 0` (LiDAR ~0.30 m above robot center, laterally centered).
- **Yaw is ~aligned** — proven by calibration (forward drive → FAST-LIO2 reads straight,
  -0.7°, within noise). RoboSense frame: +x forward, +y left, +z up (matches base_link).
- The measurement that matters most for **corridor centering** is the LiDAR's **lateral (y)
  offset** from the robot centerline. If centered (y≈0), centering works at any height.

### How to verify the tree (no motion needed)
```
ros2 run tf2_tools view_frames        # one connected tree, base_link has ONE parent
ros2 run tf2_ros tf2_echo odom_lio base_link    # should stream a pose
```

---

## 2. Localization

- **Decision (June 14):** FAST-LIO2 is the live 3D pose source. It never loses track.
- **2D AMCL FAILED** in the long featureless corridor ("corridor problem" — ambiguous
  along the hallway axis). Don't use AMCL here.
- The taught path (`results/teach_path_fastlio.csv`, 628 wp / 291 m) is in `odom_lio`.
  **Start the robot at the same physical spot the recording started** so `odom_lio`
  origin ≈ path start.
- **Future robustness upgrade (not yet needed):** continuous map-matching against the
  saved loop-closed map (`lidar_localization_ros2` / `hdl_localization` style) to kill
  FAST-LIO2's slow drift over the full 300 m / 6-8 turns.

---

## 3. Nav2 control stack (teach-and-repeat, no global planner)

We FOLLOW a recorded path (`FollowPath` action) — local costmap + MPPI handle real-time
obstacles. cmd_vel chain:
```
controller_server(cmd_vel_nav) → velocity_smoother(cmd_vel_smoothed) → collision_monitor(cmd_vel) → Scout
```

### Controller: MPPI (`nav2_mppi_controller`) — config in `config/nav2_live.yaml`
- Why MPPI over Regulated Pure Pursuit: **RPP rigidly tracks the recorded line and does
  NOT center** between walls → drifts ~0.3 m into a wall and stops. MPPI samples many
  trajectories and scores them against the costmap → it **centers** and adjusts.
- Key params: `motion_model: DiffDrive`, `vx_max` (speed cap), `wz_max` (turn cap),
  `batch_size` (# sampled trajectories), `time_steps`×`model_dt` (horizon).
- Critics (the "personality"): `PathFollowCritic`/`PathAlignCritic` keep it on the route;
  `CostCritic` pushes it off obstacles (centering); `PreferForwardCritic` discourages
  reversing; `GoalCritic`/`GoalAngleCritic` for the endpoint. **Tune weights, not code.**
- `consider_footprint: true` only meaningful once a real footprint exists (it does now).

### Costmap (`local_costmap`)
- `global_frame: odom_lio`, `robot_base_frame: base_link` (was `rslidar` in the hack).
- Use the real **footprint** rectangle (not `robot_radius`) now that we have the URDF.
- Obstacles from `/scan` (see §4). `inflation_radius` + `cost_scaling_factor` shape how
  hard it pushes away from walls — the real centering knobs.

### collision_monitor (last-resort hard stop)
- Past bug: a 0.5 m **circle** caught the side walls in a corridor → false stops.
- Fix: a **forward box** only, in `base_link`, ahead of the robot's front edge (x≥0.31).
- `base_frame_id` must match the costmap base (`base_link`).
- **GOTCHA (cost us an activation, Jazzy nav2 1.3.11):** `points` MUST be the **nested**
  string `"[[x,y],[x,y],...]"`. A flat `"[x,y,x,y,...]"` fails with `Numbers at depth
  other than 2` → the polygon won't load → **lifecycle_manager aborts the WHOLE bringup**
  → controller_server never activates → **MPPI silently stays inactive**. If MPPI won't
  activate, check the collision_monitor log first. Reference format lives in
  `/opt/ros/jazzy/share/nav2_collision_monitor/params/collision_monitor_params.yaml`.

### velocity_smoother
- `feedback: OPEN_LOOP` → does NOT need `/odom` (safe with `publish_odom:=false`).

---

## 4. Perception → costmap (`pointcloud_to_laserscan`)
- 3D `/rslidar_points` → 2D `/scan` for the costmap. File: `run/autonomous_drive.launch.py`.
- `target_frame: rslidar`, `min_height`/`max_height` slice the 3D cloud to a horizontal
  band (set so it sees walls but not the floor/ceiling). `range_min: 0.5` ignores the
  robot/mount returns.

---

## 5. Hard-won environment gotchas (cross-ref the big CLAUDE.md + memories)
- **UDP buffer:** `net.core.rmem_max=26214400` (persisted `/etc/sysctl.d/10-rslidar-udp.conf`).
  Reboot resetting this → dropped LiDAR scans → FAST-LIO2 diverges (the "800 km" blowup).
- **Two Pythons:** ROS/rclpy → `/usr/bin/python3`; rendering → conda `intel_ai`.
- **FAST-LIO2 gravity gate:** `enable_gravity_alignment: False` or it won't publish/map
  until ~2 s of motion.
- **Safety:** always test WITH a hand on e-stop; speeds capped slow; climb the ladder in
  `AUTONOMOUS_DRIVE.md` (base→localize→2 m→loop). Never jump rungs.

---

## Changelog
- **2026-06-14:** File created. Diagnosed the disconnected-TF root cause; designed the
  one-tree fix (URDF self-model + static `rslidar→base_link` + disable wheel-odom TF).
