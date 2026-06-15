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
- **CAN is DOWN after every reboot → robot won't move.** The Scout talks over `can0`
  (USB `gs_usb` adapter). After boot `can0` is `state DOWN`. Bring it up BEFORE launching
  the base: `sudo ip link set can0 up type can bitrate 500000` (Scout = 500 kbit/s; sudo
  needs no password here). Verify `state UP` / `ERROR-ACTIVE` and `candump can0` shows
  robot frames (0x251-0x254 motors, 0x241 status).
  - **ORDER MATTERS:** if the base node (`scout_cmd`) launches while `can0` is down, it
    prints `Failed to send CAN frame` forever and **TX stays 0** (commands never reach the
    wheels) — even after you later bring can0 up. Fix = bring up can0 FIRST, then launch
    the base (or restart the base after can0 is up). Health: `ip -statistics link show can0`
    → TX climbing; `ros2 topic echo /odom --once` → publishing.
- **UDP buffer:** `net.core.rmem_max=26214400` (persisted `/etc/sysctl.d/10-rslidar-udp.conf`).
  Reboot resetting this → dropped LiDAR scans → FAST-LIO2 diverges (the "800 km" blowup).
- **Two Pythons:** ROS/rclpy → `/usr/bin/python3`; rendering → conda `intel_ai`.
- **FAST-LIO2 gravity gate:** `enable_gravity_alignment: False` or it won't publish/map
  until ~2 s of motion.
- **Safety:** always test WITH a hand on e-stop; speeds capped slow; climb the ladder in
  `AUTONOMOUS_DRIVE.md` (base→localize→2 m→loop). Never jump rungs.

---

## 6. Movement debugging — the long night (2026-06-14, ~02:00–03:00)

What we **fixed and proved** (banked):
- **Self-model works.** With the URDF + one-tree fix, the robot drives **dead straight**
  (0.01 m lateral over a run). The old veer-into-the-wall is gone.
- **Perception clean.** Floor-exclusion (`min_height -0.20`) + inflation tuning →
  forward costmap channel **0/115 blocked** in a clear corridor.
- **Velocity pipeline clean.** `cmd_vel_nav` == `cmd_vel_smoothed` == `cmd_vel`.

What we **did NOT crack** — getting a clean brisk autonomous drive:
- **MPPI crawls (~0.016–0.064 m/s) even on a clear path.** It's a cost *equilibrium*:
  forward-reward (PathFollow) vs speed-penalty (`gamma`) vs short planning horizon
  (`time_steps × model_dt × vx_max` ≈ 0.4 m at our safe low `vx_max`). Boosting
  PathFollow 5→20 and gamma 0.015→0.008 raised speed 0.016→0.064 but never to full
  speed. **Conclusion: MPPI needs careful OFFLINE tuning, not live trial-and-error on
  the robot.** Canonical MPPI configs only drive briskly because they run `vx_max 0.5`.
  MPPI config preserved in `config/nav2_live_mppi.yaml`.
- **RPP (`config/nav2_live.yaml` now) commands a SET speed (no crawl)** — the right tool
  for a basic reliable drive — but it **refused to move (cmd 0)** from the test spot.

**THE KEY INSIGHT (root cause of "won't move"):** a correctly-modeled, safe robot
**will not accelerate when it's hugging a wall.** Both controllers refused for this same
reason — the robot kept ending up ~0.58 m from a wall (≈0.29 m from its body edge) in a
narrow (1.76 m) section. That's the **safety working**, not a bug. MPPI crawls, RPP
freezes; both want a **centered start**.

**Two hard rules learned:**
1. `inflation_radius` **must be ≥ the footprint inscribed radius** (here 0.30 m) or Nav2
   collision-checking is invalid and **RPP refuses to move** (logs the warning at
   costmap configure). We had it at 0.22 → wrong. Now 0.30.
2. **Always start a drive from the corridor CENTER**, pointed straight. Verify left≈right
   from `/scan` BEFORE commanding motion. Don't drive out of a wall-hug.

**Untested / next step:** a clean **RPP drive from a centered start** — never got to run
it (the robot was wall-hugging every attempt). Highest-probability quick win tomorrow.

## 7. The layered rebuild — L0 locomotion box PROVEN (2026-06-15)

After the night of Nav2 trial-and-error, we stepped back and rebuilt **bottom-up, each
layer tested in isolation** — the opposite of surgical patching. The architecture:

```
L0  robot_drive   "software remote"        ← PROVEN on hardware
L1  reactive safety / recovery             "never freeze — always act"   (next)
L2  navigation: center + follow path + turns
L3  localization (FAST-LIO2 + taught path)
```

**Key reframe (ends the "why can't we control it?" worry):** controlling the robot is
genuinely simple — it's **two numbers** (`linear.x`, `angular.z`); skid-steer math is in
Agilex firmware. Our old failures were NOT a control gap; they were a *policy* gap — Nav2
configured to **veto** (freeze near walls) instead of **act** (back off / wait / re-plan).
Delivery robots (Chicago etc.) aren't magic: their safety layer always produces a *move*,
never a freeze, with a human teleop as last resort. L1 will be built that way.

**L0 = `locomotion/robot_drive.py`** (+ `locomotion/README.md`). A reusable `RobotDrive`
class wrapping `/cmd_vel`: clamps, accel ramps, a dead-man watchdog (silent commander →
coast to 0), steady publish stream, hard-stop on exit. CLI proves each primitive.
**Verified by eye 2026-06-14:** forward, back, spin-left, spin-right, arcs — all clean.
TX 1→123 over a 2 s drive confirmed frames reach the wheels. This is the gold standard;
everything above stands on it.

## Changelog
- **2026-06-15:** Added §7 (layered rebuild) + CAN-after-reboot gotcha (§5). **L0
  locomotion box (`robot_drive`) built and PROVEN on hardware** — all primitives verified.
- **2026-06-14:** File created. Diagnosed the disconnected-TF root cause; designed the
  one-tree fix (URDF self-model + static `rslidar→base_link` + disable wheel-odom TF).
- **2026-06-14 (night):** Added §6. Self-model verified (drives straight); perception +
  velocity pipeline clean. Movement unsolved: MPPI crawls (tuning), RPP freezes when
  wall-hugging. Switched controller MPPI→RPP. Rules: inflation ≥ inscribed; start centered.
