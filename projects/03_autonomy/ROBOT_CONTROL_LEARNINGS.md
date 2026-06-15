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

## 8. FAST-LIO autonomy + the corridor problem (2026-06-15)

The session that connected all the pieces: a working autonomy stack, the *root-cause*
diagnosis of every prior corridor failure, the "human-like" centering design, and a pile
of hardware gotchas that each cost real time. Read this one first if the robot won't drive.

### 8.1 The working autonomy stack (the architecture)

```
L0  robot_drive.py     "software remote": skid-steer = TWO numbers (linear.x + angular.z)
                        + clamps + accel ramps + dead-man. Owns motion, nothing else.
L2  path_follow.py     pure-pursuit follow of a TAUGHT path, localized by FAST-LIO,
                        + a LiDAR wall-repel comfort band for centering.
        |  uses
        +-- corridor_control.py   the SHARED controller (pure function `plan()`):
                                   used by the Gazebo sim AND (conceptually) the robot.
L1  corridor_cruise.py  reactive LiDAR-only cruise — SUPERSEDED for corridors (see §8.4).
```

- **L0 `locomotion/robot_drive.py`** — unchanged from §7, the gold standard. Caps
  `max_v=0.30`, `max_w=0.80`; ramps `accel_v=0.6`, `accel_w=2.0`; dead-man
  `cmd_timeout=0.7 s`; 50 Hz publish. Skid-steer = two numbers; the rest is firmware.
- **L2 `locomotion/path_follow.py`** — the live autonomy node. Subscribes a localization
  `Odometry` (`--odom-topic`, default `/kiss/odometry`; **use `/lio/odometry`** for the
  real drive) + raw `/rslidar_points` (RELIABLE QoS). Loads a taught path CSV
  (`--path`, default `teach_path.csv`; **use `teach_path_fastlio.csv`** for the live FAST-LIO
  drive). Pure-pursuit aims at a lookahead point (`lookahead=0.8 m`), cruises at
  `cruise=0.18 m/s`, caps turn `max_w=0.45`. Writes a per-cycle movement log to
  `/tmp/pf_movement.csv` (`t,x,y,yaw,d_left,d_right,v,w,w_path,w_repel,state`) for tuning.
- **`locomotion/corridor_control.py`** — the single source of truth for the *control geometry*
  (`plan(pose, path, idx, d_left, d_right, cfg)`), pure Python / no ROS. The Gazebo +
  lightweight sims call it directly so "what we validate in sim is literally what runs on the
  robot." (Note: `path_follow.py` currently inlines an equivalent — but slightly DIFFERENT
  — control law: `w_path = 0.7*alpha` in `path_follow.py` vs `1.1*alpha` in
  `corridor_control.py`, and a comfort-band speed law vs the sim's alpha-based one. The
  reduced 0.7 gain on the robot is deliberate — see §8.3.)

### 8.2 THE CORRIDOR PROBLEM (root cause — the biggest finding of the project)

**LiDAR-only odometry cannot measure forward (along-corridor) translation in a long,
smooth, featureless corridor.** Two parallel walls shifted along their own length look
*identical* to a scan matcher — there is no feature to anchor "how far did I move
forward." This is the classic **aperture / sliding ambiguity**. The matcher locks lateral
position and heading (it sees the two walls) but slides freely forward, so it
**under-reports forward motion**.

- **PROVEN in the Gazebo sim** (commit `dacc296`): running the REAL stack
  (KISS-ICP + `path_follow`) on a simulated 16-beam LiDAR, the robot physically drove
  **12.5 m** but KISS-ICP tracked only **3.7 m** — an **8.8 m under-track**. Not corners,
  not tuning: the straight corridor itself is unobservable to LiDAR-only odometry.
- **Same ambiguity killed AMCL earlier** (§2): 2D scan-match localization is ambiguous
  along the hallway axis for the same reason.
- **THE FIX = FAST-LIO2 (LiDAR + D455 IMU).** The IMU dead-reckons the forward
  acceleration/velocity the LiDAR physically cannot see; the EKF fuses it with the LiDAR's
  good lateral+heading. **Confirmed on the real robot: FAST-LIO accurately tracked 8.6 m
  forward where KISS failed.**
- **This is WHY the project needs the IMU.** The user had briefly disabled the IMU path
  (LiDAR-only is simpler, fewer failure modes — see proj02 `device busy` gotchas). The
  corridor problem forced it back: in a corridor, IMU is not optional, it is the *only*
  thing that observes forward progress. The sim (§8.5) is what made this undeniable
  before burning robot-hours on it.

### 8.3 Centering design — the "human-like" controller

Steer by the **REAL walls** (LiDAR, frame-independent), **not** by chasing the path
exactly. Why: the taught path is in the FAST-LIO `odom_lio` frame, but on a live run that
frame is somewhat **rotated** vs the real corridor (start-alignment error at the origin +
slow FAST-LIO drift). So blindly servoing to the path's lateral coordinate walks you into
a wall. Hence in `path_follow.py`:

- **Path gives DIRECTION, at reduced trust:** `w_path = 0.7 * alpha` (deliberately <1.0;
  `path_follow.py:160`). The path's *heading* (which way the corridor goes, where the
  corners are) is reliable; its exact *lateral* aim in the live frame is not.
- **LiDAR gives CENTERING, repel-only:** `w_repel` only pushes *away* from a wall that is
  within `safe` (`safe=0.65 m`, `k_repel=2.0`). Repel-only is the key safety property:
  **cavity-safe.** A lab-doorway cavity makes a wall *farther*, never *closer*, so a
  repel-only law NEVER pulls toward a cavity (a *follow*-the-gap law would — see §8.4).
- **Comfort band (no twitching):** *inside* the band (both walls beyond `safe`) `w_repel=0`
  → go **dead straight** at **full speed**. Only when a wall enters `safe` does it ease away.
  Tolerate being off-center; act only when uncomfortable — exactly how a human drives a
  hallway.
- **Speed couples to steering, not to position:** `v = cruise * (1 - 0.55*|w|/max_w)`
  (`path_follow.py:173`) → full speed when straight, slows *only while actively turning*;
  extra-slow (≤0.08) if any wall is inside `hard_min=0.35 m`. (The shared
  `corridor_control.plan()` uses an equivalent law keyed on `alpha` instead of `w`.)
- The per-cycle `/tmp/pf_movement.csv` log is the tuning instrument: it records
  `d_left,d_right,v,w,w_path,w_repel,state` every cycle so you can see exactly *why* it
  turned (path vs repel) after a run, instead of guessing.

### 8.4 The cavity problem (why direction must come from the path)

Reactive **wall-FOLLOWING** — centering on `d_left - d_right` (the L1 `corridor_cruise.py`
gentle-centering term, `corridor_cruise.py:160-164`) — **fails at a cavity** (a lab
doorway, an alcove). When one wall recedes into the cavity, the controller reads that side
as "more open" and **steers INTO the cavity**. That is the textbook failure of pure
wall-following. The fix is the §8.3 split: **direction comes from the path** (which knows
the corridor continues straight past the doorway); **walls are used only for repel/safety**,
never to choose where to go. This is why L1 `corridor_cruise.py` is superseded for
corridors and L2 `path_follow.py` is the live node.

### 8.5 The Gazebo sim (`sim/gazebo/`) and the python sim (`sim/corridor_sim.py`)

- **Gazebo Sim (Harmonic / gz-sim 8)** — already installed. `scout_gz.urdf` is a faithful
  Scout Mini: real dims (body 0.612×0.580, wheels r=0.0875, LiDAR ~0.30 m up), a 16-beam
  `gpu_lidar` (`vertical samples=16`, ±0.2618 rad = ±15°, 10 Hz), and a `gz-sim-diff-drive`
  plugin on `cmd_vel` — the **same interface as the real robot**, so the real stack
  (KISS-ICP + controller) runs on it **unchanged**. `gen_corridor_world.py` builds a
  rectangular **loop** corridor (12×8 centerline, 1.8 m wide, 4 corners) + a matching
  `teach_path_sim.csv` centerline. Faithful enough that **it exposed the corridor problem**
  (§8.2) — the whole point of building it.
  - **GOTCHA — gz DiffDrive HOLDS the last `cmd_vel` (no dead-man).** Unlike the real Scout
    (which stops if `/cmd_vel` stalls) and unlike our L0 box, the Gazebo plugin keeps
    applying the last command forever. **Stream zeros to actually stop** in sim.
- **`sim/corridor_sim.py`** — a fast, lightweight, **pure-Python** sim (no ROS, no Gazebo):
  loads the real loop-closed map + taught path, ray-casts a LiDAR, and drives with the
  SAME `corridor_control.plan()`. `--loc perfect|lowrate|noisy` models localization quality.
  Use it for *control-geometry* validation: with **perfect** localization the controller
  follows the full ~290 m loop and every corner (reached wp 578/580, no real collisions) —
  proving the **control is sound** and the blocker is **localization**, not steering.
- **MAP-FRAME GOTCHA:** the taught path `teach_path.csv` is in the **`loopclosed_map`**
  frame (origin `[-34.280, -18.127]`, lands ~2% on walls), **NOT `nav_grid`** (origin
  `[-64.996, -19.650]` — a different frame; the path lands ~40% on its walls). Always render
  the path against `loopclosed_map.yaml`. (Live drives use `teach_path_fastlio.csv` in the
  `odom_lio` frame instead.)

### 8.6 HARDWARE GOTCHAS (each cost real debug time — 2026-06-15)

- **RC-mode lockout (cost a LONG debug — the software was fine).** If you drive the Scout
  with its handheld **RC transmitter**, the base enters **RC control mode and IGNORES all
  CAN `/cmd_vel` commands.** The symptom is maddening: the base node still publishes,
  **CAN TX climbs** (`ip -statistics link show can0` looks healthy), yet the robot **does not
  move.** The status frame **`0x211`** carries the control-mode byte (check it with
  `candump can0`). **FIX: turn the RC off, or set its mode switch to command/CAN mode.**
  Rule: before debugging "won't move" in software, confirm the robot is *in CAN command
  mode*, not RC mode.
- **D455 IMU "device busy" (FAST-LIO dies at the source).** FAST-LIO needs the D455 IMU.
  - The realsense node needs **color + depth ENABLED** — an IMU-only config fails with
    `device busy` on this rig (see proj02). And the D455 must be on a **real USB 3.x port**.
  - After a **hot unplug/replug**, the camera firmware can get stuck `Device or resource
    busy`: `depth_module get_xu` fails → "Error starting device" → **IMU dies at 0 Hz** →
    `/lio/odometry` stops. **A software USB unbind/bind reset does NOT clear it** — only a
    **PHYSICAL unplug → wait 5 s → replug** power-cycles the firmware out of it.
  - **Healthy looks like:** IMU **~198 Hz**, `/lio/odometry` **~208 Hz**. Check those rates
    first when FAST-LIO is silent.
- **CAN is DOWN after reboot.** `can0` is `state DOWN` on boot. Bring it up **BEFORE**
  launching the base: `sudo ip link set can0 up type can bitrate 500000` (see §5 for the
  full order-matters reasoning — base-before-can0 = `Failed to send CAN frame` forever).
- **LiDAR rate.** The real LiDAR has held a clean **10 Hz** this session. Earlier
  flakiness / `MSOPTIMEOUT` was **CPU contention**, not the sensor — e.g. the `whoopsie`
  crash-uploader pinning a core. If scans get flaky, check `top` for a CPU hog before
  blaming the LiDAR.

### 8.7 RUNBOOK — full real autonomy drive (FAST-LIO + path_follow)

Pre-flight: **RC transmitter OFF** (or in CAN/command mode — §8.6). Hand on the e-stop.

```bash
# 1) CAN up FIRST (every reboot; Scout = 500 kbit/s; sudo needs no password here)
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0          # expect: state UP, ERROR-ACTIVE

# 2) FAST-LIO data engine (LiDAR + D455 IMU + spark-fast-lio). Leave running.
#    Gives /lio/odometry (~208 Hz), /rslidar_points (XYZIRT, 10 Hz), IMU (~198 Hz).
bash projects/02_fast_lio2/run/start_pipeline.sh
#    Verify health: ros2 topic hz /lio/odometry  and  ros2 topic hz /camera/camera/imu

# 3) Scout base (publish_odom:=false so base_link has ONE TF parent — §1)
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch scout_cmd scout_mini.launch.py publish_odom:=false

# 4) Place the robot at the ROUTE START (where teach_path_fastlio.csv was recorded,
#    so odom_lio origin ~ path start), pointed down the corridor.

# 5) DRY-RUN first (prints the plan + what it sees, commands NO motion):
cd projects/03_autonomy/locomotion
/usr/bin/python3 path_follow.py --path ../results/teach_path_fastlio.csv \
    --odom-topic /lio/odometry --dry-run

# 6) LIVE (hand on e-stop). 3 s countdown, then it drives:
/usr/bin/python3 path_follow.py --path ../results/teach_path_fastlio.csv \
    --odom-topic /lio/odometry
#    Tuning afterwards: inspect /tmp/pf_movement.csv (path vs repel per cycle).
```

Run everything ROS with **`/usr/bin/python3`** (conda's 3.13 breaks rclpy).

## 9. EYES-ONLY corridor centering — the breakthrough (GOLDEN controller, 2026-06-15)

After two nights of the robot banging into corridor walls, the fix was to **stop trusting
the map for steering and drive by the LiDAR (the eyes)**. File: `locomotion/corridor_center.py`.
**This controller is GOLDEN — proven on hardware, do not modify.**

### 9.1 The two root causes (both proven live)
1. **The map poisoned the steering loop.** `teach_path_fastlio.csv` is in the `odom_lio`
   frame and starts at (0,0), but FAST-LIO **re-zeroes/drifts every session** — so the
   recorded path and the live `/lio/odometry` pose are in DIFFERENT frames. Parked
   dead-center, the eyes said centered (L≈R≈0.9 m) but the map said ~1.7 m off → every
   "correct toward the map" command drove the robot at a wall. FAST-LIO is *odometry*, not
   a relocalizable map. **Lesson: never use the map for lateral position.**
2. **Position-only steering fishtails.** `path_follow.py` / `corridor_cruise.py` set
   turn-rate ∝ wall distance ONLY → textbook oscillation of a turn-rate actuator; at 8 Hz
   LiDAR + too-fast speed it overshot into the wall (yaw swung −8°→−59°→back). Web research
   (F1TENTH wall-follow) confirms: steer on lateral error **and heading**, and go slow.

### 9.2 The method (F1TENTH wall-follow, both walls)
Per side, two beams: abeam `b` (±90°) and forward `a` (±45°), median over a ±6° window
(robust). Then `alpha = atan2(a·cos45 − b, a·sin45)` (wall angle) and `D = b·cos(alpha)`
(perp distance). Control law (signs verified against a parked measurement):
```
e   = D_left − D_right          # lateral error (<0 => too close to LEFT => steer right)
phi = (alphaL − alphaR)/2       # heading error (>0 => nose pointed RIGHT of corridor)
w   = kp·e + kd·phi             # kp=0.6, kd=0.8 ; the kd·phi term is the anti-fishtail damping
```
`phi` is EMA-smoothed (0.5) and clamped (±20°) to reject doorway/cavity spikes (the weave).
Comfort band (|e|<0.10 m and |phi|<0.10 rad ⇒ dead straight, no twitch). Slow (`cruise=0.10`
m/s) so the 8 Hz feedback keeps up. Safety: crawl if a wall < `hard_min` 0.35 m; HARD
steer-away if < `crit` 0.28 m; stop if blocked ahead < 0.30 m. Reuses L0 `robot_drive`.

### 9.3 Corners — handled by centering, with a reactive turn as backup
A reactive corner turn exists (front < `front_turn` 1.0 m ⇒ turn toward the more-open side —
the "arrow"), but in practice **it never fired**: as the corridor bends, the heading term
sees the nose is far off the new corridor and drives a hard correction (`w` saturates at
±0.35) until aligned, then settles back to centered. **The robot follows bends with its
eyes**, like a person. First two corners taken cleanly this way (one a bit wobbly, ~30 s).

### 9.4 Proven result (the run that worked)
Drove the straight + multiple corners of the loop, **never closer than 0.35 m to any wall**
(run-log verified min 0.351 m run1 / 0.451 m run2 — the earlier "0.47 m" was optimistic),
self-centered from 0.33 m off → centered, no fishtail, smooth small `w`. Per-cycle log:
`/tmp/corridor_center.csv` (`t,D_left,D_right,e,phi_deg,front,v,w,state`).

### 9.5 Where the map earns its keep (Phase 2, not yet built)
The map is NOT needed to avoid walls. It is only needed at an **ambiguous junction** (both
left AND right open) to choose the route — classic global-planner (A*/Dijkstra on a 2D map)
+ local-reactive (LiDAR free-space) split, exactly like Nav2. Single-opening corners are
reactive (no map). Honest caveat: scan-to-map relocalization in a long featureless corridor
hits the same aperture ambiguity (§8.2) that broke AMCL — it's a sub-project.

### 9.6 The limit we hit on the long run (2026-06-15 night) — why Phase 2 is next
On a 25-min supervised drive the golden controller did great: straight ✅, single-opening
corners ✅ (one even a cavity-corner). It hit its designed limit at **ambiguous topology**:
- At a **junction** (left branch AND straight both open) it can't *choose the route* — it
  has no map. It went straight (balanced walls ⇒ centered through the widening); the taught
  lab route actually went left. Reactive guess = "furthest-open side," not the route.
- At a **complex multi-cavity section** it **ping-ponged**: front blocked ⇒ backup turn ⇒
  turned toward the most-open direction, which was *back the way it came* ⇒ U-turn ⇒ back to
  the junction ⇒ repeat (`TURN_RIGHT` fired 3× at front≈1.0 in the log). **Root cause: the
  controller is memoryless** — no notion of "I already came from there." Safe the whole time
  (never hit a wall), just non-productive wandering.

**Phase 2 fix (two levels, lightweight first):**
1. **Route memory (tiny, no map):** forbid reversing into the corridor just exited ⇒ breaks
   the ping-pong immediately.
2. **Map + A\* router (the real one):** at each junction the planned route says "go left
   here" and overrides the reactive furthest-open guess ⇒ follows the taught route
   deterministically. This is the *only* job the map has; centering stays 100% eyes.

Run data preserved for analysis: `results/corridor_runs/run_*.csv` (per-cycle
`t,D_left,D_right,e,phi_deg,front,v,w,state`).

## Changelog
- **2026-06-15:** Added §9 (EYES-ONLY corridor centering — the GOLDEN controller
  `corridor_center.py`). Threw the map out of the steering loop (frame-stale → drove into
  walls) and switched to F1TENTH wall-follow (lateral + heading PD, slow). Proven: drove the
  straight + corners, never closer than ~0.35 m to a wall (verified min 0.351 m), no fishtail. Corners rounded by
  centering itself (heading follows the bend); reactive turn is a backup. Map reserved for
  Phase 2 (ambiguous-junction routing only). `path_follow.py` / `corridor_cruise.py` superseded.
- **2026-06-15:** Added §8 (FAST-LIO autonomy stack + the corridor problem). Diagnosed the
  ROOT CAUSE — LiDAR-only odometry can't observe along-corridor translation (aperture
  ambiguity; proven 12.5 m driven / 3.7 m tracked in Gazebo; fixed by FAST-LIO IMU, 8.6 m
  tracked on the robot). Documented the human-like centering design (path=direction at 0.7
  gain, LiDAR=repel-only comfort band, cavity-safe), the Gazebo + python sims, the
  RC-lockout / D455-busy hardware gotchas, and the full real-drive runbook.
- **2026-06-15:** Added §7 (layered rebuild) + CAN-after-reboot gotcha (§5). **L0
  locomotion box (`robot_drive`) built and PROVEN on hardware** — all primitives verified.
- **2026-06-14:** File created. Diagnosed the disconnected-TF root cause; designed the
  one-tree fix (URDF self-model + static `rslidar→base_link` + disable wheel-odom TF).
- **2026-06-14 (night):** Added §6. Self-model verified (drives straight); perception +
  velocity pipeline clean. Movement unsolved: MPPI crawls (tuning), RPP freezes when
  wall-hugging. Switched controller MPPI→RPP. Rules: inflation ≥ inscribed; start centered.
