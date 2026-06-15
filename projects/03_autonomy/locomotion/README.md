# Locomotion — the autonomy driving stack (L0 → L2)

This folder is the live driving stack. The driver that actually works is **L1**
`corridor_center.py` — **EYES-ONLY** corridor centering (no map in the loop), built on the
proven **L0** `robot_drive.py` (the software remote). The map-based `path_follow.py` and the
position-only `corridor_cruise.py` are **SUPERSEDED** (see why below). Stack map + the
golden-controller docs + runbook are at the **bottom**.

> **GOLDEN (2026-06-15):** `corridor_center.py` drove the corridor straight + multiple
> corners and **never came within 0.47 m of a wall** — no fishtailing. **Do not modify it.**
> Full story: `../ROBOT_CONTROL_LEARNINGS.md` §9.

---

# L0 — Locomotion box (`robot_drive`)

**The one reusable way this robot moves.** Every project uses this, unchanged. It is
the *software remote control*: you give it intent (forward / back / spin / arc / stop,
with a speed), it makes the wheels do it — safely. It owns **nothing** about maps,
localization, or navigation. Just motion.

> Status: **PROVEN on hardware 2026-06-14** — all primitives verified by eye
> (forward, back, spin-left, spin-right, arcs). This is the gold standard; build on it.

---

## The one fact that ends the "how do we control 4 wheels?" confusion

The Scout Mini is **skid-steer** (a tank). You do **not** steer it by driving four
wheels individually — you give **two numbers** and Agilex's `ugv_sdk` firmware turns
them into left-side / right-side wheel speeds over CAN:

| You command | Field | Robot does |
|---|---|---|
| forward / back | `linear.x`  (m/s) | both sides equal |
| turn left / right | `angular.z` (rad/s, +z = **left**/CCW per REP-103) | sides opposite (spin) or unequal (arc) |

The handheld remote sends these *same two numbers*. There is no lower level to unlock —
the hard skid-steer math is in firmware. **Anything the remote can do, this box can do.**

---

## What the box adds on top of raw `/cmd_vel` (the "non-breakable" part)

- **Clamps** — never exceeds safe caps (`max_v` 0.30 m/s, `max_w` 0.80 rad/s by default).
- **Accel ramps** — smooth starts/stops, no jerk (a fixed-rate timer ramps the *current*
  velocity toward the *setpoint*; you only ever set the setpoint).
- **Dead-man watchdog** — if the commander above goes silent for `cmd_timeout` (0.7 s),
  velocity decays to zero. A stalled brain can't run the robot away.
- **Steady publish stream** — the Scout stops if `/cmd_vel` stalls; the box keeps it fed.
- **Hard-stop on exit / Ctrl-C** — the robot *always* stops when the process dies.

---

## Use as a library (your algorithm streams setpoints)

```python
import rclpy
from robot_drive import RobotDrive

rclpy.init()
drive = RobotDrive()                 # defaults are safe
# ... your control loop, call repeatedly (ramps + safety applied automatically):
drive.set(v, w)                      # raw setpoint
drive.forward(0.2); drive.spin_left(0.4); drive.arc(0.15, 0.3); drive.stop()
# on shutdown:
drive.hard_stop(); drive.destroy_node(); rclpy.shutdown()
```

Blocking helpers for scripted moves: `drive.drive_for(v, w, seconds)`, `drive.smooth_stop()`.

## Use as a CLI (prove each primitive in isolation, hand on the e-stop)

```bash
/usr/bin/python3 robot_drive.py demo                  # full scripted sequence
/usr/bin/python3 robot_drive.py forward --v 0.15 --t 3
/usr/bin/python3 robot_drive.py back    --v 0.15 --t 3
/usr/bin/python3 robot_drive.py left    --w 0.4  --t 3   # spin in place
/usr/bin/python3 robot_drive.py right   --w 0.4  --t 3
/usr/bin/python3 robot_drive.py arc     --v 0.15 --w 0.3 --t 3
/usr/bin/python3 robot_drive.py stop
```

Run with **`/usr/bin/python3`** (conda python is 3.13 → breaks rclpy).

---

## Prerequisite to move: the base must be up and CAN must be live

**After every reboot, `can0` is DOWN.** Bring it up *before* launching the base, or the
base node prints `Failed to send CAN frame` forever (TX stuck at 0). Order matters:

```bash
# 1. bring up the CAN link (Scout = 500 kbit/s). sudo works without a password here.
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0          # expect: state UP, can state ERROR-ACTIVE

# 2. confirm the robot is alive on the bus (you should see frames stream):
timeout 3 candump can0              # frames 0x251-0x254 (motors), 0x241 (status)...

# 3. NOW launch the base (subscribes /cmd_vel, publishes /odom):
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch scout_cmd scout_mini.launch.py
```

**Health checks (objective, no eyes needed):**
- `ip -statistics link show can0` → **TX climbing** = the base is sending to the robot.
- `ros2 topic echo /odom --once` → publishing = bidirectional link confirmed.
- (Note: Scout **wheel odom is unreliable** for magnitude/heading — use it only as a
  "did it move at all" sanity check; real localization is FAST-LIO2.)

For this isolated L0 test you do **not** need FAST-LIO2 or Nav2 — base only.

---

## Where this sits in the stack

```
L0  robot_drive.py      "software remote" (PROVEN)    — clamps + ramps + dead-man
L1  corridor_center.py  EYES-ONLY centering (GOLDEN)  — F1TENTH wall-follow, NO map  ← the LIVE node
L1' corridor_cruise.py  reactive, position-only       — SUPERSEDED (fishtails: no heading term)
L2  path_follow.py      taught-path follower + repel  — SUPERSEDED (map frame is stale → walls)
        corridor_control.py = shared sim geometry (used by the sims, not the live driver)
```

## L1 — `corridor_center.py` (the GOLDEN live driver)

Drives the corridor by the **eyes only** — no map in the steering loop. Per scan it reads
each side wall with two beams (abeam ±90°, forward ±45°) → perpendicular distance `D` and
wall angle `alpha` (F1TENTH wall-follow), then steers on BOTH lateral offset and heading:
`w = kp·(D_left−D_right) + kd·phi` (the `kd·phi` heading term is what stops the fishtailing).
Slow (`cruise=0.10`) so the 8 Hz LiDAR feedback keeps up; comfort band ⇒ dead straight, no
twitch. Corners are followed by the centering itself (the heading term tracks the bend); a
reactive turn-to-the-open-side is a backup for blind/blocked corners. Safety: crawl/steer-
away/stop as walls close. Per-cycle log → `/tmp/corridor_center.csv`.

```bash
/usr/bin/python3 corridor_center.py --dry-run            # prints e/phi/w, NO motion
/usr/bin/python3 corridor_center.py --cruise 0.10        # live (hand on e-stop)
```

Prereqs: CAN up, scout base up (`publish_odom:=false` is fine — odom is NOT used), RC OFF,
`/rslidar_points` live. The map is reserved for Phase 2 (ambiguous-junction routing only) —
see `../ROBOT_CONTROL_LEARNINGS.md` §9.

---

# L2 — `path_follow.py` (the live autonomy node)

Follows a **taught path** (direction) + a **LiDAR wall-repel comfort band** (centering).
This is what actually drives the corridor. Two simple layers:

1. **Direction from the path.** Localized by FAST-LIO (`/lio/odometry`), pure-pursuit aims
   at a lookahead point (`lookahead=0.8 m`) on the taught CSV. Handles the route + corners.
   The path's *heading* is trusted; its exact lateral aim is **not** (the live FAST-LIO
   frame is somewhat rotated vs the real corridor), so the path-gain is reduced:
   `w_path = 0.7 * alpha`.
2. **Centering from the LiDAR (repel-only).** A side wall within `safe` (0.65 m) pushes the
   robot *away* from it. Repel-only ⇒ **cavity-safe**: a lab-doorway cavity makes a wall
   *farther*, never closer, so it never steers into the cavity. Inside the comfort band
   (both walls beyond `safe`) it goes **dead straight at full speed** — no twitching.
   Speed slows **only while steering**, and to ≤0.08 m/s if a wall is inside `hard_min` (0.35 m).

Defaults: `cruise=0.18 m/s`, `max_w=0.45 rad/s`. A per-cycle log
(`/tmp/pf_movement.csv`: `t,x,y,yaw,d_left,d_right,v,w,w_path,w_repel,state`) records
*why* it turned (path vs repel) for tuning after a run.

> **The cavity problem (why direction must come from the path):** reactive
> wall-*following* (centering on `d_left - d_right`, as in L1 `corridor_cruise.py`) steers
> **into** a doorway cavity (the receding wall reads as "open"). So: path gives direction,
> walls give repel/safety only. That is why L1 is superseded for corridors.

> **The corridor problem (why FAST-LIO, not KISS):** LiDAR-only odometry can't measure
> forward translation in a long smooth corridor (two parallel walls look identical shifted
> along their length — aperture ambiguity). Proven in sim: 12.5 m driven, 3.7 m tracked.
> The D455 IMU (via FAST-LIO2) dead-reckons the forward motion the LiDAR can't see. Use
> `--odom-topic /lio/odometry`, not `/kiss/odometry`. Full story: `../ROBOT_CONTROL_LEARNINGS.md` §8.

`corridor_control.py` is the **shared** control geometry (`plan()`, pure Python) used by
the Gazebo + python sims AND conceptually by the robot — so what we validate in sim is what
runs live.

---

# Runbook — full real autonomy drive

Pre-flight: **RC transmitter OFF** (or in CAN/command mode — else the base ignores
`/cmd_vel` while CAN TX still climbs; status frame `0x211`). Hand on the e-stop.

```bash
# 1) CAN up FIRST (every reboot)
sudo ip link set can0 up type can bitrate 500000        # state UP, ERROR-ACTIVE

# 2) FAST-LIO data engine — /lio/odometry (~208 Hz) + /rslidar_points (10 Hz) + IMU (~198 Hz)
bash ../../02_fast_lio2/run/start_pipeline.sh           # leave running

# 3) Scout base, wheel-odom TF OFF (base_link gets ONE parent — FAST-LIO is the localizer)
ros2 launch scout_cmd scout_mini.launch.py publish_odom:=false

# 4) Place the robot at the ROUTE START (where teach_path_fastlio.csv was recorded).

# 5) DRY-RUN first (no motion):
/usr/bin/python3 path_follow.py --path ../results/teach_path_fastlio.csv \
    --odom-topic /lio/odometry --dry-run

# 6) LIVE (hand on e-stop):
/usr/bin/python3 path_follow.py --path ../results/teach_path_fastlio.csv \
    --odom-topic /lio/odometry
```

Run everything ROS with **`/usr/bin/python3`** (conda's 3.13 breaks rclpy). The full
hardware-gotcha list (D455 `device busy` → physical replug, LiDAR-rate vs CPU contention,
CAN order) lives in `../ROBOT_CONTROL_LEARNINGS.md` §8.6.
