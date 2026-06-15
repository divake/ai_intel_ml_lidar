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
L0  robot_drive  ← YOU ARE HERE (proven)   "the software remote"
L1  reactive safety / recovery             "never freeze — always act" (next)
L2  navigation: center + follow path + turns
L3  localization (FAST-LIO2 + taught path)
```
