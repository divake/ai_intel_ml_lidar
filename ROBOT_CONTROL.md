# Scout Mini — Robot Control Quickstart

Short reference for driving the **AgileX Scout Mini Pro** from this NUC by keyboard.
Verified working 2026-06-13. For a human OR an AI agent — read this first, then follow the
"Go deeper" pointers below if you need more.

> **Why this file exists:** if the robot "doesn't move" when you run teleop, 99% of the time
> the **CAN bus is DOWN** or the **base driver isn't running**. Fix those two things (below) and
> it works.

---

## TL;DR — bring it up in 2 commands

```bash
# 1) Start the base: brings CAN up (can0 @ 500k) + launches the Scout driver
~/robotics_projects/lidar_tools/start_robot_base.sh

# 2) In a SECOND terminal, drive it
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Keep the teleop terminal **focused** — it only reads keys while its window is active.
Teleop keys: `i`/`,` fwd/back · `j`/`l` rotate · `k` stop · `u o m .` arcs · `q`/`z` change speed.
The robot auto-stops when keys stop publishing (dead-man behavior — that's normal, not a fault).

---

## Manual bring-up (what the script does, step by step)

Use this when debugging or if you don't want the script.

```bash
# A. Enable the CAN interface (resets to DOWN on every reboot / cable unplug)
sudo ip link set can0 up type can bitrate 500000
ip link show can0                       # expect: state UP ... <NOARP,UP,LOWER_UP,ECHO>

# B. Confirm the robot is powered & wired (should print CAN frames in ~1s)
candump -n 10 can0                      # IDs like 221/241/251-254/311 = Scout status feedback
                                        # (silence = robot off, e-stop pressed, or cable unplugged)

# C. Launch the driver
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch scout_cmd scout_mini.launch.py
# expect: "Connected to Scout Mini on can0" / "Scout command node ready!"
```

### Verify the ROS graph is healthy
```bash
ros2 node list                  # must include  /scout_cmd_node
ros2 topic list                 # must include  /cmd_vel  /odom  /tf
ros2 topic info /cmd_vel        # Subscription count must be >= 1  (the driver listening)
```

### One-off motion test (no keyboard) — moves ~12 cm forward then stops
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}"
sleep 1
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"      # STOP
ros2 topic echo --once /odom --field pose.pose.position          # x should have increased
```

---

## The control chain (mental model)

```
keyboard / your code → /cmd_vel (geometry_msgs/Twist) → scout_cmd_node → ugv_sdk → CAN (can0) → Scout Mini → /odom
```
- `scout_cmd` is a **custom stable wrapper** that replaces the unstable official `scout_ros2`
  (no segfaults). It talks straight to `ugv_sdk` over CAN.
- Speed limits: **1.5 m/s** linear, **2.0 rad/s** angular. Stay at **0.3 m/s / 0.5 rad/s** for testing.
- Odometry publishes at ~50 Hz on `/odom`.

---

## Other ways to drive (besides teleop_twist_keyboard)

```bash
# Command line — single Twist
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"      # forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"     # rotate CCW
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"                      # STOP

# Custom interactive w/a/s/d loop (publishes to /cmd_vel)
python3 ~/robotics_projects/robot_tests/test_scout_movements.py --interactive

# Full automated movement test suite (forward/back/rotate/arc/speed ramps)
python3 ~/robotics_projects/robot_tests/test_scout_movements.py
```

---

## When it "stops working" — quick triage

| Symptom | Cause | Fix |
|---|---|---|
| Teleop runs, robot dead | base driver not running | start the base (TL;DR step 1) |
| `/cmd_vel` Subscription count = 0 | `scout_cmd_node` not up | relaunch driver |
| `can0` state DOWN | reboot / unplug reset it | `sudo ip link set can0 up type can bitrate 500000` |
| `candump` shows nothing | robot off / e-stop / cable | power on, release e-stop, check USB-CAN cable |
| Driver already running | stale node | `pkill -f "scout_cmd scout_mini.launch.py"` then relaunch |
| Emergency stop | — | `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"` or Ctrl+C the teleop |

---

## Go deeper — where to read more (for the next AI / for me)

All robot-control code lives in **`~/robotics_projects/`** (this folder). Key files:

- **Full movement guide (long):** `~/robotics_projects/lidar_tools/ROBOT_MOVEMENT_GUIDE.md`
- **One-shot base bring-up script:** `~/robotics_projects/lidar_tools/start_robot_base.sh`
- **Start everything (base + lidar + camera + rviz):** `~/robotics_projects/lidar_tools/start_all.sh`
- **Movement test / interactive control:** `~/robotics_projects/robot_tests/test_scout_movements.py`
- **Quick CLI test:** `~/robotics_projects/robot_tests/quick_test.sh`
- **Driver SOURCE (the custom wrapper):** `~/ros2_ws/src/scout_cmd/`
  (launch file: `~/ros2_ws/src/scout_cmd/launch/scout_mini.launch.py`)
- **Driver BUILT/installed:** `~/ros2_ws/install/scout_cmd/`
- **Alt/older driver tree:** `~/scout_mini_ros2_jazzy/`  ·  official pkgs: `~/robotics_projects/scout_ros2/`, `~/robotics_projects/ugv_sdk/`
- **Configs backup repo:** https://github.com/divake/robot-configs

### Related projects on this NUC (not robot-driving, but nearby)
- **LiDAR mapping (KISS-ICP / FAST-LIO2):** `~/divek_nus/ml_lidar/` — see its `CLAUDE.md`.
  Note: that project is perception only; it has **no `/cmd_vel`** — driving lives here.
- **Vision / detection demos:** `~/divek_nus/conformal_od/`, `~/divek_nus/iros_demo/`,
  `~/robot_applications/mobile_object_detection/`

### Environment notes (important)
- Use **`/usr/bin/python3`** for anything ROS (rclpy). The conda `python3` is 3.13 and breaks rclpy.
- Always `source /opt/ros/jazzy/setup.bash` (and `~/ros2_ws/install/setup.bash` for `scout_cmd`).
- Stack: Ubuntu 24.04 · ROS 2 Jazzy · Scout Mini Pro over USB-CAN (`can0` @ 500000).
