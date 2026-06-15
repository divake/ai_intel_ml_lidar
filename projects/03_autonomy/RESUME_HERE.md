# RESUME HERE — proj03 autonomy (last updated 2026-06-15, late night)

Open this first when you come back. It says where we are, what's golden, and what to do next.

---

## TL;DR — autonomy is FIXED (the safe, no-surgery way)

The robot now drives the corridor **by its eyes (LiDAR), not by the map**, slow and safe.
On a 25-min supervised run it drove the straight + multiple corners and **never came within
0.47 m of a wall — no fishtailing, no banging.** This is the foundation; build on it.

**The one driver that works:** `locomotion/corridor_center.py` — **GOLDEN, do not modify.**

---

## What's proven ✅ vs what's the limit ⏳

| Situation | Decider | Status |
|---|---|---|
| Straight corridor | eyes — centering | ✅ smooth |
| One opening (single L/R turn, cavity-corner) | eyes — follow the bend / turn-to-open | ✅ reactive |
| **Ambiguous junction** (two ways open) | needs **map + A\*** | ⏳ Phase 2 — today picks "furthest-open side" |
| **Complex multi-cavity section** | needs **route memory** | ⏳ Phase 2 — today ping-pongs (memoryless) |

Why the limit: the controller has **no memory and no map**, so at a junction it can't choose
the taught route, and in a cavity maze it U-turns toward the most-open side (back where it
came) and loops. It stays safe the whole time — just wanders.

---

## How it works (one paragraph)

Per LiDAR scan, two beams per side (abeam ±90°, forward ±45°) → perpendicular distance `D`
and wall angle `alpha` (F1TENTH wall-follow). Steer on BOTH offset and heading:
`w = kp·(D_left−D_right) + kd·phi` — the `kd·phi` heading term is what kills the fishtail.
Slow (0.10 m/s) so 8 Hz LiDAR feedback keeps up; comfort band ⇒ dead straight (no twitch);
crawl/steer-away/stop as walls close. The map is NOT in the loop. Full story:
`ROBOT_CONTROL_LEARNINGS.md` §9.

---

## TOMORROW — Phase 2 (start here, in order)

1. **Route memory (tiny, ~1 hr, no map):** in `corridor_center.py`'s corner logic, forbid
   turning back into the corridor just exited ⇒ breaks the ping-pong. *(This is the one
   allowed touch — or better, add it as a thin wrapper so the golden core stays untouched.)*
2. **Map + A\* junction router (the real one):** at each junction, a planned route over the
   2D map says "go left here" and overrides the reactive furthest-open guess ⇒ follows the
   taught lab route deterministically. Centering stays 100% eyes. This is the ONLY job the map has.
3. **Data-collection run** (for research): `ros2 bag record /rslidar_points
   /camera/camera/imu /lio/odometry /tf /tf_static /cmd_vel` + set FAST-LIO `pcd_save_en:
   true`. One drive ⇒ point clouds + IMU + trajectory + a saved map, all aligned.
4. **Research (the north star):** uncertainty quantification in LiDAR/point-cloud + ML on
   the collected data. See memory [[research-uncertainty-lidar-ml]].

Optional tunable (don't change the centering math): faster on clear straights, auto-slow at
decision points.

---

## How to run the golden driver

```bash
# CAN up (every reboot), base up, RC OFF, hand on e-stop
sudo ip link set can0 up type can bitrate 500000
ros2 launch scout_cmd scout_mini.launch.py publish_odom:=false   # odom NOT used
# LiDAR must be live (/rslidar_points). FAST-LIO NOT required (eyes-only).
cd projects/03_autonomy/locomotion
/usr/bin/python3 corridor_center.py --dry-run         # prints e/phi/w, no motion
/usr/bin/python3 corridor_center.py --cruise 0.10     # live
```
Run all ROS with `/usr/bin/python3`. Per-cycle log → `/tmp/corridor_center.csv`; saved runs
→ `results/corridor_runs/`.

---

## Housekeeping / state at wrap-up

- **Git:** golden controller + docs committed **locally** as `f472f02`. **PUSH IS PENDING** —
  the gh token expired. Run `! gh auth login`, then push (origin: divake/ai_intel_ml_lidar).
- **Data saved:** `results/corridor_runs/run_2026-06-15_03.58.40.csv` (900 s),
  `run_2026-06-15_04.10.21.csv` (617 s).
- **Robot:** stopped (controller killed, `/cmd_vel` zeroed) before stepping away.
- Superseded (don't resurrect): `path_follow.py` (map-frame steering → walls),
  `corridor_cruise.py` (position-only → fishtail). Map-based Nav2/RPP/MPPI path abandoned.
