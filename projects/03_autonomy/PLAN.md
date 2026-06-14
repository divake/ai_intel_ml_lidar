# Project 03 — Autonomous SLAM + Localization + Nav2 (teach-and-repeat)

> **Goal:** turn the validated perception rig (proj 01/02) into an **autonomous**
> robot — close the loop (globally-consistent map), localize in it, and let the
> Scout Mini **drive itself** slowly/precisely along the taught corridor route.
> This is **stage-setting infrastructure** for the real research
> ([[research-uncertainty-lidar-ml]]): uncertainty / heavy-ML on LiDAR point clouds.
>
> **Builds on:** proj 02 FAST-LIO2 (odometry + the `corridor2.0` bag) and the
> existing robot-control stack in `~/robotics_projects/` (Scout Mini via `/cmd_vel`).
> **Touches NONE of 01/02/scout_cmd** — this folder only *borrows* configs and
> *reuses* `start_robot_base.sh`. Created 2026-06-13.

## Decisions (locked with the user 2026-06-13)
- **Start = Phase A** (offline loop-closure SLAM, robot stationary — zero risk).
- **Speed profile (geometry-aware):** straights ≤ **0.2 m/s**; near path ends /
  edges **0.1 m/s**; turns **≤ 0.1 m/s** (smaller still on tight turns). This is
  exactly Regulated Pure Pursuit's behavior — bake it into the controller config.
- **Autonomy style:** **teach-and-repeat first** (follow the corridor2.0 route),
  then add free goal-navigation on the same map.
- **Folder:** `projects/03_autonomy/` (consistent with 01/02).

## Phases
| Phase | What | Risk |
|---|---|---|
| **A. Loop-closure SLAM** | `slam_toolbox` offline on `corridor2.0` bag (3D→2D scan via `pointcloud_to_laserscan`) → loop-closed 2D occupancy grid (the 1.52 m snaps shut). **No motion.** | none |
| **B. Localization** | `nav2_map_server` (serve the grid) + `nav2_amcl` (localize live scans) → drift-free pose. Validate **stationary / hand-pushed first**. | none |
| **C. Nav2 autonomy** | Nav2 (planner + Regulated Pure Pursuit + costmaps + `nav2_collision_monitor`) drives `/cmd_vel`. Teach-and-repeat the corridor route, then goals. | **motion — see safety** |
| **D. Self-healing map** | append each run's scans → batch re-optimize the map. (Pragmatic first; true online lifelong SLAM is later/research.) | none (offline) |

## Safety (Phase C — non-negotiable)
1. **Hard speed cap in config** — caps above; gentle accel limits.
2. **`nav2_collision_monitor`** — independent stop if obstacle too close.
3. **Dead-man** (Scout auto-stops when `/cmd_vel` stops) + explicit e-stop
   (`ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"` / Ctrl-C).
4. **Incremental test ladder:** offline SLAM (no motion) → AMCL localization
   (no motion) → **one 2 m slow segment, user holding e-stop** → full path.
   Never jump straight to the full autonomous loop.
5. Secure the D455 USB cable (proj-02 lesson: vibration unseats it).

## Control interface (from ROBOT_CONTROL.md)
`/cmd_vel` (geometry_msgs/Twist) → `scout_cmd_node` → CAN → Scout Mini → `/odom`.
Base bring-up: `~/robotics_projects/lidar_tools/start_robot_base.sh`. Scout max
**1.5 m/s / 2.0 rad/s** (we run far below). `/odom` ~50 Hz.

## Status
- [x] Phase A scaffolded — building loop-closed map now.
- [ ] Phase B, C, D.
