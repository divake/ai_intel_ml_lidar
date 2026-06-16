# Path-following — the three steps, built + verified in sim (2026-06-15)

Turns "the robot knows where it is" (validated localization, `LOCALIZATION.md`) into "the robot
drives the taught route." Standard proven stack: **localized pose + global path + Regulated Pure
Pursuit + costmap.** Each step verified in code/sim BEFORE live (PNGs in `results/`).

## Step 1 — taught path → `nav_msgs/Path`  ✅ verified
- `run/path_publisher.py`: loads `results/teach_path_fastlio.csv` (x y yaw, map frame) → publishes a
  latched `/plan` (`nav_msgs/Path`) in the `map` frame for Nav2 to follow.
- **Verify:** `results/step1_path_over_map.png` — the 251.6 m / 628-pose route sits in the corridors.

## Step 2 — TF frame tree  ✅ verified (infrastructure already existed)
- Chain: **`map →[icp_relocalization]→ odom_lio →[FAST-LIO]→ rslidar →[static z=-0.30]→ base_link
  →[URDF]→ {footprint, wheels}`**. One parent per frame (the URDF work resolved the rslidar/base_link
  conflict by making base_link a CHILD of rslidar). Nav2 uses `map → base_link`.
- The only NEW edge is `map → odom_lio` = our validated localization (A). The rest is the existing
  `run/autonomous_drive.launch.py` (URDF robot_state_publisher + the static mount).
- **Verify:** `results/step2_frame_tree.png`.

## Step 3 — Regulated Pure Pursuit follows the route  ✅ verified in sim
- `sim/pp_sim.py`: the Nav2 RPP algorithm (lookahead + curvature + speed regulation) on the taught
  route over the real map, with **progress-constrained** nearest-point search (so the figure-8's
  self-overlap / start≈end can't jump to the wrong pass) and localization noise modelling our 2.8 cm.
- **Result:** reaches the goal around the **full figure-8 loop**, cross-track **mean 0.12 m / max
  0.39 m** (max at sharp corners). Wall-grid "clips" equal the taught path's own clip rate → they're
  conservative-grid thickness, not real collisions (RC drive was collision-free).
- **Verify:** `results/step3_rpp_follow.png` + `results/step3_rpp_follow.gif` (watch it drive).
- The LIVE controller is the actual **Nav2 RPP node** (`config/nav2_live.yaml`, already present) — the
  sim validates the control geometry; live uses the proven node.

## Live deployment (the next, robot-in-the-loop step — NOT done here)
Bring up, in order, then a supervised drive (hand on e-stop):
1. CAN + Scout base (`publish_odom:=false`), FAST-LIO pipeline.
2. Localization A: spark-fast-lio + `icp_relocalization` (`run/icp_loc.launch.py` style) → `map→odom_lio`.
3. Frames: `run/autonomous_drive.launch.py` (URDF + static mount).
4. `run/path_publisher.py` → `/plan`.
5. Nav2 RPP (`config/nav2_live.yaml`) consuming `/plan` + the localized pose + a live-LiDAR costmap → `/cmd_vel`.
Honest open items for live: confirm `nav2_live.yaml` frame_ids match (`map`,`odom_lio`,`base_link`);
costmap from `/rslidar_points` (scan band); start the robot at the map origin (initial-pose lock).
