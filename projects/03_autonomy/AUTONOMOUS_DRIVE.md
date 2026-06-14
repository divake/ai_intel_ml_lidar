# Phase C — Live Autonomous Drive (teach-and-repeat) — at-robot playbook

> **Do this WITH the robot, a clear corridor, and a hand on the e-stop.** Speeds are
> capped slow (0.20 m/s straight, 0.10 at turns/goals) in `config/nav2_params.yaml`,
> and `collision_monitor` hard-stops within 0.5 m. We still climb the ladder below —
> never jump to the full loop.

## Localization choice (decided)
FAST-LIO2 is the 3D pose source (it never loses track, unlike 2D AMCL in the
corridor). The taught path (`results/teach_path_fastlio.csv`, 628 wp / 291 m) is in
FAST-LIO2's `odom_lio` frame, so path + live pose are consistent. **Start the robot
at the same physical spot the recording started** (so `odom_lio` origin ≈ path start).

## Frame tree to stand up live (verify with `ros2 run tf2_tools view_frames`)
```
map --(static identity)--> odom_lio --(FAST-LIO2)--> rslidar --(static mount)--> base_link
```
- `map -> odom_lio`: `ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom_lio`
- `rslidar -> base_link`: static, from the LiDAR mount (LiDAR sits above robot center).
  Measure it; rough first guess `0 0 -0.30 0 0 0 rslidar base_link`.
- ⚠️ The **Scout driver also publishes `odom -> base_link`** from wheels — that will
  FIGHT FAST-LIO2's TF. Disable the Scout's odom TF (or remap), so FAST-LIO2 is the
  only pose source. **Verify there is exactly one parent of `base_link`.**

## The safety ladder (do every rung; abort up a rung on anything weird)
1. **Base + perception, NO autonomy.** `start_robot_base.sh` (CAN+Scout) + project-02
   `start_pipeline.sh` (LiDAR+IMU+FAST-LIO2). Confirm `/cmd_vel` sub, `/lio/odometry`
   live, `/rslidar_points` 10 Hz. Drive 1 m by **keyboard** — sanity that the base obeys.
2. **Localization only (no motion).** Add the static TFs + Nav2 (map_server optional,
   controller_server + costmaps + collision_monitor + velocity_smoother) via
   `autonomous_drive.launch.py`. In Foxglove: confirm the robot pose sits on the map,
   `/scan` + local costmap show the real walls. **Robot still parked.**
3. **One 2 m segment, hand on e-stop.** Make a 2-m stub path; run
   `follow_taught_path.py --path <stub>`. Watch it creep forward at 0.1–0.2 m/s and
   stop. Confirm `collision_monitor` stops it if you step in front.
4. **Full taught loop.** `follow_taught_path.py` with the real path. Walk alongside.
   It follows the route, avoids obstacles via the live costmap, returns near start.

## E-stop (any of these)
- **Ctrl-C** the `follow_taught_path.py` terminal (dead-man: Scout stops when `/cmd_vel`
  stops).
- `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`
- The robot's physical e-stop button.

## Capturing the autonomous run (the self-healing-map idea)
Record during the drive (`record_run.sh autonomous1` pattern): `/rslidar_points`
`/camera/camera/imu` `/lio/odometry` `/tf`. Each autonomous run is more data → re-run
KISS-SLAM on it → refine/extend the map (lifelong mapping). That's a legit research
thread ([[research-uncertainty-lidar-ml]]).

## Status
- [x] Mission executor (`run/follow_taught_path.py`), Nav2 config, teach path — built.
- [ ] `autonomous_drive.launch.py` — assembled live (needs the Scout's real frames).
- [ ] The drive.
