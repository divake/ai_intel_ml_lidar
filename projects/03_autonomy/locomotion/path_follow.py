#!/usr/bin/python3
"""L2 — follow the taught PATH (from the map) + a LiDAR wall-repel safety net.

Two layers, both simple:
  1. DIRECTION from the MAP: KISS-ICP LiDAR SLAM (/kiss/odometry) gives our pose;
     pure pursuit aims at a lookahead point on results/teach_path.csv. Handles the
     route + corners; ignores cavities (we track the path, not the walls).
  2. SAFETY from the LiDAR (independent of SLAM): if a side wall gets within `safe`,
     ease AWAY from it while still rolling forward ("drifting toward the right wall ->
     nudge left"). Repel-only => cavity-safe (a cavity makes a wall farther, never
     closer) and works even when SLAM wobbles at a corner.

Corners: ARC through them (slow down, keep rolling) — never spin in place, because an
in-place spin gives the LiDAR SLAM no translation to track at our low scan rate, so the
pose jumps and it thrashes. Always moving => SLAM stays locked, and it never freezes.

Start the robot at the route start (KISS-ICP zeroed there). Run with /usr/bin/python3.
Prereqs: L0 base up; KISS-ICP up (/kiss/odometry); LiDAR (/rslidar_points).
    /usr/bin/python3 path_follow.py --dry-run     # prints plan, NO motion
    /usr/bin/python3 path_follow.py               # live (hand on e-stop)
"""

import os
import math
import time
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from robot_drive import RobotDrive

HERE = os.path.dirname(os.path.realpath(__file__))
DEFAULT_PATH = os.path.normpath(os.path.join(HERE, "..", "results", "teach_path.csv"))
DEFAULT_ODOM = "/kiss/odometry"


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _clamp(x, lim):
    return max(-lim, min(lim, x))


def load_path(fn):
    pts = []
    with open(fn) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.replace(",", " ").split()
            pts.append((float(p[0]), float(p[1])))
    return np.array(pts)


class PathFollow(Node):
    def __init__(self, drive, path_xy, *, odom_topic=DEFAULT_ODOM,
                 cruise=0.18, max_w=0.45, lookahead=0.8,
                 safe=0.8, k_repel=2.0, hard_min=0.35,
                 turn_thresh=0.8, k_turn=0.9, chord_wp=4, turn_lookahead_wp=8,
                 zmin=-0.20, zmax=1.0, range_min=0.4, range_max=30.0,
                 dry_run=False, max_time=120.0):
        super().__init__("path_follow")
        self.drive = drive
        self.path = path_xy
        self.cruise = cruise
        self.max_w = max_w
        self.Ld = lookahead
        self.safe = safe                  # repel when a side wall is within this (m)
        self.k_repel = k_repel            # repel gain (rad/s per m of intrusion)
        self.hard_min = hard_min          # a wall this close -> also slow down (m)
        # turn detection from the path's OWN curvature (frame-independent), see _control
        self.turn_thresh = turn_thresh    # path heading-change (rad) that counts as a corner
        self.k_turn = k_turn              # arc gain through a corner (rad/s per rad of bend)
        self.chord_wp = chord_wp          # waypoints per heading chord (~1.6m, noise-robust)
        self.turn_lookahead_wp = turn_lookahead_wp  # how far ahead to sense the bend (~3.2m)
        self.zmin, self.zmax = zmin, zmax
        self.range_min, self.range_max = range_min, range_max
        self.dry_run = dry_run
        self.max_time = max_time
        self.t0 = time.monotonic()
        self._last_log = 0.0
        self._off = None
        self.idx = 0
        self.pose = None
        self.d_left = None
        self.d_right = None
        self.reached = False
        # movement log (line-buffered) — per-cycle left/right vs straight, for tuning
        self.logf = open("/tmp/pf_movement.csv", "w", buffering=1)
        self.logf.write("t,x,y,yaw_deg,d_left,d_right,v,w,w_turn,w_repel,state\n")

        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(PointCloud2, "/rslidar_points", self._on_cloud, qos)
        self.timer = self.create_timer(0.1, self._control)
        mode = "DRY-RUN (no motion)" if dry_run else "LIVE"
        self.get_logger().info(
            f"path_follow up [{mode}] — {len(path_xy)} waypoints, waiting for {odom_topic} ...")

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, _yaw_from_quat(msg.pose.pose.orientation))

    # nearest wall on each side (raw LiDAR, SLAM-independent) — for the repel safety net
    def _on_cloud(self, msg):
        if self._off is None:
            self._off = {f.name: f.offset for f in msg.fields}
        ox, oy, oz = self._off["x"], self._off["y"], self._off["z"]
        dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4", "<f4", "<f4"],
                       "offsets": [ox, oy, oz], "itemsize": msg.point_step})
        pc = np.frombuffer(msg.data, dtype=dt)
        x, y, z = pc["x"], pc["y"], pc["z"]
        rng = np.hypot(x, y)
        m = (np.isfinite(rng) & (z > self.zmin) & (z < self.zmax)
             & (rng > self.range_min) & (rng < self.range_max))
        x, y, rng = x[m], y[m], rng[m]
        ang = np.arctan2(y, x)
        # right side = front-right..back-right (-120..-60 deg); left = +60..+120 deg.
        # MIN = closest point on that side (most protective). Wide enough to catch a
        # wall the robot is turning toward, not just dead-abeam.
        rsel = (ang >= -2.10) & (ang <= -1.05)
        lsel = (ang >= 1.05) & (ang <= 2.10)
        self.d_right = float(rng[rsel].min()) if rsel.any() else None
        self.d_left = float(rng[lsel].min()) if lsel.any() else None

    def _control(self):
        if self.pose is None:
            return
        now = time.monotonic()
        rx, ry, ryaw = self.pose

        # advance progress index (monotonic forward window)
        seg = self.path[self.idx:self.idx + 80]
        d2 = (seg[:, 0] - rx) ** 2 + (seg[:, 1] - ry) ** 2
        self.idx += int(np.argmin(d2))

        # goal by progression (loop route: end ~ start)
        if self.idx >= len(self.path) - 2:
            if not self.reached:
                self.get_logger().info("REACHED end of path — stopping.")
                self.reached = True
            if not self.dry_run:
                self.drive.stop()
            return

        # --- CENTERING (lateral): ALWAYS from the REAL walls (LiDAR, frame-independent) ---
        # close RIGHT -> steer left (+); close LEFT -> steer right (-). Both walls balanced
        # => centered. Inside the comfort band (both beyond `safe`) => zero => dead straight.
        w_repel = 0.0
        if self.d_right is not None and self.d_right < self.safe:
            w_repel += self.k_repel * (self.safe - self.d_right)
        if self.d_left is not None and self.d_left < self.safe:
            w_repel -= self.k_repel * (self.safe - self.d_left)

        # --- ROUTE (turns): detect a REAL corner from the PATH's OWN curvature. This is a
        # relative heading change *within* the path, so it is immune to our lateral offset and
        # frame rotation. (Bearing-to-a-path-point is NOT usable: a lateral offset on a
        # straight makes that bearing ~90deg and fakes a turn -> the old into-the-wall bug.)
        # Compare corridor heading here vs ~turn_lookahead_wp ahead; bend past turn_thresh =>
        # a corner -> arc that way. Calibrated: straights peak ~33deg, corners 54-91deg. ---
        def _chord_h(i):
            a = self.path[max(0, min(i, len(self.path) - 1))]
            b = self.path[max(0, min(i + self.chord_wp, len(self.path) - 1))]
            return math.atan2(b[1] - a[1], b[0] - a[0])
        dh = _chord_h(self.idx + self.turn_lookahead_wp) - _chord_h(self.idx)
        dh = math.atan2(math.sin(dh), math.cos(dh))        # wrap to [-pi, pi]

        if abs(dh) > self.turn_thresh:
            w_turn = _clamp(self.k_turn * dh, self.max_w)   # arc in the path's bend direction
            w = _clamp(w_turn + 0.5 * w_repel, self.max_w)  # walls stay as a safety net
            v = self.cruise * 0.5
            state = "TURN"
        else:
            w_turn = 0.0                                    # straight: ignore the path entirely
            w = _clamp(w_repel, self.max_w)                 # steer PURELY by the walls (center)
            v = self.cruise if abs(w) < 0.05 else self.cruise * 0.7
            state = "STRAIGHT" if abs(w) < 0.05 else "CENTER"

        nearest = min([d for d in (self.d_left, self.d_right) if d is not None], default=99.0)
        if nearest < self.hard_min:
            v = min(v, 0.08)
        v = max(v, 0.08)

        if not self.dry_run:
            self.drive.set(v, w)

        # per-cycle movement log (left/right vs straight) for analysis + tuning
        dlv = self.d_left if self.d_left is not None else -1.0
        drv = self.d_right if self.d_right is not None else -1.0
        self.logf.write(f"{now - self.t0:.2f},{rx:.3f},{ry:.3f},{math.degrees(ryaw):.1f},"
                        f"{dlv:.3f},{drv:.3f},{v:.3f},{w:.3f},{w_turn:.3f},{w_repel:.3f},{state}\n")

        if now - self._last_log > 0.4:
            self._last_log = now
            dl = f"{self.d_left:.2f}" if self.d_left is not None else " -- "
            dr = f"{self.d_right:.2f}" if self.d_right is not None else " -- "
            tag = "DRY" if self.dry_run else "RUN"
            self.get_logger().info(
                f"[{tag}:{state:8s}] wp {self.idx}/{len(self.path)} pos=({rx:+.2f},{ry:+.2f}) "
                f"yaw={math.degrees(ryaw):+5.0f} dh={math.degrees(dh):+5.0f} "
                f"L={dl} R={dr} -> v={v:+.2f} w={w:+.2f}")

        if not self.dry_run and (now - self.t0) > self.max_time:
            self.get_logger().warn(f"max_time {self.max_time}s reached — stopping.")
            self.drive.hard_stop()
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description="L2 path follower + LiDAR wall-repel safety")
    ap.add_argument("--path", default=DEFAULT_PATH)
    ap.add_argument("--odom-topic", default=DEFAULT_ODOM)
    ap.add_argument("--cruise", type=float, default=0.18)
    ap.add_argument("--lookahead", type=float, default=0.8)
    ap.add_argument("--safe", type=float, default=0.65)
    ap.add_argument("--max-time", type=float, default=120.0)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    path_xy = load_path(args.path)
    print(f"loaded {len(path_xy)} waypoints from {args.path}  start={path_xy[0]} end={path_xy[-1]}")

    rclpy.init()
    drive = RobotDrive(cmd_timeout=0.8)
    follow = PathFollow(drive, path_xy, odom_topic=args.odom_topic, cruise=args.cruise,
                        lookahead=args.lookahead, safe=args.safe,
                        dry_run=args.dry_run, max_time=args.max_time)

    if not args.dry_run:
        print("\n>>> LIVE path follow + wall-repel — hand on the e-stop.")
        for i in range(3, 0, -1):
            print(f"    starting in {i} ...", end="\r", flush=True)
            time.sleep(1)
        print(" " * 40, end="\r")

    exe = MultiThreadedExecutor(num_threads=4)
    exe.add_node(drive)
    exe.add_node(follow)
    try:
        exe.spin()
    except KeyboardInterrupt:
        print("\n!!! aborted — hard stop")
    finally:
        drive.hard_stop()
        follow.destroy_node()
        drive.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
