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
                 safe=0.5, k_repel=1.6, hard_min=0.35,
                 zmin=-0.20, zmax=1.0, range_min=0.5, range_max=30.0,
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

        # lookahead point ~Ld ahead -> heading error alpha in robot frame
        j = self.idx
        while j < len(self.path) - 1 and math.hypot(self.path[j, 0] - rx, self.path[j, 1] - ry) < self.Ld:
            j += 1
        dx, dy = self.path[j, 0] - rx, self.path[j, 1] - ry
        tx = math.cos(ryaw) * dx + math.sin(ryaw) * dy
        ty = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
        alpha = math.atan2(ty, tx)

        # --- direction: pure pursuit toward the path ---
        w_path = 1.1 * alpha

        # --- safety: repel from a CLOSE side wall (raw LiDAR, SLAM-independent) ---
        w_repel = 0.0
        if self.d_right is not None and self.d_right < self.safe:
            w_repel += self.k_repel * (self.safe - self.d_right)    # close RIGHT -> steer left
        if self.d_left is not None and self.d_left < self.safe:
            w_repel -= self.k_repel * (self.safe - self.d_left)     # close LEFT  -> steer right

        w = _clamp(w_path + w_repel, self.max_w)

        # --- speed: ARC through turns (slow, never stop); extra slow very near a wall ---
        v = self.cruise * (1.0 - 0.6 * min(1.0, abs(alpha) / 0.9))
        nearest = min([d for d in (self.d_left, self.d_right) if d is not None], default=99.0)
        if nearest < self.hard_min:
            v = min(v, 0.08)
        v = max(v, 0.06)                 # never fully stop -> SLAM keeps tracking

        if not self.dry_run:
            self.drive.set(v, w)

        if now - self._last_log > 0.4:
            self._last_log = now
            dl = f"{self.d_left:.2f}" if self.d_left is not None else " -- "
            dr = f"{self.d_right:.2f}" if self.d_right is not None else " -- "
            tag = "DRY" if self.dry_run else "RUN"
            self.get_logger().info(
                f"[{tag}] wp {self.idx}/{len(self.path)} pos=({rx:+.2f},{ry:+.2f}) "
                f"yaw={math.degrees(ryaw):+5.0f} a={math.degrees(alpha):+5.0f} "
                f"L={dl} R={dr} -> v={v:+.2f} w={w:+.2f} (path{w_path:+.2f} repel{w_repel:+.2f})")

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
    ap.add_argument("--safe", type=float, default=0.5)
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
