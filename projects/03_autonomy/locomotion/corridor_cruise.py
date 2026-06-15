#!/usr/bin/python3
"""L1 — corridor cruise: reactive, never-freeze driving on top of the L0 box.

The brain the user described, in plain geometry (no Nav2, no black box):
  * GO FORWARD at a cruise speed.
  * STAY CENTERED, human-like: go straight and tolerate being off-center; only EASE
    AWAY from a wall when it gets uncomfortably close ("45/50 is fine; near a wall, nudge").
  * OBSTACLE/PERSON AHEAD: far -> full speed; close -> slow; too close -> STOP AND WAIT
    (it knows WHY), resume when clear.
  * STUCK (boxed in): REVERSE is a first-class move — back out (rear checked via the
    360° LiDAR) and turn toward the more open side, then resume. Never a dead freeze.

INPUT — why we read the raw cloud, not /scan:
  The LiDAR is healthy at ~10 Hz, but /rslidar_points is a big (~750 KB) RELIABLE-QoS
  message. A best-effort subscriber (the default /scan path) loses UDP fragments and
  only ~1.6 Hz survive. Subscribing DIRECTLY to /rslidar_points with RELIABLE QoS gets
  the frames intact (~6-10 Hz) and makes this node depend only on the LiDAR driver — no
  pointcloud_to_laserscan, no TF, no map. We compute front/left/right/rear distances
  from the cloud ourselves (numpy), in the LiDAR frame.

Frame (RoboSense): +x forward (angle 0), +y left (+pi/2), -y right (-pi/2), back (±pi).

Run with /usr/bin/python3. Prereq: base up (L0) + LiDAR driver publishing /rslidar_points.
ALWAYS dry-run first:
    /usr/bin/python3 corridor_cruise.py --dry-run        # no motion; verify what it sees
    /usr/bin/python3 corridor_cruise.py                  # live drive (hand on e-stop)
"""

import math
import time
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

from robot_drive import RobotDrive


def _median(a):
    return float(np.median(a)) if a.size else None


def _clamp(x, lim):
    return max(-lim, min(lim, x))


class CorridorCruise(Node):
    def __init__(self, drive, *,
                 cruise=0.18, max_w=0.5, kp=1.5, safe=0.6,
                 slow_dist=2.0, stop_dist=0.8,
                 side_half=0.26, y_path=0.45,
                 max_wall=3.0, hard_min=0.45,
                 zmin=-0.20, zmax=1.0, range_min=0.5, range_max=30.0,
                 stuck_time=3.0, recover_time=2.5, back_safe=0.8, back_speed=0.10,
                 dry_run=False, max_time=90.0):
        super().__init__("corridor_cruise")
        self.drive = drive
        self.cruise = cruise
        self.max_w = max_w
        self.kp = kp                      # wall-avoidance gain (rad/s per m of intrusion)
        self.safe = safe                  # comfort margin: ease away only inside this (m)
        self.slow_dist = slow_dist        # full speed beyond this (m)
        self.stop_dist = stop_dist        # v ramps to 0 at this front distance (m)
        self.side_half = side_half        # side sector half-angle (rad)
        self.y_path = y_path              # half-width of the path-ahead box (m); excludes side walls
        self.max_wall = max_wall          # beyond this = "no wall that side" (m)
        self.hard_min = hard_min          # a wall this close => ease off, steer away (m)
        self.zmin, self.zmax = zmin, zmax            # height band (rslidar frame): excl floor/ceiling
        self.range_min, self.range_max = range_min, range_max
        # recovery (reverse is a first-class move — back out + turn when boxed in)
        self.stuck_time = stuck_time
        self.recover_time = recover_time
        self.back_safe = back_safe
        self.back_speed = back_speed
        self.state = "RUN"
        self.wait_since = None
        self.recover_until = 0.0
        self.dry_run = dry_run
        self.max_time = max_time
        self.t0 = time.monotonic()
        self._last_log = 0.0
        self._off = None                  # cached x/y/z byte offsets
        # RELIABLE to match the publisher: large clouds get retransmitted, not dropped.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.sub = self.create_subscription(PointCloud2, "/rslidar_points", self._on_cloud, qos)
        mode = "DRY-RUN (no motion)" if dry_run else "LIVE"
        self.get_logger().info(f"corridor_cruise up [{mode}] — waiting for /rslidar_points ...")

    # ---- cloud -> x, y, range for points in the wall height band ----
    def _cloud_xy(self, msg):
        if self._off is None:
            self._off = {f.name: f.offset for f in msg.fields}
        ox, oy, oz = self._off["x"], self._off["y"], self._off["z"]
        dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4", "<f4", "<f4"],
                       "offsets": [ox, oy, oz], "itemsize": msg.point_step})
        p = np.frombuffer(msg.data, dtype=dt)
        x, y, z = p["x"], p["y"], p["z"]
        rng = np.hypot(x, y)
        m = (np.isfinite(rng) & (z > self.zmin) & (z < self.zmax)
             & (rng > self.range_min) & (rng < self.range_max))
        return x[m], y[m], rng[m]

    def _on_cloud(self, msg):
        now = time.monotonic()
        x, y, rng = self._cloud_xy(msg)
        ang = np.arctan2(y, x)
        # FRONT/REAR clearance: a RECTANGLE the width of the robot's path (|y|<y_path),
        # NOT an angular cone — a cone clips the side walls of a narrow corridor (~1.9 m)
        # and falsely reads "blocked ahead". x = distance straight ahead.
        infront = (x > 0.0) & (np.abs(y) < self.y_path)
        behind = (x < 0.0) & (np.abs(y) < self.y_path)
        d_front = float(x[infront].min()) if infront.any() else self.range_max
        d_back = float((-x[behind]).min()) if behind.any() else self.range_max
        # SIDES: angular sectors at ±90° give the wall distances for centering
        hs = self.side_half
        l = rng[(ang >= math.pi / 2 - hs) & (ang <= math.pi / 2 + hs)]
        r = rng[(ang >= -math.pi / 2 - hs) & (ang <= -math.pi / 2 + hs)]
        d_left = _median(l)
        d_right = _median(r)
        open_side = 1.0 if (d_left or 0.0) >= (d_right or 0.0) else -1.0  # +1 left, -1 right

        # ---------- RECOVER: timed maneuver — back out, turn toward the open side ----------
        if self.state == "RECOVER":
            if now < self.recover_until:
                if d_back > self.back_safe:
                    v, w, state = -self.back_speed, 0.4 * open_side, "RECOVER:back+turn"
                else:
                    v, w, state = 0.0, 0.5 * open_side, "RECOVER:pivot"   # boxed behind -> turn
                return self._commit(v, w, state, d_left, d_right, d_front, now)
            self.state, self.wait_since = "RUN", None

        # ---------- speed from forward clearance ----------
        if d_front >= self.slow_dist:
            v, vstate = self.cruise, "CRUISE"
        elif d_front > self.stop_dist:
            frac = (d_front - self.stop_dist) / (self.slow_dist - self.stop_dist)
            v, vstate = self.cruise * frac, "SLOW"
        else:
            v, vstate = 0.0, "WAIT"

        # ---------- steering: HUMAN-LIKE wall avoidance, not center-seeking ----------
        push = 0.0
        if d_left is not None and d_left < self.safe:
            push -= self.kp * (self.safe - d_left)    # too close LEFT  -> steer right (w<0)
        if d_right is not None and d_right < self.safe:
            push += self.kp * (self.safe - d_right)   # too close RIGHT -> steer left  (w>0)
        w = _clamp(push, self.max_w)

        # anti-pin: a side wall very close => ease speed so it steers out gently
        nearest_side = min([d for d in (d_left, d_right) if d is not None], default=99.0)
        if nearest_side < self.hard_min and v > 0.08:
            v = 0.08

        # ---------- stuck detection -> trigger RECOVER (reverse is a real option) ----------
        if vstate == "WAIT":
            w = 0.0
            if self.wait_since is None:
                self.wait_since = now
            elif now - self.wait_since > self.stuck_time:
                self.state, self.recover_until = "RECOVER", now + self.recover_time
                self.get_logger().warn("stuck — backing out to find another way")
        else:
            self.wait_since = None

        steer = "L" if w > 0.02 else "R" if w < -0.02 else "-"
        self._commit(v, w, f"{vstate}/{steer}", d_left, d_right, d_front, now)

    def _commit(self, v, w, state, d_left, d_right, d_front, now):
        """Apply (v,w) to L0 — or, in dry-run, only report — plus telemetry + safety."""
        if not self.dry_run:
            self.drive.set(v, w)
        if now - self._last_log > 0.33:
            self._last_log = now
            dl = f"{d_left:.2f}" if d_left is not None else " -- "
            dr = f"{d_right:.2f}" if d_right is not None else " -- "
            tag = "DRY:" + state if self.dry_run else state
            self.get_logger().info(
                f"[{tag:18s}] L={dl} R={dr} F={d_front:4.2f}  ->  v={v:+.2f} w={w:+.2f}")
        if not self.dry_run and (now - self.t0) > self.max_time:
            self.get_logger().warn(f"max_time {self.max_time}s reached — stopping.")
            self.drive.hard_stop()
            rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description="L1 corridor cruise (reactive, never-freeze)")
    p.add_argument("--cruise", type=float, default=0.18, help="cruise speed m/s")
    p.add_argument("--max-w", type=float, default=0.5, help="max steer rad/s")
    p.add_argument("--kp", type=float, default=1.5, help="wall-avoidance gain (rad/s per m)")
    p.add_argument("--safe", type=float, default=0.6, help="comfort margin: ease away inside this (m)")
    p.add_argument("--slow", type=float, default=2.0, help="start slowing at this front dist m")
    p.add_argument("--stop", type=float, default=0.8, help="stop/wait at this front dist m")
    p.add_argument("--max-time", type=float, default=90.0, help="auto-stop after N s (live)")
    p.add_argument("--dry-run", action="store_true", help="compute + print, do NOT move")
    args = p.parse_args()

    rclpy.init()
    # cmd_timeout 1.5s: cloud delivery is bursty (~2-6 Hz, gaps up to ~2 s), so hold
    # the last command through gaps to cruise smoothly instead of stutter-stopping;
    # still stops within ~1.5 s if perception dies entirely (safe at this slow speed).
    drive = RobotDrive(cmd_timeout=1.5)
    cruise = CorridorCruise(drive, cruise=args.cruise, max_w=args.max_w, kp=args.kp,
                            safe=args.safe, slow_dist=args.slow, stop_dist=args.stop,
                            dry_run=args.dry_run, max_time=args.max_time)

    if not args.dry_run:
        print("\n>>> LIVE corridor cruise — hand on the e-stop.")
        for i in range(3, 0, -1):
            print(f"    starting in {i} ...", end="\r", flush=True)
            time.sleep(1)
        print(" " * 40, end="\r")

    # Multi-threaded: the drive's 50 Hz timer and the (large) cloud callback run on
    # separate threads, so the steady cmd_vel stream can't starve perception.
    exe = MultiThreadedExecutor(num_threads=3)
    exe.add_node(drive)
    exe.add_node(cruise)
    try:
        exe.spin()
    except KeyboardInterrupt:
        print("\n!!! aborted — hard stop")
    finally:
        drive.hard_stop()
        cruise.destroy_node()
        drive.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
