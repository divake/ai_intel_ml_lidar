#!/usr/bin/python3
"""L1 — EYES-ONLY corridor centering. NO MAP in the loop. Drive slow by the LiDAR walls.

Why this exists (two root causes proven live 2026-06-15):
  1. The map poisoned steering. teach_path_fastlio.csv is in the odom_lio frame; FAST-LIO
     re-zeroes/drifts between sessions, so the map coords and the live pose are in DIFFERENT
     frames. Parked dead-center the eyes said centered but the map said ~1.7 m off -> every
     "correct toward the map" drove us at a wall. So: the map does NOT touch steering here.
  2. Steering on wall-distance ALONE (position-only) fishtails -> overshoots into the wall
     (textbook: P control of a turn-rate actuator oscillates). Fix (F1TENTH wall-follow):
     steer on BOTH lateral offset AND heading-to-the-walls, and go SLOW so the ~8 Hz LiDAR
     feedback keeps up.

Method (validated read-only, parked):
  Per side, two beams: abeam b (+-90 deg) and forward a (+-45 deg). Then
     wall angle  alpha = atan2(a*cos45 - b, a*sin45)
     perp dist   D     = b * cos(alpha)
  Control:  e = D_left - D_right            (lateral error; <0 => too close to LEFT)
            phi = (alphaL - alphaR)/2       (heading; >0 => nose pointed RIGHT of corridor)
            w   = kp*e + kd*phi             (centered + straighten; both gains > 0)
  Signs verified against the parked measurement (e=-0.66 -> w<0 steer right; correct).
  Repel/steer-away dominates near a wall; one wall open (cavity/junction) => go straight
  cautiously (cavity-safe); front blocked or no walls => stop. Reuses L0 robot_drive.

Run with /usr/bin/python3.  Prereqs: CAN up, scout base up, RC OFF, /rslidar_points live.
    /usr/bin/python3 corridor_center.py --dry-run        # prints e/phi/w, NO motion
    /usr/bin/python3 corridor_center.py --cruise 0.10    # live (hand on e-stop)
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

DEG = math.pi / 180.0


def _clamp(x, lim):
    return max(-lim, min(lim, x))


class CorridorCenter(Node):
    def __init__(self, drive, *, cruise=0.10, max_w=0.35, kp=0.6, kd=0.8,
                 target=0.82, band=0.10, head_band=0.10,
                 hard_min=0.35, crit=0.28, crawl=0.05,
                 front_turn=1.0, front_clear=1.6, turn_w=0.30,
                 phi_lp=0.5, phi_cap=0.35,
                 zmin=-0.20, zmax=1.0, range_min=0.4, range_max=8.0,
                 beam_half=6.0, front_half=12.0, dry_run=False, max_time=40.0):
        super().__init__("corridor_center")
        self.drive = drive
        self.cruise = cruise
        self.max_w = max_w
        self.kp = kp                      # lateral gain (rad/s per m of offset)
        self.kd = kd                      # heading gain (rad/s per rad of yaw) -> the damping
        self.target = target             # desired distance to a wall when only ONE is seen
        self.band = band                 # lateral comfort band (m): inside => go straight
        self.head_band = head_band       # heading comfort band (rad): inside => go straight
        self.hard_min = hard_min         # side wall closer than this => crawl
        self.crit = crit                 # side wall closer than this => HARD steer-away (safety)
        self.crawl = crawl               # crawl speed (m/s)
        # --- reactive corner turn (Phase 2a): front blocked => turn toward the open side ---
        self.front_turn = front_turn     # wall ahead closer than this => enter TURN
        self.front_clear = front_clear   # corridor ahead farther than this => exit TURN
        self.turn_w = turn_w             # turn rate while rounding a corner (rad/s)
        # --- heading estimate conditioning (kill the weave / doorway spikes) ---
        self.phi_lp = phi_lp             # EMA factor on phi (0..1, higher = less smoothing)
        self.phi_cap = phi_cap           # clamp phi used for control (rad) -> reject spikes
        self.zmin, self.zmax = zmin, zmax
        self.range_min, self.range_max = range_min, range_max
        self.beam_half = beam_half * DEG
        self.front_half = front_half * DEG
        self.dry_run = dry_run
        self.max_time = max_time
        self.t0 = time.monotonic()
        self._last_log = 0.0
        self._off = None
        self._stopped = False
        self._turning = False            # corner state machine
        self._turn_dir = 0               # +1 = left (CCW), -1 = right (CW)
        self._phi_f = None               # filtered heading

        self.logf = open("/tmp/corridor_center.csv", "w", buffering=1)
        self.logf.write("t,D_left,D_right,e,phi_deg,front,v,w,state\n")

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(PointCloud2, "/rslidar_points", self._on_cloud, qos)
        mode = "DRY-RUN (no motion)" if dry_run else "LIVE"
        self.get_logger().info(
            f"corridor_center up [{mode}] cruise={cruise} kp={kp} kd={kd} "
            f"band={band}m head_band={head_band:.2f}rad — waiting for LiDAR ...")

    # ---- beam helper: median range in a +-beam_half window around a0 (robust to noise) ----
    def _beam(self, ang, rng, a0):
        s = (ang >= a0 - self.beam_half) & (ang <= a0 + self.beam_half)
        return float(np.median(rng[s])) if s.any() else None

    def _on_cloud(self, msg):
        if self._off is None:
            self._off = {f.name: f.offset for f in msg.fields}
        ox, oy, oz = self._off["x"], self._off["y"], self._off["z"]
        dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4", "<f4", "<f4"],
                       "offsets": [ox, oy, oz], "itemsize": msg.point_step})
        pc = np.frombuffer(msg.data, dtype=dt)
        x, y, z = pc["x"].astype(float), pc["y"].astype(float), pc["z"].astype(float)
        rng = np.hypot(x, y)
        ang = np.arctan2(y, x)
        m = (np.isfinite(rng) & (z > self.zmin) & (z < self.zmax)
             & (rng > self.range_min) & (rng < self.range_max))
        ang, rng = ang[m], rng[m]
        if ang.size == 0:
            self._act(None, None, 0.0, 99.0)
            return

        c45, s45 = math.cos(45 * DEG), math.sin(45 * DEG)
        bL, aL = self._beam(ang, rng, 90 * DEG), self._beam(ang, rng, 45 * DEG)
        bR, aR = self._beam(ang, rng, -90 * DEG), self._beam(ang, rng, -45 * DEG)

        D_left = alphaL = None
        if bL is not None and aL is not None:
            alphaL = math.atan2(aL * c45 - bL, aL * s45)
            D_left = bL * math.cos(alphaL)
        D_right = alphaR = None
        if bR is not None and aR is not None:
            alphaR = math.atan2(aR * c45 - bR, aR * s45)
            D_right = bR * math.cos(alphaR)

        # heading error phi: >0 => nose pointed RIGHT of the corridor axis
        if alphaL is not None and alphaR is not None:
            phi = (alphaL - alphaR) / 2.0
        elif alphaL is not None:
            phi = alphaL
        elif alphaR is not None:
            phi = -alphaR
        else:
            phi = 0.0

        # front clearance: 15th-pct range in a narrow forward cone (robust vs a few stray
        # side points); needs enough returns to count as a real wall ahead.
        fsel = (ang >= -self.front_half) & (ang <= self.front_half)
        front = float(np.percentile(rng[fsel], 15)) if int(fsel.sum()) >= 5 else 99.0

        # which side is OPEN (for the reactive corner turn): how far we can see fwd-left vs
        # fwd-right (85th-pct range). The more-open side is the corridor continuation.
        lsel = (ang >= 35 * DEG) & (ang <= 110 * DEG)
        rsel = (ang >= -110 * DEG) & (ang <= -35 * DEG)
        openL = float(np.percentile(rng[lsel], 85)) if lsel.any() else 0.0
        openR = float(np.percentile(rng[rsel], 85)) if rsel.any() else 0.0

        self._act(D_left, D_right, phi, front, openL, openR)

    def _act(self, D_left, D_right, phi_raw, front, openL, openR):
        now = time.monotonic()

        # condition the heading estimate: EMA smooth + clamp (kills weave & doorway spikes)
        self._phi_f = phi_raw if self._phi_f is None else (
            self.phi_lp * phi_raw + (1 - self.phi_lp) * self._phi_f)
        phi = max(-self.phi_cap, min(self.phi_cap, self._phi_f))

        present = [d for d in (D_left, D_right) if d is not None]
        nearest = min(present) if present else None
        ev = (D_left - D_right) if (D_left is not None and D_right is not None) else 0.0

        # ---- reactive corner state machine (hysteresis on front) ----
        # front blocked => a corner/dead-end ahead; turn toward whichever side is OPEN (the
        # corridor continuation). Latch direction at entry so it doesn't flip-flop. Exit when
        # the new corridor opens up ahead. This is the "arrow" that harnesses the robot.
        if self._turning and front > self.front_clear:
            self._turning = False
        if (not self._turning) and front < self.front_turn:
            self._turning = True
            self._turn_dir = +1 if openL >= openR else -1

        if self._turning:
            w = self._turn_dir * self.turn_w
            v = self.crawl if front > 0.45 else 0.0               # crawl round, or pivot if tight
            state = "TURN_LEFT" if self._turn_dir > 0 else "TURN_RIGHT"
        elif nearest is None:
            v, w, state = 0.0, 0.0, "NO_WALLS_STOP"               # blind -> stop
        else:
            # lateral error e (<0 => steer right; >0 => steer left)
            if D_left is not None and D_right is not None:
                e, base = D_left - D_right, "CENTER"
            elif D_left is not None:
                e, base = D_left - self.target, "LEFT_ONLY"        # hold target off the one wall
            else:
                e, base = self.target - D_right, "RIGHT_ONLY"

            if abs(e) < self.band and abs(phi) < self.head_band:
                w, state = 0.0, "STRAIGHT"                         # comfort band -> dead straight
            else:
                w, state = _clamp(self.kp * e + self.kd * phi, self.max_w), base

            v = self.cruise if abs(w) < 0.05 else self.cruise * 0.6
            if nearest < self.hard_min:
                v, state = self.crawl, "CRAWL_" + state

            # HARD safety: a wall critically close -> override, steer away at max, crawl
            if nearest < self.crit:
                if D_left is not None and (D_right is None or D_left <= D_right):
                    w = -self.max_w                               # left wall closest -> steer right
                else:
                    w = self.max_w                                # right wall closest -> steer left
                v, state = self.crawl, "CRIT_AVOID"

        # hard emergency: something very close dead ahead -> no forward motion (pivot still ok)
        if front < 0.30:
            v = 0.0
            if not self._turning:
                state = "FRONT_EMERG"

        if not self.dry_run:
            self.drive.set(v, w)

        # log every cycle (line-buffered) for after-run analysis
        dl = D_left if D_left is not None else -1.0
        dr = D_right if D_right is not None else -1.0
        ev = (D_left - D_right) if (D_left is not None and D_right is not None) else 0.0
        self.logf.write(f"{now - self.t0:.2f},{dl:.3f},{dr:.3f},{ev:.3f},"
                        f"{math.degrees(phi):.1f},{front:.2f},{v:.3f},{w:.3f},{state}\n")

        if now - self._last_log > 0.3:
            self._last_log = now
            dls = f"{D_left:.2f}" if D_left is not None else " -- "
            drs = f"{D_right:.2f}" if D_right is not None else " -- "
            tag = "DRY" if self.dry_run else "RUN"
            self.get_logger().info(
                f"[{tag}:{state:12s}] L={dls} R={drs} e={ev:+.2f} "
                f"phi={math.degrees(phi):+5.1f} front={front:4.1f} -> v={v:+.2f} w={w:+.2f}")

        if not self.dry_run and (now - self.t0) > self.max_time and not self._stopped:
            self._stopped = True
            self.get_logger().warn(f"max_time {self.max_time}s reached — stopping.")
            self.drive.hard_stop()
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description="L1 eyes-only corridor centering (no map)")
    ap.add_argument("--cruise", type=float, default=0.10)
    ap.add_argument("--max-w", type=float, default=0.35)
    ap.add_argument("--kp", type=float, default=0.6)
    ap.add_argument("--kd", type=float, default=0.8)
    ap.add_argument("--target", type=float, default=0.82)
    ap.add_argument("--band", type=float, default=0.10)
    ap.add_argument("--max-time", type=float, default=40.0)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    drive = RobotDrive(cmd_timeout=0.7)
    node = CorridorCenter(drive, cruise=args.cruise, max_w=args.max_w, kp=args.kp,
                          kd=args.kd, target=args.target, band=args.band,
                          dry_run=args.dry_run, max_time=args.max_time)

    if not args.dry_run:
        print("\n>>> LIVE eyes-only corridor centering — hand on the e-stop.")
        for i in range(3, 0, -1):
            print(f"    starting in {i} ...", end="\r", flush=True)
            time.sleep(1)
        print(" " * 40, end="\r")

    exe = MultiThreadedExecutor(num_threads=4)
    exe.add_node(drive)
    exe.add_node(node)
    try:
        exe.spin()
    except KeyboardInterrupt:
        print("\n!!! aborted — hard stop")
    finally:
        drive.hard_stop()
        node.destroy_node()
        drive.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
