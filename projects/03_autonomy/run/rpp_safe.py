#!/usr/bin/python3
"""LIVE path-following + a SIMPLE wall-repel nudge (no modes — the user wanted less logic).

Design (2026-06-15, after run 3 SPUN when a separate "GOLDEN mode" pivoted in place):
  * ALWAYS follow the route (Regulated Pure Pursuit on the localized map pose). FOLLOW drove
    the first 11 m of the corridor cleanly and (earlier) rounded the left corner nicely.
  * Add a GENTLE, CONTINUOUS repel nudge: only when a wall is CLOSER than the path normally
    runs (repel_dist, below the path's ~0.29 m hug), bias the steering AWAY from it,
    proportional and capped, WHILE STILL DRIVING FORWARD. So the robot ARCS off a wall —
    it never crawl-pivots, so it cannot spin (the run-3 failure). No mode switch, no stop.

Hard stops ONLY: localization lost (pose collapsed >6 m off the path), something dead ahead
(<0.30 m), the loop goal, or max_time. No stall watchdog (removed at the user's request —
driving close to a wall without hitting it is fine).

The dense per-sector LiDAR sensing (sees any close point, incl. off-beam protrusions) feeds
the repel; clean STOPPING streams zeros so the base can't latch the last velocity.

  /usr/bin/python3 rpp_safe.py --dry-run            # prints clearances, NO motion
  /usr/bin/python3 rpp_safe.py --cruise 0.12 --max-time 600
Run with /usr/bin/python3. Prereqs (LIVE): CAN up, base up (publish_odom:=false), FAST-LIO +
icp localization live (map->odom_lio), robot parked at the mapped start, RC OFF, hand on e-stop.
"""
import os, sys, math, time, argparse, collections, threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import tf2_ros

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "locomotion"))
from robot_drive import RobotDrive
import corridor_eyes as ce

PATH_CSV = os.path.join(HERE, "..", "results", "teach_path_fastlio.csv")
DEG = math.pi / 180.0


def regulated_pure_pursuit(pose, path, cfg, prog=0):
    """Nav2-style RPP, progress-constrained (verified in pp_sim). Returns (v,w,look,cte,near)."""
    x, y, yaw = pose
    win = cfg.get("search_win", 60)
    lo, hi = prog, min(len(path), prog + win)
    d = np.hypot(path[lo:hi, 0] - x, path[lo:hi, 1] - y)
    near = lo + int(np.argmin(d)); cte = float(d.min())
    Ld = cfg["lookahead"]; j = near
    while j < len(path) - 1 and np.hypot(path[j, 0] - x, path[j, 1] - y) < Ld:
        j += 1
    lx, ly = path[j, 0], path[j, 1]
    a = math.atan2(ly - y, lx - x) - yaw
    a = math.atan2(math.sin(a), math.cos(a))
    dist = max(1e-3, math.hypot(lx - x, ly - y))
    curv = 2.0 * math.sin(a) / dist
    v = cfg["cruise"]; rmin = cfg["regulate_radius"]
    if abs(curv) > 1e-3:
        radius = 1.0 / abs(curv)
        if radius < rmin:
            v *= max(cfg["min_speed_frac"], radius / rmin)
    w = max(-cfg["max_w"], min(cfg["max_w"], v * curv))
    return v, w, j, cte, near


def yaw_from_quat(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _sector_min(ang, rng, a_lo, a_hi, k=5, pct=5.0):
    """Robust nearest range in an angular sector [a_lo,a_hi] (rad). Low percentile (not raw
    min) so a couple of stray points can't false-trigger; None if too few returns."""
    s = (ang >= a_lo) & (ang <= a_hi)
    n = int(s.sum())
    if n < k:
        return None
    return float(np.percentile(rng[s], pct))


class RPPSafe(Node):
    def __init__(self, drive, *, cruise=0.12, dry_run=False, max_time=60.0,
                 repel_dist=0.25, repel_gain=4.0, range_floor=0.25):
        super().__init__("rpp_safe")
        self.drive = drive; self.dry = dry_run; self.cruise = cruise; self.max_time = max_time
        self.path = np.loadtxt(PATH_CSV); self.prog = 0
        self.p = ce.EyesParams()                                  # for beams_from_scan (eyes geometry)
        self.cfg = dict(cruise=cruise, max_w=0.45, lookahead=1.0, regulate_radius=0.9,
                        min_speed_frac=0.3, search_win=60)
        # ---- simple wall-repel (no modes): nudge away ONLY when closer than repel_dist ----
        self.repel_dist = repel_dist             # only nudge when a wall is closer than this (m) — below the path's normal hug
        self.repel_gain = repel_gain             # nudge strength (rad/s per m inside repel_dist)
        self.range_floor = range_floor           # see returns down to here
        self._lost_count = 0                      # consecutive ticks the pose is far off the path (collapse)
        # No stall watchdog, no mode switch. ALWAYS follow the route; add a gentle continuous
        # repel that keeps moving forward (arcs off a wall, never pivots/spins). Only hard stops:
        # localization lost (pose collapsed), something dead ahead, the loop goal, or max_time.
        self.tfbuf = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
        self._scan = None
        # BEST_EFFORT so this node can NEVER back-pressure FAST-LIO (the corner-divergence cause).
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=2)
        self.create_subscription(PointCloud2, "/rslidar_points", self._on_cloud, qos)
        self.logf = open("/tmp/rpp_safe.csv", "w", buffering=1)
        self.logf.write("t,x,y,yaw,prog,cte,Dl,Dr,front,dmin_l,dmin_r,dmin_f,near,v,w,state\n")
        self.t0 = time.monotonic(); self._stopped = False; self._stopping_t = None; self._last = 0.0
        self.done = False                                         # main loop watches this for a clean exit
        self.create_timer(0.1, self._tick)                        # 10 Hz control
        self.get_logger().info(f"rpp_safe up [{'DRY' if dry_run else 'LIVE'}] cruise={cruise} "
                               f"repel<{repel_dist}m gain={repel_gain} floor={range_floor}m path={len(self.path)} pts")

    def _on_cloud(self, msg):
        if not hasattr(self, "_off"):
            self._off = {f.name: f.offset for f in msg.fields}
        dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                       "offsets": [self._off["x"], self._off["y"], self._off["z"]], "itemsize": msg.point_step})
        pc = np.frombuffer(msg.data, dtype=dt)
        x, y, z = pc["x"].astype(float), pc["y"].astype(float), pc["z"].astype(float)
        rng = np.hypot(x, y); ang = np.arctan2(y, x)
        # keep returns down to range_floor (so a 0.3 m edge is visible), z band rejects floor/ceiling
        m = np.isfinite(rng) & (z > -0.2) & (z < 1.0) & (rng > self.range_floor) & (rng < 8.0)
        self._scan = (ang[m], rng[m])

    def _pose(self):
        try:
            tf = self.tfbuf.lookup_transform("map", "rslidar", rclpy.time.Time())
            t = tf.transform.translation; q = tf.transform.rotation
            return t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w)
        except Exception:
            return None

    def _clearances(self):
        """Dense per-sector nearest ranges (m) + the golden beam eyes. Returns a dict or None."""
        if self._scan is None or not self._scan[0].size:
            return None
        ang, rng = self._scan
        # dense sector minima (robust low-pct). Forward hemisphere drives the danger trigger.
        dmin_f = _sector_min(ang, rng, -30 * DEG, 30 * DEG)               # front
        dmin_l = _sector_min(ang, rng, 20 * DEG, 160 * DEG)              # whole left side
        dmin_r = _sector_min(ang, rng, -160 * DEG, -20 * DEG)            # whole right side
        dmin_fl = _sector_min(ang, rng, 20 * DEG, 70 * DEG)              # forward-left (where the edge hid)
        dmin_fr = _sector_min(ang, rng, -70 * DEG, -20 * DEG)            # forward-right
        fwd = [d for d in (dmin_f, dmin_l, dmin_r, dmin_fl, dmin_fr) if d is not None]
        nearest = min(fwd) if fwd else None
        # golden beam eyes (for the smooth centering law inside GOLDEN mode)
        Dl, Dr, phi, front, oL, oR = ce.beams_from_scan(ang, rng, self.p)
        return dict(dmin_f=dmin_f, dmin_l=dmin_l, dmin_r=dmin_r, dmin_fl=dmin_fl, dmin_fr=dmin_fr,
                    nearest=nearest, Dl=Dl, Dr=Dr, phi=phi, front=front)

    def _stop(self, reason):
        """Begin a clean STOPPING: stream zeros for ~0.6 s (base decelerates, sees sustained
        zero) then shut down. Avoids the v1 race where the base latched the last velocity."""
        if self._stopping_t is None:
            self.get_logger().warn(f"STOP: {reason}")
            self._stopping_t = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        # ---- clean shutdown: stream zeros, then exit ----
        if self._stopping_t is not None:
            if not self.dry:
                self.drive.set(0.0, 0.0)
            if now - self._stopping_t > 0.6 and not self._stopped:
                self._stopped = True
                if not self.dry:
                    self.drive.hard_stop()
                self.done = True                                  # signal main to exit (no shutdown in-callback)
            return

        pose = self._pose()
        c = self._clearances()
        x = y = yaw = None
        if pose is not None:
            x, y, yaw = pose
            # Only a HOPELESSLY LOST pose stops us (outside the building, e.g. a catastrophic
            # blowup to 500,660). Normal jitter / close-to-wall driving is allowed — the GOLDEN
            # net handles physical safety. No per-cycle "jump" trip (that was too twitchy).
            in_map = (-65.0 < x < 45.0) and (-23.0 < y < 33.0)
            if (not in_map) and not self.dry:
                self._stop(f"pose left the map ({x:.0f},{y:.0f}) — lost"); return

        near = c["nearest"] if c else None                 # dense nearest (forward hemisphere), for logging

        # ---- progress tracking (always, so FOLLOW resumes at the right spot) ----
        cte = float("nan")
        if pose is not None:
            _, _, _, cte, near_idx = regulated_pure_pursuit(pose, self.path, self.cfg, self.prog)
            self.prog = near_idx
            # ---- LOCALIZATION-LOST: pose collapsed far off the path (NOT a slow/close stop —
            # this is the 13 m teleport-back-to-start failure). Stop rather than drive blind. ----
            self._lost_count = self._lost_count + 1 if cte > 6.0 else 0
            if self._lost_count > 12 and not self.dry:
                self._stop(f"localization lost — pose {cte:.0f} m off the path"); return

        # ---- command: ALWAYS follow the route + a GENTLE wall-repel nudge (no mode, no spin) ----
        if pose is None:
            v, w, state = 0.0, 0.0, "NO_POSE_HOLD"          # no localization -> hold, never drive blind
        else:
            v, w, _, _, _ = regulated_pure_pursuit(pose, self.path, self.cfg, self.prog)
            state = "FOLLOW"
            if abs(w) > 0.10:
                v = min(v, self.cruise * 0.5); state = "TURN"
            # GENTLE continuous repel: only when a wall is CLOSER than the path normally runs
            # (repel_dist), so it never fights the taught route. Bias steering AWAY, proportional
            # and capped — and KEEP moving forward so the robot ARCS off the wall (never pivots/spins).
            if c is not None:
                left = min([d for d in (c["dmin_l"], c["dmin_fl"]) if d is not None], default=99.0)
                right = min([d for d in (c["dmin_r"], c["dmin_fr"]) if d is not None], default=99.0)
                repel = 0.0
                if left < self.repel_dist:
                    repel -= self.repel_gain * (self.repel_dist - left)    # steer RIGHT (away from left)
                if right < self.repel_dist:
                    repel += self.repel_gain * (self.repel_dist - right)   # steer LEFT (away from right)
                if repel != 0.0:
                    w = max(-self.cfg["max_w"], min(self.cfg["max_w"], w + repel))
                    state = "REPEL"
                    if min(left, right) < 0.22:            # really close -> ease speed for a tighter arc
                        v = min(v, 0.07)
                front_eff = min([d for d in (c["dmin_f"], c["front"]) if d is not None], default=99.0)
                if front_eff < 0.30:                        # something dead ahead -> no forward (RPP can still turn)
                    v = 0.0; state = "FRONT_STOP"

        # (stall watchdog removed — no auto-stop for slow/non-advancing pose, per user)

        # ---- GOAL: looped back near the start ----
        if pose is not None and self.prog > len(self.path) - 6 and \
           math.hypot(x - self.path[-1, 0], y - self.path[-1, 1]) < 2.0 and (now - self.t0) > 10:
            self._stop("GOAL — loop complete"); return

        if not self.dry:
            self.drive.set(v, w)

        # ---- log every cycle ----
        def f(z): return "%.2f" % z if z is not None else "--"
        xs = "%.2f" % x if x is not None else "--"; ys = "%.2f" % y if y is not None else "--"
        yw = "%.0f" % math.degrees(yaw) if yaw is not None else "--"
        cl = c or {}
        self.logf.write(f"{now-self.t0:.2f},{xs},{ys},{yw},{self.prog},{cte:.2f},"
                        f"{f(cl.get('Dl'))},{f(cl.get('Dr'))},{f(cl.get('front'))},"
                        f"{f(cl.get('dmin_l'))},{f(cl.get('dmin_r'))},{f(cl.get('dmin_f'))},"
                        f"{f(near)},{v:.3f},{w:.3f},{state}\n")
        if now - self._last > 0.4:
            self._last = now
            tag = "DRY" if self.dry else "RUN"
            self.get_logger().info(
                f"[{tag}:{state:11s}] pose({xs},{ys}) yaw={yw} prog={self.prog} "
                f"cte={cte:.2f} | near={f(near)} L={f(cl.get('dmin_l'))} R={f(cl.get('dmin_r'))} "
                f"F={f(cl.get('dmin_f'))} -> v={v:+.2f} w={w:+.2f}")

        if (now - self.t0) > self.max_time:                       # ends the run in BOTH dry and live
            self._stop(f"max_time {self.max_time}s")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cruise", type=float, default=0.12)
    ap.add_argument("--max-time", type=float, default=60.0)
    ap.add_argument("--repel-dist", type=float, default=0.25)     # only nudge when a wall is closer than this (below the path's normal hug)
    ap.add_argument("--repel-gain", type=float, default=4.0)      # nudge strength (rad/s per m inside repel-dist)
    ap.add_argument("--range-floor", type=float, default=0.25)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    rclpy.init(); drive = RobotDrive(cmd_timeout=0.7)
    node = RPPSafe(drive, cruise=a.cruise, dry_run=a.dry_run, max_time=a.max_time,
                   repel_dist=a.repel_dist, repel_gain=a.repel_gain, range_floor=a.range_floor)
    if not a.dry_run:
        print("\n>>> LIVE rpp_safe (FOLLOW + gentle wall-repel nudge) — hand on the e-stop.")
        for i in range(3, 0, -1):
            print(f"    starting in {i} ...", end="\r", flush=True); time.sleep(1)
    exe = MultiThreadedExecutor(num_threads=4); exe.add_node(drive); exe.add_node(node)
    spin = threading.Thread(target=exe.spin, daemon=True); spin.start()   # spin in bg; main watches node.done
    try:
        while rclpy.ok() and not node.done:
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n!!! aborted — hard stop")
    finally:
        try:
            if rclpy.ok():
                drive.hard_stop()                                 # guaranteed final zero (guarded)
        except Exception:
            pass
        exe.shutdown(timeout_sec=1.0)
        node.destroy_node(); drive.destroy_node()
        rclpy.ok() and rclpy.shutdown()


if __name__ == "__main__":
    main()
