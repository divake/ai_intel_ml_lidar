#!/usr/bin/python3
"""LIVE path-following WITH the golden safety net (two clean modes).

The user's spec (2026-06-15), after the right-wall protrusion grind:
  * MOST of the time -> FOLLOW the route (Regulated Pure Pursuit on the localized map pose).
    This is proven: it rounded the corner that used to diverge.
  * The MOMENT any wall/edge comes within a DANGER RADIUS (right / left / front), instantly
    hand control to the GOLDEN eyes-only centering ("wobble to center, drive straight, peel
    off the wall") — corridor_center.py's proven law. When back OUT of the danger zone, hand
    control back to FOLLOW. Since the taught path runs down the middle, it rejoins naturally.

Why the protrusion grind happened (fixed here):
  1. The old eyes sampled only a few fixed beams (+-45, +-90) -> a thin protrusion slipped
     between them and never triggered. FIX: DENSE per-sector minimum range (sees any close
     point), with a small range floor so we can actually see a 0.3 m edge.
  2. There was no stall watchdog -> when wedged, the follower kept commanding v=0.12 forward
     for 90 s. FIX: if forward motion is commanded but the localized pose isn't advancing,
     STOP. A robot that cannot make progress must stop, never grind.
  3. The stop on exit raced/failed (base latched the last velocity). FIX: a STOPPING state
     that streams zeros cleanly for ~0.6 s before shutdown, plus a guarded finally.

Steering authority:
  - FOLLOW: RPP (route) — the ONLY thing that chooses direction / turns at junctions.
  - GOLDEN: the eyes recenter + hard steer-away from the DENSE nearest obstacle. Never both.

  /usr/bin/python3 rpp_safe.py --dry-run            # prints clearances + mode, NO motion
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
                 danger_enter=0.40, danger_exit=0.55, range_floor=0.25):
        super().__init__("rpp_safe")
        self.drive = drive; self.dry = dry_run; self.cruise = cruise; self.max_time = max_time
        self.path = np.loadtxt(PATH_CSV); self.prog = 0
        self.p = ce.EyesParams()                                  # golden thresholds (crit/hard_min/kp/kd...)
        self.cfg = dict(cruise=cruise, max_w=0.45, lookahead=1.0, regulate_radius=0.9,
                        min_speed_frac=0.3, search_win=60)
        # ---- danger radius (the safety net) ----
        self.danger_enter = danger_enter        # any wall closer than this (m) -> GOLDEN
        self.danger_exit = danger_exit           # all clear beyond this (m) -> back to FOLLOW (hysteresis)
        self.range_floor = range_floor           # see returns down to here (the eyes use 0.4; too coarse for a 0.3 m edge)
        self.mode = "FOLLOW"
        # ---- stall watchdog ----
        self._hist = collections.deque()         # (t, x, y, v_cmd) over a sliding window
        self.stall_win = 4.0                     # s
        self.stall_cmd = 0.12                    # if commanded travel over the window exceeds this ...
        self.stall_act = 0.08                    # ... but actual displacement is below this -> wedged
        self._prev_xy = None                     # divergence guard
        self.tfbuf = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
        self._scan = None
        # BEST_EFFORT so this node can NEVER back-pressure FAST-LIO (the corner-divergence cause).
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=2)
        self.create_subscription(PointCloud2, "/rslidar_points", self._on_cloud, qos)
        self.logf = open("/tmp/rpp_safe.csv", "w", buffering=1)
        self.logf.write("t,x,y,yaw,prog,cte,Dl,Dr,front,dmin_l,dmin_r,dmin_f,near,v,w,mode,state\n")
        self.t0 = time.monotonic(); self._stopped = False; self._stopping_t = None; self._last = 0.0
        self.done = False                                         # main loop watches this for a clean exit
        self.create_timer(0.1, self._tick)                        # 10 Hz control
        self.get_logger().info(f"rpp_safe up [{'DRY' if dry_run else 'LIVE'}] cruise={cruise} "
                               f"danger<{danger_enter}m floor={range_floor}m path={len(self.path)} pts")

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

    def _golden_avoid(self, c):
        """GOLDEN danger action — the proven golden law (corridor_center.py), but fed DENSE eyes
        so an off-beam edge can't slip through (the protrusion's lesson). Two terms, both golden:
          centering  w_center = kp*e + kd*phi   (smooth hold of the centerline, perpendicular beams)
          steer-away from the DENSE nearest side, RAMPED by closeness (0 at danger_enter -> max at crit)
        + crawl speed + no forward motion if blocked dead ahead. Purely reactive; no map here."""
        p = self.p
        Dl, Dr, phi = c["Dl"], c["Dr"], c["phi"]
        phi = max(-p.phi_cap, min(p.phi_cap, phi if phi is not None else 0.0))
        # --- golden centering from the perpendicular beams ---
        if Dl is not None and Dr is not None:
            e = Dl - Dr
        elif Dl is not None:
            e = Dl - p.target
        elif Dr is not None:
            e = p.target - Dr
        else:
            e = 0.0
        w_center = p.kp * e + p.kd * phi
        # --- DENSE steer-away (whole-side + forward-diagonal nearest), ramped by severity ---
        left = min([d for d in (c["dmin_l"], c["dmin_fl"]) if d is not None], default=99.0)
        right = min([d for d in (c["dmin_r"], c["dmin_fr"]) if d is not None], default=99.0)
        nearest_side = min(left, right)
        away = 0.0
        if nearest_side < self.danger_enter:
            sev = (self.danger_enter - nearest_side) / max(0.05, self.danger_enter - p.crit)
            sev = max(0.0, min(1.0, sev))                            # 0 at danger_enter -> 1 at crit
            away = (p.max_w if right <= left else -p.max_w) * sev    # +w (left) away from a RIGHT obstacle
        w = max(-p.max_w, min(p.max_w, w_center + away))
        v = p.crawl                                                  # slow & deliberate in danger
        # never drive into something dead ahead (rotation to peel off still allowed)
        front_eff = min([d for d in (c["dmin_f"], c["front"]) if d is not None], default=99.0)
        if front_eff < 0.30:
            v = 0.0
        side = "R" if right <= left else "L"
        state = f"GOLDEN_AWAY_{side}" if away != 0.0 else "GOLDEN_CENTER"
        return v, w, state

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
            # ---- DIVERGENCE GUARD: localization jumped / left the map => STOP ----
            jump = math.hypot(x - self._prev_xy[0], y - self._prev_xy[1]) if self._prev_xy else 0.0
            self._prev_xy = (x, y)
            in_map = (-62.0 < x < 42.0) and (-20.0 < y < 30.0)
            if (jump > 1.0 or not in_map) and not self.dry:          # guard is a LIVE safety; dry just observes
                self._stop(f"divergence jump {jump:.1f} m / pos({x:.0f},{y:.0f})"); return

        # ---- DANGER detection (hysteresis) drives the FOLLOW <-> GOLDEN switch ----
        near = c["nearest"] if c else None
        if self.mode == "FOLLOW" and near is not None and near < self.danger_enter:
            self.mode = "GOLDEN"; self.get_logger().warn(f"DANGER {near:.2f} m -> GOLDEN recenter")
        elif self.mode == "GOLDEN" and (near is None or near > self.danger_exit):
            self.mode = "FOLLOW"; self.get_logger().info(f"clear ({near}) -> back to FOLLOW")

        # ---- progress tracking (always, so FOLLOW resumes at the right spot) ----
        cte = float("nan")
        if pose is not None:
            _, _, _, cte, near_idx = regulated_pure_pursuit(pose, self.path, self.cfg, self.prog)
            self.prog = near_idx

        # ---- choose command by mode ----
        if self.mode == "GOLDEN" and c is not None:
            v, w, state = self._golden_avoid(c)
        elif pose is not None:
            v, w, _, _, _ = regulated_pure_pursuit(pose, self.path, self.cfg, self.prog)
            state = "FOLLOW"
            if abs(w) > 0.10:
                v = min(v, self.cruise * 0.5); state = "FOLLOW_TURN"
        else:
            # no localization and not in danger -> hold (don't drive blind)
            v, w, state = 0.0, 0.0, "NO_POSE_HOLD"

        # ---- STALL watchdog: commanded to move but pose not advancing => wedged => STOP ----
        if pose is not None:
            self._hist.append((now, x, y, v))
            while self._hist and now - self._hist[0][0] > self.stall_win:
                self._hist.popleft()
            if len(self._hist) > 5 and (now - self._hist[0][0]) > self.stall_win * 0.8:
                cmd_dist = sum(h[3] for h in self._hist) * 0.1                  # ~v*dt integrated (10 Hz)
                act_dist = math.hypot(x - self._hist[0][1], y - self._hist[0][2])
                if cmd_dist > self.stall_cmd and act_dist < self.stall_act and not self.dry:
                    self._stop(f"stalled: commanded {cmd_dist:.2f} m but moved {act_dist:.2f} m"); return

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
                        f"{f(near)},{v:.3f},{w:.3f},{self.mode},{state}\n")
        if now - self._last > 0.4:
            self._last = now
            tag = "DRY" if self.dry else "RUN"
            self.get_logger().info(
                f"[{tag}:{self.mode}:{state:14s}] pose({xs},{ys}) yaw={yw} prog={self.prog} "
                f"cte={cte:.2f} | near={f(near)} L={f(cl.get('dmin_l'))} R={f(cl.get('dmin_r'))} "
                f"F={f(cl.get('dmin_f'))} -> v={v:+.2f} w={w:+.2f}")

        if (now - self.t0) > self.max_time:                       # ends the run in BOTH dry and live
            self._stop(f"max_time {self.max_time}s")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cruise", type=float, default=0.12)
    ap.add_argument("--max-time", type=float, default=60.0)
    ap.add_argument("--danger-enter", type=float, default=0.40)
    ap.add_argument("--danger-exit", type=float, default=0.55)
    ap.add_argument("--range-floor", type=float, default=0.25)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    rclpy.init(); drive = RobotDrive(cmd_timeout=0.7)
    node = RPPSafe(drive, cruise=a.cruise, dry_run=a.dry_run, max_time=a.max_time,
                   danger_enter=a.danger_enter, danger_exit=a.danger_exit, range_floor=a.range_floor)
    if not a.dry_run:
        print("\n>>> LIVE rpp_safe (FOLLOW + golden danger net) — hand on the e-stop.")
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
