#!/usr/bin/python3
"""LIVE path-following: Regulated Pure Pursuit on the localized map-pose (route) + the proven
eyes-only LiDAR wall-safety (backstop) + L0 RobotDrive (clamps/dead-man/hard-stop). Slow.

Pose source: TF map->rslidar (our validated localization). Path: teach_path_fastlio.csv, whose
coords ARE the map frame (same teach-session odom_lio the prior map lives in). RPP gives the
route direction; the eyes only OVERRIDE to steer-away/stop if a wall gets too close.

  /usr/bin/python3 rpp_drive.py --dry-run            # prints pose/v/w, NO motion
  /usr/bin/python3 rpp_drive.py --cruise 0.12 --max-time 60
Run with /usr/bin/python3. Prereqs: CAN up, base up (publish_odom:=false), FAST-LIO + icp
localization live (map->odom_lio), RC OFF, hand on e-stop.
"""
import os, sys, math, time, argparse
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


def regulated_pure_pursuit(pose, path, cfg, prog=0):
    """Nav2-style RPP, progress-constrained (verified in pp_sim). Returns (v,w,look,cte,near)."""
    x, y, yaw = pose
    win = cfg.get("search_win", 60)
    lo, hi = prog, min(len(path), prog+win)
    d = np.hypot(path[lo:hi, 0]-x, path[lo:hi, 1]-y)
    near = lo + int(np.argmin(d)); cte = float(d.min())
    Ld = cfg["lookahead"]; j = near
    while j < len(path)-1 and np.hypot(path[j, 0]-x, path[j, 1]-y) < Ld:
        j += 1
    lx, ly = path[j, 0], path[j, 1]
    a = math.atan2(ly - y, lx - x) - yaw
    a = math.atan2(math.sin(a), math.cos(a))
    dist = max(1e-3, math.hypot(lx-x, ly-y))
    curv = 2.0*math.sin(a)/dist
    v = cfg["cruise"]; rmin = cfg["regulate_radius"]
    if abs(curv) > 1e-3:
        radius = 1.0/abs(curv)
        if radius < rmin:
            v *= max(cfg["min_speed_frac"], radius/rmin)
    w = max(-cfg["max_w"], min(cfg["max_w"], v*curv))
    return v, w, j, cte, near


def yaw_from_quat(x, y, z, w):
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


class RPPDrive(Node):
    def __init__(self, drive, *, cruise=0.12, dry_run=False, max_time=60.0):
        super().__init__("rpp_drive")
        self.drive = drive; self.dry = dry_run; self.cruise = cruise; self.max_time = max_time
        self.path = np.loadtxt(PATH_CSV)
        self.prog = 0
        self.p = ce.EyesParams()                         # for safety thresholds (crit/hard_min)
        self.cfg = dict(cruise=cruise, max_w=0.45, lookahead=1.0, regulate_radius=0.9,
                        min_speed_frac=0.3, search_win=60)
        self.tfbuf = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
        self._scan = None
        self._prev_xy = None                             # divergence guard
        # BEST_EFFORT so this node can NEVER back-pressure FAST-LIO (the live failure cause).
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=2)
        self.create_subscription(PointCloud2, "/rslidar_points", self._on_cloud, qos)
        self.logf = open("/tmp/rpp_drive.csv", "w", buffering=1)
        self.logf.write("t,x,y,yaw,prog,cte,v,w,state\n")
        self.t0 = time.monotonic(); self._stopped = False; self._last = 0.0
        self.create_timer(0.1, self._tick)              # 10 Hz control
        self.get_logger().info(f"rpp_drive up [{'DRY' if dry_run else 'LIVE'}] cruise={cruise} "
                               f"path={len(self.path)} pts — waiting for TF map->rslidar ...")

    def _on_cloud(self, msg):
        if not hasattr(self, "_off"):
            self._off = {f.name: f.offset for f in msg.fields}
        dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"]*3,
                       "offsets": [self._off["x"], self._off["y"], self._off["z"]], "itemsize": msg.point_step})
        pc = np.frombuffer(msg.data, dtype=dt)
        x, y, z = pc["x"].astype(float), pc["y"].astype(float), pc["z"].astype(float)
        rng = np.hypot(x, y); ang = np.arctan2(y, x)
        m = np.isfinite(rng) & (z > -0.2) & (z < 1.0) & (rng > 0.4) & (rng < 8.0)
        self._scan = (ang[m], rng[m])

    def _pose(self):
        try:
            tf = self.tfbuf.lookup_transform("map", "rslidar", rclpy.time.Time())
            t = tf.transform.translation; q = tf.transform.rotation
            return t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w)
        except Exception:
            return None

    def _tick(self):
        now = time.monotonic()
        pose = self._pose()
        if pose is None:
            return
        x, y, yaw = pose
        # ---- DIVERGENCE GUARD: localization jumped or left the map => HARD STOP (the live failure) ----
        jump = math.hypot(x-self._prev_xy[0], y-self._prev_xy[1]) if self._prev_xy else 0.0
        self._prev_xy = (x, y)
        in_map = (-62.0 < x < 42.0) and (-20.0 < y < 30.0)
        if jump > 1.0 or not in_map:
            if not self.dry:
                self.drive.set(0.0, 0.0); self.drive.hard_stop()
            self.get_logger().error(f"DIVERGENCE GUARD: jump {jump:.1f} m / pos({x:.0f},{y:.0f}) "
                                    f"{'outside map' if not in_map else ''} -> HARD STOP")
            self._stopped = True; rclpy.shutdown(); return
        v, w, look, cte, near = regulated_pure_pursuit(pose, self.path, self.cfg, self.prog)
        self.prog = near
        state = "FOLLOW"
        # speed schedule: slower while turning (extra margin so FAST-LIO keeps up through corners)
        if abs(w) > 0.10:
            v = min(v, self.cruise * 0.5); state = "TURN"
        # ---- eyes wall-safety BACKSTOP (proven) — only overrides to be safer ----
        if self._scan is not None and self._scan[0].size:
            Dl, Dr, phi, front, oL, oR = ce.beams_from_scan(self._scan[0], self._scan[1], self.p)
            present = [d for d in (Dl, Dr) if d is not None]
            nearest = min(present) if present else None
            if nearest is not None and nearest < self.p.hard_min:
                v = min(v, self.p.crawl); state = "CRAWL"
            if nearest is not None and nearest < self.p.crit:        # a wall critically close
                w = -self.p.max_w if (Dl is not None and (Dr is None or Dl <= Dr)) else self.p.max_w
                v = self.p.crawl; state = "CRIT_AVOID"
            if front < 0.30:
                v = 0.0; state = "FRONT_STOP"
        # goal: completed the loop, back near start
        if self.prog > len(self.path)-6 and math.hypot(x-self.path[-1, 0], y-self.path[-1, 1]) < 2.0 and (now-self.t0) > 10:
            v, w, state = 0.0, 0.0, "GOAL"
        if not self.dry:
            self.drive.set(v, w)
        self.logf.write(f"{now-self.t0:.2f},{x:.2f},{y:.2f},{math.degrees(yaw):.0f},{self.prog},{cte:.2f},{v:.3f},{w:.3f},{state}\n")
        if now - self._last > 0.4:
            self._last = now
            tag = "DRY" if self.dry else "RUN"
            self.get_logger().info(f"[{tag}:{state:10s}] map({x:+.1f},{y:+.1f}) yaw={math.degrees(yaw):+4.0f} "
                                   f"prog={self.prog}/{len(self.path)} cte={cte:.2f} -> v={v:+.2f} w={w:+.2f}")
        if state == "GOAL" and not self._stopped:
            self._stopped = True; self.get_logger().info("GOAL — loop complete, stopping."); self.drive.hard_stop(); rclpy.shutdown()
        if not self.dry and (now-self.t0) > self.max_time and not self._stopped:
            self._stopped = True; self.get_logger().warn(f"max_time {self.max_time}s — stopping."); self.drive.hard_stop(); rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cruise", type=float, default=0.12)
    ap.add_argument("--max-time", type=float, default=60.0)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    rclpy.init(); drive = RobotDrive(cmd_timeout=0.7)
    node = RPPDrive(drive, cruise=a.cruise, dry_run=a.dry_run, max_time=a.max_time)
    if not a.dry_run:
        print("\n>>> LIVE RPP path-following — hand on the e-stop.");
        for i in range(3, 0, -1):
            print(f"    starting in {i} ...", end="\r", flush=True); time.sleep(1)
    exe = MultiThreadedExecutor(num_threads=4); exe.add_node(drive); exe.add_node(node)
    try:
        exe.spin()
    except KeyboardInterrupt:
        print("\n!!! aborted — hard stop")
    finally:
        drive.hard_stop(); node.destroy_node(); drive.destroy_node()
        rclpy.ok() and rclpy.shutdown()


if __name__ == "__main__":
    main()
