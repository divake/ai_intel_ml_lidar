#!/usr/bin/env python3
"""Corridor simulator — develop/validate the autonomy controller against OUR map, fast.

No robot, no ROS. Loads the loop-closed occupancy map + the taught path, simulates a
skid-steer robot with a ray-cast LiDAR, and drives it with the SAME controller the real
robot uses (locomotion/corridor_control.plan). Renders the trajectory over the map so we
can see — in seconds — whether it follows the route and gets through the corners.

Localization model (--loc):
  perfect  : controller sees ground-truth pose (validates the CONTROL geometry)
  lowrate  : controller pose updates only at --loc-hz (mimics our ~2-4 Hz LiDAR-SLAM)
  noisy    : lowrate + yaw noise that grows with turn rate (mimics SLAM losing track in
             a spin — this is what reproduces the real corner failure)

Run with the conda python (rendering):
  /home/nus-ai/miniconda3/envs/intel_ai/bin/python corridor_sim.py
  ... --loc noisy --gif    # reproduce the corner problem + save an animation
"""
import os
import sys
import math
import argparse

import numpy as np
import yaml
from PIL import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import collections as mc

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "locomotion"))
import corridor_control as cc

RESULTS = os.path.normpath(os.path.join(HERE, "..", "results"))


# ----------------------------------------------------------------------------- map
class Grid:
    def __init__(self, yaml_path):
        with open(yaml_path) as f:
            m = yaml.safe_load(f)
        img = Image.open(os.path.join(os.path.dirname(yaml_path), m["image"])).convert("L")
        self.a = np.array(img)                      # (H, W) uint8, row 0 = top
        self.H, self.W = self.a.shape
        self.res = float(m["resolution"])
        self.ox, self.oy = m["origin"][0], m["origin"][1]
        self.occ = self.a < 90                      # dark = occupied (walls)

    def w2p(self, x, y):
        c = int((x - self.ox) / self.res)
        r = self.H - 1 - int((y - self.oy) / self.res)
        return c, r

    def occupied(self, x, y):
        c, r = self.w2p(x, y)
        if r < 0 or r >= self.H or c < 0 or c >= self.W:
            return True                             # off-map = blocked (for ray-cast)
        return bool(self.occ[r, c])

    def is_wall(self, x, y):
        """True only on a MAPPED wall cell (for collision counting) — unmapped gray and
        off-map do NOT count, so an incomplete map doesn't inflate collisions."""
        c, r = self.w2p(x, y)
        if r < 0 or r >= self.H or c < 0 or c >= self.W:
            return False
        return bool(self.occ[r, c])

    def raycast(self, x, y, ang, max_r=6.0, step=0.05):
        ca, sa = math.cos(ang), math.sin(ang)
        r = step
        while r <= max_r:
            if self.occupied(x + ca * r, y + sa * r):
                return r
            r += step
        return max_r


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


def side_dists(grid, x, y, yaw):
    """nearest wall on each side — mirrors path_follow._on_cloud windows."""
    def sector(a_lo, a_hi):
        best = 9.9
        for a in np.linspace(a_lo, a_hi, 12):
            best = min(best, grid.raycast(x, y, yaw + a))
        return best
    d_right = sector(-2.10, -1.05)
    d_left = sector(1.05, 2.10)
    return (d_left if d_left < 9.8 else None), (d_right if d_right < 9.8 else None)


# ------------------------------------------------------------------------- sim loop
def simulate(grid, path, cfg, loc="perfect", loc_hz=3.0, noise=0.0, dt=0.1, max_t=2000.0):
    # start at path[0], facing along the path
    x, y = float(path[0][0]), float(path[0][1])
    yaw = math.atan2(path[5][1] - path[0][1], path[5][0] - path[0][0])
    idx = 0
    traj, states = [], []
    seen_pose = (x, y, yaw)
    last_loc_t = -1e9
    collisions = 0
    rng = np.random.default_rng(0)
    t = 0.0
    while t < max_t:
        # what pose does the CONTROLLER see?
        if loc == "perfect":
            seen_pose = (x, y, yaw)
        else:  # lowrate / noisy: refresh only every 1/loc_hz s
            if t - last_loc_t >= 1.0 / loc_hz:
                ny = yaw + (rng.normal(0, noise) if loc == "noisy" else 0.0)
                seen_pose = (x, y, ny)
                last_loc_t = t
        d_left, d_right = side_dists(grid, x, y, yaw)
        v, w, idx, reached, st, alpha = cc.plan(seen_pose, path, idx, d_left, d_right, cfg)
        if reached:
            states.append("GOAL")
            traj.append((x, y))
            break
        # noisy localization: a fast turn injects extra yaw error next refresh
        if loc == "noisy":
            noise = 0.06 + 0.5 * abs(w)            # spinning -> SLAM drifts more
        # integrate skid-steer kinematics (ground truth)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
        if grid.is_wall(x, y):
            collisions += 1
        traj.append((x, y))
        states.append(st)
        t += dt
    return np.array(traj), states, reached, collisions, idx, t


# ---------------------------------------------------------------------------- render
def render(grid, path, traj, states, out_png, title):
    fig, ax = plt.subplots(figsize=(12, 8))
    ext = [grid.ox, grid.ox + grid.W * grid.res, grid.oy, grid.oy + grid.H * grid.res]
    ax.imshow(grid.a, cmap="gray", extent=ext, origin="upper")
    ax.plot(path[:, 0], path[:, 1], "-", color="deepskyblue", lw=1.5, label="taught path")
    if len(traj):
        st = np.array(states[:len(traj)])
        ax.plot(traj[:, 0], traj[:, 1], "-", color="red", lw=2.0, label="robot (sim)")
        turn = traj[st == "TURN"] if len(st) == len(traj) else traj[:0]
        if len(turn):
            ax.plot(turn[:, 0], turn[:, 1], ".", color="orange", ms=3, label="cornering")
        ax.plot(traj[0, 0], traj[0, 1], "go", ms=10, label="start")
        ax.plot(traj[-1, 0], traj[-1, 1], "ks", ms=9, label="end")
    ax.set_title(title)
    ax.legend(loc="upper right")
    ax.set_aspect("equal")
    # zoom to the path bounds + margin
    ax.set_xlim(path[:, 0].min() - 3, path[:, 0].max() + 3)
    ax.set_ylim(path[:, 1].min() - 3, path[:, 1].max() + 3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    print("saved", out_png)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=os.path.join(RESULTS, "loopclosed_map.yaml"))
    ap.add_argument("--path", default=os.path.join(RESULTS, "teach_path.csv"))
    ap.add_argument("--loc", choices=["perfect", "lowrate", "noisy"], default="perfect")
    ap.add_argument("--loc-hz", type=float, default=3.0)
    ap.add_argument("--cruise", type=float, default=0.18)
    ap.add_argument("--out", default=os.path.join(HERE, "sim_result.png"))
    args = ap.parse_args()

    grid = Grid(args.map)
    path = load_path(args.path)
    print(f"map {grid.W}x{grid.H} res={grid.res} origin=({grid.ox},{grid.oy}); "
          f"path {len(path)} wp  x[{path[:,0].min():.1f},{path[:,0].max():.1f}] "
          f"y[{path[:,1].min():.1f},{path[:,1].max():.1f}]")
    cfg = cc.Cfg(cruise=args.cruise)
    traj, states, reached, coll, idx, t = simulate(grid, path, cfg, loc=args.loc, loc_hz=args.loc_hz)
    print(f"loc={args.loc}: reached={reached} wp={idx}/{len(path)} sim_t={t:.0f}s "
          f"collisions={coll} traj_pts={len(traj)}")
    render(grid, path, traj, states, args.out,
           f"corridor sim [loc={args.loc}]  reached={reached} wp={idx}/{len(path)} collisions={coll}")


if __name__ == "__main__":
    main()
