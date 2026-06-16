#!/usr/bin/env python3
"""Closed-loop sim for the EYES-ONLY controller + route-memory router (sim == robot).

Ray-casts a LiDAR fan against the occupancy grid, builds the SAME polar scan the robot's
cloud yields, runs corridor_eyes.beams_from_scan + EyesController (golden law + router),
integrates skid-steer, and renders the trajectory over the map. This validates the
route-memory junction logic on the REAL figure-8 BEFORE any hardware.

  /home/nus-ai/miniconda3/envs/intel_ai/bin/python junction_sim.py            # eyes-only baseline (guesses)
  /home/nus-ai/miniconda3/envs/intel_ai/bin/python junction_sim.py --route L,R,R,L
"""
import os, sys, math, argparse
import numpy as np
import yaml
from PIL import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "locomotion"))
import corridor_eyes as ce

RESULTS = os.path.normpath(os.path.join(HERE, "..", "results"))


class Grid:
    def __init__(self, yaml_path):
        m = yaml.safe_load(open(yaml_path))
        img = Image.open(os.path.join(os.path.dirname(yaml_path), m["image"])).convert("L")
        self.a = np.array(img)
        self.H, self.W = self.a.shape
        self.res = float(m["resolution"])
        self.ox, self.oy = m["origin"][0], m["origin"][1]
        self.occ = self.a < 90

    def w2p(self, x, y):
        c = int((x - self.ox) / self.res)
        r = self.H - 1 - int((y - self.oy) / self.res)
        return c, r

    def occupied(self, x, y):
        c, r = self.w2p(x, y)
        if r < 0 or r >= self.H or c < 0 or c >= self.W:
            return True
        return bool(self.occ[r, c])

    def is_wall(self, x, y):
        c, r = self.w2p(x, y)
        if r < 0 or r >= self.H or c < 0 or c >= self.W:
            return False
        return bool(self.occ[r, c])

    def raycast(self, x, y, ang, max_r=8.0, step=0.05):
        ca, sa = math.cos(ang), math.sin(ang)
        r = step
        while r <= max_r:
            if self.occupied(x + ca * r, y + sa * r):
                return r
            r += step
        return max_r


def load_path(fn):
    pts = []
    for line in open(fn):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        p = line.replace(",", " ").split()
        pts.append((float(p[0]), float(p[1])))
    return np.array(pts)


def scan(grid, x, y, yaw, fov_deg=150.0, n=151, max_r=8.0, step=0.07):
    """Synthetic polar scan in the robot frame (VECTORIZED ray-cast): angles in [-fov,fov],
    ranges from stepping all rays outward together against the occupancy grid."""
    angs = np.linspace(-fov_deg * ce.DEG, fov_deg * ce.DEG, n)
    ca, sa = np.cos(yaw + angs), np.sin(yaw + angs)
    rng = np.full(n, max_r)
    hit = np.zeros(n, dtype=bool)
    r = step
    while r <= max_r:
        px, py = x + ca * r, y + sa * r
        c = ((px - grid.ox) / grid.res).astype(int)
        rr = grid.H - 1 - ((py - grid.oy) / grid.res).astype(int)
        inb = (rr >= 0) & (rr < grid.H) & (c >= 0) & (c < grid.W)
        occ = ~inb                                  # off-map counts as occupied
        ii = inb & ~hit
        if ii.any():
            occ[ii] = grid.occ[rr[ii], c[ii]]
        newhit = (~hit) & occ
        rng[newhit] = r
        hit |= occ
        if hit.all():
            break
        r += step
    good = (rng > 0.4) & (rng < max_r - 1e-6)
    return angs[good], rng[good]


def simulate(grid, path, route, dt=0.1, max_t=4000.0, max_r=8.0, start_idx=0, params=None):
    si = start_idx
    x, y = float(path[si][0]), float(path[si][1])
    yaw = math.atan2(path[si + 5][1] - path[si][1], path[si + 5][0] - path[si][0])
    ctrl = ce.EyesController(params or ce.EyesParams(), route=route)
    traj, states = [], []
    s_travel = 0.0
    collisions = 0
    t = 0.0
    started_away = False
    while t < max_t:
        ang, rng = scan(grid, x, y, yaw, max_r=max_r)
        if ang.size == 0:
            v, w, st = 0.0, 0.0, "NO_WALLS_STOP"
        else:
            Dl, Dr, phi, front, oL, oR = ce.beams_from_scan(ang, rng, ctrl.p)
            v, w, st = ctrl.step(Dl, Dr, phi, front, oL, oR, s_travel)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
        s_travel += abs(v) * dt
        if grid.is_wall(x, y):
            collisions += 1
        traj.append((x, y)); states.append(st)
        # loop-complete: traveled enough, took all route turns, back near start
        d_start = math.hypot(x - path[0][0], y - path[1][1])
        if s_travel > 8.0:
            started_away = True
        if (started_away and ctrl.jcount >= len(route) and len(route) > 0
                and math.hypot(x - path[0][0], y - path[0][1]) < 2.0 and s_travel > 200.0):
            states[-1] = "GOAL"
            break
        t += dt
    return np.array(traj), states, collisions, ctrl, s_travel, t


def path_coverage(path, traj):
    """For each taught waypoint, nearest distance to the robot trajectory. Tells us how far
    along the route the robot actually tracked before diverging."""
    if len(traj) == 0:
        return 0.0, 0.0, 0.0
    covered = []
    for px, py in path:
        d = np.min(np.hypot(traj[:, 0] - px, traj[:, 1] - py))
        covered.append(d)
    covered = np.array(covered)
    frac = float(np.mean(covered < 1.5))
    return float(covered.mean()), float(covered.max()), frac


def render(grid, path, traj, states, ctrl, out_png, title):
    fig, ax = plt.subplots(figsize=(13, 8))
    ext = [grid.ox, grid.ox + grid.W * grid.res, grid.oy, grid.oy + grid.H * grid.res]
    ax.imshow(grid.a, cmap="gray", extent=ext, origin="upper")
    ax.plot(path[:, 0], path[:, 1], "-", color="deepskyblue", lw=1.5, label="taught path")
    if len(traj):
        st = np.array(states[:len(traj)])
        ax.plot(traj[:, 0], traj[:, 1], "-", color="red", lw=1.8, label="robot (sim)")
        turn = traj[np.char.startswith(st.astype(str), "TURN")] if len(st) == len(traj) else traj[:0]
        if len(turn):
            ax.plot(turn[:, 0], turn[:, 1], ".", color="orange", ms=3, label="turning")
        ax.plot(traj[0, 0], traj[0, 1], "go", ms=11, label="start")
        ax.plot(traj[-1, 0], traj[-1, 1], "ks", ms=9, label="end")
    ax.set_title(title)
    ax.legend(loc="upper right")
    ax.set_aspect("equal")
    ax.set_xlim(path[:, 0].min() - 3, path[:, 0].max() + 3)
    ax.set_ylim(path[:, 1].min() - 3, path[:, 1].max() + 3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    print("saved", out_png)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=os.path.join(RESULTS, "nav_grid.yaml"))
    ap.add_argument("--path", default=os.path.join(RESULTS, "teach_path_fastlio.csv"))
    ap.add_argument("--route", default="", help="comma turns at junctions e.g. L,R,R,L")
    ap.add_argument("--start-idx", type=int, default=0, help="path index to start at (skip junction clutter)")
    ap.add_argument("--jopen", type=float, default=2.2, help="both-sides-open threshold (m) for a junction")
    ap.add_argument("--out", default=os.path.join(HERE, "junction_sim.png"))
    args = ap.parse_args()

    route = []
    if args.route.strip():
        route = [(+1 if c.strip().upper() == "L" else -1) for c in args.route.split(",")]

    grid = Grid(args.map)
    path = load_path(args.path)
    params = ce.EyesParams(j_open=args.jopen)
    print(f"map {grid.W}x{grid.H} res={grid.res}; path {len(path)} wp "
          f"x[{path[:,0].min():.1f},{path[:,0].max():.1f}] y[{path[:,1].min():.1f},{path[:,1].max():.1f}]")
    print(f"route (turns at junctions): {['L' if t>0 else 'R' for t in route] or '(none - eyes guess)'}")

    traj, states, coll, ctrl, s_travel, t = simulate(grid, path, route,
                                                      start_idx=args.start_idx, params=params)
    cmean, cmax, cfrac = path_coverage(path, traj)
    reached = states[-1] == "GOAL" if states else False
    print(f"\nRESULT: reached_goal={reached} sim_t={t:.0f}s travel={s_travel:.1f}m collisions={coll}")
    print(f"junctions detected: {ctrl.jcount}")
    for s, jc, td, src in ctrl.events:
        print(f"   J{jc} @ travel {s:5.1f}m -> turn {'LEFT' if td>0 else 'RIGHT'}  [{src}]")
    print(f"path coverage: mean {cmean:.2f}m  max {cmax:.2f}m  within-1.5m {cfrac*100:.0f}%")

    global ctrl_total_s
    ctrl_total_s = s_travel
    render(grid, path, traj, states, ctrl, args.out,
           f"junction sim  route={args.route or 'none'}  reached={reached} "
           f"junctions={ctrl.jcount} collisions={coll} cover={cfrac*100:.0f}%")


if __name__ == "__main__":
    main()
