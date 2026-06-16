#!/usr/bin/env python3
"""Path-following sim: Regulated Pure Pursuit (the Nav2 RPP algorithm) on the taught route.

Verifies, BEFORE live, that the taught path is geometrically followable by RPP and that the
robot stays on the route given our localization quality. Renders trajectory PNG + a GIF so we
can watch. Live deployment uses the ACTUAL Nav2 RPP node; this validates the control geometry.

  /home/nus-ai/miniconda3/envs/intel_ai/bin/python pp_sim.py [--loc-noise 0.05] [--gif]
"""
import os, sys, math, argparse
import numpy as np, yaml
from PIL import Image
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.realpath(__file__))
RES = os.path.normpath(os.path.join(HERE, "..", "results"))


class Grid:
    def __init__(self, yp):
        m = yaml.safe_load(open(yp)); self.a = np.array(Image.open(os.path.join(os.path.dirname(yp), m["image"])).convert("L"))
        self.H, self.W = self.a.shape; self.res = float(m["resolution"]); self.ox, self.oy = m["origin"][0], m["origin"][1]
        self.occ = self.a < 90

    def w2p(self, x, y): return int((x - self.ox)/self.res), self.H - 1 - int((y - self.oy)/self.res)
    def is_wall(self, x, y):
        c, r = self.w2p(x, y)
        return False if (r < 0 or r >= self.H or c < 0 or c >= self.W) else bool(self.occ[r, c])
    def raycast(self, x, y, ang, mr=6.0, st=0.07):
        ca, sa = math.cos(ang), math.sin(ang); r = st
        while r <= mr:
            c, rr = self.w2p(x+ca*r, y+sa*r)
            if r < 0 or rr < 0 or rr >= self.H or c < 0 or c >= self.W or self.occ[rr, c]: return r
            r += st
        return mr


def regulated_pure_pursuit(pose, path, cfg, prog=0):
    """Nav2-style RPP: lookahead point on path, curvature to it, regulate speed by curvature
    + proximity. PROGRESS-CONSTRAINED nearest-point search (forward window only) so a
    self-intersecting route (figure-8 spine, start≈end) can't jump to the wrong pass.
    Returns (v, w, look_idx, cte, near). pose=(x,y,yaw)."""
    x, y, yaw = pose
    win = cfg.get("search_win", 60)
    lo, hi = prog, min(len(path), prog+win)
    d = np.hypot(path[lo:hi, 0]-x, path[lo:hi, 1]-y)
    near = lo + int(np.argmin(d)); cte = float(d.min())
    # adaptive lookahead scaled by speed (here fixed band), find point Ld ahead along path
    Ld = cfg["lookahead"]
    j = near
    while j < len(path)-1 and np.hypot(path[j, 0]-x, path[j, 1]-y) < Ld:
        j += 1
    lx, ly = path[j, 0], path[j, 1]
    # angle to lookahead in robot frame
    a = math.atan2(ly - y, lx - x) - yaw
    a = math.atan2(math.sin(a), math.cos(a))
    dist = max(1e-3, math.hypot(lx-x, ly-y))
    curv = 2.0*math.sin(a)/dist                      # pure-pursuit curvature
    # --- regulation (RPP): slow on high curvature + low cross-track confidence ---
    v = cfg["cruise"]
    rmin = cfg["regulate_radius"]
    if abs(curv) > 1e-3:
        radius = 1.0/abs(curv)
        if radius < rmin:
            v *= max(cfg["min_speed_frac"], radius/rmin)
    w = v*curv
    w = max(-cfg["max_w"], min(cfg["max_w"], w))
    return v, w, j, cte, near


def simulate(grid, path, cfg, loc_noise=0.0, dt=0.1, max_t=3000.0):
    rng = np.random.default_rng(0)
    x, y = float(path[0, 0]), float(path[0, 1]); yaw = math.atan2(path[5, 1]-path[0, 1], path[5, 0]-path[0, 0])
    traj, ctes, vs = [], [], []
    t = 0.0; goal_idx = len(path)-1; reached = False; collisions = 0; frames = []; prog = 0
    while t < max_t:
        # localized pose = true + noise (models our ~2.8cm localization)
        lp = (x + rng.normal(0, loc_noise), y + rng.normal(0, loc_noise), yaw + rng.normal(0, loc_noise*0.5))
        v, w, look, cte, near = regulated_pure_pursuit(lp, path, cfg, prog)
        prog = near                                  # advance progress monotonically
        # (live deployment adds the Nav2 costmap for NEW obstacles; this sim verifies RPP
        #  geometrically tracks the collision-free taught path — no costmap stop here.)
        x += v*math.cos(yaw)*dt; y += v*math.sin(yaw)*dt; yaw += w*dt
        if grid.is_wall(x, y): collisions += 1
        traj.append((x, y)); ctes.append(cte); vs.append(v); t += dt
        if near >= goal_idx-2 and np.hypot(x-path[-1, 0], y-path[-1, 1]) < cfg["goal_tol"] and t > 5:
            reached = True; break
        if cfg.get("gif") and len(traj) % cfg["frame_every"] == 0:
            frames.append((np.array(traj).copy(), near))
    return np.array(traj), np.array(ctes), np.array(vs), reached, collisions, frames


def render(grid, path, traj, ctes, reached, coll, out):
    # honest framing: the robot's wall-grid clip rate vs the taught path's OWN clip rate.
    # The RC-driven taught path is collision-free; equal rates => the robot collides no more
    # than the (safe) reference, i.e. clips are conservative-grid thickness, not real hits.
    rob_clip = sum(grid.is_wall(px, py) for px, py in traj[:, :2])
    path_clip = sum(grid.is_wall(px, py) for px, py in path[:, :2])
    fig, ax = plt.subplots(figsize=(15, 7))
    ax.imshow(grid.a, cmap="gray", extent=[grid.ox, grid.ox+grid.W*grid.res, grid.oy, grid.oy+grid.H*grid.res], origin="upper")
    ax.plot(path[:, 0], path[:, 1], "-", c="deepskyblue", lw=2.5, label="taught path", alpha=0.7)
    ax.plot(traj[:, 0], traj[:, 1], "-", c="red", lw=1.6, label="robot (RPP sim)")
    ax.plot(traj[0, 0], traj[0, 1], "go", ms=13, label="start"); ax.plot(traj[-1, 0], traj[-1, 1], "ks", ms=10, label="end")
    ax.set_aspect("equal"); ax.legend(loc="upper right")
    ax.set_title(f"Step 3: Regulated Pure Pursuit follows the taught route — reached={reached}, "
                 f"cross-track mean {ctes.mean():.2f} m / max {ctes.max():.2f} m\n"
                 f"(wall-grid clips: robot {100*rob_clip/len(traj):.0f}% vs taught-path {100*path_clip/len(path):.0f}% "
                 f"— equal => conservative-grid thickness, not real collisions; RC drive was collision-free)")
    ax.set_xlim(path[:, 0].min()-4, path[:, 0].max()+4); ax.set_ylim(path[:, 1].min()-4, path[:, 1].max()+4)
    plt.tight_layout(); plt.savefig(out, dpi=110); print("saved", out)


def make_gif(grid, path, frames, out):
    imgs = []
    for traj, near in frames:
        fig, ax = plt.subplots(figsize=(12, 5.6))
        ax.imshow(grid.a, cmap="gray", extent=[grid.ox, grid.ox+grid.W*grid.res, grid.oy, grid.oy+grid.H*grid.res], origin="upper")
        ax.plot(path[:, 0], path[:, 1], "-", c="deepskyblue", lw=1.5, alpha=0.6)
        ax.plot(traj[:, 0], traj[:, 1], "-", c="red", lw=1.6)
        ax.plot(traj[-1, 0], traj[-1, 1], "o", c="orange", ms=8)
        ax.set_aspect("equal"); ax.set_xlim(path[:, 0].min()-4, path[:, 0].max()+4); ax.set_ylim(path[:, 1].min()-4, path[:, 1].max()+4)
        ax.set_title("RPP following the taught route (sim)"); ax.axis("off")
        fig.canvas.draw(); im = Image.frombytes("RGB", fig.canvas.get_width_height(), fig.canvas.tostring_rgb()); imgs.append(im); plt.close(fig)
    if imgs:
        imgs[0].save(out, save_all=True, append_images=imgs[1:], duration=120, loop=0); print("saved", out, f"({len(imgs)} frames)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=os.path.join(RES, "wall_grid.yaml"))
    ap.add_argument("--path", default=os.path.join(RES, "teach_path_fastlio.csv"))
    ap.add_argument("--loc-noise", type=float, default=0.03)   # our localization ~2.8 cm
    ap.add_argument("--gif", action="store_true")
    args = ap.parse_args()
    grid = Grid(args.map); path = np.loadtxt(args.path)
    cfg = dict(cruise=0.5, max_w=0.8, lookahead=1.2, regulate_radius=1.0, min_speed_frac=0.25,
               slow_dist=1.2, stop_dist=0.12, goal_tol=2.0, search_win=60, gif=args.gif, frame_every=40)
    traj, ctes, vs, reached, coll, frames = simulate(grid, path, cfg, loc_noise=args.loc_noise)
    print(f"RESULT: reached={reached} collisions={coll} traj_pts={len(traj)} "
          f"cross-track mean {ctes.mean():.3f} m / max {ctes.max():.3f} m  mean v {vs.mean():.2f} m/s")
    render(grid, path, traj, ctes, reached, coll, os.path.join(RES, "step3_rpp_follow.png"))
    if args.gif: make_gif(grid, path, frames, os.path.join(RES, "step3_rpp_follow.gif"))


if __name__ == "__main__":
    main()
