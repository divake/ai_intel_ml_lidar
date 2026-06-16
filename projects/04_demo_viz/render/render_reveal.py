#!/usr/bin/env python3
"""STYLE 1 - "The building draws itself" (signature reveal). The figure-8 paints
on voxel-by-voxel as the robot drives; a glowing white trail traces the route.
Camera starts near top-down and eases into a 3/4 tilt. CUDA rasterizer.

  /home/divake/miniconda3/envs/env_py311/bin/python render_reveal.py [seconds]
"""
import os, sys, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import common as C
from gpu_raster import Renderer

W, H, FPS = 1920, 1080, 30
SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
N = int(SECS * FPS)

d = C.load_cache()
vox, inten, first, traj, nscans = d["vox"], d["inten"], d["first"], d["traj"], d["nscans"]
zlo, zhi = np.percentile(vox[:, 2], 2), np.percentile(vox[:, 2], 98)
order = np.argsort(first, kind="stable")
vox, inten, first = vox[order], inten[order], first[order]
ctr = vox.mean(0)
ext = np.percentile(np.linalg.norm(vox[:, :2] - ctr[:2], axis=1), 99.5)


def trail_points(c):
    """densify trajectory polyline up to scan c into bright points (for the glowing trail)."""
    p = traj[:max(c, 2)].astype(np.float32)
    seg = p[1:] - p[:-1]
    L = np.linalg.norm(seg, axis=1)
    reps = np.clip((L / 0.04).astype(int), 1, 60)
    out = []
    for a, s, r in zip(p[:-1], seg, reps):
        tt = np.linspace(0, 1, r, endpoint=False)[:, None]
        out.append(a[None] + tt * s[None])
    return np.concatenate(out) if out else p


R = Renderer(W, H, ssaa=2)
for pal in ("turbo", "intensity", "ice"):
    rgb = C.colorize(vox, inten, pal, zlo=zlo, zhi=zhi)
    bg = tuple(C.BG[pal])
    out = os.path.join(C.OUTDIR, f"reveal_{pal}.mp4")
    vw = C.FFWriter(out, W, H, FPS)
    for fr in range(N):
        t = fr / max(N - 1, 1)
        cutoff = int(round(t * nscans))
        k = int(np.searchsorted(first, cutoff, side="right"))
        k = max(k, 50)
        # base map revealed so far + bright white trail
        tp = trail_points(min(cutoff, len(traj)))
        xyz = np.vstack([vox[:k], tp.astype(np.float32)])
        col = np.vstack([rgb[:k], np.tile([1.0, 1.0, 1.0], (len(tp), 1)).astype(np.float32)])
        R.set_cloud(xyz, col)
        e = C.ease(t)
        elev = np.deg2rad(85 - 47 * e)
        dist = (1.55 + 0.5 * e) * ext
        th = -np.pi / 2 + 0.55 * e
        eye = [ctr[0] + dist * np.cos(elev) * np.cos(th),
               ctr[1] + dist * np.cos(elev) * np.sin(th),
               ctr[2] + dist * np.sin(elev)]
        img = R.render(eye, ctr, up=(0, 0, 1), fov_deg=45, bg=bg, point_px=2.0, edl=0.85, bloom=0.85)
        vw.write(img)
    vw.close()
    print(f"  wrote {out}  ({N} frames)")
print("DONE")
