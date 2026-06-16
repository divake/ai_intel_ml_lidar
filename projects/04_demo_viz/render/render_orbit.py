#!/usr/bin/env python3
"""STYLE 2 - Orbit beauty (hero). Finished map rotates slowly; CUDA rasterizer
with eye-dome lighting + bloom. One ~6 s sample per palette.

  /home/divake/miniconda3/envs/env_py311/bin/python render_orbit.py [seconds]
"""
import os, sys, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import common as C
from gpu_raster import Renderer

W, H, FPS = 1920, 1080, 30
SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
N = int(SECS * FPS)

d = C.load_cache(); vox = d["vox"]
zlo, zhi = np.percentile(vox[:, 2], 2), np.percentile(vox[:, 2], 98)
ctr = vox.mean(0)
ext = np.percentile(np.linalg.norm(vox[:, :2] - ctr[:2], axis=1), 99.5)
dist, elev = 1.9 * ext, np.deg2rad(36.0)
Rd, zc = dist * np.cos(elev), ctr[2] + dist * np.sin(elev)

R = Renderer(W, H, ssaa=2)
for pal in ("turbo", "intensity", "ice"):
    rgb = C.colorize(vox, d["inten"], pal, zlo=zlo, zhi=zhi)
    R.set_cloud(vox, rgb)
    bg = tuple(C.BG[pal])
    out = os.path.join(C.OUTDIR, f"orbit_{pal}.mp4")
    vw = C.FFWriter(out, W, H, FPS)
    for fr in range(N):
        th = 2 * np.pi * fr / N - np.pi / 2
        eye = [ctr[0] + Rd * np.cos(th), ctr[1] + Rd * np.sin(th), zc]
        img = R.render(eye, ctr, up=(0, 0, 1), fov_deg=45, bg=bg, point_px=2.0, edl=0.9, bloom=0.8)
        vw.write(img)
    vw.close()
    print(f"  wrote {out}  ({N} frames)")
print("DONE")
