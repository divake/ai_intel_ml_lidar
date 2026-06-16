#!/usr/bin/env python3
"""Hero STILL of the corridor2.0 map in all 3 palettes, via the CUDA rasterizer.
Validation that the GPU path produces a premium frame.

  /home/divake/miniconda3/envs/env_py311/bin/python gpu_still.py
"""
import os, sys, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import common as C
from gpu_raster import Renderer
import matplotlib.image as mpimg

W, H, SS = 2560, 1440, 2
os.makedirs(C.OUTDIR, exist_ok=True)

d = C.load_cache(); vox = d["vox"]
zlo, zhi = np.percentile(vox[:, 2], 2), np.percentile(vox[:, 2], 98)
ctr = vox.mean(0)
ext = np.percentile(np.linalg.norm(vox[:, :2] - ctr[:2], axis=1), 99.5)
eye = ctr + np.array([0.10 * ext, -1.25 * ext, 1.05 * ext], np.float32)

R = Renderer(W, H, ssaa=SS)
for pal in ("turbo", "intensity", "ice"):
    rgb = C.colorize(vox, d["inten"], pal, zlo=zlo, zhi=zhi)
    R.set_cloud(vox, rgb)
    bg = tuple(C.BG[pal])
    img = R.render(eye, ctr, up=(0, 0, 1), fov_deg=46, bg=bg,
                   point_px=2.0, edl=0.9, bloom=0.8)
    out = os.path.join(C.OUTDIR, f"still_{pal}.png")
    mpimg.imsave(out, img)
    print(f"  wrote {out}  {img.shape[1]}x{img.shape[0]}")
print("DONE")
