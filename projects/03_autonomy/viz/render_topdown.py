#!/usr/bin/env python3
"""Render a single high-quality top-down proof frame of the full accumulated
LiDAR map (height-coloured), with the driven trajectory overlaid.
Headless: numpy + matplotlib colormap + PIL. No GPU, no display.
"""
import numpy as np
from matplotlib import cm
from PIL import Image

CACHE = 'projects/03_autonomy/results/video/cache.npz'
OUT   = 'projects/03_autonomy/results/video/proof_topdown.png'
W     = 2400
BG    = np.array([8, 9, 14], np.uint8)

d = np.load(CACHE)
vox, traj = d['vox'], d['traj']

# height colour (turbo) with robust z range
z = vox[:, 2]
zlo, zhi = np.percentile(z, 2), np.percentile(z, 98)
norm = np.clip((z - zlo) / max(zhi - zlo, 1e-6), 0, 1)
col = (cm.turbo(norm)[:, :3] * 255).astype(np.uint8)

# draw low points first so higher structures sit on top
o = np.argsort(z)
vox, col = vox[o], col[o]

x, y = vox[:, 0], vox[:, 1]
pad = 0.03
xmin, xmax = x.min(), x.max(); ymin, ymax = y.min(), y.max()
dx, dy = xmax - xmin, ymax - ymin
xmin -= pad*dx; xmax += pad*dx; ymin -= pad*dy; ymax += pad*dy
dx, dy = xmax - xmin, ymax - ymin
s = (W - 1) / dx
H = int(round((ymax - ymin) * s)) + 1
img = np.empty((H, W, 3), np.uint8); img[:] = BG

px = ((x - xmin) * s).astype(np.int32)
py = ((ymax - y) * s).astype(np.int32)
for ax in (-1, 0, 1):
    for ay in (-1, 0, 1):
        xx, yy = px + ax, py + ay
        m = (xx >= 0) & (xx < W) & (yy >= 0) & (yy < H)
        img[yy[m], xx[m]] = col[m]

# trajectory as a thin white path
tx = ((traj[:, 0] - xmin) * s).astype(np.int32)
ty = ((ymax - traj[:, 1]) * s).astype(np.int32)
m = (tx >= 0) & (tx < W) & (ty >= 0) & (ty < H)
img[ty[m], tx[m]] = (255, 255, 255)

Image.fromarray(img).save(OUT)
print(f"{OUT}  {W}x{H}  voxels={len(vox):,}")
