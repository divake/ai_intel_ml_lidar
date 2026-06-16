#!/usr/bin/env python3
"""Premium-look top-down still: supersampled (anti-aliased), soft additive
round points (glow), bloom post, dark gradient background. Headless CPU.
Run with conda python (numpy + PIL + matplotlib).
"""
import numpy as np
from matplotlib import cm
from PIL import Image, ImageFilter

CACHE = 'projects/03_autonomy/results/video/cache.npz'
OUT   = 'projects/03_autonomy/results/video/premium_still.png'
W     = 1920
SS    = 2                       # supersample factor (anti-aliasing)
Wb    = W * SS

d = np.load(CACHE)
vox = d['vox'].astype(np.float32)
z = vox[:, 2]
zlo, zhi = np.percentile(z, 2), np.percentile(z, 98)
norm = np.clip((z - zlo) / max(zhi - zlo, 1e-6), 0, 1)
col = cm.turbo(norm)[:, :3].astype(np.float32)          # 0..1

x, y = vox[:, 0], vox[:, 1]
pad = 0.03
xmin, xmax = x.min(), x.max(); ymin, ymax = y.min(), y.max()
dx, dy = xmax-xmin, ymax-ymin
xmin -= pad*dx; xmax += pad*dx; ymin -= pad*dy; ymax += pad*dy
dx, dy = xmax-xmin, ymax-ymin
s = (Wb-1)/dx
Hb = int(round((ymax-ymin)*s)) + 1
px = ((x-xmin)*s)
py = ((ymax-y)*s)

# soft round additive splat (5x5 gaussian kernel)
buf = np.zeros((Hb, Wb, 3), np.float32)
ix = np.round(px).astype(np.int32); iy = np.round(py).astype(np.int32)
rad = 2
ker = []
for ax in range(-rad, rad+1):
    for ay in range(-rad, rad+1):
        w = np.exp(-(ax*ax+ay*ay)/2.0)
        ker.append((ax, ay, w))
for ax, ay, w in ker:
    xx, yy = ix+ax, iy+ay
    m = (xx>=0)&(xx<Wb)&(yy>=0)&(yy<Hb)
    np.add.at(buf, (yy[m], xx[m]), col[m]*w)

# tonemap (HDR-ish) so dense walls glow but don't clip harshly
k = 1.0 / max(np.percentile(buf[buf>0], 60), 1e-3)
tm = 1.0 - np.exp(-buf * k * 1.3)
img8 = (np.clip(tm, 0, 1) * 255).astype(np.uint8)

# bloom: blur a bright-pass and screen-add
im = Image.fromarray(img8)
lum = img8.max(2)
bright = (img8 * (lum[..., None] > 90)).astype(np.uint8)
bloom = Image.fromarray(bright).filter(ImageFilter.GaussianBlur(SS*5))
a = img8.astype(np.float32); b = np.asarray(bloom, np.float32)*0.8
screen = 255.0 - (255.0-a)*(255.0-b)/255.0
out = np.clip(screen, 0, 255).astype(np.uint8)

# subtle vignette + dark gradient bg already (black). downscale for AA.
final = Image.fromarray(out).resize((W, Hb//SS), Image.LANCZOS)
final.save(OUT)
print(f"{OUT}  {final.size}")
