#!/usr/bin/env python3
"""Beauty shot: the FINISHED figure-8 map rotating slowly (height-coloured),
camera orbiting the building at a fixed elevation. Loops seamlessly.
Headless: numpy + matplotlib colormap, frames piped to ffmpeg (libx264).
Run with conda python.
"""
import subprocess, numpy as np
from matplotlib import cm

CACHE  = 'projects/03_autonomy/results/video/cache.npz'
OUT    = 'projects/03_autonomy/results/video/lidar_orbit.mp4'
FFMPEG = '/home/nus-ai/miniconda3/envs/intel_ai/bin/ffmpeg'

W, H, FPS = 1920, 1080, 30
NFRAMES = 450                      # 15 s, one full revolution
ELEV    = np.deg2rad(33.0)
HFOV    = np.deg2rad(62.0)
BG      = np.array([8, 9, 14], np.uint8)

d = np.load(CACHE)
vox = d['vox'].astype(np.float64)
z = vox[:, 2]
zlo, zhi = np.percentile(z, 2), np.percentile(z, 98)
col = (cm.turbo(np.clip((z - zlo)/(zhi - zlo), 0, 1))[:, :3] * 255).astype(np.uint8)

c = vox.mean(0)
Dh = np.percentile(np.linalg.norm(vox[:, :2] - c[:2], axis=1), 99.5)
dist = 2.15 * Dh                                   # 3-D camera distance (auto-fit)
R = dist * np.cos(ELEV)
zc_cam = c[2] + dist * np.sin(ELEV)
focal = (W / 2) / np.tan(HFOV / 2)
WUP = np.array([0, 0, 1.0])

# depth-sort once is impossible (changes with view); sort per frame.
cmd = [FFMPEG, '-y', '-hide_banner', '-loglevel', 'error',
       '-f', 'rawvideo', '-pixel_format', 'rgb24', '-video_size', f'{W}x{H}',
       '-framerate', str(FPS), '-i', '-',
       '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '23',
       '-preset', 'slow', '-movflags', '+faststart', OUT]
pipe = subprocess.Popen(cmd, stdin=subprocess.PIPE)

for fr in range(NFRAMES):
    th = 2*np.pi * fr / NFRAMES
    eye = np.array([c[0] + R*np.cos(th), c[1] + R*np.sin(th), zc_cam])
    f = c - eye; f /= np.linalg.norm(f)
    r_ = np.cross(f, WUP); r_ /= np.linalg.norm(r_)
    u_ = np.cross(r_, f)
    rel = vox - eye
    xc = rel @ r_; yc = rel @ u_; zz = rel @ f
    vis = zz > 0.5
    px = (W/2 + focal * xc[vis]/zz[vis])
    py = (H/2 - focal * yc[vis]/zz[vis])
    order = np.argsort(-zz[vis])
    px = np.round(px[order]).astype(np.int32)
    py = np.round(py[order]).astype(np.int32)
    cc = col[vis][order]
    img = np.empty((H, W, 3), np.uint8); img[:] = BG
    for ax in (0, 1):
        for ay in (0, 1):
            xx, yy = px+ax, py+ay
            m = (xx>=0)&(xx<W)&(yy>=0)&(yy<H)
            img[yy[m], xx[m]] = cc[m]
    pipe.stdin.write(img.tobytes())
    if fr % 60 == 0:
        print(f"  orbit {fr}/{NFRAMES}", flush=True)

pipe.stdin.close(); pipe.wait()
print(f"{OUT}  {NFRAMES} frames @ {FPS}fps")
