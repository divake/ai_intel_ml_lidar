#!/usr/bin/env python3
"""WIDE 'map draws itself' top-down timelapse from the cached voxel map.
The figure-8 grows as the robot drives; a bright marker + trail show the robot.
Headless: numpy + matplotlib colormap + cv2 VideoWriter.  Run with conda python.
"""
import sys, numpy as np, cv2
from matplotlib import cm

CACHE = 'projects/03_autonomy/results/video/cache.npz'
OUT   = 'projects/03_autonomy/results/video/lidar_wide_timelapse.mp4'
W     = 1920
FPS   = 30
SECS  = 42                 # target video length
BG    = np.array([8, 9, 14], np.uint8)

d = np.load(CACHE)
vox, first, traj = d['vox'], d['first'], d['traj']
nscans = int(d['nscans'])

# ---- static top-down projection of every voxel (computed once) ----------
z = vox[:, 2]
zlo, zhi = np.percentile(z, 2), np.percentile(z, 98)
norm = np.clip((z - zlo) / max(zhi - zlo, 1e-6), 0, 1)
rgb = (cm.turbo(norm)[:, :3] * 255).astype(np.uint8)
bgr = rgb[:, ::-1].copy()                                  # cv2 wants BGR

x, y = vox[:, 0], vox[:, 1]
pad = 0.03
xmin, xmax = x.min(), x.max(); ymin, ymax = y.min(), y.max()
dx, dy = xmax - xmin, ymax - ymin
xmin -= pad*dx; xmax += pad*dx; ymin -= pad*dy; ymax += pad*dy
dx, dy = xmax - xmin, ymax - ymin
s = (W - 1) / dx
H = int(round((ymax - ymin) * s)) + 1
H += H % 2                                                  # even height for codec

def to_px(wx, wy):
    return (((wx - xmin) * s).astype(np.int32),
            ((ymax - wy) * s).astype(np.int32))

px, py = to_px(x, y)

# order voxels by first-seen so we can paint incrementally
o = np.argsort(first, kind='stable')
px, py, bgr, first = px[o], py[o], bgr[o], first[o]
tpx, tpy = to_px(traj[:, 0], traj[:, 1])

# ---- incremental paint ---------------------------------------------------
canvas = np.empty((H, W, 3), np.uint8); canvas[:] = BG[::-1]
nframes = FPS * SECS
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
vw = cv2.VideoWriter(OUT, fourcc, FPS, (W, H))
if not vw.isOpened():
    sys.exit("VideoWriter failed to open")

OFF = [(ax, ay) for ax in (-1, 0, 1) for ay in (-1, 0, 1)]
ptr = 0
for f in range(nframes):
    cutoff = int(round((f + 1) / nframes * nscans))
    nxt = np.searchsorted(first, cutoff, side='right')
    if nxt > ptr:                                          # splat newly-seen voxels
        sx, sy, sc = px[ptr:nxt], py[ptr:nxt], bgr[ptr:nxt]
        for ax, ay in OFF:
            xx, yy = sx + ax, sy + ay
            m = (xx >= 0) & (xx < W) & (yy >= 0) & (yy < H)
            canvas[yy[m], xx[m]] = sc[m]
        ptr = nxt
    frame = canvas.copy()
    # robot trail (faint) + recent (brighter) + head dot
    c = max(cutoff, 1)
    cv2.polylines(frame, [np.stack([tpx[:c], tpy[:c]], 1)], False, (90, 90, 90), 1, cv2.LINE_AA)
    r0 = max(0, c - 120)
    cv2.polylines(frame, [np.stack([tpx[r0:c], tpy[r0:c]], 1)], False, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.circle(frame, (int(tpx[c-1]), int(tpy[c-1])), 7, (0, 230, 255), -1, cv2.LINE_AA)
    cv2.circle(frame, (int(tpx[c-1]), int(tpy[c-1])), 7, (255, 255, 255), 1, cv2.LINE_AA)
    vw.write(frame)

vw.release()
print(f"{OUT}  {W}x{H}  {nframes} frames @ {FPS}fps  (~{SECS}s)")
