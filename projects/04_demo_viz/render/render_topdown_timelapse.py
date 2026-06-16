#!/usr/bin/env python3
"""Faithful remake of lidar_wide_timelapse (top-down 'map draws itself' + robot
trail) -- but with the Z-DRIFT color gradient removed. The original colored by
raw height, so odometry Z-drift painted the left side red and the right blue.
Here we subtract the robot's own trajectory height (it drove the flat floor, so
its z IS the drift) -> floor is uniform, walls pop, no left/right gradient.

  color=height  (default, de-drifted turbo)  |  color=intensity (reflectivity)
  lidar_viz python render_topdown_timelapse.py [height|intensity] [secs]
"""
import os, sys, numpy as np, cv2
from matplotlib import cm
from scipy.spatial import cKDTree
sys.path.insert(0, os.path.dirname(__file__))
import common as C

COLOR = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].isdigit() else "height"
SECS = float([a for a in sys.argv[1:] if a.replace(".", "").isdigit()][0]) if any(
    a.replace(".", "").isdigit() for a in sys.argv[1:]) else 42.0
W, FPS = 1920, 30

d = C.load_cache()
vox, inten, first, traj, nscans = d["vox"].astype(np.float64), d["inten"], d["first"], d["traj"], d["nscans"]
x, y, z = vox[:, 0], vox[:, 1], vox[:, 2]

if COLOR == "intensity":
    ni = np.clip((np.sqrt(inten) - np.percentile(np.sqrt(inten), 3)) /
                 (np.percentile(np.sqrt(inten), 99) - np.percentile(np.sqrt(inten), 3) + 1e-6), 0, 1)
    rgb = (cm.inferno(ni)[:, :3] * 255).astype(np.uint8)
else:  # de-drifted height
    _, idx = cKDTree(traj[:, :2]).query(vox[:, :2], k=1)
    zf = z - traj[idx, 2]
    n = np.clip((zf - np.percentile(zf, 3)) / (np.percentile(zf, 97) - np.percentile(zf, 3) + 1e-6), 0, 1)
    rgb = (cm.turbo(n)[:, :3] * 255).astype(np.uint8)

# ---- top-down pixel projection (matches the original framing) ----
pad = 0.03
xmin, xmax, ymin, ymax = x.min(), x.max(), y.min(), y.max()
dx, dy = xmax - xmin, ymax - ymin
xmin -= pad * dx; xmax += pad * dx; ymin -= pad * dy; ymax += pad * dy
dx, dy = xmax - xmin, ymax - ymin
s = (W - 1) / dx
H = int(round(dy * s)) + 1; H += H % 2
def to_px(wx, wy):
    return (((wx - xmin) * s).astype(np.int32), ((ymax - wy) * s).astype(np.int32))
px, py = to_px(x, y)
tpx, tpy = to_px(traj[:, 0], traj[:, 1])

# paint in first-seen order so the map grows with the drive
o = np.argsort(first, kind="stable")
px, py, rgb, first = px[o], py[o], rgb[o], first[o]

BG = np.array([8, 9, 14], np.uint8)
canvas = np.empty((H, W, 3), np.uint8); canvas[:] = BG
out = os.path.join(C.OUTDIR, f"wide_timelapse_fixed_{COLOR}.mp4")
vw = C.FFWriter(out, W, H, FPS)           # small, PPT-friendly
nframes = int(FPS * SECS)
OFF = [(ax, ay) for ax in (-1, 0, 1) for ay in (-1, 0, 1)]
ptr = 0
for f in range(nframes):
    cutoff = int(round((f + 1) / nframes * nscans))
    nxt = int(np.searchsorted(first, cutoff, side="right"))
    if nxt > ptr:
        sx, sy, sc = px[ptr:nxt], py[ptr:nxt], rgb[ptr:nxt]
        for ax, ay in OFF:
            xx, yy = sx + ax, sy + ay
            m = (xx >= 0) & (xx < W) & (yy >= 0) & (yy < H)
            canvas[yy[m], xx[m]] = sc[m]
        ptr = nxt
    frame = canvas.copy()
    c = max(cutoff, 2)
    cv2.polylines(frame, [np.stack([tpx[:c], tpy[:c]], 1)], False, (90, 90, 90), 1, cv2.LINE_AA)
    r0 = max(0, c - 120)
    cv2.polylines(frame, [np.stack([tpx[r0:c], tpy[r0:c]], 1)], False, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.circle(frame, (int(tpx[c - 1]), int(tpy[c - 1])), 7, (255, 210, 40), -1, cv2.LINE_AA)  # RGB head dot
    cv2.circle(frame, (int(tpx[c - 1]), int(tpy[c - 1])), 7, (255, 255, 255), 1, cv2.LINE_AA)
    vw.write(frame)
vw.close()
print(f"DONE -> {out}  ({W}x{H} -> 720p, {nframes} frames, ~{SECS:.0f}s)")
