#!/usr/bin/env python3
"""CLOSE chase-cam: a perspective camera trails the robot; the live LiDAR scan
radiates out bright while the already-built map glows dim behind it.
Reads the bag for live per-scan points; pipes frames to ffmpeg (libx264).
Headless, no GPU.  Run with ROS python:
  source /opt/ros/jazzy/setup.bash && /usr/bin/python3 follow_cam.py [scan_lo scan_hi]
"""
import sys, subprocess, numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from matplotlib import cm

BAG    = 'projects/02_fast_lio2/results/corridor2.0'
POSES  = 'projects/03_autonomy/results/kiss_slam/poses_loopclosed.npy'
CACHE  = 'projects/03_autonomy/results/video/cache.npz'
OUT    = 'projects/03_autonomy/results/video/lidar_chase_cam.mp4'
FFMPEG = '/home/nus-ai/miniconda3/envs/intel_ai/bin/ffmpeg'

W, H, FPS = 1920, 1080, 30
HFOV  = np.deg2rad(74.0)
BACK, HEIGHT, LOOKAHEAD = 7.0, 4.2, 5.0   # camera geometry (m)
RADIUS = 22.0                              # only draw map within this of the robot
SMOOTH = 12                                # camera path smoothing (scans)
LO = int(sys.argv[1]) if len(sys.argv) > 2 else 0
HI = int(sys.argv[2]) if len(sys.argv) > 2 else 1500

PT = np.dtype({"names":["x","y","z","intensity","ring","timestamp"],
               "formats":["<f4","<f4","<f4","<f4","<u2","<f8"],
               "offsets":[0,4,8,12,16,18], "itemsize":26})

poses = np.load(POSES).astype(np.float64)
HI = min(HI, len(poses))
d = np.load(CACHE)
vox = d['vox'].astype(np.float32)

# global height colour map (shared with the wide video)
def hcol(z):
    zlo, zhi = -2.2, 3.2
    n = np.clip((z - zlo) / (zhi - zlo), 0, 1)
    return (cm.turbo(n)[:, :3] * 255)

vcol = hcol(vox[:, 2])

# ---- read the live scans for the chosen range (map frame, downsampled) ----
def vdown(p, leaf=0.10):
    q = np.floor(p / leaf).astype(np.int64)
    _, idx = np.unique(q[:,0]*73856093 ^ q[:,1]*19349663 ^ q[:,2]*83492791, return_index=True)
    return p[idx]

scans = [None] * HI
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'),
       rosbag2_py.ConverterOptions('cdr','cdr'))
i = -1
while r.has_next():
    topic, data, _ = r.read_next()
    if topic != '/rslidar_points':
        continue
    i += 1
    if i >= HI:
        break
    if i < LO - 1:                          # still need a few before LO? no — only [LO,HI)
        if i < LO:
            continue
    m = deserialize_message(data, PointCloud2)
    a = np.frombuffer(bytes(m.data), dtype=PT)
    p = np.stack([a["x"], a["y"], a["z"]], axis=1).astype(np.float32)
    rng = np.linalg.norm(p, axis=1)
    good = np.isfinite(p).all(1) & (rng > 0.5) & (rng < 35.0)
    p = p[good]
    T = poses[i]
    mp = (p @ T[:3,:3].T + T[:3,3]).astype(np.float32)
    scans[i] = vdown(mp)
    if i % 300 == 0:
        print(f"  read scan {i}/{HI}", flush=True)

# ---- smoothed camera path -------------------------------------------------
pos = poses[:HI, :3, 3].copy()
k = SMOOTH
kern = np.ones(k) / k
sm = pos.copy()
for c in range(3):
    sm[:, c] = np.convolve(pos[:, c], kern, mode='same')
vel = np.gradient(sm, axis=0)
fwd = vel.copy(); fwd[:, 2] = 0
nrm = np.linalg.norm(fwd, axis=1, keepdims=True)
for j in range(HI):                          # fallback to pose x-axis when ~stationary
    if nrm[j, 0] < 1e-3:
        f = poses[j, :3, 0].copy(); f[2] = 0
        fwd[j] = f
nrm = np.linalg.norm(fwd, axis=1, keepdims=True)
fwd = fwd / np.clip(nrm, 1e-6, None)

focal = (W / 2) / np.tan(HFOV / 2)
WUP = np.array([0, 0, 1.0])

def project(pts, eye, f):
    r_ = np.cross(f, WUP); r_ /= np.linalg.norm(r_)
    u_ = np.cross(r_, f)
    rel = pts - eye
    xc = rel @ r_; yc = rel @ u_; zc = rel @ f
    vis = zc > 0.25
    px = (W/2 + focal * xc[vis]/zc[vis])
    py = (H/2 - focal * yc[vis]/zc[vis])
    return px, py, zc[vis], vis

# ---- render + pipe to ffmpeg ---------------------------------------------
cmd = [FFMPEG, '-y', '-hide_banner', '-loglevel', 'error',
       '-f', 'rawvideo', '-pixel_format', 'rgb24', '-video_size', f'{W}x{H}',
       '-framerate', str(FPS), '-i', '-',
       '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '26',
       '-preset', 'slow', '-movflags', '+faststart', OUT]
pipe = subprocess.Popen(cmd, stdin=subprocess.PIPE)

BG = np.array([8, 9, 14], np.uint8)
def splat(img, px, py, col, size):
    px = np.round(px).astype(np.int32); py = np.round(py).astype(np.int32)
    offs = [(0,0)] if size == 1 else [(ax,ay) for ax in (-1,0,1) for ay in (-1,0,1)]
    for ax, ay in offs:
        xx, yy = px+ax, py+ay
        m = (xx>=0)&(xx<W)&(yy>=0)&(yy<H)
        img[yy[m], xx[m]] = col[m]

for i in range(LO, HI):
    eye = sm[i] - fwd[i]*BACK + WUP*HEIGHT
    tgt = sm[i] + fwd[i]*LOOKAHEAD
    f = tgt - eye; f /= np.linalg.norm(f)
    img = np.empty((H, W, 3), np.uint8); img[:] = BG

    near = np.linalg.norm(vox - sm[i], axis=1) < RADIUS      # dim built map
    if near.any():
        px, py, zc, vis = project(vox[near], eye, f)
        if len(zc):
            o = np.argsort(-zc)
            fade = np.clip(1.0 - zc/(RADIUS+6), 0.15, 1.0)[:, None]
            col = (vcol[near][vis] * 0.32 * fade).astype(np.uint8)
            splat(img, px[o], py[o], col[o], 1)

    cur = scans[i]                                            # bright live scan
    if cur is not None and len(cur):
        px, py, zc, vis = project(cur, eye, f)
        if len(zc):
            o = np.argsort(-zc)
            base = hcol(cur[vis, 2])
            col = np.clip(base + 85, 0, 255)[o].astype(np.uint8)  # bright, height-tinted
            splat(img, px[o], py[o], col, 2)

    pipe.stdin.write(img.tobytes())
    if i % 200 == 0:
        print(f"  render {i}/{HI}", flush=True)

pipe.stdin.close(); pipe.wait()
print(f"{OUT}  scans[{LO},{HI})  {HI-LO} frames @ {FPS}fps")
