#!/usr/bin/env python3
"""Read corridor2.0 mcap ONCE (no ROS), pull the live per-scan LiDAR points,
transform each into the map frame with the per-scan SLAM pose, downsample, and
cache. This is the bright 'laser sweep' overlay for the hero film.

Run with the mcap env:  /home/divake/miniconda3/envs/lidar_viz/bin/python extract_live.py
"""
import os, numpy as np
from mcap_ros2.reader import read_ros2_messages

REPO = "/ssd_4TB/divake/ml_lidar"
BAG = os.path.join(REPO, "projects/02_fast_lio2/results/corridor2.0/corridor2.0_0.mcap")
POSES = os.path.join(REPO, "projects/03_autonomy/results/kiss_slam/poses_loopclosed.npy")
OUT = os.path.join(REPO, "projects/04_demo_viz/cache/live_scans.npz")
VOX, CAP = 0.12, 8000

PT = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "timestamp"],
               "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f8"],
               "offsets": [0, 4, 8, 12, 16, 18], "itemsize": 26})

poses = np.load(POSES).astype(np.float64)
N = len(poses)
fwd = poses[:, :3, 0].astype(np.float32)          # robot forward (map frame) per scan
pos = poses[:, :3, 3].astype(np.float32)

def vdown(p, leaf=VOX):
    q = np.floor(p / leaf).astype(np.int64)
    key = q[:, 0] * 73856093 ^ q[:, 1] * 19349663 ^ q[:, 2] * 83492791
    _, idx = np.unique(key, return_index=True)
    return idx

pts_chunks, int_chunks, offs = [], [], [0]
i = -1
for m in read_ros2_messages(BAG, topics=["/rslidar_points"]):
    i += 1
    if i >= N:
        break
    msg = m.ros_msg
    a = np.frombuffer(bytes(msg.data), dtype=PT)
    p = np.stack([a["x"], a["y"], a["z"]], axis=1).astype(np.float32)
    inten = a["intensity"].astype(np.float32)
    rng = np.linalg.norm(p, axis=1)
    good = np.isfinite(p).all(1) & (rng > 0.5) & (rng < 45.0)
    p, inten = p[good], inten[good]
    T = poses[i]
    mp = (p @ T[:3, :3].T + T[:3, 3]).astype(np.float32)
    zin = (mp[:, 2] > -2.6) & (mp[:, 2] < 4.5)
    mp, inten = mp[zin], inten[zin]
    idx = vdown(mp)
    if len(idx) > CAP:
        idx = idx[np.linspace(0, len(idx) - 1, CAP).astype(int)]
    mp, inten = mp[idx], inten[idx]
    pts_chunks.append(mp.astype(np.float16)); int_chunks.append(inten.astype(np.float16))
    offs.append(offs[-1] + len(mp))
    if i % 500 == 0:
        print(f"  scan {i}/{N}  cum_pts={offs[-1]:,}", flush=True)

PTS = np.concatenate(pts_chunks); INT = np.concatenate(int_chunks)
np.savez(OUT, pts=PTS, inten=INT, offs=np.array(offs, np.int64),
         pos=pos[:i + 1], fwd=fwd[:i + 1], nscans=np.int32(i + 1))
print(f"scans={i + 1}  live_pts={len(PTS):,}  -> {OUT}  ({PTS.nbytes/1e6:.0f} MB)")
