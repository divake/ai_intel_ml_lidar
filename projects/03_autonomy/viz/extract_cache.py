#!/usr/bin/env python3
"""Extract pass: read the corridor2.0 bag ONCE, transform every scan into the
map frame using the per-scan SLAM poses, and accumulate a voxel map where each
voxel remembers the scan index it first appeared in. Cached to an .npz so the
render passes (many video versions) never touch the bag again.

Run with ROS python:  source /opt/ros/jazzy/setup.bash && /usr/bin/python3 extract_cache.py
"""
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

BAG   = 'projects/02_fast_lio2/results/corridor2.0'
POSES = 'projects/03_autonomy/results/kiss_slam/poses_loopclosed.npy'   # (7619,4,4) world<-sensor
OUT   = 'projects/03_autonomy/results/video/cache.npz'
VOX   = 0.10        # map voxel size (m)

PT = np.dtype({"names":["x","y","z","intensity","ring","timestamp"],
               "formats":["<f4","<f4","<f4","<f4","<u2","<f8"],
               "offsets":[0,4,8,12,16,18], "itemsize":26})

poses = np.load(POSES).astype(np.float64)
N = len(poses)
traj = poses[:, :3, 3].astype(np.float32)

OFF = 2048   # voxel-index offset so packed keys stay positive (12 bits/axis)
def pack(q):
    return (((q[:,0]+OFF).astype(np.int64) << 24)
            | ((q[:,1]+OFF).astype(np.int64) << 12)
            |  (q[:,2]+OFF).astype(np.int64))

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'),
       rosbag2_py.ConverterOptions('cdr','cdr'))

Ks, XYZs, Is, INT = [], [], [], []
i = -1
while r.has_next():
    topic, data, _ = r.read_next()
    if topic != '/rslidar_points':
        continue
    i += 1
    if i >= N:
        break
    m = deserialize_message(data, PointCloud2)
    a = np.frombuffer(bytes(m.data), dtype=PT)
    p = np.stack([a["x"], a["y"], a["z"]], axis=1).astype(np.float32)
    inten = a["intensity"].astype(np.float32)
    rng = np.linalg.norm(p, axis=1)
    good = np.isfinite(p).all(1) & (rng > 0.5) & (rng < 50.0)
    p, inten = p[good], inten[good]
    T = poses[i]
    mp = (p @ T[:3,:3].T + T[:3,3]).astype(np.float32)
    zin = (mp[:,2] > -3.0) & (mp[:,2] < 4.5)
    mp, inten = mp[zin], inten[zin]
    q = np.floor(mp / VOX).astype(np.int64)
    key = pack(q)
    uk, ui = np.unique(key, return_index=True)
    Ks.append(uk); XYZs.append(mp[ui]); INT.append(inten[ui])
    Is.append(np.full(uk.shape, i, np.int32))
    if i % 1000 == 0:
        print(f"  scan {i}/{N}  cum-chunks={len(Ks)}", flush=True)

K   = np.concatenate(Ks)
XYZ = np.concatenate(XYZs)
INT = np.concatenate(INT)
I   = np.concatenate(Is)
print(f"raw voxel-observations: {len(K):,}")

order = np.lexsort((I, K))             # for equal key, smallest scan-index first
K, XYZ, INT, I = K[order], XYZ[order], INT[order], I[order]
uk, ui = np.unique(K, return_index=True)
vox, inten, first = XYZ[ui], INT[ui], I[ui]

np.savez_compressed(OUT,
                    vox=vox.astype(np.float32),
                    inten=inten.astype(np.float32),
                    first=first.astype(np.int32),
                    traj=traj,
                    nscans=np.int32(i))
print(f"scans={i}  unique voxels={len(vox):,}  ->  {OUT}")
print("extent x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f]" % (
    vox[:,0].min(), vox[:,0].max(), vox[:,1].min(), vox[:,1].max(),
    vox[:,2].min(), vox[:,2].max()))
