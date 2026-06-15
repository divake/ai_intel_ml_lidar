"""Loop-close the FAST-LIO (IMU-aided) trajectory and rebuild the global map.

FAST-LIO gives a low-drift trajectory in the corridor (the IMU dead-reckons the
forward motion LiDAR can't see), but it has NO loop-closure step, so the ~1.25 m
of accumulated drift just sits between END and START. We KNOW it's a loop (robot
returned to start), so we add the closure by hand: snap END->START and distribute
the gap along the trajectory by time, then re-accumulate every scan at its
corrected pose. Simple linear closure (translation only) — right call for a small,
mostly along-corridor drift; a full pose-graph BA is the survey-grade upgrade.
"""
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

BAG = "/home/nus-ai/divek_nus/ml_lidar/projects/02_fast_lio2/results/corridor2.0"
OUTDIR = "/home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/results"
OUT_NPZ = OUTDIR + "/corridor2_loopclosed_fastlio.npz"
OUT_PNG = OUTDIR + "/corridor2_loopclosed_fastlio.png"
VOX = 0.05


def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


def Rq(q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]], dtype=np.float64)


def vidx(w, vox):
    g = np.floor(w/vox).astype(np.int64); g -= g.min(0); span = g.max(0)+1
    packed = (g[:, 0]*span[1]+g[:, 1])*span[2]+g[:, 2]
    _, idx = np.unique(packed, return_index=True); return idx


# --- Pass 1: FAST-LIO odometry ---
print("pass 1: reading /lio/odometry ...", flush=True)
ot, opos, oq = [], [], []
r = reader(["/lio/odometry"])
while r.has_next():
    _, d, _ = r.read_next(); m = deserialize_message(d, Odometry)
    ot.append(m.header.stamp.sec + m.header.stamp.nanosec*1e-9)
    p = m.pose.pose.position; o = m.pose.pose.orientation
    opos.append([p.x, p.y, p.z]); oq.append([o.x, o.y, o.z, o.w])
ot = np.array(ot); opos = np.array(opos); oq = np.array(oq)
t0, t1 = ot[0], ot[-1]
gap = opos[0] - opos[-1]                       # vector that brings END onto START
print(f"  {len(ot)} odom poses, span {t1-t0:.1f}s, END-START gap = {np.linalg.norm(gap):.3f} m", flush=True)

# --- Pass 2: clouds -> corrected world map ---
print("pass 2: accumulating scans at loop-closed poses ...", flush=True)
dt = np.dtype({'names': ['x', 'y', 'z', 'intensity'], 'formats': ['<f4']*4,
               'offsets': [0, 4, 8, 12], 'itemsize': 26})
acc, rawacc, traj, trajraw = [], [], [], []
i = 0
r = reader(["/rslidar_points"])
while r.has_next():
    _, d, _ = r.read_next(); m = deserialize_message(d, PointCloud2)
    t = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
    j = int(np.clip(np.searchsorted(ot, t), 0, len(ot)-1))
    pos = opos[j]; Rm = Rq(oq[j]); frac = (t-t0)/(t1-t0)
    pos_c = pos + frac*gap
    a = np.frombuffer(m.data, dtype=dt)
    xyz = np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)
    g = np.isfinite(xyz).all(1); rr = np.linalg.norm(xyz, axis=1); g &= (rr > 0.5) & (rr < 60)
    xyz = xyz[g]
    if len(xyz):
        w = (Rm@xyz.T).T + pos_c; k = vidx(w, VOX); acc.append(w[k].astype(np.float32))
        wr = (Rm@xyz.T).T + pos;  rawacc.append(wr[k].astype(np.float32))
    traj.append(pos_c); trajraw.append(pos.copy()); i += 1
    if i % 1000 == 0:
        W = np.concatenate(acc); acc = [W[vidx(W, VOX)]]
        WR = np.concatenate(rawacc); rawacc = [WR[vidx(WR, VOX)]]
        print(f"  scan {i}: {len(acc[0])} pts", flush=True)
W = np.concatenate(acc); W = W[vidx(W, VOX)]
WR = np.concatenate(rawacc); WR = WR[vidx(WR, VOX)]
traj = np.array(traj); trajraw = np.array(trajraw)
res = np.linalg.norm(traj[0]-traj[-1])
print(f"DONE: closed map {len(W)} pts; residual END-START {res:.4f} m (was {np.linalg.norm(gap):.3f} m)", flush=True)
np.savez_compressed(OUT_NPZ, world=W, traj=traj, trajraw=trajraw)

import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig, axs = plt.subplots(1, 2, figsize=(22, 8))
for ax, (P, T, ttl) in zip(axs, [(WR, trajraw, f"FAST-LIO raw (open, {np.linalg.norm(gap):.2f} m gap)"),
                                  (W, traj, "loop-closed")]):
    ax.scatter(P[:, 0], P[:, 1], s=0.05, c=P[:, 2], cmap="turbo", linewidths=0)
    ax.plot(T[:, 0], T[:, 1], "-", c="white", lw=1.0)
    ax.scatter([T[0, 0]], [T[0, 1]], c="lime", s=140, zorder=5)
    ax.scatter([T[-1, 0]], [T[-1, 1]], c="red", s=90, marker="s", zorder=5)
    ax.set_facecolor("black"); ax.set_aspect("equal"); ax.tick_params(colors="white")
    ax.set_title(ttl, color="white")
fig.patch.set_facecolor("black"); plt.tight_layout(); plt.savefig(OUT_PNG, dpi=110, facecolor="black")
print("saved", OUT_PNG, flush=True)
