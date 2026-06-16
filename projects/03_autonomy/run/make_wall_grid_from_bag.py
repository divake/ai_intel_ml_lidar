"""Clean 2D WALL occupancy grid from RAW scans (sensor-height horizontal slice).

The 3D-map projection fails at the twice-driven spine (Z-drift stacks two floors -> false
walls). Fix: for each scan keep only points near the SENSOR's horizontal plane (|z_sensor|
small) -- that's a 2D-laser slice that hits WALLS, not floor/ceiling -- transform by the
per-scan FAST-LIO pose, and accumulate hits into a grid. Drift-robust (per-scan) and uses
RAW /lio/odometry poses, so it's in the SAME frame as teach_path_fastlio.csv.
"""
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

BAG = "/home/nus-ai/divek_nus/ml_lidar/projects/02_fast_lio2/results/corridor2.0"
OUT_PNG = "/home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/results/wall_grid.png"
OUT_YAML = "/home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/results/wall_grid.yaml"
PATH = "/home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/results/teach_path_fastlio.csv"
RES = 0.10
ZLO, ZHI = 0.10, 0.70  # m sensor-frame: keep a slice ABOVE horizontal (excludes the floor below)
RMIN, RMAX = 0.6, 30.0
MINHITS = 6           # grid cell needs this many hits to be a wall (rejects stray points / floor specks)
DILATE = 0


def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


def Rq(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]], dtype=np.float64)


print("pass1: /lio/odometry ...", flush=True)
ot, opos, oq = [], [], []
r = reader(["/lio/odometry"])
while r.has_next():
    _, d, _ = r.read_next(); m = deserialize_message(d, Odometry)
    ot.append(m.header.stamp.sec + m.header.stamp.nanosec*1e-9)
    p = m.pose.pose.position; o = m.pose.pose.orientation
    opos.append([p.x, p.y, p.z]); oq.append([o.x, o.y, o.z, o.w])
ot = np.array(ot); opos = np.array(opos); oq = np.array(oq)

path = np.loadtxt(PATH)
# grid bounds from the trajectory + margin (covers the whole figure-8)
allx = np.concatenate([opos[:, 0], path[:, 0]]); ally = np.concatenate([opos[:, 1], path[:, 1]])
pad = 4.0
ox, oy = allx.min() - pad, ally.min() - pad
W = int((allx.max() + pad - ox) / RES) + 1
H = int((ally.max() + pad - oy) / RES) + 1
hits = np.zeros((H, W), dtype=np.int32)
print(f"grid {W}x{H} res={RES} origin=({ox:.2f},{oy:.2f})", flush=True)

dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"]*3, "offsets": [0, 4, 8], "itemsize": 26})
print("pass2: raw scans -> horizontal slice -> grid ...", flush=True)
r = reader(["/rslidar_points"]); i = 0
while r.has_next():
    _, d, _ = r.read_next(); m = deserialize_message(d, PointCloud2)
    t = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
    j = int(np.clip(np.searchsorted(ot, t), 0, len(ot)-1))
    a = np.frombuffer(m.data, dtype=dt)
    xs, ys, zs = a["x"].astype(np.float64), a["y"].astype(np.float64), a["z"].astype(np.float64)
    rr = np.hypot(xs, ys)
    sel = np.isfinite(xs) & (zs > ZLO) & (zs < ZHI) & (rr > RMIN) & (rr < RMAX)   # above-floor wall slice
    if sel.any():
        pts = np.stack([xs[sel], ys[sel], zs[sel]], 1)
        wpts = (Rq(oq[j]) @ pts.T).T + opos[j]
        gx = ((wpts[:, 0] - ox) / RES).astype(np.int64)
        gy = ((wpts[:, 1] - oy) / RES).astype(np.int64)
        ok = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
        np.add.at(hits, (gy[ok], gx[ok]), 1)
    i += 1
    if i % 1500 == 0:
        print(f"  scan {i}", flush=True)

occ = hits >= MINHITS
if DILATE:
    from scipy import ndimage
    occ = ndimage.binary_dilation(occ, iterations=DILATE)
print(f"wall cells {occ.sum()} ({100*occ.sum()/(H*W):.1f}%)", flush=True)

img = np.full((H, W), 254, dtype=np.uint8)
img[occ] = 0
img = np.flipud(img)                         # row 0 = top (max y) — ROS convention
from PIL import Image
Image.fromarray(img).save(OUT_PNG)
open(OUT_YAML, "w").write(
    f"image: wall_grid.png\nresolution: {RES}\norigin: [{ox:.3f}, {oy:.3f}, 0.0]\n"
    f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n")

# verify with the SAME convention the sim uses (Grid.w2p): c=(x-ox)/res, r=H-1-(y-oy)/res
def occ_img(px, py):
    c = int((px - ox) / RES); rrow = H - 1 - int((py - oy) / RES)
    if rrow < 0 or rrow >= H or c < 0 or c >= W:
        return True
    return img[rrow, c] < 90
on = sum(occ_img(px, py) for px, py in path[:, :2])
print(f"saved. teach-path pts on wall: {on}/{len(path)} ({100*on/len(path):.1f}%) -- want ~0%", flush=True)
