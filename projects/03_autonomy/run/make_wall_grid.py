"""Build a clean 2D WALL occupancy grid from the loop-closed map, aligned to the path.

A cell is a WALL if its points span a tall vertical extent (z-range > thresh) -- robust to
the 2.3 m Z-drift and excludes the flat floor (which would otherwise fill the corridor).
Output: an occupancy PNG + ROS-style yaml the sim ray-casts against. Same frame as the
loop-closed npz traj and teach_path_fastlio.csv, so they're guaranteed aligned.
"""
import numpy as np
from PIL import Image
from scipy import ndimage

NPZ = "/home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/results/corridor2_loopclosed_fastlio.npz"
OUT_PNG = "/home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/results/wall_grid.png"
OUT_YAML = "/home/nus-ai/divek_nus/ml_lidar/projects/03_autonomy/results/wall_grid.yaml"
RES = 0.10
BAND_LO = 0.4            # m above local floor: bottom of the wall-detection band
BAND_HI = 2.0           # m above local floor: top of the band (below the ~2.5 m ceiling)
MINPTS = 5              # points in the wall-band for a cell to count as a wall
DILATE = 1               # cells: thicken walls so rays don't leak through

d = np.load(NPZ)
w = d["world"]; traj = d["traj"]
x, y, z = w[:, 0], w[:, 1], w[:, 2]
pad = 2.0
ox, oy = x.min() - pad, y.min() - pad
W = int((x.max() + pad - ox) / RES) + 1
H = int((y.max() + pad - oy) / RES) + 1
print(f"grid {W}x{H} res={RES} origin=({ox:.2f},{oy:.2f})")

gx = ((x - ox) / RES).astype(np.int64)
gy = ((y - oy) / RES).astype(np.int64)
cid = gx * H + gy
order = np.argsort(cid, kind="stable")
cid_s = cid[order]; z_s = z[order]
bnd = np.concatenate([[0], np.flatnonzero(np.diff(cid_s)) + 1, [len(cid_s)]])
cnt = np.diff(bnd)
cell_ids = cid_s[bnd[:-1]]
# local floor per cell = low percentile of z (robust to stray low points)
zlo = np.minimum.reduceat(z_s, bnd[:-1])
zlo_pt = np.repeat(zlo, cnt)                          # broadcast floor back to each point
inband = (z_s > zlo_pt + BAND_LO) & (z_s < zlo_pt + BAND_HI)
band_cnt = np.add.reduceat(inband.astype(np.int64), bnd[:-1])
is_wall = band_cnt >= MINPTS                          # vertical structure rising from floor = wall
wall_cells = cell_ids[is_wall]
wgx, wgy = wall_cells // H, wall_cells % H

occ = np.zeros((H, W), dtype=bool)
occ[wgy, wgx] = True
if DILATE:
    occ = ndimage.binary_dilation(occ, iterations=DILATE)
print(f"wall cells: {occ.sum()} ({100*occ.sum()/(H*W):.1f}% of grid)")

# image: row 0 = top (max y). occupied=0 (black), free=254 (white) -- matches Grid (a<90=occ)
img = np.full((H, W), 254, dtype=np.uint8)
img[occ] = 0
img = np.flipud(img)                 # so row 0 = top
Image.fromarray(img).save(OUT_PNG)
with open(OUT_YAML, "w") as f:
    f.write(f"image: wall_grid.png\nresolution: {RES}\norigin: [{ox:.3f}, {oy:.3f}, 0.0]\n"
            f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n")
print("saved", OUT_PNG, OUT_YAML)

# verify: how many trajectory points land in FREE space?
def occupied(px, py):
    c = int((px - ox) / RES); r = H - 1 - int((py - oy) / RES)
    if r < 0 or r >= H or c < 0 or c >= W:
        return True
    return bool(occ[r, c])
on = sum(occupied(px, py) for px, py in traj[:, :2])
print(f"trajectory pts in WALL cells: {on}/{len(traj)} ({100*on/len(traj):.1f}%) -- want ~0%")
