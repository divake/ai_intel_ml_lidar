#!/usr/bin/env python
"""Re-render an exported KISS-ICP dataset into a dense, intensity-colored map
(PNGs + interactive HTML). Does NOT touch the dataset; writes to a new folder.

Usage: python render_map.py <dataset_dir> <out_dir> [--voxel 0.02]
"""
import os, sys, argparse
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("dataset")
ap.add_argument("out")
ap.add_argument("--voxel", type=float, default=0.02)
ap.add_argument("--html_max", type=int, default=300000, help="max pts in interactive html")
args = ap.parse_args()

os.makedirs(args.out, exist_ok=True)
NAME = os.path.basename(args.out.rstrip("/")).replace("_render", "") or "map"
frames_dir = os.path.join(args.dataset, "frames")
poses = np.loadtxt(os.path.join(args.dataset, "poses_tum.txt"))  # t x y z qx qy qz qw
n = poses.shape[0]
print(f"poses: {n}")


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


files = sorted(os.listdir(frames_dir))
files = [f for f in files if f.endswith(".npy")]
print(f"frames: {len(files)}")

worlds = []
intens = []
for i, f in enumerate(files):
    if i >= n:
        break
    arr = np.load(os.path.join(frames_dir, f))  # (N,5) x,y,z,intensity,ring
    if arr.shape[0] == 0:
        continue
    pts = arr[:, :3].astype(np.float64)
    inten = arr[:, 3].astype(np.float32)
    tx, ty, tz = poses[i, 1], poses[i, 2], poses[i, 3]
    R = quat_to_R(poses[i, 4], poses[i, 5], poses[i, 6], poses[i, 7])
    w = (R @ pts.T).T + np.array([tx, ty, tz])
    worlds.append(w.astype(np.float32))
    intens.append(inten)
    if i % 400 == 0:
        print(f"  transformed {i}/{len(files)}")

world = np.concatenate(worlds, axis=0)
inten = np.concatenate(intens, axis=0)
print(f"raw world points: {world.shape[0]:,}")

# voxel downsample (keep first occurrence) at fine voxel.
# Pack the 3D voxel index into a single int64 (memory-light, fast unique)
# instead of np.unique(..., axis=0) which is heavy at hundreds of millions of pts.
g = np.floor(world / args.voxel).astype(np.int64)
g -= g.min(axis=0)                     # shift to non-negative
span = g.max(axis=0) + 1               # voxel extent per axis
assert int(span[0]) * int(span[1]) * int(span[2]) < 9_000_000_000_000_000_000, "voxel grid too large to pack"
packed = (g[:, 0] * span[1] + g[:, 1]) * span[2] + g[:, 2]
_, idx = np.unique(packed, return_index=True)
world = world[idx]
inten = inten[idx]
print(f"voxel({args.voxel} m) points: {world.shape[0]:,}")

# robust intensity normalization (2-98 percentile)
lo, hi = np.percentile(inten, 2), np.percentile(inten, 98)
inorm = np.clip((inten - lo) / max(hi - lo, 1e-6), 0, 1)

traj = poses[:, 1:4]

# ---- write a dense intensity PLY (so they can open in CloudCompare/Foxglove) ----
ply = os.path.join(args.out, "map_dense.ply")
with open(ply, "w") as fp:
    fp.write("ply\nformat ascii 1.0\n")
    fp.write(f"element vertex {world.shape[0]}\n")
    fp.write("property float x\nproperty float y\nproperty float z\nproperty float intensity\n")
    fp.write("end_header\n")
    np.savetxt(fp, np.column_stack([world, inten]), fmt="%.4f %.4f %.4f %.1f")
print("wrote", ply)

# ---- matplotlib renders ----
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def render(view, fname, title):
    fig = plt.figure(figsize=(16, 16), facecolor="black")
    ax = fig.add_subplot(111, facecolor="black")
    if view == "top":
        X, Y = world[:, 0], world[:, 1]
    elif view == "side":  # look along X: show Y (horizontal) vs Z (height)
        X, Y = world[:, 1], world[:, 2]
    ax.scatter(X, Y, c=inorm, s=0.5, cmap="jet", linewidths=0, marker=".")
    if view == "top":
        ax.plot(traj[:, 0], traj[:, 1], "-", color="white", lw=2.0, alpha=0.95)
        ax.plot(traj[0, 0], traj[0, 1], "o", color="lime", ms=12)
        ax.plot(traj[-1, 0], traj[-1, 1], "s", color="red", ms=12)
    ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_title(title, color="white", fontsize=14)
    plt.tight_layout()
    plt.savefig(os.path.join(args.out, fname), dpi=150, facecolor="black")
    plt.close(fig)
    print("wrote", fname)

render("top", "map_topdown_intensity.png",
       f"{NAME}  top-down (intensity)  |  {world.shape[0]:,} pts @ {args.voxel} m voxel")
render("side", "map_side_intensity.png",
       f"{NAME}  side view (intensity)  |  height = up")

# ---- interactive plotly HTML (subsampled) ----
import plotly.graph_objects as go
m = world.shape[0]
if m > args.html_max:
    sel = np.random.default_rng(0).choice(m, args.html_max, replace=False)
else:
    sel = np.arange(m)
fig = go.Figure(data=[go.Scatter3d(
    x=world[sel, 0], y=world[sel, 1], z=world[sel, 2],
    mode="markers",
    marker=dict(size=1.2, color=inorm[sel], colorscale="Jet", opacity=0.9,
                colorbar=dict(title="intensity")),
)])
fig.add_trace(go.Scatter3d(
    x=traj[:, 0], y=traj[:, 1], z=traj[:, 2], mode="lines",
    line=dict(color="white", width=4), name="robot path"))
fig.update_layout(
    title=f"{NAME} interactive ({len(sel):,} pts shown)",
    scene=dict(aspectmode="data",
               xaxis=dict(backgroundcolor="black", color="white"),
               yaxis=dict(backgroundcolor="black", color="white"),
               zaxis=dict(backgroundcolor="black", color="white")),
    paper_bgcolor="black", font=dict(color="white"))
html = os.path.join(args.out, "map_interactive.html")
fig.write_html(html)
print("wrote", html)
print("DONE")
