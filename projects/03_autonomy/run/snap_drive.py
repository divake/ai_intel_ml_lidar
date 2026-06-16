#!/usr/bin/env python3
"""Snapshot the live RPP drive: overlay /tmp/rpp_drive.csv (actual robot trajectory) on the
wall grid + taught path. For progress milestones (e.g. the bottom-right corner). Renders with
conda python (matplotlib/PIL/yaml).

  /home/nus-ai/miniconda3/envs/intel_ai/bin/python snap_drive.py [--out .../snap.png]
"""
import os, argparse
import numpy as np, yaml
from PIL import Image
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.realpath(__file__))
RES = os.path.normpath(os.path.join(HERE, "..", "results"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=os.path.join(RES, "wall_grid.yaml"))
    ap.add_argument("--path", default=os.path.join(RES, "teach_path_fastlio.csv"))
    ap.add_argument("--csv", default="/tmp/rpp_drive.csv")
    ap.add_argument("--out", default="/tmp/rpp_snap.png")
    a = ap.parse_args()

    m = yaml.safe_load(open(a.map)); img = np.array(Image.open(os.path.join(os.path.dirname(a.map), m["image"])).convert("L"))
    H, W = img.shape; res = float(m["resolution"]); ox, oy = m["origin"][0], m["origin"][1]
    path = np.loadtxt(a.path)
    d = np.genfromtxt(a.csv, delimiter=",", names=True)
    tx, ty = np.atleast_1d(d["x"]), np.atleast_1d(d["y"])
    prog = int(np.atleast_1d(d["prog"])[-1]); cte = float(np.atleast_1d(d["cte"])[-1])
    state = str(d["state"][-1]) if d["state"].ndim else str(d["state"])

    fig, ax = plt.subplots(figsize=(15, 7))
    ax.imshow(img, cmap="gray", extent=[ox, ox+W*res, oy, oy+H*res], origin="upper")
    ax.plot(path[:, 0], path[:, 1], "-", c="deepskyblue", lw=2.2, alpha=0.6, label="taught route")
    ax.plot(tx, ty, "-", c="red", lw=2.0, label="robot (live RPP)")
    ax.plot(tx[0], ty[0], "go", ms=12, label="start")
    ax.plot(tx[-1], ty[-1], "o", c="orange", ms=14, label=f"robot now (prog {prog}/{len(path)})")
    # mark the corner (prog ~91) for reference
    if len(path) > 91:
        ax.plot(path[91, 0], path[91, 1], "m*", ms=18, label="bottom-right corner (~prog 91)")
    ax.set_aspect("equal"); ax.legend(loc="upper right", fontsize=9)
    ax.set_title(f"LIVE drive — robot at prog {prog}/{len(path)}, cross-track {cte:.2f} m, state {state}")
    ax.set_xlim(path[:, 0].min()-4, path[:, 0].max()+4); ax.set_ylim(path[:, 1].min()-4, path[:, 1].max()+4)
    plt.tight_layout(); plt.savefig(a.out, dpi=110); print("saved", a.out, "| prog", prog, "cte", cte, "state", state)


if __name__ == "__main__":
    main()
