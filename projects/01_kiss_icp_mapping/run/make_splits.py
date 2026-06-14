#!/usr/bin/env python3
"""Write splits.json (train/val/test) for an exported dataset.

LiDAR frames at 10 Hz are highly correlated frame-to-frame, so a RANDOM
per-frame split leaks (a val frame is nearly identical to its train neighbours).
We instead use CONTIGUOUS temporal blocks: first 70% train, next 15% val,
last 15% test. This mirrors how KITTI/SemanticKITTI split by sequence.

  python make_splits.py <dataset_dir> [train_frac val_frac]
"""
import os, sys, json

root = sys.argv[1]
tr = float(sys.argv[2]) if len(sys.argv) > 2 else 0.70
va = float(sys.argv[3]) if len(sys.argv) > 3 else 0.15
n = len([f for f in os.listdir(os.path.join(root, "frames")) if f.endswith(".npy")])
a = int(n * tr)
b = int(n * (tr + va))
splits = {
    "_scheme": "contiguous temporal blocks (no shuffling) to avoid leakage between adjacent 10 Hz frames",
    "train": list(range(0, a)),
    "val": list(range(a, b)),
    "test": list(range(b, n)),
}
with open(os.path.join(root, "splits.json"), "w") as f:
    json.dump(splits, f)
print(f"{root}: {n} frames -> train {a}  val {b-a}  test {n-b}")
