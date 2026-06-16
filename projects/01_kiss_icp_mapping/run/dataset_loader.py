#!/usr/bin/env python3
"""Minimal, dependency-light loader for the ml_lidar KISS-ICP datasets.

Only needs numpy. Works on any `<name>_dataset/` folder produced by
export_dataset.py. Designed to feel familiar to KITTI / Waymo users.

    from dataset_loader import LidarDataset
    ds = LidarDataset("corridor1_dataset")
    pts = ds.frame(0)                 # (N,5) float32: x,y,z,intensity,ring (sensor frame)
    T   = ds.pose(0)                  # (4,4) world<-sensor (KISS-ICP estimate)
    wpts = ds.frame_world(0)          # (N,5) with xyz transformed into the world/map frame
    for i in ds.split("train"):       # iterate a split (train/val/test)
        ...

Conventions (see DATASET.md at the repo root for the full dataset card):
  * coordinate frame: ROS REP-103, x forward / y left / z up, metres, right-handed
  * intensity: 0-255, uncalibrated sensor reflectivity
  * ring: laser/beam id 0..15
  * poses: KISS-ICP LiDAR-odometry ESTIMATE, not survey/RTK ground truth
"""
import os, json
import numpy as np


def quat_to_R(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w),     s * (x * z + y * w)],
        [s * (x * y + z * w),     1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w),     s * (y * z + x * w),     1 - s * (x * x + y * y)],
    ])


class LidarDataset:
    def __init__(self, root):
        self.root = root
        self.frames_dir = os.path.join(root, "frames")
        self.files = sorted(f for f in os.listdir(self.frames_dir) if f.endswith(".npy"))
        self.poses_tum = np.loadtxt(os.path.join(root, "poses_tum.txt")).reshape(-1, 8)
        with open(os.path.join(root, "meta.json")) as f:
            self.meta = json.load(f)
        sp = os.path.join(root, "splits.json")
        self.splits = json.load(open(sp)) if os.path.exists(sp) else None

    def __len__(self):
        return len(self.files)

    def frame(self, i):
        """(N,5) float32: x,y,z,intensity,ring  in the sensor frame."""
        return np.load(os.path.join(self.frames_dir, self.files[i]))

    def pose(self, i):
        """(4,4) homogeneous world<-sensor transform for frame i."""
        _, tx, ty, tz, qx, qy, qz, qw = self.poses_tum[i]
        T = np.eye(4)
        T[:3, :3] = quat_to_R(qx, qy, qz, qw)
        T[:3, 3] = (tx, ty, tz)
        return T

    def frame_world(self, i):
        """frame(i) with xyz mapped into the world/map frame (intensity,ring kept)."""
        p = self.frame(i)
        T = self.pose(i)
        xyz = (p[:, :3] @ T[:3, :3].T) + T[:3, 3]
        return np.column_stack([xyz.astype(np.float32), p[:, 3:5]])

    def split(self, name):
        """Yield frame indices for 'train' | 'val' | 'test' (contiguous, leak-free)."""
        if self.splits is None:
            raise FileNotFoundError("no splits.json in this dataset")
        return list(self.splits[name])


if __name__ == "__main__":
    import sys
    ds = LidarDataset(sys.argv[1] if len(sys.argv) > 1 else ".")
    print(f"{len(ds)} frames | sensor: {ds.meta.get('sensor')}")
    print(f"fields: {ds.meta.get('frame_fields')}  frame0 -> {ds.frame(0).shape}")
    print(f"pose0:\n{ds.pose(0)}")
    if ds.splits:
        print({k: len(v) for k, v in ds.splits.items()})
