#!/usr/bin/env python3
"""One-off: correct the `ring` column of an already-exported dataset.

Early exports stored the azimuth index (0..1799) in the 5th column instead of
the laser/beam id (0..15). This re-reads the source bag and rewrites ONLY the
ring column of each frames/*.npy; x,y,z,intensity are recomputed identically,
so map.ply / poses / timestamps are untouched and do not need regenerating.

Run with SYSTEM python:
  /usr/bin/python3 fix_rings.py <bag_dir_or_mcap> <dataset_dir>
"""
import os, sys
import numpy as np

# reuse the corrected reader so the fix can never drift from the exporter
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from export_dataset import cloud_to_array  # noqa: E402

import rclpy.serialization as ser  # noqa: E402
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402

if len(sys.argv) != 3:
    print(__doc__); sys.exit(1)
bag, out = sys.argv[1], sys.argv[2]
frames_dir = os.path.join(out, "frames")

r = SequentialReader()
r.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))

idx, fixed, mism = 0, 0, 0
while r.has_next():
    topic, data, _t = r.read_next()
    if topic != "/rslidar_points":
        continue
    pts = cloud_to_array(ser.deserialize_message(data, PointCloud2)).astype(np.float32)
    fp = os.path.join(frames_dir, f"{idx:06d}.npy")
    if os.path.exists(fp):
        old = np.load(fp)
        if old.shape != pts.shape:
            mism += 1
            print(f"  ! shape mismatch {idx:06d}: old {old.shape} new {pts.shape}")
    np.save(fp, pts)
    fixed += 1
    if idx % 1000 == 0:
        print(f"  {idx}: ring now in [{pts[:,4].min():.0f},{pts[:,4].max():.0f}]")
    idx += 1

print(f"DONE: rewrote {fixed} frames, {mism} shape mismatches -> {frames_dir}")
