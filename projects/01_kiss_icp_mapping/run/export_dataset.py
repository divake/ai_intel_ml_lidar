#!/usr/bin/env python3
"""Export a KISS-ICP mapping run (rosbag .mcap) into an ML-ready dataset.

Reads a rosbag2 recording of:
  /rslidar_points  (sensor_msgs/PointCloud2  : x,y,z,intensity float32, 16x1800 organized)
  /kiss/odometry   (nav_msgs/Odometry        : per-frame pose from KISS-ICP)

Writes into <out>/:
  frames/000000.npy ...   # each: (N,5) float32 -> x,y,z,intensity,ring   (sensor frame)
  poses_tum.txt           # timestamp tx ty tz qx qy qz qw   (KISS-ICP trajectory)
  timestamps.txt          # one cloud timestamp (sec) per frame, matching frames/
  map.ply                 # merged + voxel-downsampled global cloud (world frame)
  meta.json               # summary (counts, fields, voxel size, extents)

Run with SYSTEM python (rclpy/rosbag2 are built for system 3.12):
  /usr/bin/python3 export_dataset.py <bag_dir_or_mcap> <out_dir> [--voxel 0.05]

Pure system deps: rosbag2_py, rclpy serialization, numpy. No open3d/conda needed.
"""
import sys, os, json, argparse
import numpy as np

import rclpy.serialization as ser
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


def cloud_to_array(msg):
    """PointCloud2 -> (N,5) x,y,z,intensity,ring  (ring derived from organized row)."""
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    xyzi = raw[:, :16].copy().view(np.float32).reshape(-1, 4)  # x,y,z,intensity at offsets 0,4,8,12
    # ring = row index for an organized HxW cloud (height=16 beams)
    if msg.height > 1:
        ring = np.repeat(np.arange(msg.height, dtype=np.float32), msg.width)
    else:
        ring = np.zeros(xyzi.shape[0], dtype=np.float32)
    pts = np.column_stack([xyzi, ring])
    finite = np.isfinite(pts[:, :3]).all(axis=1)
    nz = ~(np.abs(pts[:, :3]) < 1e-6).all(axis=1)  # drop (0,0,0) padding
    return pts[finite & nz]


def quat_to_R(x, y, z, w):
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    s = 2.0 / n
    return np.array([
        [1-s*(y*y+z*z),   s*(x*y-z*w),   s*(x*z+y*w)],
        [s*(x*y+z*w),   1-s*(x*x+z*z),   s*(y*z-x*w)],
        [s*(x*z-y*w),     s*(y*z+x*w), 1-s*(x*x+y*y)],
    ], dtype=np.float64)


def stamp_sec(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--voxel", type=float, default=0.05, help="map voxel size (m)")
    ap.add_argument("--cloud-topic", default="/rslidar_points")
    ap.add_argument("--odom-topic", default="/kiss/odometry")
    args = ap.parse_args()

    os.makedirs(os.path.join(args.out, "frames"), exist_ok=True)

    storage = StorageOptions(uri=args.bag, storage_id="mcap")
    conv = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage, conv)

    clouds, odoms = [], []
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic == args.cloud_topic:
            clouds.append(ser.deserialize_message(data, PointCloud2))
        elif topic == args.odom_topic:
            odoms.append(ser.deserialize_message(data, Odometry))

    print(f"read {len(clouds)} clouds, {len(odoms)} odometry msgs")
    if not clouds:
        print("ERROR: no clouds found — check --cloud-topic"); sys.exit(1)

    # odom timestamps for nearest-pose association
    o_ts = np.array([stamp_sec(o.header) for o in odoms]) if odoms else np.array([])

    def pose_at(ts):
        if o_ts.size == 0:
            return np.eye(3), np.zeros(3)
        i = int(np.argmin(np.abs(o_ts - ts)))
        p = odoms[i].pose.pose
        R = quat_to_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        t = np.array([p.position.x, p.position.y, p.position.z])
        return R, t

    # voxel-hash accumulator for the merged map (memory-safe)
    vox = {}
    ts_list, n_total = [], 0
    extents_min = np.array([np.inf]*3); extents_max = np.array([-np.inf]*3)
    for idx, c in enumerate(clouds):
        pts = cloud_to_array(c)
        np.save(os.path.join(args.out, "frames", f"{idx:06d}.npy"), pts.astype(np.float32))
        ts = stamp_sec(c.header); ts_list.append(ts); n_total += pts.shape[0]
        # transform to world and accumulate into voxel grid
        R, t = pose_at(ts)
        world = (pts[:, :3].astype(np.float64) @ R.T) + t
        if world.size:
            extents_min = np.minimum(extents_min, world.min(axis=0))
            extents_max = np.maximum(extents_max, world.max(axis=0))
        keys = np.floor(world / args.voxel).astype(np.int64)
        inten = pts[:, 3]
        for k, w, ii in zip(map(tuple, keys), world, inten):
            if k not in vox:
                vox[k] = (w[0], w[1], w[2], ii)

    # write poses (TUM)
    with open(os.path.join(args.out, "poses_tum.txt"), "w") as f:
        for o in odoms:
            ts = stamp_sec(o.header); p = o.pose.pose
            f.write(f"{ts:.6f} {p.position.x:.6f} {p.position.y:.6f} {p.position.z:.6f} "
                    f"{p.orientation.x:.6f} {p.orientation.y:.6f} {p.orientation.z:.6f} {p.orientation.w:.6f}\n")
    with open(os.path.join(args.out, "timestamps.txt"), "w") as f:
        f.write("\n".join(f"{t:.6f}" for t in ts_list))

    # write merged map PLY (ascii, x y z intensity)
    mp = np.array(list(vox.values()), dtype=np.float64) if vox else np.zeros((0, 4))
    ply = os.path.join(args.out, "map.ply")
    with open(ply, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {mp.shape[0]}\n")
        f.write("property float x\nproperty float y\nproperty float z\nproperty float intensity\n")
        f.write("end_header\n")
        for r in mp:
            f.write(f"{r[0]:.4f} {r[1]:.4f} {r[2]:.4f} {r[3]:.2f}\n")

    meta = {
        "frames": len(clouds), "odom_msgs": len(odoms),
        "total_points_raw": int(n_total), "map_points_voxelized": int(mp.shape[0]),
        "voxel_size_m": args.voxel,
        "frame_fields": ["x", "y", "z", "intensity", "ring"],
        "map_extent_min": extents_min.tolist() if np.isfinite(extents_min).all() else None,
        "map_extent_max": extents_max.tolist() if np.isfinite(extents_max).all() else None,
        "cloud_topic": args.cloud_topic, "odom_topic": args.odom_topic,
    }
    with open(os.path.join(args.out, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2)
    print("EXPORT DONE")
    print(json.dumps(meta, indent=2))


if __name__ == "__main__":
    main()
