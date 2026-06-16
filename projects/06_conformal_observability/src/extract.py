"""PART 1 — Data foundation.

Stream the corridor2.0 mcap once → per-scan downsampled clouds + raw-cloud scalar features +
nearest /lio/odometry pose & 6x6 covariance. Load the loop-closed npz → per-scan pose error
(trajraw - traj = the pseudo-GT error signal). Cache everything for Parts 2-5.

Usage:
  python src/extract.py            # full run (7619 scans)
  python src/extract.py --limit 50 # smoke test
"""
from __future__ import annotations
import argparse
import os
import time
import numpy as np

import common as C
from mcap_ros2.reader import read_ros2_messages


def _stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def stream_bag(limit: int | None, voxel: float):
    """Single streaming pass over the mcap. Returns scan + odom records."""
    scan_t, scan_pts, scan_off = [], [], [0]
    raw_count, n_rings, r_mean, r_std, r_p10 = [], [], [], [], []
    odom_t, odom_pose, odom_cov = [], [], []

    t0 = time.time()
    n_scan = 0
    for m in read_ros2_messages(C.BAG, topics=[C.TOPIC_LIDAR, C.TOPIC_ODOM]):
        topic = m.channel.topic
        msg = m.ros_msg
        if topic == C.TOPIC_ODOM:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            odom_t.append(_stamp_s(msg.header.stamp))
            odom_pose.append([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            odom_cov.append(np.asarray(msg.pose.covariance, dtype=np.float64))
        elif topic == C.TOPIC_LIDAR:
            xyz, inten, ring = C.parse_pointcloud2(msg.data)
            rng = np.linalg.norm(xyz, axis=1) if len(xyz) else np.zeros(0)
            raw_count.append(len(xyz))
            n_rings.append(int(np.unique(ring).size))
            r_mean.append(float(rng.mean()) if len(rng) else 0.0)
            r_std.append(float(rng.std()) if len(rng) else 0.0)
            r_p10.append(float(np.percentile(rng, 10)) if len(rng) else 0.0)
            ds = C.voxel_downsample(xyz, voxel)
            scan_pts.append(ds.astype(np.float32))
            scan_off.append(scan_off[-1] + len(ds))
            scan_t.append(_stamp_s(msg.header.stamp))
            n_scan += 1
            if n_scan % 500 == 0:
                print(f"  scan {n_scan}  ({time.time()-t0:.0f}s, {len(odom_t)} odom)")
            if limit and n_scan >= limit:
                break
    print(f"  streamed {n_scan} scans, {len(odom_t)} odom msgs in {time.time()-t0:.0f}s")

    return dict(
        scan_t=np.asarray(scan_t, np.float64),
        pts=np.concatenate(scan_pts, axis=0) if scan_pts else np.zeros((0, 3), np.float32),
        offsets=np.asarray(scan_off, np.int64),
        raw_count=np.asarray(raw_count, np.int32),
        n_rings=np.asarray(n_rings, np.int32),
        r_mean=np.asarray(r_mean, np.float32),
        r_std=np.asarray(r_std, np.float32),
        r_p10=np.asarray(r_p10, np.float32),
        odom_t=np.asarray(odom_t, np.float64),
        odom_pose=np.asarray(odom_pose, np.float64),
        odom_cov=np.asarray(odom_cov, np.float64),  # (M,36)
    )


def pair_odom(scan_t, odom_t, odom_pose, odom_cov):
    """Nearest-in-time odometry sample for each scan."""
    order = np.argsort(odom_t)
    ot, op, oc = odom_t[order], odom_pose[order], odom_cov[order]
    idx = np.searchsorted(ot, scan_t)
    idx = np.clip(idx, 1, len(ot) - 1)
    left, right = idx - 1, idx
    pick = np.where(np.abs(scan_t - ot[left]) <= np.abs(scan_t - ot[right]), left, right)
    dt = scan_t - ot[pick]
    return op[pick], oc[pick].reshape(-1, 6, 6), dt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--limit", type=int, default=0, help="0 = full run")
    ap.add_argument("--voxel", type=float, default=0.10)
    args = ap.parse_args()
    limit = args.limit or None

    print(f"[extract] bag = {C.BAG}")
    print(f"[extract] voxel = {args.voxel} m, limit = {limit}")
    R = stream_bag(limit, args.voxel)
    n = len(R["scan_t"])

    odom_pose, odom_cov, odom_dt = pair_odom(R["scan_t"], R["odom_t"], R["odom_pose"], R["odom_cov"])

    # ---- pseudo-GT pose error from the loop-closed npz ----
    d = np.load(C.NPZ_LOOPCLOSED)
    traj = d["traj"].astype(np.float64)        # loop-closed (pseudo-GT)
    trajraw = d["trajraw"].astype(np.float64)  # raw FAST-LIO
    if limit:
        traj, trajraw = traj[:n], trajraw[:n]
    assert len(traj) == n, f"npz has {len(traj)} poses but extracted {n} scans"
    pose_err_xyz = trajraw - traj
    pose_err = np.linalg.norm(pose_err_xyz, axis=1)

    # ---- save ----
    print("[extract] saving caches:")
    C.save_npz(os.path.join(C.CACHE, "scans.npz"),
               scan_t=R["scan_t"], pts=R["pts"], offsets=R["offsets"],
               raw_count=R["raw_count"], n_rings=R["n_rings"],
               r_mean=R["r_mean"], r_std=R["r_std"], r_p10=R["r_p10"],
               voxel=np.float32(args.voxel))
    C.save_npz(os.path.join(C.CACHE, "poses.npz"),
               odom_pose=odom_pose, odom_cov=odom_cov, odom_dt=odom_dt,
               traj=traj, trajraw=trajraw,
               pose_err_xyz=pose_err_xyz, pose_err=pose_err)

    # ---- sanity report ----
    print("\n========== PART 1 SANITY REPORT ==========")
    print(f"scans extracted        : {n}  (expected {C.N_SCANS_EXPECTED})")
    print(f"odom msgs              : {len(R['odom_t'])}")
    print(f"odom pairing |dt| (ms) : mean {1e3*np.abs(odom_dt).mean():.2f}  max {1e3*np.abs(odom_dt).max():.2f}")
    print(f"raw pts/scan           : mean {R['raw_count'].mean():.0f}  min {R['raw_count'].min()}  max {R['raw_count'].max()}")
    dspc = np.diff(R["offsets"])
    print(f"downsampled pts/scan   : mean {dspc.mean():.0f}  min {dspc.min()}  max {dspc.max()}")
    print(f"rings present/scan      : mean {R['n_rings'].mean():.1f}  min {R['n_rings'].min()}  (of {C.N_RINGS})")
    print(f"--- pseudo-GT error (trajraw - traj) ---")
    print(f"pose_err (m)           : mean {pose_err.mean():.3f}  med {np.median(pose_err):.3f}  max {pose_err.max():.3f}")
    print(f"pose_err first / last  : {pose_err[0]:.3f} / {pose_err[-1]:.3f}")
    print(f"trajraw != traj?       : max|diff| = {np.abs(pose_err_xyz).max():.3f} m  "
          f"(identical={np.allclose(traj, trajraw)})")
    print(f"odom cov diag[0:3] mean: {odom_cov[:, [0,1,2], [0,1,2]].mean(0)}")
    print("==========================================")


if __name__ == "__main__":
    main()
