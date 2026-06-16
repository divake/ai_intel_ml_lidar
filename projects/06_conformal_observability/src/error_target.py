"""PART 1 (error target) — injected-GT recovery.

Reference map = EVEN scans (transformed to map frame). On ODD scans we inject a KNOWN pose
perturbation and recover it by point-to-plane ICP against that map; the residual ‖recovered −
true‖ is the per-scan localization error with TRUE ground truth (non-circular: test scan ∉ map).

Outputs cache/error.npz:
  odd_idx           (K,)         odd scan indices (the test set)
  resid_rand        (K,R)        residual error (m) after recovering R random perturbations
  dmag_rand         (K,R)        the injected magnitudes
  resid_along       (K,)         residual after a 0.5 m perturbation ALONG robot heading (corridor axis)
  resid_cross       (K,)         residual after a 0.5 m perturbation CROSS (lateral)
  plane_resid,ninl  (K,)         ICP final plane RMS + inlier count (random[0])
  yaw               (K,)         heading
"""
from __future__ import annotations
import argparse, os, time
import numpy as np
from multiprocessing import Pool, cpu_count
import sys; sys.path.insert(0, os.path.dirname(__file__))
import common as C
import registration as reg

DPERT = 0.5          # directional perturbation magnitude (m)
N_RAND = 2           # random perturbations per odd scan
RAND_LO, RAND_HI = 0.2, 1.0

# globals shared with workers via fork
_G = {}

def _worker(args):
    i, seed = args
    pts, off = _G["pts"], _G["off"]
    R_all, t_all = _G["R"], _G["t"]
    M, nrm, tree = _G["M"], _G["nrm"], _G["tree"]
    src = pts[off[i]:off[i+1]].astype(np.float64)
    Ttrue = np.eye(4); Ttrue[:3, :3] = R_all[i]; Ttrue[:3, 3] = t_all[i]
    rng = np.random.default_rng(seed)
    out = dict(idx=i)
    Rt = R_all[i]            # world<-robot; robot-frame error = Rt^T @ world_error
    # ---- random perturbations (conformal dataset) ----
    rr, dm, evec = [], [], []
    for r in range(N_RAND):
        d = rng.normal(size=3); d /= (np.linalg.norm(d) + 1e-12); d *= rng.uniform(RAND_LO, RAND_HI)
        Tin = Ttrue.copy(); Tin[:3, 3] = Ttrue[:3, 3] + d
        Trec, info = reg.point_to_plane_icp(src, Tin, M, nrm, tree, iters=30, max_corr=1.0)
        e_world = Trec[:3, 3] - t_all[i]
        e_robot = Rt.T @ e_world                     # [along, cross, vert]
        rr.append(float(np.linalg.norm(e_world)))
        evec.append(e_robot.tolist())
        dm.append(float(np.linalg.norm(d)))
        if r == 0:
            out["plane_resid"] = info["resid"]; out["ninl"] = info["ninl"]
    out["resid_rand"] = rr; out["dmag_rand"] = dm; out["evec_robot"] = evec
    # ---- directional: along heading (corridor axis) vs cross (lateral) ----
    fwd = R_all[i] @ np.array([1.0, 0, 0]); fwd /= np.linalg.norm(fwd)
    lat = R_all[i] @ np.array([0, 1.0, 0]); lat /= np.linalg.norm(lat)
    for key, vec in (("along", fwd), ("cross", lat)):
        Tin = Ttrue.copy(); Tin[:3, 3] = Ttrue[:3, 3] + DPERT * vec
        Trec, _ = reg.point_to_plane_icp(src, Tin, M, nrm, tree, iters=30, max_corr=1.0)
        out["resid_" + key] = float(np.linalg.norm(Trec[:3, 3] - t_all[i]))
    out["yaw"] = float(np.arctan2(R_all[i][1, 0], R_all[i][0, 0]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--voxel", type=float, default=0.10)
    ap.add_argument("--procs", type=int, default=min(24, cpu_count()))
    args = ap.parse_args()

    S = np.load(os.path.join(C.CACHE, "scans.npz"))
    P = np.load(os.path.join(C.CACHE, "poses.npz"))
    pts, off = S["pts"], S["offsets"]
    quat, traj = P["odom_pose"][:, 3:7], P["traj"]
    n = len(traj)
    R_all = np.stack([reg.pose_to_T([0, 0, 0], q)[:3, :3] for q in quat])
    t_all = traj.astype(np.float64)

    even = np.arange(0, n, 2)
    odd = np.arange(1, n, 2)
    if args.limit:
        keep = args.limit
        even = even[even < keep]; odd = odd[odd < keep]

    print(f"[error] building reference map from {len(even)} even scans ...")
    t0 = time.time()
    M, nrm, tree = reg.build_reference_map(even, pts, off, R_all, t_all, voxel=args.voxel)
    print(f"  map {len(M)} pts, normals+tree in {time.time()-t0:.0f}s")

    _G.update(pts=pts, off=off, R=R_all, t=t_all, M=M, nrm=nrm, tree=tree)

    print(f"[error] injected-GT recovery on {len(odd)} odd scans, {args.procs} procs ...")
    t0 = time.time()
    jobs = [(int(i), int(i) * 7 + 1) for i in odd]
    with Pool(args.procs) as pool:
        res = pool.map(_worker, jobs, chunksize=16)
    print(f"  done in {time.time()-t0:.0f}s")

    res.sort(key=lambda d: d["idx"])
    odd_idx = np.array([r["idx"] for r in res])
    resid_rand = np.array([r["resid_rand"] for r in res])
    evec_robot = np.array([r["evec_robot"] for r in res])      # (K, N_RAND, 3) [along,cross,vert]
    dmag_rand = np.array([r["dmag_rand"] for r in res])
    resid_along = np.array([r["resid_along"] for r in res])
    resid_cross = np.array([r["resid_cross"] for r in res])
    plane_resid = np.array([r.get("plane_resid", np.nan) for r in res])
    ninl = np.array([r.get("ninl", 0) for r in res])
    yaw = np.array([r["yaw"] for r in res])

    C.save_npz(os.path.join(C.CACHE, "error.npz"),
               odd_idx=odd_idx, resid_rand=resid_rand, evec_robot=evec_robot, dmag_rand=dmag_rand,
               resid_along=resid_along, resid_cross=resid_cross,
               plane_resid=plane_resid, ninl=ninl, yaw=yaw, dpert=np.float32(DPERT))
    # quick anisotropy check on the natural (random-perturbation) error vectors
    ev = evec_robot.reshape(-1, 3)
    print(f"\nnatural recovery-error vector (robot frame), |component| mean:")
    print(f"  along {np.abs(ev[:,0]).mean():.4f}  cross {np.abs(ev[:,1]).mean():.4f}  vert {np.abs(ev[:,2]).mean():.4f}  "
          f"(along/cross = {np.abs(ev[:,0]).mean()/max(np.abs(ev[:,1]).mean(),1e-9):.2f}x)")

    rr = resid_rand.ravel()
    print("\n========== ERROR-TARGET REPORT ==========")
    print(f"odd test scans         : {len(odd_idx)}   (x{N_RAND} random = {rr.size} samples)")
    print(f"injected mag (m)       : mean {dmag_rand.mean():.3f}")
    print(f"residual after recovery: mean {rr.mean():.3f}  med {np.median(rr):.3f}  p90 {np.percentile(rr,90):.3f}  max {rr.max():.3f}")
    print(f"  recovered (<0.1m)    : {100*np.mean(rr<0.1):.1f}%   failed (>0.3m): {100*np.mean(rr>0.3):.1f}%")
    print(f"directional @ {DPERT} m:")
    print(f"  resid ALONG heading  : mean {resid_along.mean():.3f}  med {np.median(resid_along):.3f}  (corridor axis → should be WORSE)")
    print(f"  resid CROSS lateral  : mean {resid_cross.mean():.3f}  med {np.median(resid_cross):.3f}")
    print(f"  along/cross ratio    : {resid_along.mean()/max(resid_cross.mean(),1e-6):.2f}x")
    print(f"plane resid (m)        : mean {np.nanmean(plane_resid):.4f}   inliers mean {ninl.mean():.0f}")
    print("=========================================")


if __name__ == "__main__":
    main()
