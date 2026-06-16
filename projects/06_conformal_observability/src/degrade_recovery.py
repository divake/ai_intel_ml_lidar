"""PART 1 (degrade-to-reveal) — induce the corridor aperture by degrading the LIVE scan.

Reference map = EVEN scans at FULL quality (the prior map). For each ODD scan and each
max-range level R, truncate the live scan to points within R (removes far end-wall returns that
constrain forward motion), compute observability on the DEGRADED scan, inject a known
perturbation, and recover by ICP against the full map. As R shrinks the corridor aperture
appears: forward (along) becomes unobservable -> recovery fails along -> observability predicts it.

Cache cache/degrade.npz: per (scan,level) sample -> observability + recovery error vector.
"""
from __future__ import annotations
import os, time
import numpy as np
from multiprocessing import Pool, cpu_count
import sys; sys.path.insert(0, os.path.dirname(__file__))
import common as C
import registration as reg

RANGE_LEVELS = [40.0, 12.0, 8.0, 6.0, 5.0, 4.0, 3.0]   # 40 ~ full; small R = degenerate
_G = {}


def _obs_horizontal(src, Ri):
    """Horizontal walls-only observability of a (degraded) scan, robot frame."""
    n = reg.estimate_normals(src, k=12, workers=1)
    nw = n @ Ri.T
    wall = np.abs(nw[:, 2]) < 0.4
    fwd = (Ri @ np.array([1.0, 0, 0]))[:2]; fwd /= (np.linalg.norm(fwd) + 1e-9)
    lat = np.array([-fwd[1], fwd[0]])
    if wall.sum() < 15:
        return np.nan, np.nan, np.nan
    nh = nw[wall, :2]; nh /= (np.linalg.norm(nh, axis=1, keepdims=True) + 1e-9)
    Hh = (nh.T @ nh) / len(nh)
    a_along = float(fwd @ Hh @ fwd)          # info along corridor (small => blind)
    a_cross = float(lat @ Hh @ lat)
    vh = np.linalg.eigh(Hh)[1]; halign = float(abs(vh[:, 0] @ fwd))
    return a_along, a_cross, halign


def _worker(args):
    i, li, seed = args
    pts, off, R_all, t_all = _G["pts"], _G["off"], _G["R"], _G["t"]
    M, nrm, tree = _G["M"], _G["nrm"], _G["tree"]
    full = pts[off[i]:off[i+1]].astype(np.float64)
    rng_pt = np.linalg.norm(full, axis=1)
    src = full[rng_pt <= RANGE_LEVELS[li]]
    if len(src) < 50:
        return None
    Ri = R_all[i]
    a_along, a_cross, halign = _obs_horizontal(src, Ri)
    Ttrue = np.eye(4); Ttrue[:3, :3] = Ri; Ttrue[:3, 3] = t_all[i]
    rg = np.random.default_rng(seed)
    d = rg.normal(size=3); d /= (np.linalg.norm(d) + 1e-12); d *= rg.uniform(0.2, 1.0)
    Tin = Ttrue.copy(); Tin[:3, 3] += d
    Trec, info = reg.point_to_plane_icp(src, Tin, M, nrm, tree, iters=30, max_corr=1.0)
    e = Ri.T @ (Trec[:3, 3] - t_all[i])      # robot frame [along, cross, vert]
    return dict(i=i, li=li, R=RANGE_LEVELS[li], a_along=a_along, a_cross=a_cross,
                halign=halign, npts=len(src),
                e_along=float(e[0]), e_cross=float(e[1]), e_vert=float(e[2]),
                resid=info["resid"])


def main():
    ap = __import__("argparse").ArgumentParser()
    ap.add_argument("--procs", type=int, default=40)
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    S = np.load(os.path.join(C.CACHE, "scans.npz")); P = np.load(os.path.join(C.CACHE, "poses.npz"))
    pts, off = S["pts"], S["offsets"]
    R_all = np.stack([reg.pose_to_T([0, 0, 0], q)[:3, :3] for q in P["odom_pose"][:, 3:7]])
    t_all = P["traj"].astype(np.float64)
    n = off.size - 1
    even = np.arange(0, n, 2); odd = np.arange(1, n, 2)
    if args.limit:
        even = even[even < args.limit]; odd = odd[odd < args.limit]

    print(f"[degrade] building FULL reference map from {len(even)} even scans ...", flush=True)
    t0 = time.time()
    M, nrm, tree = reg.build_reference_map(even, pts, off, R_all, t_all, voxel=0.10)
    print(f"  map {len(M)} pts in {time.time()-t0:.0f}s", flush=True)
    _G.update(pts=pts, off=off, R=R_all, t=t_all, M=M, nrm=nrm, tree=tree)

    jobs = [(int(i), li, int(i)*13 + li + 1) for i in odd for li in range(len(RANGE_LEVELS))]
    print(f"[degrade] {len(odd)} scans x {len(RANGE_LEVELS)} range levels = {len(jobs)} recoveries, {args.procs} procs", flush=True)
    t0 = time.time()
    with Pool(args.procs) as pool:
        res = [r for r in pool.map(_worker, jobs, chunksize=16) if r]
    print(f"  done in {time.time()-t0:.0f}s  ({len(res)} valid)", flush=True)

    keys = ["i", "li", "R", "a_along", "a_cross", "halign", "npts", "e_along", "e_cross", "e_vert", "resid"]
    A = {k: np.array([r[k] for r in res]) for k in keys}
    C.save_npz(os.path.join(C.CACHE, "degrade.npz"), **A)

    # ---- the decisive check: does observability now predict along-error? ----
    print("\n========== DEGRADE-TO-REVEAL REPORT ==========")
    print(f"{'R(m)':>6s} {'npts':>6s} {'|e_along|':>9s} {'|e_cross|':>9s} {'along/cross':>11s} {'a_along':>8s} {'fail>0.3':>8s}")
    for li, R in enumerate(RANGE_LEVELS):
        m = A["li"] == li
        ea, ec = np.abs(A["e_along"][m]), np.abs(A["e_cross"][m])
        print(f"{R:6.1f} {A['npts'][m].mean():6.0f} {ea.mean():9.3f} {ec.mean():9.3f} "
              f"{ea.mean()/max(ec.mean(),1e-9):11.2f} {np.nanmean(A['a_along'][m]):8.4f} {100*np.mean(np.abs(A['e_along'][m])>0.3):7.1f}%")
    ok = np.isfinite(A["a_along"]) & np.isfinite(A["e_along"])
    unc = 1.0 / np.clip(A["a_along"][ok], 1e-3, None)
    from scipy.stats import spearmanr
    print(f"\nSpearman(1/a_along, |e_along|) = {spearmanr(unc, np.abs(A['e_along'][ok]))[0]:+.3f}   (want strongly POSITIVE)")
    print(f"Spearman(1/a_cross, |e_cross|) = {spearmanr(1/np.clip(A['a_cross'][ok],1e-3,None), np.abs(A['e_cross'][ok]))[0]:+.3f}")
    print(f"Pearson (1/a_along, |e_along|) = {np.corrcoef(unc, np.abs(A['e_along'][ok]))[0,1]:+.3f}")
    print("==============================================")


if __name__ == "__main__":
    main()
