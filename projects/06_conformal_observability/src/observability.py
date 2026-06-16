"""PART 2 — Observability signal (map-free, per scan, single-pass).

For every scan: surface normals (KNN+PCA) → translation constraint matrix T=(Σ nnᵀ)/N and full
6x6 C=(Σ [a×n;n][·]ᵀ)/N. The scale-free degeneracy signals:
  lam_min_t   smallest eigenvalue of T  (∈[0,1]; ~0 = a blind translation axis, ~0.33 isotropic)
  kappa_t     condition number of T (λ_max/λ_min)  (large = degenerate)
  blind_dir   eigenvector of lam_min_t, in WORLD frame (should ∥ corridor axis in a corridor)
  heading_align = |blind_dir · forward|  (≈1 in a straight corridor)
Plus density / range / ring features. Cache cache/signal.npz (all scans).

Then validate: does the signal predict the injected-GT recovery error? (AUSE + correlation.)
"""
from __future__ import annotations
import os, time
import numpy as np
from multiprocessing import Pool, cpu_count
import sys; sys.path.insert(0, os.path.dirname(__file__))
import common as C
import registration as reg

_G = {}

def _worker(i):
    pts, off, R = _G["pts"], _G["off"], _G["R"]
    p = pts[off[i]:off[i+1]].astype(np.float64)
    if len(p) < 25:
        return dict(idx=i, bad=True)
    n = reg.estimate_normals(p, k=15, workers=1)  # workers=1: we're already inside a Pool
    a = p @ R[i].T          # world-oriented points (rotation center = sensor origin)
    nw = n @ R[i].T         # world normals
    N = len(p)
    # full 3D translation block (Σ nnᵀ)/N  — dominated by floor/ceiling (vertical normals)
    Tm = (nw.T @ nw) / N
    wt, vt = np.linalg.eigh(Tm)
    fwd = R[i] @ np.array([1.0, 0, 0]); fwd /= np.linalg.norm(fwd)
    # HORIZONTAL observability (walls only) — the part that constrains x,y localization
    wall = np.abs(nw[:, 2]) < 0.4
    f2 = fwd[:2] / (np.linalg.norm(fwd[:2]) + 1e-9)      # forward (along) in world-horizontal
    l2 = np.array([-f2[1], f2[0]])                        # lateral (cross)
    if wall.sum() >= 20:
        nh = nw[wall, :2]; nh /= (np.linalg.norm(nh, axis=1, keepdims=True) + 1e-9)
        Hh = (nh.T @ nh) / len(nh)               # 2x2 horizontal information
        wh, vh = np.linalg.eigh(Hh)              # ascending: wh[0]=blind, wh[1]=well-constrained
        blind_h = vh[:, 0]
        lam_h_min, lam_h_max = float(wh[0]), float(wh[1])
        halign_h = float(abs(blind_h @ f2))
        a_along = float(f2 @ Hh @ f2)            # info along robot-forward  (small => uncertain)
        a_cross = float(l2 @ Hh @ l2)            # info along robot-lateral
        wall_frac = float(wall.mean())
    else:
        blind_h = np.array([1.0, 0.0]); lam_h_min = lam_h_max = a_along = a_cross = np.nan
        halign_h = np.nan; wall_frac = 0.0
    Cf = reg.constraint_matrix(a, nw) / N
    wf = np.linalg.eigvalsh(Cf)
    return dict(idx=i, bad=False,
                lam_t=wt.astype(np.float32),
                blind_h=blind_h.astype(np.float32), lam_h_min=lam_h_min, lam_h_max=lam_h_max,
                a_along=a_along, a_cross=a_cross,
                heading_align_h=halign_h, wall_frac=wall_frac,
                lam6_min=float(wf[0]), lam6_max=float(wf[-1]), npts=N)


def main():
    S = np.load(os.path.join(C.CACHE, "scans.npz"))
    pts, off = S["pts"], S["offsets"]
    raw_count, n_rings = S["raw_count"], S["n_rings"]
    r_mean, r_std, r_p10 = S["r_mean"], S["r_std"], S["r_p10"]
    P = np.load(os.path.join(C.CACHE, "poses.npz"))
    R_all = np.stack([reg.pose_to_T([0, 0, 0], q)[:3, :3] for q in P["odom_pose"][:, 3:7]])
    n = off.size - 1
    _G.update(pts=pts, off=off, R=R_all)

    print(f"[obs] computing observability for {n} scans ...")
    t0 = time.time()
    with Pool(min(24, cpu_count())) as pool:
        res = pool.map(_worker, range(n), chunksize=32)
    print(f"  done in {time.time()-t0:.0f}s")
    res.sort(key=lambda d: d["idx"])

    lam_t = np.array([r.get("lam_t", [np.nan]*3) for r in res])   # (n,3)
    blind_h = np.array([r.get("blind_h", [np.nan]*2) for r in res])      # (n,2) horizontal blind dir
    lam_h_min = np.array([r.get("lam_h_min", np.nan) for r in res])
    lam_h_max = np.array([r.get("lam_h_max", np.nan) for r in res])
    a_along = np.array([r.get("a_along", np.nan) for r in res])
    a_cross = np.array([r.get("a_cross", np.nan) for r in res])
    halign_h = np.array([r.get("heading_align_h", np.nan) for r in res])
    wall_frac = np.array([r.get("wall_frac", np.nan) for r in res])
    lam6_min = np.array([r.get("lam6_min", np.nan) for r in res])
    lam6_max = np.array([r.get("lam6_max", np.nan) for r in res])

    lam_min_t = lam_t[:, 0]
    kappa_t = lam_t[:, 2] / np.clip(lam_t[:, 0], 1e-9, None)
    # horizontal anisotropy ratio: how elongated the uncertainty is (well/blind). >1, large=anisotropic
    aniso_h = np.clip(lam_h_max, 1e-6, None) / np.clip(lam_h_min, 1e-6, None)

    C.save_npz(os.path.join(C.CACHE, "signal.npz"),
               lam_t=lam_t, lam_min_t=lam_min_t, kappa_t=kappa_t,
               blind_h=blind_h, lam_h_min=lam_h_min, lam_h_max=lam_h_max,
               a_along=a_along, a_cross=a_cross,
               aniso_h=aniso_h, heading_align_h=halign_h, wall_frac=wall_frac,
               lam6_min=lam6_min, lam6_max=lam6_max,
               density=raw_count.astype(np.float32), n_rings=n_rings.astype(np.float32),
               r_mean=r_mean, r_std=r_std, r_p10=r_p10)

    # ---- validation vs injected-GT recovery error (odd scans) ----
    print("\n========== OBSERVABILITY SIGNAL REPORT ==========")
    print(f"HORIZONTAL observability (walls only):")
    print(f"  blind_h · forward      : med {np.nanmedian(halign_h):.3f}   frac>0.7 (forward IS blind axis): {100*np.nanmean(halign_h>0.7):.1f}%")
    print(f"  anisotropy ratio (max/min): med {np.nanmedian(aniso_h):.2f}  p90 {np.nanpercentile(aniso_h,90):.2f}")
    print(f"  wall-point fraction    : med {np.nanmedian(wall_frac):.2f}")

    epath = os.path.join(C.CACHE, "error.npz")
    if os.path.exists(epath):
        E = np.load(epath)
        oi = E["odd_idx"]
        ha = halign_h[oi]
        ra, rc = E["resid_along"], E["resid_cross"]
        ok = np.isfinite(ha) & np.isfinite(ra)
        fb = ha[ok] > 0.6                          # forward is the blind axis
        print(f"\n-- aperture validation (odd scans, n={ok.sum()}) --")
        print(f"forward-blind scans ({100*fb.mean():.0f}%): resid_along {ra[ok][fb].mean():.3f} vs cross {rc[ok][fb].mean():.3f}  "
              f"({ra[ok][fb].mean()/max(rc[ok][fb].mean(),1e-6):.2f}x)")
        print(f"other scans         ({100*(~fb).mean():.0f}%): resid_along {ra[ok][~fb].mean():.3f} vs cross {rc[ok][~fb].mean():.3f}  "
              f"({ra[ok][~fb].mean()/max(rc[ok][~fb].mean(),1e-6):.2f}x)")
        ev = E["evec_robot"].reshape(-1, 3)
        print(f"natural error |along|/|cross| = {np.abs(ev[:,0]).mean()/max(np.abs(ev[:,1]).mean(),1e-9):.2f}x")
    print("================================================")


def ause(err, unc):
    """Area Under Sparsification Error vs oracle (>=0; lower=better).
    Remove the most-uncertain fraction f, KEEP the least-uncertain rest, plot remaining mean error."""
    n = len(err); fr = np.linspace(0, 0.9, 19)
    def curve(rank):                       # rank ascending: keep the first k (lowest-rank = kept)
        order = np.argsort(rank)
        return np.array([err[order][:max(int(n*(1-f)), 1)].mean() for f in fr])
    by_unc = curve(unc)                    # keep least-uncertain
    by_err = curve(err)                    # oracle: keep lowest-error
    by_unc /= (by_unc[0]+1e-12); by_err /= (by_err[0]+1e-12)
    return float(np.trapz(by_unc - by_err, fr))


if __name__ == "__main__":
    main()
