"""PART 3 (reframed) — Anisotropic / directional conformal localization regions.

Core result: LiDAR localization in corridors is anisotropically uncertain — confident laterally,
blind along the corridor axis. The horizontal observability gives per-axis info (a_along,a_cross);
we form a conformal ellipse with semi-axes (q*sigma_along, q*sigma_cross) and a distribution-free
JOINT coverage guarantee. Because the error is elongated along the blind axis, the anisotropic
region is much SMALLER (area) than an isotropic conformal disk at the same coverage.

Target: the robot-frame recovery-error vector (along, cross) from injected-GT recovery.
Methods (all conformalized identically — only sigma differs):
  isotropic         sigma_along = sigma_cross = 1          (standard scalar conformal disk)
  fixed-anisotropic sigma = global cal std per axis        (constant ellipse, not adaptive)
  sys-cov           sigma from FAST-LIO EKF xy covariance  (the system's own shape, over-confident)
  obs (ours)        sigma_axis = 1/sqrt(info_axis)         (per-scan, observability-adaptive)
Outputs results/T_aniso.csv + cache/aniso.npz.
"""
from __future__ import annotations
import os, csv
import numpy as np
import sys; sys.path.insert(0, os.path.dirname(__file__))
import common as C

ALPHA = 0.10


def load():
    E = np.load(os.path.join(C.CACHE, "error.npz"))
    Sg = np.load(os.path.join(C.CACHE, "signal.npz"))
    P = np.load(os.path.join(C.CACHE, "poses.npz"))
    oi = E["odd_idx"]
    ev = E["evec_robot"]                                   # (K, R, 3) along,cross,vert
    K, R, _ = ev.shape
    ea = ev[:, :, 0].reshape(-1); ec = ev[:, :, 1].reshape(-1)   # per (scan,pert) sample
    rep = lambda v: np.repeat(v[oi], R)
    a_along = np.clip(rep(Sg["a_along"]), 1e-4, None)
    a_cross = np.clip(rep(Sg["a_cross"]), 1e-4, None)
    cov = P["odom_cov"][oi]                                # (K,6,6)
    # EKF xy covariance rotated into the robot frame (along,cross)
    cxx, cyy, cxy = cov[:, 0, 0], cov[:, 1, 1], cov[:, 0, 1]
    cs, sn = np.cos(E["yaw"]), np.sin(E["yaw"])
    s_along = cs*cs*cxx + sn*sn*cyy + 2*cs*sn*cxy
    s_cross = sn*sn*cxx + cs*cs*cyy - 2*cs*sn*cxy
    sys_a = np.clip(np.repeat(np.sqrt(np.abs(s_along)), R), 1e-6, None)
    sys_c = np.clip(np.repeat(np.sqrt(np.abs(s_cross)), R), 1e-6, None)
    ok = np.isfinite(ea) & np.isfinite(ec) & np.isfinite(a_along) & np.isfinite(a_cross)
    return dict(ea=ea[ok], ec=ec[ok], a_along=a_along[ok], a_cross=a_cross[ok],
                sys_a=sys_a[ok], sys_c=sys_c[ok], scan=np.repeat(oi, R)[ok])


def split(n, frac=0.5):
    k = int(n*frac); return np.arange(k), np.arange(k, n)


def conformalize(ea, ec, sa, sc, cal, te, alpha=ALPHA):
    """Diagonal-Mahalanobis conformal ellipse. Returns dict of metrics + per-test ellipse axes."""
    s = np.sqrt((ea/sa)**2 + (ec/sc)**2)
    q = np.quantile(s[cal], min(1.0, (1-alpha)*(1+1/len(cal))), method="higher")
    inside = s[te] <= q
    area = np.pi * q**2 * sa[te] * sc[te]
    # per-axis marginal coverage (1D interval +-q*sigma)
    cov_along = np.mean(np.abs(ea[te]) <= q*sa[te])
    cov_cross = np.mean(np.abs(ec[te]) <= q*sc[te])
    return dict(PICP=float(inside.mean()), area=float(area.mean()),
                cov_along=float(cov_along), cov_cross=float(cov_cross),
                q=float(q), semi_a=q*sa[te], semi_c=q*sc[te])


def aci(ea, ec, sa, sc, cal, te, alpha=ALPHA, gamma=0.02):
    s_cal = np.sort(np.sqrt((ea[cal]/sa[cal])**2 + (ec[cal]/sc[cal])**2))
    a_t = alpha; hit = 0; cov_run = np.empty(len(te))
    for j, i in enumerate(te):
        lvl = min(1.0, max(0.0, 1-a_t))
        q = np.quantile(s_cal, lvl, method="higher") if lvl > 0 else s_cal[-1]
        si = np.sqrt((ea[i]/sa[i])**2 + (ec[i]/sc[i])**2)
        err = 0.0 if si <= q else 1.0
        hit += (1-err); a_t += gamma*(alpha-err); cov_run[j] = hit/(j+1)
    return cov_run


def main():
    D = load()
    ea, ec = D["ea"], D["ec"]
    n = len(ea); cal, te = split(n)
    one = np.ones(n)
    fa, fc = np.std(ea[cal]), np.std(ec[cal])
    methods = {
        "isotropic":        (one, one),
        "fixed-anisotropic": (one*fa, one*fc),
        "sys-cov (EKF)":    (D["sys_a"], D["sys_c"]),
        "obs (ours)":       (1/np.sqrt(D["a_along"]), 1/np.sqrt(D["a_cross"])),
    }
    print(f"[aniso] samples={n}  cal={len(cal)} test={len(te)}  target={1-ALPHA:.0%}")
    print(f"  natural error std: along {np.std(ea):.3f}  cross {np.std(ec):.3f}  ratio {np.std(ea)/max(np.std(ec),1e-9):.2f}x")

    rows = []
    iso_area = None
    for name, (sa, sc) in methods.items():
        r = conformalize(ea, ec, sa, sc, cal, te)
        if name == "isotropic": iso_area = r["area"]
        rows.append(dict(method=name, **{k: r[k] for k in ["PICP", "area", "cov_along", "cov_cross", "q"]}))
    for r in rows:
        r["area_vs_iso"] = r["area"]/iso_area

    # ACI on ours (online coverage)
    sa, sc = methods["obs (ours)"]
    cov_run = aci(ea, ec, sa, sc, cal, te)

    # save
    rr = conformalize(ea, ec, sa, sc, cal, te)
    C.save_npz(os.path.join(C.CACHE, "aniso.npz"),
               ea_te=ea[te], ec_te=ec[te], semi_a=rr["semi_a"], semi_c=rr["semi_c"],
               scan_te=D["scan"][te], cov_run_aci=cov_run, q_ours=np.float32(rr["q"]))
    out = os.path.join(C.RESULTS, "T_aniso.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["method", "PICP", "area", "area_vs_iso", "cov_along", "cov_cross", "q"])
        w.writeheader()
        for r in rows:
            w.writerow({**r, **{k: round(r[k], 4) for k in ["PICP","area","area_vs_iso","cov_along","cov_cross","q"]}})

    print(f"\n{'method':20s} {'PICP':>6s} {'area(m2)':>9s} {'vs.iso':>7s} {'cov_along':>9s} {'cov_cross':>9s}")
    for r in rows:
        print(f"{r['method']:20s} {r['PICP']:6.3f} {r['area']:9.4f} {r['area_vs_iso']:7.2f} {r['cov_along']:9.3f} {r['cov_cross']:9.3f}")
    print(f"\nACI running coverage: start {cov_run[0]:.2f} -> end {cov_run[-1]:.3f} (target {1-ALPHA:.2f})")
    print(f"(target PICP {1-ALPHA:.2f}; smaller area = tighter; isotropic over/under-covers per-axis)")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
