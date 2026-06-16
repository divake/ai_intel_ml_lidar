"""PART 5 — figures. Plots P1-P7 (matplotlib) + maps M1-M5 (reuse proj-04 GPU rasterizer).

Run after parts 1-4 have populated cache/ and results/. Each function is independent so we can
regenerate one figure at a time.
"""
from __future__ import annotations
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import sys; sys.path.insert(0, os.path.dirname(__file__))
import common as C

plt.rcParams.update({"figure.dpi": 130, "font.size": 11, "axes.grid": True, "grid.alpha": 0.3})


def _save(fig, name):
    p = os.path.join(C.FIGS, name)
    fig.savefig(p, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"  figs/{name}")


# ---------------------------------------------------------------- plots
def p1_sparsification():
    """P1 — sparsification curves: observability vs constant vs oracle."""
    Cf = np.load(os.path.join(C.CACHE, "conformal.npz"))
    y = Cf["y_test"]; sig = Cf["sig_obs"]
    n = len(y); fr = np.linspace(0, 0.9, 19)
    def curve(rank):                       # keep least-uncertain (lowest rank) (1-f) fraction
        order = np.argsort(rank)
        return np.array([y[order][:max(int(n*(1-f)), 1)].mean() for f in fr])
    obs = curve(sig); ora = curve(y); rnd = curve(np.zeros(n))   # rnd: no ranking -> stable order
    fig, ax = plt.subplots(figsize=(5.2, 4))
    ax.plot(fr, obs/obs[0], "-o", ms=3, label="observability (ours)")
    ax.plot(fr, rnd/rnd[0], "--", label="random / constant")
    ax.plot(fr, ora/ora[0], "-", color="k", label="oracle")
    ax.set_xlabel("fraction of most-uncertain scans removed")
    ax.set_ylabel("normalized mean error of remainder")
    ax.set_title("P1 · Sparsification (lower = uncertainty tracks error)")
    ax.legend()
    _save(fig, "P1_sparsification.png")


def p2_calibration():
    """P2 — coverage vs nominal for split-conformal(obs) vs temp-scaling(syscov)."""
    import conformal as CF
    y, sig, _ = CF.load_dataset()
    cal, te = CF.temporal_split(len(y))
    alphas = np.linspace(0.02, 0.4, 15)
    cov_obs, cov_sys = [], []
    for a in alphas:
        U, _ = CF.split_conformal(y, sig["obs_1/lam"], cal, te, alpha=a)
        cov_obs.append(np.mean(y[te] <= U))
        Ut, _ = CF.temp_scaling(y, sig["sys_cov"], cal, te, alpha=a)
        cov_sys.append(np.mean(y[te] <= Ut))
    nom = 1 - alphas
    fig, ax = plt.subplots(figsize=(5, 4.6))
    ax.plot([0, 1], [0, 1], "k--", lw=1, label="ideal")
    ax.plot(nom, cov_obs, "-o", ms=3, label="split-conformal · obs (ours)")
    ax.plot(nom, cov_sys, "-s", ms=3, label="temp-scale · sys-cov")
    ax.set_xlabel("nominal coverage 1−α"); ax.set_ylabel("empirical coverage")
    ax.set_title("P2 · Calibration"); ax.legend(); ax.set_xlim(0.55, 1); ax.set_ylim(0.4, 1.02)
    _save(fig, "P2_calibration.png")


def p3_running_coverage():
    """P3 — running coverage on the stream: split (drifts) vs ACI/PID (recover to nominal)."""
    Cf = np.load(os.path.join(C.CACHE, "conformal.npz"))
    y = Cf["y_test"]; Us = Cf["U_split"]
    cov_split = np.cumsum(y <= Us) / np.arange(1, len(y)+1)
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.axhline(0.9, color="k", ls="--", lw=1, label="target 0.90")
    ax.plot(cov_split, label="split conformal")
    ax.plot(Cf["cov_aci"], label="ACI (ours)")
    ax.plot(Cf["cov_pid"], label="Conformal-PID (ours)")
    ax.set_xlabel("test scan (stream order)"); ax.set_ylabel("running coverage")
    ax.set_title("P3 · Online coverage on the corridor stream"); ax.legend(); ax.set_ylim(0.6, 1.0)
    _save(fig, "P3_running_coverage.png")


def p6_directional():
    """P6 — recovery error along vs cross corridor (the aperture asymmetry)."""
    E = np.load(os.path.join(C.CACHE, "error.npz"))
    fig, ax = plt.subplots(figsize=(5, 4))
    ax.boxplot([E["resid_cross"], E["resid_along"]], labels=["cross\n(lateral)", "along\n(corridor axis)"],
               showfliers=False)
    ax.set_ylabel(f"residual after recovering a {float(E['dpert']):.1f} m perturbation (m)")
    ax.set_title("P6 · Aperture: along-corridor error is unrecoverable")
    _save(fig, "P6_directional.png")


# ---------------------------------------------------------------- maps (GPU)
def _renderer():
    sys.path.insert(0, os.path.join(C.ROOT, "projects/04_demo_viz/render"))
    from gpu_raster import Renderer
    return Renderer


def _topdown(R, xyz, rgb, pad=3.0, W=2000, H=1400):
    ctr = xyz.mean(0)
    span = max(xyz[:, 0].ptp(), xyz[:, 1].ptp()) + pad
    eye = np.array([ctr[0], ctr[1], ctr[2] + span*1.1], np.float32)
    Rnd = R(W=W, H=H, ssaa=2)
    Rnd.set_cloud(xyz.astype(np.float32), rgb.astype(np.float32))
    return Rnd.render(eye, ctr.astype(np.float32), up=(1, 0, 0), fov_deg=55,
                      bg=(0.04, 0.045, 0.06), point_px=1.6, edl=0.85, bloom=0.5)


def m1_observability_map():
    """M1 — trajectory colored by observability (hot=degenerate) over the dim map."""
    Renderer = _renderer()
    vm = np.load(C.VOX_MAP_CACHE)
    vox = vm["vox"].astype(np.float32)
    Sg = np.load(os.path.join(C.CACHE, "signal.npz"))
    P = np.load(os.path.join(C.CACHE, "poses.npz"))
    traj = P["trajraw"].astype(np.float32)
    lam = Sg["lam_min_t"]
    unc = np.clip(1.0/np.clip(lam, 1e-3, None), 0, None)
    unc = (unc - np.percentile(unc, 5)) / (np.percentile(unc, 95) - np.percentile(unc, 5) + 1e-9)
    unc = np.clip(unc, 0, 1)
    import matplotlib.cm as cm
    bg_rgb = np.full((len(vox), 3), 0.18, np.float32)            # dim grey map
    traj_rgb = cm.inferno(unc)[:, :3].astype(np.float32)         # hot = uncertain
    xyz = np.concatenate([vox, traj], 0)
    rgb = np.concatenate([bg_rgb, traj_rgb], 0)
    img = _topdown(Renderer, xyz, rgb)
    import matplotlib.image as mpimg
    mpimg.imsave(os.path.join(C.FIGS, "M1_observability_map.png"), img)
    print("  figs/M1_observability_map.png")


ALL = {"P1": p1_sparsification, "P2": p2_calibration, "P3": p3_running_coverage,
       "P6": p6_directional, "M1": m1_observability_map}


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*", default=list(ALL))
    args = ap.parse_args()
    for k in args.only:
        try:
            ALL[k]()
        except Exception as e:
            print(f"  [skip {k}] {type(e).__name__}: {e}")


if __name__ == "__main__":
    main()
