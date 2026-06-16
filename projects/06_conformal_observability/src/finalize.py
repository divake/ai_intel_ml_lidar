"""FINAL — honest 'sensor-quality + conformal safety' story. Tables + figures from caches.

Acts: (1) localization reliability vs sensing range [degradation curve]; (2) the EKF is grossly
over-confident; (3) conformal prediction gives calibrated, online (ACI), anisotropic bounds the
EKF can't; (4) risk-aware speed control gated on sensing quality. Honest signal = scan density/
range (directly observable, AUROC~0.62), conformalized.
"""
from __future__ import annotations
import os, csv
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import sys; sys.path.insert(0, os.path.dirname(__file__))
import common as C

# Presentation style: Times-family serif (Nimbus Roman = metric-compatible Times clone if TNR
# absent), no titles (added on the slide), thin clean spines, non-overlapping legends.
plt.rcParams.update({
    "figure.dpi": 200, "savefig.dpi": 200,
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 13, "axes.labelsize": 14, "legend.fontsize": 12,
    "xtick.labelsize": 12, "ytick.labelsize": 12,
    "axes.grid": True, "grid.alpha": 0.25, "grid.linewidth": 0.6,
    "axes.linewidth": 0.9, "legend.frameon": True, "legend.framealpha": 0.92,
    "legend.edgecolor": "0.8", "legend.fancybox": False,
})
TAU = 0.30; TARGET = 0.90


def _clean(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def _save(fig, name):
    fig.tight_layout()
    fig.savefig(os.path.join(C.FIGS, name), bbox_inches="tight", facecolor="white"); plt.close(fig)
    print(f"  figs/{name}")


def load():
    D = np.load(os.path.join(C.CACHE, "degrade.npz"))
    P = np.load(os.path.join(C.CACHE, "poses.npz"))
    ekf_sig = np.sqrt(np.clip(P["odom_cov"][:, 0, 0] + P["odom_cov"][:, 1, 1], 1e-12, None))  # per scan
    eh = np.sqrt(D["e_along"]**2 + D["e_cross"]**2)
    return D, P, ekf_sig, eh


def degradation_table(D, eh):
    levels = np.unique(D["li"]); rows = []
    for li in levels:
        m = D["li"] == li
        rows.append(dict(R=float(D["R"][m][0]), npts=float(D["npts"][m].mean()),
                         e_along=float(np.abs(D["e_along"][m]).mean()),
                         e_cross=float(np.abs(D["e_cross"][m]).mean()),
                         ratio=float(np.abs(D["e_along"][m]).mean()/max(np.abs(D["e_cross"][m]).mean(),1e-9)),
                         fail=float(np.mean(eh[m] > TAU)),
                         err_med=float(np.median(eh[m]))))
    rows.sort(key=lambda r: -r["R"])
    with open(os.path.join(C.RESULTS, "T_degradation.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader()
        for r in rows: w.writerow({k: round(v, 4) for k, v in r.items()})
    return rows


def overconfidence(D, ekf_sig, eh):
    """What fraction does the EKF's nominal 90% interval actually contain?"""
    sig = ekf_sig[D["i"]]
    ekf_90 = 1.645 * sig                       # 1D 90% half-width
    actual_cov = float(np.mean(eh <= ekf_90))
    # factor by which EKF under-states uncertainty (median)
    factor = float(np.median(eh) / np.median(sig))
    return dict(ekf_sigma_mm=float(np.median(sig)*1e3), ekf_90_mm=float(np.median(ekf_90)*1e3),
                actual_err_med_mm=float(np.median(eh)*1e3), ekf_claimed=0.90,
                ekf_actual_coverage=actual_cov, underconf_factor=factor)


def safe_action(D, ekf_sig, eh):
    """FAIL-Detect-style gate on observable sensing quality (1/sqrt density). Threshold calibrated
    on GOOD (full-range) scans for a 10% false-slow guarantee; selective by construction."""
    npts = np.clip(D["npts"], 1, None); scan = D["i"]; R = D["R"]
    z = 1.0/np.sqrt(npts)                         # higher = sparser = worse sensing (observable)
    good = R >= R.max()                           # full-range scans = calibration of "normal"
    tau_slow = np.quantile(z[good], 0.90)         # <=10% false-slow on good sensing
    tau_stop = np.quantile(z[good], 0.98)         # <=2% false-stop on good sensing
    dang = eh > TAU
    z_ekf = 1.645*ekf_sig[scan]                   # EKF "uncertainty" — mm-scale, ~constant w/ degradation

    def gate(zz):
        ts, tp = np.quantile(zz[good], 0.90), np.quantile(zz[good], 0.98)
        v = np.ones(len(zz)); v[zz > ts] = 0.3; v[zz > tp] = 0.0; return v

    def mkstats(v):
        return dict(mean_speed=float(v.mean()), unsafe=int(np.sum((v > 0.3) & dang)),
                    frac_slowed=float(np.mean(v < 1.0)), v=v)
    pol = {"ungated": mkstats(np.ones(len(z))),
           "EKF-gated": mkstats(gate(z_ekf)),
           "conformal-gated (ours)": mkstats(gate(z))}
    # per-level slow rate + unsafe for the conformal gate vs ungated
    vc = pol["conformal-gated (ours)"]["v"]
    perlevel = []
    for li in np.unique(D["li"]):
        m = D["li"] == li
        perlevel.append(dict(R=float(R[m][0]), slow=float(np.mean(vc[m] < 1.0)),
                             unsafe_gated=int(np.sum((vc[m] > 0.3) & dang[m])),
                             unsafe_ungated=int(np.sum(dang[m]))))
    perlevel.sort(key=lambda r: -r["R"])
    # Pareto: sweep the false-slow budget
    pareto = []
    for fa in np.linspace(0.0, 0.6, 25):
        ts = np.quantile(z[good], 1-fa)
        v = np.ones(len(z)); v[z > ts] = 0.0
        pareto.append((float(v.mean()), int(np.sum((v > 0.3) & dang))))
    with open(os.path.join(C.RESULTS, "T_safety.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["policy", "mean_speed", "unsafe_events", "frac_slowed", "n_danger"])
        for k, s2 in pol.items(): w.writerow([k, round(s2["mean_speed"],3), s2["unsafe"], round(s2["frac_slowed"],3), int(dang.sum())])
    return pol, np.array(pareto), int(dang.sum()), perlevel


# ----------------------------------------------------------------- figures
BLUE, RED, GREEN, INK = "#1f5fb4", "#c0392b", "#1f8a4c", "#2b2b2b"


def fig_degradation(rows):
    R = [r["R"] for r in rows]
    fig, ax = plt.subplots(figsize=(6.4, 4.3))
    ax.plot(R, [r["e_along"] for r in rows], "-o", color=RED, lw=2, ms=6, label="along-corridor error")
    ax.plot(R, [r["e_cross"] for r in rows], "-s", color=BLUE, lw=2, ms=6, label="cross (lateral) error")
    ax.set_xlabel("LiDAR effective range (m)   —  degradation →")
    ax.set_ylabel("recovery error (m)")
    ax.invert_xaxis(); ax.spines["top"].set_visible(False)
    ax2 = ax.twinx()
    ax2.plot(R, [100*r["fail"] for r in rows], "--^", color=INK, lw=1.8, ms=6, alpha=0.7, label="failure rate")
    ax2.set_ylabel("failure rate (> 0.3 m), %"); ax2.grid(False); ax2.spines["top"].set_visible(False)
    h1, l1 = ax.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc="upper left", borderaxespad=0.6)
    _save(fig, "F1_degradation_curve.png")


def fig_overconfidence(oc, D, ekf_sig, eh):
    fig, ax = plt.subplots(figsize=(6.4, 4.3))
    ax.hist(np.clip(eh, 0, 1.0), bins=60, color=BLUE, alpha=0.75, edgecolor="white", linewidth=0.2,
            label="actual localization error")
    ax.axvline(oc["ekf_90_mm"]/1e3, color=RED, lw=2.2, label=f"EKF “90%” bound ({oc['ekf_90_mm']:.0f} mm)")
    ax.set_xlabel("localization error (m)"); ax.set_ylabel("count"); ax.set_xlim(0, 1.0)
    _clean(ax); ax.legend(loc="upper right", borderaxespad=0.6)
    _save(fig, "F2_overconfidence.png")


def fig_calibration():
    try:
        import degrade_conformal as DC
        Dd = DC.load(); cal, te = DC.split_by_scan(Dd["scan"])
        ea, ec = Dd["ea"], Dd["ec"]; aa, ac = Dd["aa"], Dd["ac"]
        meth = {"isotropic": (np.ones_like(ea), np.ones_like(ea), BLUE),
                "anisotropic (ours)": (1/np.sqrt(aa), 1/np.sqrt(ac), RED)}
        alphas = np.linspace(0.02, 0.4, 12); fig, ax = plt.subplots(1, 2, figsize=(9.6, 4.0))
        ax[0].plot([0.6, 1], [0.6, 1], "--", color="0.5", lw=1.2, label="ideal")
        for nm, (sa, sc, col) in meth.items():
            cov = [np.mean(DC.score(ea, ec, sa, sc)[te] <=
                   np.quantile(DC.score(ea, ec, sa, sc)[cal], min(1.0, (1-a)*(1+1/cal.sum())), method="higher"))
                   for a in alphas]
            ax[0].plot(1-alphas, cov, "-o", color=col, lw=2, ms=4, label=nm)
        ax[0].set_xlabel("nominal coverage  1−α"); ax[0].set_ylabel("empirical coverage")
        _clean(ax[0]); ax[0].legend(loc="lower right", borderaxespad=0.6)
        cov_aci = np.load(os.path.join(C.CACHE, "degrade_conformal.npz"))["cov_aci"]
        ax[1].axhline(0.9, color="0.5", ls="--", lw=1.2, label="target 0.90")
        ax[1].plot(cov_aci, color=GREEN, lw=2, label="ACI running coverage")
        ax[1].set_xlabel("test sample (stream order)"); ax[1].set_ylabel("running coverage"); ax[1].set_ylim(0.7, 1.0)
        _clean(ax[1]); ax[1].legend(loc="lower right", borderaxespad=0.6)
        _save(fig, "F3_calibration.png")
    except Exception as e:
        print(f"  [skip F3] {type(e).__name__}: {e}")


def fig_safety(pol, pareto, ndang):
    fig, ax = plt.subplots(1, 2, figsize=(9.6, 4.0))
    names = ["ungated", "EKF-gated", "conformal-gated (ours)"]
    short = ["ungated", "EKF-gated", "conformal\n(ours)"]
    sp = [pol[k]["mean_speed"] for k in names]; un = [pol[k]["unsafe"]/max(ndang, 1) for k in names]
    x = np.arange(len(names))
    ax[0].bar(x-0.2, sp, 0.38, color=BLUE, label="mean speed (efficiency)")
    ax[0].bar(x+0.2, un, 0.38, color=RED, label="unsafe fraction (of danger)")
    ax[0].set_xticks(x); ax[0].set_xticklabels(short); ax[0].set_ylabel("fraction"); ax[0].set_ylim(0, 1.08)
    _clean(ax[0])
    ax[0].legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=2, frameon=False, fontsize=11)
    ax[1].plot(pareto[:, 0], pareto[:, 1], "-o", color=GREEN, lw=2, ms=4)
    ax[1].set_xlabel("mean speed (efficiency)"); ax[1].set_ylabel("unsafe events")
    _clean(ax[1])
    _save(fig, "F4_safety.png")


def fig_map():
    try:
        vm = np.load(C.VOX_MAP_CACHE); vox = vm["vox"]; traj = vm["traj"]
        fig, ax = plt.subplots(figsize=(7.0, 5.0))
        ax.scatter(vox[::6, 0], vox[::6, 1], s=0.2, c="0.72", alpha=0.45, linewidths=0)
        ax.plot(traj[:, 0], traj[:, 1], "-", color=RED, lw=1.8, label="autonomous route")
        ax.set_aspect("equal"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        ax.grid(False); _clean(ax)
        ax.legend(loc="lower right", borderaxespad=0.6)
        _save(fig, "F5_map.png")
    except Exception as e:
        print(f"  [skip F5] {type(e).__name__}: {e}")


def main():
    D, P, ekf_sig, eh = load()
    rows = degradation_table(D, eh)
    oc = overconfidence(D, ekf_sig, eh)
    pol, pareto, ndang, perlevel = safe_action(D, ekf_sig, eh)

    print("\n========== FINAL HONEST RESULTS ==========")
    print("F1 degradation:")
    for r in rows:
        print(f"  R={r['R']:5.1f}m  npts={r['npts']:5.0f}  along={r['e_along']:.3f} cross={r['e_cross']:.3f}  "
              f"ratio={r['ratio']:.2f}  fail={100*r['fail']:.1f}%")
    print(f"\nF2 over-confidence: EKF median σ={oc['ekf_sigma_mm']:.1f}mm, its '90%' bound={oc['ekf_90_mm']:.0f}mm, "
          f"actual error median={oc['actual_err_med_mm']:.0f}mm")
    print(f"   -> EKF's nominal 90% interval ACTUALLY covers {100*oc['ekf_actual_coverage']:.1f}%  "
          f"(over-confident (σ off) {oc['underconf_factor']:.0f}×)")
    print("\nF4 safe action (gate calibrated for 10% false-slow on good sensing):")
    for k, s in pol.items():
        print(f"  {k:24s} mean_speed={s['mean_speed']:.2f}  unsafe={s['unsafe']}/{ndang}  slowed={100*s['frac_slowed']:.0f}%")
    print("  per-level (conformal gate): slow-rate & unsafe gated vs ungated")
    for r in perlevel:
        print(f"    R={r['R']:5.1f}m  slow={100*r['slow']:4.0f}%  unsafe {r['unsafe_gated']:3d} (was {r['unsafe_ungated']:3d})")

    print("\nfigures:")
    fig_degradation(rows); fig_overconfidence(oc, D, ekf_sig, eh); fig_calibration()
    fig_safety(pol, pareto, ndang); fig_map()
    print("==========================================")


if __name__ == "__main__":
    main()
