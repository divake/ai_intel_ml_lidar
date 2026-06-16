"""PART 3 — Conformal upper bounds on localization error + Tier 1&2 baselines.

Target Y = injected-GT recovery error (m, >=0). We build a ONE-SIDED conformal UPPER BOUND
U(x) = q_hat * sigma(x) with P(Y <= U) >= 1-alpha.  sigma(x) is a per-scan "difficulty" signal.

Tier 1 (which SIGNAL?) : sigma in {constant, 1/lam_min_t, kappa_t, system-cov, ICP-residual, 1/density}
                          all via split conformal -> compare efficiency (MPIW) at fixed coverage.
Tier 2 (which METHOD?) : on the best signal -> temp-scaling (no guarantee), split conformal,
                          ACI, Conformal-PID -> compare coverage & coverage-over-time on the stream.

Outputs results/T1_master.csv and cache/conformal.npz (bounds + running-coverage curves).
"""
from __future__ import annotations
import os
import numpy as np
import csv
import sys; sys.path.insert(0, os.path.dirname(__file__))
import common as C

ALPHA = 0.10  # target 90% coverage


# ----------------------------------------------------------------- data
def load_dataset():
    E = np.load(os.path.join(C.CACHE, "error.npz"))
    Sg = np.load(os.path.join(C.CACHE, "signal.npz"))
    P = np.load(os.path.join(C.CACHE, "poses.npz"))
    oi = E["odd_idx"]
    y = E["resid_rand"].mean(axis=1).astype(float)        # per-scan recovery error
    # candidate sigma signals (all > 0), indexed to odd scans
    lam = np.clip(Sg["lam_min_t"][oi], 1e-3, None)
    kap = np.clip(Sg["kappa_t"][oi], 1.0, None)
    dens = np.clip(Sg["density"][oi], 1.0, None)
    cov = P["odom_cov"][oi]                                # (K,6,6)
    covtr = np.clip(np.sqrt(np.abs(cov[:, 0, 0] + cov[:, 1, 1] + cov[:, 2, 2])), 1e-6, None)
    resid = np.clip(E["plane_resid"], 1e-4, None)
    sig = {
        "constant":   np.ones_like(y),
        "obs_1/lam":  1.0 / lam,
        "obs_kappa":  kap,
        "sys_cov":    covtr,
        "icp_resid":  resid,
        "inv_density": 1.0 / dens,
    }
    ok = np.isfinite(y)
    for k in sig:
        ok &= np.isfinite(sig[k])
    y = y[ok]
    sig = {k: v[ok] for k, v in sig.items()}
    return y, sig, oi[ok]


def temporal_split(n, frac=0.5):
    """Block split: first frac = calibration, rest = test (no shuffling — stream is correlated)."""
    k = int(n * frac)
    return np.arange(k), np.arange(k, n)


# ----------------------------------------------------------------- metrics
def picp(y, U):       return float(np.mean(y <= U))
def mpiw(U):          return float(np.mean(U))
def winkler(y, U, a):  # one-sided interval [0,U]: width + miss penalty
    pen = (2.0 / a) * np.maximum(y - U, 0.0)
    return float(np.mean(U + pen))

def aurc(y, U, sigma):
    """Risk-coverage: sort by certainty (small sigma), at coverage c the risk = miss-rate among kept."""
    order = np.argsort(sigma)            # most-certain first
    miss = (y > U).astype(float)[order]
    cov = np.arange(1, len(y)+1)/len(y)
    risk = np.cumsum(miss)/np.arange(1, len(y)+1)
    return float(np.trapz(risk, cov))

def ause(y, unc):
    """>=0, lower=better. Keep the LEAST-uncertain (1-f) fraction; oracle keeps lowest-error."""
    n = len(y); fr = np.linspace(0, 0.9, 19)
    def curve(rank):
        order = np.argsort(rank)
        return np.array([y[order][:max(int(n*(1-f)),1)].mean() for f in fr])
    bu = curve(unc); bo = curve(y)
    bu /= bu[0]+1e-12; bo /= bo[0]+1e-12
    return float(np.trapz(bu-bo, fr))


# ----------------------------------------------------------------- calibrators
def split_conformal(y, sig, cal, te, alpha=ALPHA):
    s = y[cal] / sig[cal]
    q = np.quantile(s, min(1.0, (1-alpha)*(1+1/len(cal))), method="higher")
    U = q * sig[te]
    return U, q

def temp_scaling(y, sig, cal, te, alpha=ALPHA):
    """Scale sigma by a single factor so cal coverage == 1-alpha (calibration, NOT conformal)."""
    facs = np.linspace(0.1, 50, 2000)
    target = 1-alpha
    best = facs[np.argmin([abs(np.mean(y[cal] <= f*sig[cal]) - target) for f in facs])]
    return best * sig[te], best

def aci(y, sig, cal, te, alpha=ALPHA, gamma=0.02):
    """Adaptive Conformal Inference over the test stream. Returns U_t and running coverage."""
    s_cal = np.sort(y[cal] / sig[cal])
    a_t = alpha
    U = np.empty(len(te)); cov_run = np.empty(len(te)); hit = 0
    for j, i in enumerate(te):
        lvl = min(1.0, max(0.0, 1-a_t))
        q = np.quantile(s_cal, lvl, method="higher") if lvl > 0 else s_cal[-1]
        U[j] = q * sig[i]
        err = 0.0 if y[i] <= U[j] else 1.0
        hit += (1-err)
        a_t = a_t + gamma*(alpha - err)
        cov_run[j] = hit/(j+1)
    return U, cov_run

def conformal_pid(y, sig, cal, te, alpha=ALPHA, kp=0.05, ki=0.005):
    s_cal = np.sort(y[cal] / sig[cal])
    a_t = alpha; integ = 0.0
    U = np.empty(len(te)); cov_run = np.empty(len(te)); hit = 0
    for j, i in enumerate(te):
        lvl = min(1.0, max(0.0, 1-a_t))
        q = np.quantile(s_cal, lvl, method="higher") if lvl > 0 else s_cal[-1]
        U[j] = q * sig[i]
        err = 0.0 if y[i] <= U[j] else 1.0
        hit += (1-err)
        e = alpha - err; integ += e
        a_t = alpha + kp*e + ki*integ
        cov_run[j] = hit/(j+1)
    return U, cov_run


# ----------------------------------------------------------------- main
def main():
    y, sig, oi = load_dataset()
    n = len(y)
    cal, te = temporal_split(n)
    print(f"[conformal] n={n}  cal={len(cal)} test={len(te)}  target coverage={1-ALPHA:.0%}")

    rows = []
    # ---- Tier 1: signal comparison via split conformal ----
    for name, s in sig.items():
        U, q = split_conformal(y, s, cal, te)
        rows.append(dict(tier="T1-signal", method=f"split·{name}",
                         PICP=picp(y[te], U), MPIW=mpiw(U),
                         AUSE=ause(y[te], s[te]), AURC=aurc(y[te], U, s[te]),
                         Winkler=winkler(y[te], U, ALPHA)))

    # ---- Tier 2: best signal (observability) with different calibrators ----
    best = "obs_1/lam"
    s = sig[best]
    U, _ = split_conformal(y, s, cal, te)
    rows.append(dict(tier="T2-method", method="split·obs", PICP=picp(y[te], U), MPIW=mpiw(U),
                     AUSE=ause(y[te], s[te]), AURC=aurc(y[te], U, s[te]), Winkler=winkler(y[te], U, ALPHA)))
    Ut, _ = temp_scaling(y, sig["sys_cov"], cal, te)
    rows.append(dict(tier="T2-method", method="tempscale·syscov", PICP=picp(y[te], Ut), MPIW=mpiw(Ut),
                     AUSE=ause(y[te], sig["sys_cov"][te]), AURC=aurc(y[te], Ut, sig["sys_cov"][te]),
                     Winkler=winkler(y[te], Ut, ALPHA)))
    Ua, cov_a = aci(y, s, cal, te)
    rows.append(dict(tier="T2-method", method="ACI·obs (ours)", PICP=picp(y[te], Ua), MPIW=mpiw(Ua),
                     AUSE=ause(y[te], s[te]), AURC=aurc(y[te], Ua, s[te]), Winkler=winkler(y[te], Ua, ALPHA)))
    Up, cov_p = conformal_pid(y, s, cal, te)
    rows.append(dict(tier="T2-method", method="CPID·obs (ours)", PICP=picp(y[te], Up), MPIW=mpiw(Up),
                     AUSE=ause(y[te], s[te]), AURC=aurc(y[te], Up, s[te]), Winkler=winkler(y[te], Up, ALPHA)))

    # ---- write table ----
    out = os.path.join(C.RESULTS, "T1_master.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["tier", "method", "PICP", "MPIW", "AUSE", "AURC", "Winkler"])
        w.writeheader()
        for r in rows:
            w.writerow({**r, **{k: round(r[k], 4) for k in ["PICP","MPIW","AUSE","AURC","Winkler"]}})
    C.save_npz(os.path.join(C.CACHE, "conformal.npz"),
               y_test=y[te], U_split=split_conformal(y, s, cal, te)[0],
               U_aci=Ua, U_pid=Up, cov_aci=cov_a, cov_pid=cov_p,
               sig_obs=s[te], test_scan=oi[te])

    print(f"\n{'method':22s} {'PICP':>6s} {'MPIW':>8s} {'AUSE':>7s} {'AURC':>7s} {'Winkler':>8s}")
    for r in rows:
        print(f"{r['method']:22s} {r['PICP']:6.3f} {r['MPIW']:8.3f} {r['AUSE']:7.3f} {r['AURC']:7.4f} {r['Winkler']:8.3f}")
    print(f"\n(target PICP = {1-ALPHA:.2f}; lower MPIW/AUSE/AURC/Winkler better)")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
