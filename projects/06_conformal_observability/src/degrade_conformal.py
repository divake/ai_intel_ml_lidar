"""PART 3 (on degrade dataset) — anisotropic conformal, validity + HONEST matched-coverage efficiency.

Dataset = per (scan,range-level) sample from cache/degrade.npz. Target = robot-frame error
(e_along,e_cross); sigma per method. We report:
  - VALIDITY: split-conformal cal->test coverage (and ACI online) — does the guarantee hold under shift?
  - EFFICIENCY: region AREA AT MATCHED test coverage 0.90 (sweep q to the same coverage for every
    method, then compare area) — the fair comparison the verification gate demanded.
"""
from __future__ import annotations
import os, csv
import numpy as np
import sys; sys.path.insert(0, os.path.dirname(__file__))
import common as C

ALPHA = 0.10
TARGET = 1 - ALPHA


def load():
    D = np.load(os.path.join(C.CACHE, "degrade.npz"))
    ea, ec = D["e_along"], D["e_cross"]
    aa = np.clip(D["a_along"], 1e-3, None); ac = np.clip(D["a_cross"], 1e-3, None)
    ok = np.isfinite(ea) & np.isfinite(ec) & np.isfinite(aa) & np.isfinite(ac)
    return dict(ea=ea[ok], ec=ec[ok], aa=aa[ok], ac=ac[ok], scan=D["i"][ok], li=D["li"][ok], R=D["R"][ok])


def split_by_scan(scan):
    u = np.unique(scan); cut = u[len(u)//2]
    cal = scan < cut; te = scan >= cut
    return cal, te


def score(ea, ec, sa, sc):
    return np.sqrt((ea/sa)**2 + (ec/sc)**2)


def area_at_coverage(ea, ec, sa, sc, c):
    """Empirical region scale q s.t. coverage == c; return mean ellipse area."""
    s = score(ea, ec, sa, sc)
    q = np.quantile(s, c, method="higher")
    return float(np.mean(np.pi * q**2 * sa * sc)), q


def aci(ea, ec, sa, sc, cal, te, alpha=ALPHA, gamma=0.02):
    s = score(ea, ec, sa, sc)
    scal = np.sort(s[cal]); a_t = alpha; hit = 0; cov = np.empty(te.sum()); idx = np.where(te)[0]
    for j, i in enumerate(idx):
        lvl = min(1.0, max(0.0, 1 - a_t))
        q = np.quantile(scal, lvl, method="higher") if lvl > 0 else scal[-1]
        err = 0.0 if s[i] <= q else 1.0
        hit += (1 - err); a_t += gamma*(alpha - err); cov[j] = hit/(j+1)
    return cov


def main():
    D = load(); ea, ec = D["ea"], D["ec"]
    cal, te = split_by_scan(D["scan"])
    fa, fc = np.std(ea[cal]), np.std(ec[cal])
    methods = {
        "isotropic":         (np.ones_like(ea), np.ones_like(ea)),
        "fixed-anisotropic": (np.full_like(ea, fa), np.full_like(ea, fc)),
        "obs (ours)":        (1/np.sqrt(D["aa"]), 1/np.sqrt(D["ac"])),
    }
    print(f"[degrade-conformal] n={len(ea)}  cal={cal.sum()} test={te.sum()}  target={TARGET:.0%}")
    print(f"  test-set natural error std: along {np.std(ea[te]):.3f}  cross {np.std(ec[te]):.3f}  ratio {np.std(ea[te])/max(np.std(ec[te]),1e-9):.2f}x")

    rows = []
    iso_area_matched = None
    for name, (sa, sc) in methods.items():
        # validity: conformal q from cal, measure test coverage + that-q area
        q_cal = np.quantile(score(ea, ec, sa, sc)[cal], min(1.0, TARGET*(1+1/cal.sum())), method="higher")
        s_te = score(ea, ec, sa, sc)[te]
        picp = float(np.mean(s_te <= q_cal))
        area_conf = float(np.mean(np.pi*q_cal**2*sa[te]*sc[te]))
        # efficiency at MATCHED 0.90 test coverage
        area_m, _ = area_at_coverage(ea[te], ec[te], sa[te], sc[te], TARGET)
        if name == "isotropic": iso_area_matched = area_m
        rows.append(dict(method=name, PICP=picp, area_conf=area_conf, area_matched=area_m))
    for r in rows:
        r["area_vs_iso_matched"] = r["area_matched"]/iso_area_matched

    sa, sc = methods["obs (ours)"]
    cov_aci = aci(ea, ec, sa, sc, cal, te)

    out = os.path.join(C.RESULTS, "T_degrade_conformal.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["method", "PICP", "area_conf", "area_matched", "area_vs_iso_matched"])
        w.writeheader()
        for r in rows:
            w.writerow({**r, **{k: round(r[k], 4) for k in ["PICP","area_conf","area_matched","area_vs_iso_matched"]}})
    C.save_npz(os.path.join(C.CACHE, "degrade_conformal.npz"), cov_aci=cov_aci,
               ea_te=ea[te], ec_te=ec[te], sa_te=sa[te], sc_te=sc[te], R_te=D["R"][te])

    print(f"\n{'method':20s} {'PICP':>6s} {'area@conf':>10s} {'area@0.90':>10s} {'vs.iso(matched)':>16s}")
    for r in rows:
        print(f"{r['method']:20s} {r['PICP']:6.3f} {r['area_conf']:10.4f} {r['area_matched']:10.4f} {r['area_vs_iso_matched']:16.3f}")
    print(f"\nACI running coverage: {cov_aci[0]:.2f} -> {cov_aci[-1]:.3f} (target {TARGET:.2f})")
    print(f"** HONEST headline = 'vs.iso(matched)' for obs(ours): <1.0 means genuinely tighter at EQUAL coverage **")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
