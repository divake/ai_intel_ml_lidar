"""PART 5 — risk-controlled safe action.

The conformal upper bound U(x) on localization error governs speed:
  U < u_go    -> full speed
  U < u_slow  -> crawl
  else        -> stop / re-localize
"Danger" = a scan whose TRUE recovery error y exceeds tau (localization actually unreliable).
We compare gated vs ungated: unsafe events (full speed while in danger), mean speed (efficiency),
and trace the safety-efficiency Pareto by sweeping the coverage level alpha.

Outputs results/T_safety.csv + cache/safe.npz (per-scan speed/U/y, Pareto curve).
"""
from __future__ import annotations
import os, csv
import numpy as np
import sys; sys.path.insert(0, os.path.dirname(__file__))
import common as C
import conformal as CF

TAU = 0.30          # m: true error above this = "lost" (unsafe to drive fast)
V_FULL, V_CRAWL, V_STOP = 1.0, 0.3, 0.0


def gate_speed(U, u_go, u_slow):
    v = np.full(len(U), V_STOP)
    v[U < u_slow] = V_CRAWL
    v[U < u_go] = V_FULL
    return v


def main():
    y, sig, scan = CF.load_dataset()
    n = len(y); cal, te = CF.temporal_split(n)
    s = sig["obs_1/lam"]
    yte = y[te]
    danger = yte > TAU

    # conformal bound at target coverage, threshold chosen so a SAFE-data quantile triggers slow
    U, _ = CF.split_conformal(y, s, cal, te, alpha=0.10)
    u_go = np.quantile(U, 0.70)     # below this 70% -> go
    u_slow = np.quantile(U, 0.90)   # 70-90% -> crawl, top 10% -> stop
    v_gate = gate_speed(U, u_go, u_slow)
    v_ungated = np.full(n - len(cal), V_FULL)

    def stats(v):
        unsafe = int(np.sum((v > V_CRAWL) & danger))          # full speed while in danger
        return dict(mean_speed=float(v.mean()), unsafe=unsafe,
                    frac_slowed=float(np.mean(v < V_FULL)),
                    frac_slowed_safe=float(np.mean((v < V_FULL) & ~danger)))
    g, u = stats(v_gate), stats(v_ungated)

    # ---- safety-efficiency Pareto: sweep alpha ----
    pareto = []
    for a in np.linspace(0.02, 0.5, 25):
        Ua, _ = CF.split_conformal(y, s, cal, te, alpha=a)
        ug, us = np.quantile(Ua, 0.70), np.quantile(Ua, 0.90)
        va = gate_speed(Ua, ug, us)
        pareto.append((float(va.mean()), int(np.sum((va > V_CRAWL) & danger))))
    pareto = np.array(pareto)

    C.save_npz(os.path.join(C.CACHE, "safe.npz"),
               U=U, y=yte, danger=danger, v_gate=v_gate, test_scan=scan[te],
               pareto=pareto, u_go=np.float32(u_go), u_slow=np.float32(u_slow), tau=np.float32(TAU))

    out = os.path.join(C.RESULTS, "T_safety.csv")
    with open(out, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["policy", "mean_speed", "unsafe_events", "frac_slowed", "frac_slowed_unnecessarily"])
        w.writerow(["ungated", round(u["mean_speed"], 3), u["unsafe"], u["frac_slowed"], round(u["frac_slowed_safe"], 3)])
        w.writerow(["conformal-gated", round(g["mean_speed"], 3), g["unsafe"], round(g["frac_slowed"], 3), round(g["frac_slowed_safe"], 3)])

    print("========== SAFE-ACTION REPORT ==========")
    print(f"test scans {len(te)}   danger (err>{TAU}m): {int(danger.sum())} ({100*danger.mean():.1f}%)")
    print(f"{'policy':18s} {'mean_speed':>10s} {'unsafe':>7s} {'%slowed':>8s} {'%slowed_safe':>12s}")
    print(f"{'ungated':18s} {u['mean_speed']:10.3f} {u['unsafe']:7d} {100*u['frac_slowed']:7.1f}% {100*u['frac_slowed_safe']:11.1f}%")
    print(f"{'conformal-gated':18s} {g['mean_speed']:10.3f} {g['unsafe']:7d} {100*g['frac_slowed']:7.1f}% {100*g['frac_slowed_safe']:11.1f}%")
    print(f"unsafe events cut: {u['unsafe']} -> {g['unsafe']}  ({100*(1-g['unsafe']/max(u['unsafe'],1)):.0f}% reduction) "
          f"at {100*g['mean_speed']:.0f}% of full speed")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
