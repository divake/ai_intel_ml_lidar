# Project 06 — Findings (honest, holistic)

*Conformal safety bounds for LiDAR localization, and how much sensing it takes.*
Last updated 2026-06-16. Data: **corridor2.0 only** (FAST-LIO2, XYZIRT + IMU). Full method/plan
in `PLAN.md`; this file is the conclusions + the numbers to present.

---

## TL;DR

We built a real autonomous LiDAR robot and asked whether it can *bound its own localization
error with a guarantee, and slow down when sensing is insufficient.* The headline results:

1. **The on-board filter's covariance is not a safety bound.** FAST-LIO2's reported (per-step)
   pose σ ≈ **3.7 mm**; treated as a global localization bound, its nominal **"90%" interval
   contains the true pose only ~1.3% of the time** (~**9× optimistic**). *Apples-to-oranges
   caveat:* this compares a local/incremental filter covariance to a global recovery residual —
   but even read charitably it's 7–9× off, so the filter's confidence cannot gate safety.
2. **Localization reliability is governed by sensing quality, and degrades anisotropically.**
   As the LiDAR's effective range drops 40 m → 3 m (sparse / occluded / cheaper sensing), recovery
   error grows ~**4×** and the **corridor aperture emerges** (along-corridor error grows faster than
   lateral: ratio **1.06 → 1.77**); failure rate rises **8% → 23%**. (Trend is the claim; absolute
   rates are ICP-from-injection residuals, not real-drive rates.)
3. **Conformal prediction gives a calibrated, online, distribution-free bound the filter can't —
   this is the strongest result.** Split-conformal under a real temporal shift under-covers (0.88);
   **online ACI restores nominal 0.90** (no leakage; split-by-scan). The error is directional, so a
   **fixed** anisotropic region is **8.4% tighter** than isotropic at matched coverage. (Per-scan
   observability-adaptive earns only 3.6% — a negative result; don't credit "ours" with the 8.4%.)
4. **A sensor-aware speed policy slows when it should.** Gating speed on the robot's *observable
   sensing quality* (scan density) **cuts unsafe events 70%** (3886 → 1148, to zero in the worst
   conditions), slowing only as sensing degrades (10% → 100%); the over-confident EKF can't gate
   (it doesn't know it's degraded). **Honest scope:** the gating signal is sensing density — a raw
   range/density gate does comparably, so conformal does *not* beat it at gating; conformal's role
   here is the **calibrated bound + held-out false-alarm guarantee (≈10%)**, not gating skill.

**Honest negative result:** *per-scan geometric observability does not predict per-scan
localization failure on this data* (within-level AUROC ≈ 0.5). The aperture is a real
*structural / per-condition* effect, not a per-scan-discriminative signal — so reliability is
driven by sensing density/range, which the robot can observe directly.

---

## What we built

- **Demo (the anchor):** Scout Mini drives a **251.6 m** taught corridor loop autonomously —
  FAST-LIO2 LiDAR-inertial SLAM → loop-closed **801k-pt** map → ICP localization → pure-pursuit.
  corridor2.0 = **7,619 scans @ 10 Hz + IMU @ 200 Hz, 12.7 min**, 0 drops.
- **Pipeline (`src/`):** `extract` (bag → per-scan clouds + IMU + EKF covariance + poses),
  `registration` (point-to-plane ICP + constraint matrix + normals), `degrade_recovery`
  (injected-GT recovery under sensor degradation = the true-GT error target),
  `observability` (per-scan horizontal degeneracy), `anisotropic`/`degrade_conformal` (conformal
  ellipse + ACI + baselines), `finalize` (tables + figures). All parallelized, thread-capped,
  fully cached/reproducible.

## The numbers (presentation-ready)

| Result | Figure / Table | Number |
|---|---|---|
| Autonomous loop | demo video, `F5_map` | 251.6 m, 801k-pt map, 7,619 scans |
| FAST-LIO2 vs LiDAR-only drift | (COMPARISON.md) | 1.52 m vs 4.00 m loop drift |
| **EKF over-confidence** | `F2_overconfidence` | **"90%" interval covers 1.3%; σ off ~9×** |
| **Degradation curve** | `F1_degradation_curve`, `T_degradation` | range 40→3 m: along-err 0.043→0.167 m, fail 8%→23%, aperture 1.06→1.77 |
| Conformal calibration + online | `F3_calibration` | split 0.88 → **ACI 0.90** |
| Anisotropic region (**fixed**) | `T_degrade_conformal` | **8.4% smaller** area at matched 0.90 coverage (obs-adaptive only 3.6%) |
| Sensor-aware safe action | `F4_safety`, `T_safety` | unsafe **3886→1148 (−70%)**, EKF-gate 3886→3309; slows 10%→100% by degradation (gate = density; conformal = the calibrated bound) |

## How to read the journey (why the story changed)

The original idea — "observability eigenvalue predicts per-scan localization error → conformal
bound → safe action" — did **not** survive contact with the data, for honest reasons we verified
adversarially (4-agent gate, `PLAN.md` §16):
1. The loop-closed trajectory is a **linear ramp** → unusable as error GT (§15). Fixed with
   injected-GT recovery.
2. The full 3-D constraint matrix is **dominated by floor/ceiling** → masks horizontal degeneracy.
   Fixed with walls-only horizontal observability (forward = blind axis in **88%** of scans).
3. **Single-scan registration is robust** (cm-level) with a 360° sensor + dense map → the
   per-scan observability signal has ~zero correlation with error (Spearman −0.16; the "16% area
   win" reversed at matched coverage). The aperture only bites under **sensor degradation**.

→ Reframed (your call) to the **sensor-quality + conformal-safety** story above, which the data
*does* support, keeping the demo, the conformal/ACI machinery, and the over-confidence punch.

## Caveats (state these)

- **Pseudo-GT via injected-perturbation recovery**, not survey GT (no survey GT exists; absolute
  drift along the corridor is unobservable to LiDAR — that's the aperture).
- **Degradation is emulated** (range-truncating the live scan) — a realistic stand-in for sparse/
  occluded/cheaper sensing, not a second physical sensor.
- **Single building, single run, single 16-beam sensor** → mechanism, not a population rate.
- The safe-action mean-speed reflects a *uniform* mix of degradation levels (artificial); the
  honest read is **per-level** (near-full speed when sensing is good, slow/stop when poor). The
  gate is a **scan-density/range gate**; conformal supplies the calibrated bound + false-alarm
  guarantee, not the gating decision (a raw density/range gate performs comparably).
- Per-scan geometric observability is reported as a **negative** result, not a working predictor.
- Conformal coverage is **marginal**, not conditional — per-degradation-level coverage ranges
  0.80–0.94 under the shift (ACI flattens it). Statistical **n ≈ #scans (~3,800)**, not the 26,663
  (scan×level) rows. The false-slow ≤10% guarantee holds **out-of-sample** (held-out 9.9%).

## Suggested talk arc (5 beats)

1. **We built a self-driving LiDAR robot** (demo + map + 251.6 m loop).
2. **Can it trust itself?** No — the filter's "90%" confidence is right 1.3% of the time (F2).
3. **Reliability depends on how much it can see** — the degradation curve + the corridor aperture (F1).
4. **Conformal prediction gives an honest, online safety bound** the filter can't (F3).
5. **So it slows down exactly when it should** — 70% fewer unsafe events, full speed when sensing
   is good (F4). *Distribution-free, no Bayesian priors, runs on the edge.*
