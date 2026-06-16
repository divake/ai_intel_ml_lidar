# Final cross-check ("the game") — verdict (2026-06-16)

3 adversarial agents (red-team skeptic, conformal statistician, reproducibility re-runner) on the
honest reframed results. **Overall: reproducible; core results solid; the safe-action headline is
overstated and must be reframed.**

## Reproduces exactly (determinism PASS)
All FINDINGS numbers re-derived from cache + re-ran scripts byte-identical: 7,619 scans; EKF 90%
interval covers 1.31% (σ off 9.49×); degradation 40→3 m along 0.043→0.167, ratio 1.06→1.77, fail
8.3%→22.7%; split PICP 0.884 → ACI 0.900; matched-coverage area iso 1.0 / fixed-aniso 0.916 / obs
0.964; safe-action unsafe 3886→1148 (−70.5%). Source notes: 251.6 m = teach_path (proj03);
801,837-pt map = prior_map_loopclosed.pcd.

## SOLID — present with confidence
- **ACI online calibration** — strongest result. Split under-covers under a real temporal shift
  (no leakage; split-by-scan), ACI restores long-run 0.90 (windowed 0.85–0.96). Gibbs-Candès-correct.
- **EKF is not a safety bound** — ~7–9× optimistic even full-range; its 90% interval covers ~1.3%.
- **Degradation trend + aperture geometry** — real; a_along < a_cross at every level (geometry,
  not artifact); aperture emerges as range shrinks.
- **Fixed-anisotropic region 8.4% tighter** at matched 0.90 coverage (the prior matched-coverage
  bug is fixed).

## MUST FIX / SOFTEN before presenting
1. **Safe-action gate is a density/range gate, NOT conformal.** `v_conformal == v_density` exactly;
   at matched mean speed a trivial range gate is 3.6× better (317 vs 1148 unsafe @ speed 0.40). The
   conformal contribution is the *calibrated bound + held-out false-alarm guarantee*, not gating
   skill. Reframe: "a calibrated sensor-aware speed policy slows as sensing degrades"; do NOT claim
   conformal beats baselines for safety.
2. **Do not credit "ours" (obs) with 8.4%** — that's the FIXED global anisotropy; obs earns 3.6%
   and slightly under-covers. Per-scan observability remains a negative result.
3. **EKF wording:** "its per-step covariance is not a localization safety bound (~9× optimistic)"
   — note it's a local/incremental covariance vs a global recovery residual (apples-to-oranges).
4. **False-alarm guarantee was in-sample** (tautological). Held-out split recovers it (9.9% false-
   slow, 1.9% false-stop) — state as "≈10% by calibration (held-out)".
5. **Disclose:** coverage is MARGINAL (per-level 0.80–0.94); statistical n ≈ #scans (~3,800), not
   26,663 rows; absolute failure rates are ICP-from-injection residuals (trend, not real-drive rates);
   degradation is emulated (range-truncated live scan vs full map).

## Net
A defensible, honest study: real LiDAR autonomy demo + "the filter's confidence is not a safety
bound" + "reliability degrades with sensing, anisotropically" + "conformal/ACI gives the honest,
online-valid bound" + "a sensor-aware speed policy slows when it should." Lead with ACI +
over-confidence + degradation; present safe-action as sensor-aware (not a conformal-vs-baseline win).
