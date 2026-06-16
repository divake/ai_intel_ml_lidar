# Project 06 — Conformalized Observability

*Online, distribution-free localization safety from LiDAR geometry.*

**Status:** PLAN ONLY — nothing implemented yet. This is the agreed idea, written down so we
don't lose it. Nothing here is mandatory; it's a menu we curate as results come in. We start
building only on an explicit "go".

**Working title:** *Conformalized Observability — knowing when the robot is lost, and acting
safely, with a guarantee, from LiDAR alone.*
**Method nickname (to finalize):** **CO-Loc** / **COSA** (Conformal Observability + Safe Action).

---

## 0. Why this exists

- **North star:** uncertainty quantification for 3D LiDAR / point-cloud ML (conformal,
  distribution-free — not Bayesian as the headline, no CQR).
- **Audience:** the CogniSense (DARPA JUMP2.0) program. Their standing ask: *do real LiDAR*,
  not images. This project is the first in our line that uses **real 3D LiDAR point clouds +
  real robot + IMU + a closed control loop** — which is exactly that ask, and exactly the
  white space across our prior papers (LCP, COINS abstention, TRIAGE, model-hopping all used
  images or LiDAR-as-a-noise-model, never real point-cloud localization).
- **What matters most (stated by Divake):** a *strong story*, a *nice demo*, and *clean
  results with many baselines* shown on **maps, tables, and plots*. Picking methods straight
  from the literature is fine — the contribution is the synthesis + the demo, not method
  invention. (That said, the core below is genuinely unclaimed; see §3.)

---

## 1. The one-line thesis

A point-estimate LiDAR localizer fails **silently** exactly where the corridor geometry is
**degenerate** (the along-corridor axis is unobservable — the aperture problem). We read an
**observability** uncertainty straight from the LiDAR cloud, **conformalize** it into a
distribution-free guarantee on pose error, keep that guarantee valid **online** on the
non-exchangeable scan stream, and let it **govern a safe action** (slow / stop). Result: the
robot *knows when it's lost* and acts on it — fast when confident, cautious only where it must
be — with coverage you can put a number on.

---

## 2. The story (one arc, every piece is load-bearing)

Six beats; each beat creates the problem the next one solves. Letters map to the ideas in §4–§5.

1. **The robot drives itself (demo).** FAST-LIO2 SLAM → loop-closed map → ICP localization →
   pure-pursuit → Scout Mini. It runs clean. *Hook: it has no honest sense of when it will fail.*
2. **Silent failure (A + B).** Self-similar corridor ⇒ along-corridor axis unobservable.
   The point estimate slides and teleports — and the system's **own covariance says it's
   confident** (provably over-optimistic). *⇒ we need an honest uncertainty.*
3. **The signal (A).** Read uncertainty from LiDAR geometry: smallest eigenvalue / condition
   number of the registration constraint matrix; its eigenvector = the blind direction.
   Single-pass, no training. *⇒ it correlates with real drift… but a raw signal isn't a guarantee.*
4. **The guarantee (C — the novel core).** Wrap the signal in conformal prediction → coverage
   on **pose error** (observability-normalized score, no CQR; optional LCP-learned score).
   *⇒ but the corridor stream isn't exchangeable.*
5. **The stream (D).** Naive split conformal silently under-covers; **online ACI /
   Conformal-PID** restores coverage live. *⇒ now the interval is trustworthy — use it.*
6. **The safe action (E + counterfactual).** The interval / risk budget governs speed: fast
   when certain, slow/stop when blind. Replay shows it would have triggered **N scans before**
   the teleport. Safety–efficiency Pareto. *Punchline.*

> Out of scope (keeps the story clean): LiDAR **semantic segmentation** on SemanticKITTI.
> No labels for our 16-beam sensor + severe domain gap; it's a separate domain. Optional
> appendix only, never the spine.

---

## 3. What's actually new (honest ledger)

| Ingredient | Continuity (already ours / standard) | New |
|---|---|---|
| Conformal prediction, coverage guarantee | ✅ our flagship | — |
| Edge / NUC / Agilex / sampling-free / Pareto framing | ✅ our signature | — |
| Learnable nonconformity score (LCP) | ✅ published | — |
| Observability/degeneracy **as the uncertainty** (A,B) | | ✅ new to us (geometry-native, not model-confidence) |
| Conformal on **LiDAR odometry pose-error** (C) | | ✅✅ open gap in the field ("to our knowledge") |
| **Online** ACI / Conformal-PID on the scan stream (D) | | ✅ new to us (our own stated future work) |
| Conformal → **motion** safe-action (E) | | ✅ new to us (action, not abstention/model-hop) |

Novel core = *a LiDAR-physics uncertainty (registration observability) conformalized into an
online, distribution-free localization-safety guarantee that drives a safe motion.* That
intersection is unclaimed. Possible extra novelty knob (optional): a **directional** prediction
region shaped by the degeneracy *eigenvector*, not just the scalar eigenvalue.

---

## 4. The method (ours) + ablation knobs

**CO-Loc = observability-normalized, online-conformal pose-error intervals driving a
risk-controlled action.**

Pipeline per scan:
1. **Observability signal (A).** Estimate surface normals on the scan (KNN + PCA), build the
   point-to-plane constraint matrix `C = Σ cᵢcᵢᵀ`, `cᵢ = [pᵢ×nᵢ ; nᵢ]` (6×6). Eigen-decompose →
   `λ_min`, condition number `κ = λ_max/λ_min`, and the eigenvector of `λ_min` = unobservable
   direction. Single cloud, single pass, no training (Gelfand 2003 / Zhang 2016 / X-ICP).
2. **Nonconformity score.** Pose error normalized by a difficulty estimate from the
   observability features (locally-weighted split conformal — heteroscedastic, **no CQR**).
   - *Knob:* analytic score (raw `1/λ_min` style) → normalized → **LCP-learned** score (tiny MLP
     over geometric features). Shown as an ablation (each rung tightens intervals at fixed coverage).
3. **Conformalize.** Calibrate on a held-out **temporal block** (not random — corridors are
   correlated) → distribution-free interval on pose error.
4. **Online (D).** ACI updates the level from realized miscoverage; Conformal-PID as the
   stronger variant (ablation). Restores nominal coverage under corridor distribution shift.
5. **Act (E).** Map interval width / Conformal-Risk-Control budget → speed command
   (full / crawl / stop). Hysteresis so it doesn't chatter.

Ablation table (T2) covers: score (analytic → normalized → LCP) × calibrator
(split → ACI → PID), plus directional-region on/off.

---

## 5. Baselines (the comparison they came to see) — three tiers, three different stories

**Tier 1 — which *signal* tracks localization error?**
| Baseline | Story it tells |
|---|---|
| Constant / global uncertainty | uniform conservatism is wasteful ("redistribute risk") |
| System covariance (FAST-LIO2 EKF, from corridor2.0) | the system's own confidence **lies** (over-optimistic) |
| ICP fitness / residual | low residual even while sliding → **misses** degeneracy |
| Point density / sparsity | catches sparse frames, misses *directional* blindness |
| **Observability (ours, raw)** | tracks the real error — but uncalibrated |

**Tier 2 — which *method* gives an honest guarantee?** (best signal + each calibrator)
| Baseline | Story |
|---|---|
| Temperature / variance scaling | fixes the average, no finite-sample guarantee, breaks under shift |
| Split conformal (offline) | valid i.i.d. — but **under-covers** the correlated stream |
| **Online conformal — ACI / PID (ours)** | recovers nominal coverage live |

**Tier 3 — tightest interval at guaranteed coverage, cheapest compute?** (our signature)
| Baseline | Story |
|---|---|
| MC-dropout / Deep ensemble / Evidential (on a small learned error-regressor) | strong but **M× cost / energy** |
| **LCP-learned score + online conformal (ours)** | same coverage, **tighter** intervals, fraction of the energy |

→ Up to ~10 baselines → one master table (T1), three punchlines.

---

## 6. Metrics (grouped by the question each answers)

- **Does uncertainty predict error?** AUSE + sparsification plot (primary); AUROC for
  failure-detection; Spearman ρ (intuition only).
- **Is the interval honest?** PICP (coverage) vs nominal; MPIW (width/sharpness); Winkler /
  interval score; calibration curve; **running coverage over time** (the ACI story).
- **Safe *and* efficient?** lead-time (scans/sec before failure); unsafe-events & min
  wall-distance gated vs ungated; avg speed / % at full speed / % needlessly slowed; AURC
  (risk-coverage); **safety–efficiency Pareto**.
- **Edge cost (our fingerprint)?** ms/scan + mJ/scan, ours vs ensembles.

---

## 7. Deliverables — maps, plots, tables

**On the map ("this thing did that"):**
- **M1** Trajectory colored by observability — hot in corridors, cool at junctions.
- **M2** Quiver of the *blind direction* eigenvector — arrows point down the hallway.
- **M3** Failure overlay — pseudo-GT vs raw odometry, teleport highlighted, observability spiking there.
- **M4** **Gated vs ungated trajectory** side-by-side — ungated into the wall (red), gated stays safe (speed-colored). *The money map.*
- **M5** Conformal "uncertainty tube" — interval band widening in corridors, tight at junctions.

**Plots:** P1 sparsification/AUSE · P2 coverage-vs-nominal calibration · P3 running coverage
(split under-covers → ACI recovers) · **P4 safety–efficiency Pareto (headline)** · P5 the money
time-series (signal + frozen threshold + lead-time arrow + speed→0) · P6 interval-width by scene ·
P7 energy/latency bars (ours vs ensembles).

**Tables:** **T1 master baseline comparison** (≈10 rows × AUSE / PICP / MPIW / AURC / lead-time /
ms / mJ — bold ours) · T2 ablation (score × calibrator) · T3 SLAM demo-context
(FAST-LIO2 corridor2.0 drift / z-sag). *Open question §13: whether to also run KISS-ICP on the
**corridor2.0 bag** as an apples-to-apples "weaker localizer" — that would still be corridor2.0
data, not project 01.*

---

## 8. Data, feasibility, hardware

> **DATA SOURCE — `corridor2.0` ONLY (project 02). Project 01 / corridor1 is NOT used.**
> corridor1 was the early try-and-test run: **X,Y,Z (+intensity) only, no IMU, no real ring,
> no per-point timestamp** (its `.npy` col5 is azimuth 0..1799, *not* ring). corridor2.0 is the
> deliberate, final dataset: **full LiDAR XYZIRT = x, y, z, intensity, real ring (0..15),
> per-point timestamp**, plus **D455 IMU (~200 Hz)** and **FAST-LIO2** poses. Same physical
> corridor, better rig + algorithm. Everything in this project is corridor2.0.

**Anchor sequence:** `projects/02_fast_lio2/results/corridor2.0/` (XYZIRT + IMU + FAST-LIO2,
loop-closed). The crown jewel. The only sequence we use.

On disk / ready (verified 2026-06-16):
- **Pseudo-GT error** = `trajraw` − `traj` from
  `projects/03_autonomy/results/corridor2_loopclosed_fastlio.npz` (loop-closed vs raw, 7,619 scans). Direct subtraction.
- Per-scan **FAST-LIO2 poses** for corridor2.0 (loop-closed = pseudo-GT; raw = the localizer
  under study). All from the corridor2.0 bag / its loop-closed npz — no other sequence.
- `sklearn` + `scipy` present (`env_py311`) → normals (KNN+PCA) + eigendecomposition. No open3d needed.
- 2× RTX 6000 Ada (48 GB) → Tier-3 learned regressor + MC-dropout / ensemble; energy via NVML.
- proj-04 custom CUDA rasterizer → publication-grade map renders (no OpenGL on this box).

To set up (approved):
- `pip install mcap mcap-ros2-support` in `env_py311` → read corridor2.0 per-scan clouds +
  `/camera/camera/imu` + `/lio/odometry` (incl. **system covariance** for the Tier-1 strawman).
  PointCloud2 is XYZIRT, `point_step=26` (offsets x0 y4 z8 int12 ring16 ts18). Mask NaN/zero padding.

**Failure case for the counterfactual (corridor2.0 only):** we use the **corridor2.0 raw-odometry
error against the loop-closed pseudo-GT** (`trajraw` − `traj` already encodes where FAST-LIO
drifts on this run) and/or a **held-out degenerate corridor2.0 segment** as the honest failure —
stated plainly as an open-loop counterfactual. (The live autonomous teleport survives only as
PNGs, `live_drive_progress.png`, not a clean per-scan log, so we don't depend on it.)

**Rough effort:** strong core (Tiers 1–2, all maps/plots, T1) ≈ 2–3 days; full incl. Tier-3
(LCP + ensembles + energy/Pareto) ≈ +1.5 days.

---

## 9. Proposed build order

1. Extract corridor2.0 → cache per-scan clouds + IMU + odometry covariance + pseudo-GT error.
2. Observability signal (normals → constraint matrix → λ_min, κ, eigenvector) + per-scan features.
3. Pseudo-GT pose error; decompose into along- vs cross-corridor components (validates A).
4. Conformal pipeline + all baselines + metrics (AUSE, PICP, MPIW, AURC, coverage-over-time).
5. Maps / plots / tables.
6. Safe-action gate + counterfactual + Pareto.
7. (Tier 3) learned LCP score + MC-dropout / ensemble / evidential + energy story.

---

## 10. Honesty / caveats (state these on the slides — don't get caught on them)

- **Pseudo-GT, not survey GT.** Error is vs the loop-closed/optimized trajectory; labeled as such.
- **Open-loop counterfactual.** Replay shows the gate *would have fired in time*; it does not
  prove the crash was prevented (stopping changes future inputs). n=1 = mechanism, not a rate —
  inject failures at multiple points / perturb to make statistical plots defensible.
- **Exchangeability.** Corridor scans are temporally correlated → block/temporal split, and ACI
  for the online guarantee. Marginal ≠ conditional coverage; report stratified by scene.
- **Sensor domain.** 16-beam RoboSense; all results stay on our own data (no cross-sensor claims).

---

## 11. Decisions locked

- **Data = corridor2.0 ONLY. Project 01 / corridor1 is never used** (older, xyz-only, no IMU,
  no real ring) — see the box in §8.
- Anchor = corridor2.0; pseudo-GT = loop-closed trajectory.
- Multiple baselines = the 3-tier ladder (§5).
- Tier 3 (learned + ensembles + energy/Pareto) = **in**.
- `pip install mcap` (and anything else needed) = approved.
- Segmentation/KITTI = out of the main story (optional appendix only).

---

## 12. Literature (the spine's backbone — verified arXiv IDs)

**Degeneracy / observability:** Zhang, Kaess, Singh, *On Degeneracy of Optimization-based State
Estimation*, ICRA 2016 (no arXiv) · Tuna et al., **X-ICP**, T-RO 2024, arXiv 2211.16335 ·
Hatleskog & Alexis, *Probabilistic Degeneracy Detection*, RA-L 2024, arXiv 2410.10784 (repo
`drpm`) · Gelfand et al., *Geometrically Stable Sampling for ICP*, 3DIM 2003 (no arXiv) · Nubert
et al., *Learning-based Localizability*, IROS 2022, arXiv 2203.05698.

**ICP/odometry covariance is over-optimistic:** Censi, ICRA 2007 · Brossard, Bonnabel, Barrau,
*A New Approach to 3D ICP Covariance*, RA-L 2020, arXiv 1909.05722 · Bonnabel et al., ACC 2016,
arXiv 1410.7632 (point-to-point caveat → KISS-ICP) · Landry et al., **CELLO-3D**, ICRA 2019,
arXiv 1810.01470.

**Our systems:** Vizzo et al., **KISS-ICP**, RA-L 2023, arXiv 2209.15397 · Xu et al.,
**FAST-LIO2**, T-RO 2022, arXiv 2107.06829.

**Conformal foundations:** Vovk, Gammerman, Shafer, *Algorithmic Learning in a Random World*,
2005/2022 · Angelopoulos & Bates, *A Gentle Introduction…*, arXiv 2107.07511 · Lei et al.,
*Distribution-Free Predictive Inference for Regression*, JASA 2018, arXiv 1604.04173.

**Online / adaptive conformal:** Gibbs & Candès, **ACI**, NeurIPS 2021, arXiv 2106.00170 ·
Zaffran et al., *Adaptive CP for Time Series*, ICML 2022, arXiv 2202.07282 · Angelopoulos,
Candès, Tibshirani, **Conformal PID**, NeurIPS 2023, arXiv 2307.16895 · Barber et al., *CP Beyond
Exchangeability*, Ann. Stat. 2023, arXiv 2202.13415.

**Risk control / safe action:** Angelopoulos et al., **Conformal Risk Control**, ICLR 2024, arXiv
2208.02814 · Angelopoulos et al., *Learn then Test*, arXiv 2110.01052 · Lindemann et al., *Safe
Planning … Conformal Prediction*, arXiv 2210.10254 · Dixit et al., *Adaptive CP for Motion
Planning*, L4DC 2023, arXiv 2212.00278 · Sinha, Schmerling, Pavone, **Fallback-Safe MPC**, CDC
2023, arXiv 2309.08603 · Lekeufack et al., *Conformal Decision Theory*, ICRA 2024, arXiv
2310.05921 · Xu et al. (TRI), **FAIL-Detect**, RSS 2025, arXiv 2503.08558.

**Evaluation:** Ilg et al., *Uncertainty … Optical Flow* (**AUSE**), ECCV 2018, arXiv 1802.07095 ·
Poggi et al., *Uncertainty of Monocular Depth*, CVPR 2020, arXiv 2005.06209 · Kuleshov et al.,
*Calibrated Regression*, ICML 2018, arXiv 1807.00263.

**UQ baselines:** Hendrycks & Gimpel (MSP) arXiv 1610.02136 · Gal & Ghahramani (MC-dropout) arXiv
1506.02142 · Lakshminarayanan et al. (deep ensembles) arXiv 1612.01474 · Sensoy et al.
(evidential) arXiv 1806.01768; Amini et al. (deep evidential regression) arXiv 1910.02600 · Guo
et al. (temperature scaling) arXiv 1706.04599.

**Our prior work to connect (not repeat):** **LCP** — Kumar et al., *Learnable Conformal
Prediction with Context-Aware Nonconformity*, arXiv 2509.21955 · **COINS** — Kumar et al.,
*Uncertainty-Aware LiDAR-Camera Autonomy via Conformal Prediction and Principled Abstention*,
IEEE COINS 2025 · **TRIAGE** — arXiv 2603.08128.

---

## 13. Resolved: FAST-LIO2 only

Decided 2026-06-16: **FAST-LIO2 only — no KISS-ICP anywhere.** The localizer under study is
FAST-LIO2 on corridor2.0; the "system covariance" strawman is FAST-LIO2's EKF covariance;
pseudo-GT = loop-closed FAST-LIO2 trajectory (`traj`) vs raw (`trajraw`). T3 = FAST-LIO2
corridor2.0 drift / z-sag context only. KISS-ICP remains only as a cited related method.

---

## 14. Implementation & Verification Plan (execution)

Engineering conventions for the whole build:
- **Language/env:** Python in `env_py311` (torch 2.12 + cu130, 2× RTX 6000 Ada, sklearn, scipy,
  matplotlib; `pip install mcap mcap-ros2-support` for the bag).
- **Parallelization is mandatory for speed:**
  - CPU-bound (mcap parsing, per-scan I/O) → `multiprocessing.Pool` across all cores.
  - Math over the 7,619 scans → **vectorized / GPU-batched** (no per-scan Python loops):
    normals + 6×6 constraint-matrix eigendecomposition via batched `torch.linalg.eigh` on CUDA;
    voxel-downsample each scan first so KNN is cheap.
  - Tier-3 learned models → distribute ensemble members across **both GPUs**; large batches.
  - Determinism: fixed seeds; cache every stage to `cache/*.npz` so reruns are instant.
- **Directory layout:**
  ```
  06_conformal_observability/
    PLAN.md  FINDINGS.md(final)
    src/  extract.py observability.py conformal.py baselines.py learned.py
          safe_action.py figures.py common.py
    cache/    per-scan extracted + computed arrays (npz)
    results/  tables (csv/json) + per-part verification reports (md)
    figs/     maps (M*) + plots (P*)
  ```

### The five parts (each ends with a parallel adversarial validation gate)

**PART 1 — Data foundation.** `extract.py`. Read corridor2.0 mcap → per-scan XYZIRT clouds,
IMU, `/lio/odometry` + EKF covariance, timestamps; load pseudo-GT (`traj`,`trajraw`). Mask
NaN/zero padding; align each scan to nearest odometry by timestamp; compute per-scan pose error
(`trajraw`−`traj`). Cache to `cache/scans.npz`, `cache/poses.npz`. *Parallel: multiprocessing over
the mcap message stream.* Deliverable: clean cached dataset + a sanity report.

**PART 2 — Observability signal (A).** `observability.py`. Per-scan normals (KNN+PCA, GPU),
constraint matrix `C=Σcᵢcᵢᵀ`, batched eigendecomp → `λ_min, κ`, blind eigenvector; geometric
features (planarity, density, sparsity, normal-space coverage); decompose pose error into
along-/cross-corridor. *Parallel: GPU-batched over all scans.* Deliverable: per-scan signal arrays
+ first validation that the signal predicts error (sparsification/AUSE, correlation).

**PART 3 — Conformal + Tier 1&2 baselines (B, C, D).** `conformal.py`, `baselines.py`. Temporal
block split; scores (analytic, normalized); split conformal, **ACI**, **Conformal-PID**, **CRC**;
Tier-1 signal baselines (constant, FAST-LIO2 EKF covariance, ICP residual, density) + Tier-2
calibrators (temp-scaling, split, ACI/PID); metrics (PICP, MPIW, AUSE, AURC, Winkler,
coverage-over-time). Deliverable: T1 master table (Tiers 1–2) + calibration/coverage plots.

**PART 4 — Tier 3 learned (LCP + Bayesian baselines) + energy.** `learned.py`. Tiny **LCP** score
MLP over geometric features; **MC-dropout**, **deep ensemble**, **evidential** pose-error
regressors. Measure latency + energy (NVML). *Parallel: ensemble members across both GPUs.*
Deliverable: Tier-3 rows in T1 + energy/Pareto-vs-ensembles plot. (Punchline: same coverage,
tighter intervals, fraction of the energy.)

**PART 5 — Safe action + maps + synthesis (E, F).** `safe_action.py`, `figures.py`. Uncertainty→
speed gate (full/crawl/stop, hysteresis); counterfactual lead-time (trigger calibrated on good
data only); safety–efficiency Pareto. Render maps **M1–M5** (GPU rasterizer) + plots **P1–P7** +
final tables **T1–T3**. Write **FINDINGS.md** = holistic conclusions, what's solid vs shaky, the
numbers that tell each story.

### Per-part validation gate (parallel sub-agents)

After EACH part, before proceeding, launch **3–4 parallel sub-agents** with distinct adversarial
lenses; each returns a structured verdict (PASS / ISSUES + specifics):
- **V1 Code auditor** — read the part's code: bugs, logic errors, edge cases, parallelization
  correctness (races, dtype, masking), determinism.
- **V2 Numerical validator** — independently recompute/spot-check key numbers from the cached
  arrays; check for **data leakage / circularity** (e.g., test info in calibration; thresholding
  the error itself); confirm shapes/units/frames.
- **V3 Adversarial skeptic** — try to *refute* the part's headline claim; argue what's off,
  missed, or wrongly assumed; flag overclaiming.
- **V4 Methodology checker** — verify math/stats against the literature (conformal quantile
  formula, AUSE/sparsification definition, constraint-matrix construction, ACI update).

The main loop collects verdicts → fixes any real issues → re-verifies → only then advances.
Each gate's consolidated verdict is saved to `results/verification_partN.md`.

### Final cross-check — "the game" (adversarial tournament)

When all five parts are done, launch a multi-agent adversarial review in distinct roles, each
told to *break* the work, then a synthesis judges what survives:
- **Red-team skeptic** — attack the headline claims (does CO-Loc really beat the baselines? is
  the counterfactual honest?).
- **Conformal statistician** — coverage validity, exchangeability handling, marginal vs
  conditional, calibration-set size.
- **SLAM/LiDAR geometry expert** — is the observability signal physically right (aperture/
  degeneracy), frames/extrinsics, deskew, the eigenvector=corridor-axis claim.
- **Reproducibility re-runner** — re-run key scripts from cache, confirm the numbers in T1/figs.

Output: `results/FINAL_CROSSCHECK.md` (objections that survive + fixes), folded into FINDINGS.md.
Nothing ships as a "result" until it survives this round.

---

## 15. PART-1 FINDING — pseudo-GT corrected (2026-06-16)

**The agreed `trajraw − traj` pose-error target is INVALID.** `build_loopclosed_fastlio.py:71`
closes the loop with a **linear ramp** (`pos_c = pos + frac·gap`), so `trajraw − traj` is a pure
line in scan index: `corr=0.99999999998`, linear R²=`0.99999999998`, detrended residual = 2.7 cm
(numerical noise). It carries zero per-scan information.

Root cause is fundamental: **no survey GT exists, and the along-corridor error is unobservable
from LiDAR** (our thesis). The loop-closed map is also self-consistent with the FAST-LIO poses
(built from them) → registering an untouched scan to it is **circular** (≈0 residual).

**Corrected error methodology (approved — "Both"):**
- **Injected-GT recovery (rigorous core).** Build a reference map from **even** scans (each
  transformed by its true pose = odom rotation + `traj` position). On **odd** scans inject a
  *known* perturbation δ (random for the conformal dataset; controlled along-/cross-corridor for
  the aperture validation), run point-to-plane ICP back to the map, and measure the residual
  error Y = ‖recovered − true‖. True error is known → rigorous coverage. Non-circular (test scan
  not in the reference map).
- **Real-drive narrative (no synthetic GT):** the per-scan **observability** signal on real
  scans (maps M1/M2) + the **loop-closure drift budget** (~1.25 m) shown to concentrate in
  low-observability stretches (∫ low-λ_min arc-length predicts the drift).
- Drop `trajraw − traj`. Conformal target Y = post-recovery residual error; X = observability
  features; ACI over scan order. Caveat stated in FINDINGS.

---

## 16. PART 1+2 VERIFICATION GATE — verdict (2026-06-16)

4 parallel adversarial agents. **Code/math correct & reproducible; headline scientific claims do
NOT hold on this data.**
- Anisotropic "16% smaller area" REVERSES at matched coverage (obs ellipse becomes larger). It was
  an artifact of obs under-covering by 0.3pp. DROP the efficiency claim.
- Observability signal does NOT predict recovery error (Spearman ≈ −0.16, wrong sign; AUSE worse
  than random). Root cause: TARGET/SIGNAL MISMATCH — scan-to-GLOBAL-map recovery is well-conditioned,
  so it cannot exhibit the corridor aperture the signal measures (incremental odometry is where the
  aperture lives, but consecutive scans overlap too much to show it either).
- Natural error is isotropic (1.05×); the 1.93× only under a fixed directional probe (presupposes
  the answer). Single-scan recovery is robust (cm-level) → conformalized error is near-trivial.
- AUSE metric BUG: sparsification ordering inverted (keeps most-uncertain instead of removing).
- HOLDS: EKF covariance over-confident 13–100× (real); conformal quantile / ACI correct; geometric
  aperture (forward=blind 88%) is a real geometric fact but doesn't drive error in the full-sensor regime.

**Conclusion:** the full 360° dataset is too well-conditioned for the "observability predicts error"
thesis. Need a premise decision: (1) degrade-to-reveal (sparsify/limit FOV/range to create a real
degenerate regime where observability predicts failure), (2) pivot to the modest-but-true
"conformal calibration of over-confident LiDAR-odometry uncertainty" story, or (3) rethink scope.
AWAITING DIVAKE'S STEER.

---

## 17. DEGRADE-TO-REVEAL RESULT — definitive (2026-06-16)

Degradation (range-truncate the live scan) DOES reveal the aperture at the AGGREGATE level:
range 40→3 m ⇒ along/cross error ratio 1.06→1.77, a_along 0.282→0.191, failure(>0.3m) 6%→18.6%.
AND the error is globally anisotropic ⇒ a FIXED anisotropic conformal region is 8.4% tighter than
isotropic at matched 0.90 coverage (real, modest).

BUT the core thesis is FALSIFIED: the per-scan observability signal does NOT predict per-scan
error/failure.
- Spearman(1/a_along, |e_along|) = +0.13 (weak); Pearson +0.04.
- Failure-detection AUROC: 1/a_along 0.557, 1/npts 0.607, 1/range 0.618 (all weak; the predictive
  ones are CONTROLLED INPUTS, not a signal). WITHIN each degradation level AUROC ≈ 0.5 (chance),
  0.38 at full range. The observability-ADAPTIVE region (0.964) does NOT beat the FIXED one (0.916).

Conclusion: per-scan geometric observability is not a useful localization-uncertainty predictor on
this dataset. What is SOLID & presentable: (1) the autonomy demo; (2) localization reliability vs
sensing quality (degradation curve) + the corridor aperture; (3) EKF over-confidence 13-100x;
(4) conformal + ACI give valid, anisotropic, online bounds the EKF cannot. The "observability
predicts uncertainty" claim must be dropped or reported as an honest negative. AWAITING STEER on
the honest reframe.

---

## 18. FINAL CROSS-CHECK ("the game") — verdict (2026-06-16)

3 adversarial agents on the reframed results. Full record: results/FINAL_CROSSCHECK.md. All
numbers reproduce deterministically. SOLID: ACI calibration (strongest), EKF-not-a-safety-bound
(~9×), degradation+aperture trend, fixed-anisotropic 8.4% at matched coverage. REFRAMED/SOFTENED
in FINDINGS: (1) the safe-action "conformal gate" is really a density/range gate — conformal is
the calibrated bound, not the gating skill (a raw range gate matches it); (2) 8.4% = fixed
anisotropic, not obs-adaptive (3.6%); (3) EKF wording softened (local cov vs global residual);
(4) false-alarm guarantee stated as held-out (~10%); (5) coverage is marginal, n≈#scans.
Project complete: honest sensor-quality + conformal-safety story + the autonomy demo.
