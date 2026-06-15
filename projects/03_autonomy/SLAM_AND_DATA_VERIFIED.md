# SLAM & Data — verified reference (2026-06-15)

> **Read this before touching SLAM / the map / loop closure again.** It exists so we
> never re-have the "KISS-SLAM vs FAST-LIO" confusion or re-derive the data audit.
> Every number here was produced by **independent adversarial verification agents**
> (3 of them, run in parallel), not by a single pass. Verdicts at the bottom.

---

## 0. The confusion-killer: the TWO SLAM systems

| | **FAST-LIO2** | **KISS-SLAM / KISS-ICP** |
|---|---|---|
| Sensors | LiDAR **+ IMU** | LiDAR **only** (no IMU) |
| Loop closure? | ❌ none (pure odometry) | ✅ has a back-end… |
| …did it close our loop? | n/a | ❌ **"0 closures found"** — back-end was inert |
| Drift on corridor2 | **1.52 m over 266 m (0.57%)** | hand-closed by linear ramp (same as us) |
| Trajectory quality | smooth, jump-free (max step 160 mm) | **7× jitterier** (Z-step std 34.7 vs 4.8 mm) |

**The trap (this is what kept biting us):** the system *with the IMU* (FAST-LIO) is the
one that does **not** close loops; the system that *has* loop-closure (KISS-SLAM) is the
one **without** the IMU — **and its loop-closure detector found nothing** on our data.
So neither tool, as run, closed the loop. We close it ourselves, offline (§3).

**Decision: build the map on the FAST-LIO trajectory.** ✅ Verified correct (§4).

---

## 1. Data audit — `corridor2.0` bag — VERIFIED **PASS**

Bag: `projects/02_fast_lio2/results/corridor2.0/corridor2.0_0.mcap` (5.9 GB, 761.9 s,
RC-driven by hand). Recorded **with only the IMU from the D455 — no color/depth images.**

- **`/rslidar_points`**: 7619 scans @ **10.001 Hz**, headers strictly monotonic, **0 dropped
  scans** (no gap > 0.2 s). Organized 1800×16, `point_step 26`.
  - Fields (full XYZIRT): `x,y,z (f32)  intensity (f32)  ring (uint16)  timestamp (f64)`.
  - Per scan 28 800 pts; ~1% NaN (invalid returns are cleanly all-NaN, `is_dense=False`);
    `ring` spans 0..15 every scan; per-point `timestamp` spans exactly one 0.0995 s rev →
    **deskew-ready**.
- **`/camera/camera/imu`**: 151 910 msgs @ **199.4 Hz**, 0 NaN; at rest **|accel| 9.584**
  (−2.3% factory bias on the uncalibrated D455 — FAST-LIO estimates it online), gyro ≈ 0.
- **`/lio/odometry`**: 159 515 @ ~209 Hz, continuous, **0 jumps**.

**Conclusion:** the data is sound for BOTH loop-closed SLAM and point-cloud ML. The only
gap for research: **no RGB/depth images** (only IMU was recorded from the camera). Fine for
point-cloud work; add images on a future run if RGB-D fusion / a richer demo is wanted.

---

## 2. What we have vs what "proper SLAM" means

- We have: a **FAST-LIO odometry map** (`corridor2.0_map/map_dense.ply`, 3.5 M pts) +
  the **loop-closed map** we just built (§3). Both LiDAR-inertial odometry under the hood.
- True survey-grade SLAM would add a **pose-graph bundle adjustment** (corrects rotation +
  non-uniform drift across the whole loop). We have **not** done that — and for our
  purposes (teach-and-repeat + point-cloud ML) we do **not** need it. The manual closure
  below is sufficient and verified.

---

## 3. Loop closure (the offline step) — VERIFIED **PASS**

Builder: `run/build_loopclosed_fastlio.py`. Output:
`results/corridor2_loopclosed_fastlio.npz` (keys `world` 3.70 M pts, `traj`, `trajraw`) +
`results/corridor2_loopclosed_fastlio.png`. Seam proof:
`results/seam_zoom_loopclosure.png`.

**Method (simple, deliberately):** we KNOW it's a loop (robot returned to start), so we don't
rely on auto loop-detection (which found 0). We compute `gap = pose[start] − pose[end]` and
add `frac·gap` to every pose, `frac = (t−t0)/(t1−t0)` → 0 at start, 1 at end. Translation-only,
linear-in-time. Then re-accumulate every scan at its corrected pose.

**Verified results (agent rebuilt the seam from the bag, independently):**
- Residual END–START: 1.52 m → **0.0106 m** *(≈0 BY CONSTRUCTION — not evidence by itself)*.
- **The real test (seam alignment):** full-3D nearest-neighbour of END-wall to START-wall:
  **0.585 m → 0.143 m (4.1× better)**. The double-wall was **49% vertical** — a **0.75 m
  double-floor** that a top-down view hides entirely; the closure collapses it (see the XZ
  panels of `seam_zoom_loopclosure.png`).
- **No distortion introduced:** the correction is provably parallel to the gap (max
  perpendicular 0.010 m) → an affine ramp that **cannot bend a straight wall**. Long walls
  stay straight (PCA-fit RMS 0.17–0.20 m = sensor noise); loop length (266 m) and footprint
  (85.8 × 25.9 m) unchanged.

**Honest limits of this closure:** translation-only (a residual end-of-loop *heading* error
is uncorrected — ~0.14 m leftover at the seam); linear-by-time assumes uniform drift accrual
(benign here at 1.5 m; would distort if drift were large/burst-like). The survey-grade
upgrade is a pose-graph BA — not needed now.

---

## 4. CORRECTED understanding — read this, the old rationale was wrong

We long justified FAST-LIO with: *"a long featureless corridor is degenerate → LiDAR-only
can't measure forward motion → KISS-ICP UNDER-travels (sim: 12.5 m driven / 3.7 m tracked)."*

**On the real `corridor2.0` data that is empirically FALSE.** Verified:
- KISS-SLAM path length **345 m vs FAST-LIO 260 m — KISS is LONGER (+32.5%), not shorter.**
  Its corridor-axis span is **identical** (85.5 m) → **no compression / no under-travel.**
- The extra 85 m is **per-scan jitter, not travel**: KISS Z-step std 7× FAST-LIO's, 991
  steps > 100 mm (vs 6), Z swings ±5.7 m on a flat floor. Smoothed, both → ~254 m.

**Why FAST-LIO is still the right choice (corrected reason):** not because KISS under-travels,
but because **KISS is far noisier/jitterier** and its loop-closure gave **zero** benefit,
while FAST-LIO is **smooth, jump-free (max step 160 mm), and geometrically self-consistent**
with the clean single-walled dense map. *(The sim degeneracy is real physics; the real
building just has enough features — doors, alcoves, junctions — that it didn't dominate here.
Don't cite "KISS under-travels" as fact for this dataset.)*

---

## 5. Deployment caveats (carry these into teach-and-repeat)

Both systems are **open-loop** mid-route (real loop closure only at the seam). Therefore:
- Far-loop position error reaches **~1.5–5 m** before the seam. **Junction logic must
  tolerate ±2 m position slack** — gate decisions by approximate progress + **reactive**
  junction detection (geometry), never tight global alignment.
- **The start pose must be reproduced accurately** or the whole path is offset (frame is set
  at FAST-LIO init = wherever the robot is at session start).
- FAST-LIO has a slow **2.36 m Z-drift** ramp → **do not use map Z** for anything.
- Keep the **map/path OUT of the lateral steering loop** — eyes-only centering owns lateral;
  the path is for **along-track progress + junction turn decisions only**. (See
  `ROBOT_CONTROL_LEARNINGS.md` §9; memory `eyes-only-corridor-centering`.)

---

## 6. Verification verdicts (2026-06-15, 3 parallel adversarial agents)

| Agent | Scope | Verdict |
|---|---|---|
| Data integrity | bag fields/rates/timestamps/IMU | **PASS** |
| Loop-closure correctness | seam alignment, distortion | **PASS** (4.1× seam, no distortion) |
| Trajectory / methodology | FAST-LIO vs KISS, drift, deployment | **CONCERN** — FAST-LIO is the right source, but (a) open-loop drift budget 1.5–5 m needs explicit ±2 m junction slack, (b) the "KISS under-travels" rationale is false for this data (see §4) |

**Net:** data ✅, closure ✅, trajectory trustworthy for teach-and-repeat **with** the §5
caveats engineered in. Not yet survey-grade (no pose-graph BA) — fine for our goals.

## 7. Artifacts
- Loop-closed map: `results/corridor2_loopclosed_fastlio.npz` (+ `.png`)
- Seam proof: `results/seam_zoom_loopclosure.png`
- Trajectory checks: `results/verify_fastlio_shape.png`,
  `results/verify_overlay_fastlio_vs_kiss.png`
- Builder: `run/build_loopclosed_fastlio.py` (FAST-LIO) ; `run/build_loopclosed_3d_map.py` (KISS, superseded for the map)
- Source bag: `../02_fast_lio2/results/corridor2.0/corridor2.0_0.mcap`
- Taught route: `results/teach_path_fastlio.csv` (251 m, 10 turns — see `taught_route_turns.png`)
