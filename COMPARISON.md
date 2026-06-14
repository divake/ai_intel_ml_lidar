# Odometry / Drift Comparison — KISS-ICP vs FAST-LIO2

> **Living document.** Each experiment that improves (or tests) localization drift
> gets a row/section here. Updated **2026-06-13**.
> Sources: [project 01 (KISS-ICP)](projects/01_kiss_icp_mapping/FINDINGS.md) ·
> [project 02 (FAST-LIO2)](projects/02_fast_lio2/README.md).

---

## TL;DR — adding the IMU roughly halved drift and flattened the floor

| Metric (same corridor loop) | **KISS-ICP** (LiDAR-only, proj 01) | **FAST-LIO2** (LiDAR + IMU, proj 02) | Improvement |
|---|---|---|---|
| **Loop-closure drift** (‖end − start‖) | **4.00 m** | **1.52 m** | **2.6× lower (−62 %)** |
| **Z-sag** (floor flatness) | **3.40 m** | **0.75 m** | **4.5× lower (−78 %)** |

These two are the trustworthy metrics. (Path length is **not** a reliable
comparison metric — see [§4](#4-why-the-path-length-differs-392-m-vs-305-m).)

---

## 1. What changed between the two

| | KISS-ICP (project 01) | FAST-LIO2 (project 02) |
|---|---|---|
| Sensors | LiDAR only | LiDAR + **D455 IMU** (200 Hz, the rig's only inertial source) |
| Algorithm | frame-to-frame ICP (pure odometry) | tightly-coupled **iterated-EKF LiDAR-inertial** |
| Point format | XYZI (ring rebuilt by hand, **no deskew**) | **XYZIRT** — native ring + per-point timestamps → real motion deskew |
| Drift correction | none (no IMU, no loop closure) | IMU **slows** drift + pins roll/pitch to gravity |
| Loop closure | none | none (still odometry — see [§6](#6-whats-next)) |

The IMU is **drift *reduction*, not a *cure*** — it does not close the loop, it
makes each step more accurate and uses gravity to keep the floor flat.

---

## 2. The two runs

| | corridor1 (KISS-ICP) | corridor2.0 (FAST-LIO2) |
|---|---|---|
| Date | project 01 | 2026-06-13 |
| Duration | — | 12.7 min (761.9 s) |
| Bag | `01_kiss_icp_mapping/results/corridor1` | `02_fast_lio2/results/corridor2.0` (5.6 GB) |
| Sensor health | — | IMU continuous **199.4 Hz**, LiDAR **10.0 Hz**, 0 UDP drops |

> corridor2.0 was re-recorded after a first attempt (`corridor2`) was scrapped:
> a loose D455 USB cable shook free mid-drive (vibration) and killed the IMU for
> ~10 min. Cable reseated + strain-relieved; the clean run has continuous IMU
> end-to-end (verified from the bag).

---

## 3. Full metrics (rate-independent route descriptors included)

| | corridor1 KISS-ICP | corridor2.0 FAST-LIO2 |
|---|---|---|
| Loop-closure drift | **4.00 m** | **1.52 m** |
| Z-sag | **3.40 m** | **0.75 m** |
| Max excursion from start | 54.1 m | 52.6 m |
| XY extent | **86 × 39 m** | **86 × 27 m** |
| Z range | 3.38 m | 2.37 m |
| Path length (at native rate) | 392.5 m @ ~10 Hz | 305.2 m @ ~209 Hz |
| Trajectory samples | 11 093 | 159 515 |

---

## 4. Why the path length differs (392 m vs 305 m) — *it's the same route*

The robot drove the **same physical route** — proven by the rate-independent
descriptors: **max excursion (54.1 vs 52.6 m) and X-extent (86 vs 86 m) match
almost exactly.** The path-length gap is a **measurement artifact**, from two
effects — and both actually favor FAST-LIO2:

**(a) KISS-ICP's drift inflates its path.** On the same route its trajectory is
*wider in Y* (39 vs 27 m) and *taller in Z* (3.38 vs 2.37 m). That extra spread
is drift-induced **wander**, and wander gets summed as extra distance.

**(b) Sample-rate inflation.** Path-length-by-summing grows with sample rate
(high-frequency jitter adds up). Re-sampling corridor2.0 to KISS-ICP's rate:

| corridor2.0 sampled at | path length |
|---|---|
| 209 Hz (raw) | 305.2 m |
| ~10 Hz (matched to KISS-ICP) | **267.0 m** |
| ~2 Hz (≈ true geometry) | 258.2 m |

**Rate-matched at 10 Hz: FAST-LIO2 = 267 m vs KISS-ICP = 392 m on the identical
corridor.** That ~125 m of extra "ghost distance" is KISS-ICP's accumulated drift
physically showing up as a longer, wider, more sagging trajectory.

---

## 5. Lesson — which metrics to trust

- ✅ **Loop-closure error** (‖end − start‖, robot returns to start) and
  **Z-sag** — robust, and both clearly favor FAST-LIO2.
- ⚠️ **Path length / drift-as-%-of-path** — unreliable: inflated by sample rate
  *and* by the very drift you're trying to measure. Do **not** use it to compare
  odometry systems.

---

## 6. Caveats & what's next

- **Pending (definitive apples-to-apples):** replay **KISS-ICP on `corridor2.0`**
  (the bag has raw XYZIRT) so both algorithms process the *identical* drive —
  removes any route-difference caveat and shows KISS-ICP's loop error + path
  inflation head-to-head. Plus a **top-down trajectory overlay** plot.
- Neither system has **loop closure** yet. Next big lever for drift is a
  loop-closing SLAM / map-based relocalization — gated decision after the
  apples-to-apples result (see project-02 plan).

---

### Result log (append future comparisons here)

| Run | System | Loop drift | Z-sag | Notes |
|---|---|---|---|---|
| corridor1 | KISS-ICP (LiDAR-only) | 4.00 m | 3.40 m | baseline |
| corridor2.0 | FAST-LIO2 (LiDAR+IMU) | 1.52 m | 0.75 m | IMU added; XYZIRT deskew |
