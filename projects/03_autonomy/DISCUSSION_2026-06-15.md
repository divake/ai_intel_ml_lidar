# Discussion Log — 2026-06-15 (proj03 autonomy + research direction)

> **HOW TO READ THIS:** This is a record of what we *discussed*, **not** a committed plan
> or a TODO list. "Discussed" = talking points + reasoning we want to keep in mind.
> Nothing here is a decision to build unless we explicitly choose it later.
>
> **For the main Claude Code thread:** use this as context, then think and plan. The user
> asked a series of conceptual/strategic questions; my answers are digested below. Full
> detail lives in the fork transcript; this is the canonical digest.

---

## 0. Operational state at end of this session (what we actually DID today)

- **The robot did NOT drive this session.** We brought the rig up, found and fixed sensor
  issues, and then spent the session answering the user's strategic questions. The user
  explicitly held the drive ("let me know before you start the robot — I have questions").
- **Rig is healthy and GO** (verified, holding): CAN up; LiDAR clean **10 Hz** full-res
  (1800×16); D455 **IMU healthy** (proven by FAST-LIO odometry at 209 Hz); Scout base
  connected; controller processes at ~10 Hz with sane geometry.
- **NEW root-cause found today (important):** the "erratic LiDAR" was **never the sensor**.
  A single fault chain — camera `device busy` → no IMU → **FAST-LIO stalls → as a RELIABLE
  subscriber it back-pressures the 750 KB cloud → bursty delivery to *every* consumer**
  (incl. the controller). One fault made both sensors look broken. CPU was idle (0.00) the
  whole time, so it was a **QoS problem, not compute**.
  - **Fix for the camera:** physical D455 unplug/replug (software restart did NOT clear
    `device busy`). User did this; camera came up clean.
  - **Consequence for driving:** **FAST-LIO must stay OFF during driving** (eyes-only). The
    proper fix to let FAST-LIO run live alongside driving is a **one-line QoS change**:
    publish `/rslidar_points` as **`best_effort`** (sensor QoS) so no slow subscriber can
    back-pressure it. Not done yet — flagged as the enabler for live map + live dashboard.
  - Verified: `ros2 bag record` does **not** back-pressure (controller stayed 9.99 Hz while
    recording) → drive + record raw data is safe.
- **Golden controller unchanged** (`locomotion/corridor_center.py`) — do not modify.

---

## The discussion — questions top-to-bottom, with answers

### Q1. What is SLAM? Are we doing it? Did the first attempt work?
- **SLAM = Simultaneous Localization And Mapping** — build a map *and* localize within it at
  the same time (chicken-and-egg). Three parts: **front-end** (odometry / scan-matching),
  **back-end** (pose-graph optimization), **loop closure** (recognize a revisited place,
  correct accumulated drift). **Loop closure is the line between true SLAM and mere
  odometry.**
- **Are we doing SLAM? Honest answer: we've been doing the front-end half — odometry + live
  mapping — NOT yet full loop-closing SLAM.**
  - Project 01 (KISS-ICP): LiDAR-**only** odometry + mapping, **no loop closure → drifts
    ~1%**. The authors themselves call it *odometry*.
  - Project 02 (FAST-LIO2): LiDAR-**inertial** *Odometry* (name says it) — IMU+LiDAR fused,
    no loop closure, **~4 m loop error** measured.
  - KISS-SLAM (planned): adds loop closure → *actually* SLAM.
- **"Did the first thing do SLAM?"** Project 01 produced a map + trajectory so it *looked*
  like SLAM, but it was odometry without loop closure: it drifted and couldn't self-correct.
  That drift (esp. the corridor failure) is *why* we moved to FAST-LIO2 (added IMU) and then
  toward KISS-SLAM (loop closure).
- **Note:** today's *driving* uses **no SLAM at all** — it's eyes-only reactive. SLAM is the
  mapping/data infrastructure, separate from the live steering loop.

### Q2. What is an IMU? Why does it matter? What do we get from ours; is it healthy?
- **IMU = Inertial Measurement Unit.** Ours (inside the RealSense D455) is **6-axis**:
  3-axis **accelerometer** (linear accel incl. gravity) + 3-axis **gyroscope** (angular
  velocity). No magnetometer.
- **Why it matters for US specifically:** the LiDAR can't measure forward translation in a
  long featureless corridor (aperture ambiguity — proven in sim: 12.5 m driven, 3.7 m
  tracked). **The IMU dead-reckons the forward motion the LiDAR can't see.** That is the
  entire reason Project 02 added the IMU after Project 01's pure-LiDAR KISS-ICP failed in the
  corridor. Also: ~200 Hz (vs LiDAR 10 Hz) fills motion between scans; gravity gives roll/
  pitch leveling. **Must be FUSED** (IMU alone drifts via double-integration; LiDAR corrects
  it, IMU fills LiDAR gaps) — that's FAST-LIO2's tightly-coupled iterated EKF.
- **What we gather:** topic `/camera/camera/imu`, **best_effort** QoS, ~190–200 Hz, frame
  `camera_imu_optical_frame`, `unite_imu_method:2` (accel+gyro merged into one stream). Plus
  a LiDAR↔IMU extrinsic in `config/helios16p_d455.yaml`.
- **How we know it's healthy:** (1) today FAST-LIO produced `/lio/odometry` at **209 Hz** —
  the pure IMU-propagation publisher can only run that fast if the IMU streams ~200 Hz;
  (2) documented physics check: at rest **|accel| ≈ 9.6 m/s²** (sees gravity correctly).

### Q3. Is my framework right? (input → processing → output) + visualization + status + north star
- **User's model:** inputs = LiDAR + IMU; middle = feed to A* / future ML, match map +
  localize; output = control wheels + capture localization/mapping/autonomy.
- **3 corrections:**
  1. **It's TWO separate pipelines, not one.** TRACK 1 (eyes-only: LiDAR → reactive
     centering → wheels) is the ONLY thing that steers today. TRACK 2 (LiDAR+IMU → FAST-LIO →
     map+pose) runs alongside as infrastructure and does **not** touch steering yet.
  2. **A\* is a *planner*, not a *localizer*.** A* picks the *route* ("at this junction go
     left"); it does NOT figure out *where you are* — that's localization (scan-matching).
     A* *consumes* a map + known position.
  3. **The IMU does NOT feed driving.** It feeds FAST-LIO (mapping). The live controller
     ignores it. A*, map-localization, and ML are **not implemented yet**.
- **Visualization:** `framework_2026-06-15.png` (+ `.gv` source) in this folder. Color key:
  green = working, orange dashed = to build, blue = research north star.
- **Status:**
  - ✅ DONE: LiDAR+IMU sensors; eyes-only driving (`corridor_center.py`); FAST-LIO2
    odometry+mapping; raw data capture (rosbag).
  - 🟠 NEED: (1) route memory (don't U-turn back — quick win, no map); (2) localization
    (scan↔map match); (3) loop closure / true SLAM (KISS-SLAM); (4) A* junction router.
  - 🔵 NORTH STAR: ML + uncertainty quantification on LiDAR/point clouds. Everything else is
    the rig that produces clean data for this.
  - Footnotes: wheel encoders exist but unused (unreliable); FAST-LIO off during driving
    (back-pressure) → live map for A* comes from recorded run rebuilt offline.

### Q4. Do people use A*/Dijkstra in navigation, or are we unique? What algorithm going forward?
- **Yes — it's THE textbook approach; the user re-derived the standard.** A*/Dijkstra are the
  **global planner** layer (not part of SLAM itself). Canonical stack:
  `Mapping(SLAM) → Localization → GLOBAL PLANNER (A*/Dijkstra) → Local controller`.
  Used everywhere: ROS **Nav2** ships A* (NavFn, Smac); self-driving uses **Hybrid-A\***;
  Roombas/warehouse/Google Maps routing — all this family. (A* = Dijkstra + heuristic.)
- **Correction:** A* is **not** what stopped the wall-banging and couldn't be — that was a
  control+localization failure (stale map frame + fishtail). **Garbage localization in →
  garbage path out.** The **eyes-only reactive controller** fixed the banging. A*'s real job
  is the *next* problem: junction route decisions.
- **Algorithm going forward:** for a known corridor loop with ~6–8 junctions, use **A\* over
  a topological graph** (nodes = junctions, edges = corridor segments) → tiny, instant, robust;
  lighter than grid-A* (Nav2). The eyes still drive; A* only whispers the turn at forks.
  **Key insight:** the planner is the easy 10%; the **map + localization underneath is the
  hard 90%** — that's where effort goes. Order: clean loop-closed map → localization → then
  A* is a ~20-line afterthought.

### Q5. Is the heavy GPU server (ADA-6000, 48 GB) a useful resource?
- **Yes — for the *research* half, not the driving half.**
- **Honest caveat:** it does **NOT** fix the FAST-LIO back-pressure (that's QoS, and FAST-LIO
  doesn't even use the GPU — it's a CPU EKF). And the **safety/control loop must stay on the
  NUC** (never put a network round-trip in the wheel-command path).
- **Where it's a game-changer:** training point-cloud ML + **uncertainty quantification** (the
  north star — NUC can't do this); offline map building / heavy SLAM; heavy live perception
  (semantic seg, detection) streamed back; dataset processing at scale.
- **Principle:** split by **latency-criticality** — NUC = real-time/safety; server = heavy/
  latency-tolerant.
- **Bridge:** tier 1 = record bags on NUC → `rsync` to server → train (do this first, trivial);
  tier 2 = **Zenoh bridge** (`zenoh-bridge-ros2dds`) for live streaming later. Wired gigabit
  handles the 10 Hz cloud (~60 Mbps); Wi-Fi marginal.

### Q6. Dashboard / GUI for demo + investor visualization?
- What's described **is** a robotics observability dashboard; **Foxglove (already in stack)
  does all of it**: 3D panel (reconstructed map + pose + heading arrow), camera image panels,
  telemetry plots (speed/direction from `/cmd_vel`), MCAP record + **replay with timeline
  scrub + variable speed**, and remote viewing over **Tailscale** (`foxglove_bridge`).
- **Recommended demo strategy:** **record run → rebuild map offline → replay in Foxglove →
  screen-capture at 10×.** Deterministic, controllable, no live failures. (10× only works in
  replay — the live robot stays slow for safety; user already knew this.) Live remote view is
  a nice secondary touch.
- **`rerun.io`** = code-first Python viz — use it **later for research** (e.g., coloring each
  point by its uncertainty / conformal set size). Foxglove for ops, rerun for research viz.
- **One dependency for a *live* reconstructing-map dashboard:** needs FAST-LIO running while
  driving → blocked by the back-pressure → same **best_effort QoS one-liner** unblocks it.

### Q7. The uncertainty research angle — plan it (baselines, ViT, validity, matrix, novelty)
- **The gap is real and precise** (grounded in current lit):
  - Conformal prediction for **2D image** segmentation → **done** (Mossina et al. CVPR-W 2024;
    deel-ai).
  - UQ for **LiDAR** → mostly MC-dropout / ensembles / evidential — **no finite-sample
    guarantee**, and **poorly calibrated under distribution shift** (IEEE T-ITS 2025 review).
  - **Conformal / distribution-free *validity* for 3D LiDAR point clouds → emerging / largely
    open.** ← this is the "validity angle people haven't done."
- **Backbone (the "ViT for point clouds"):** **Point Transformer V3 (PTv3, CVPR'24 Oral)** in
  the **Pointcept** codebase — SOTA, runs on the ADA-6000.
- **Comparison matrix (the paper's main table):** rows = {softmax, MC-dropout, deep
  ensembles, evidential, **conformal/CRC (ours)**}; cols = {mIoU, **coverage (≥1−α?)**,
  **efficiency (set size↓)**, calibration (ECE), OOD/shift, compute}. Story: baselines give
  uncertainty but no guarantee and break under shift; ours gives **provable coverage**, stays
  tight, holds under real-robot shift, and the robot acts on it.
- **Novel contribution (recommended combo):** (1) a **3D-native conformal score** (lift 2D
  conformal seg to point clouds via neighborhood geometry); (2) **shift-robust conformal**
  (adaptive/weighted CP, Conformal Risk Control) validated with **our corridor data as the
  natural distribution shift**; (3) **closed-loop risk-controlled autonomy** — CRC *guarantees*
  a safety bound and drives the robot's slow/stop/flag policy from prediction-set size.
  One-liner: *"Distribution-free, guaranteed uncertainty for 3D LiDAR perception that a robot
  can act on — validated in closed loop on real hardware."* (Embodied + formal-guarantee angle
  is underexplored — most UQ work is static-benchmark.)
- **Incremental roadmap (no hit-and-trial):** R0 baseline (collect data + train PTv3 on
  SemanticKITTI/nuScenes) → R1 UQ baselines + eval harness → R2 conformal for 3D seg (Paper 1)
  → R3 distribution shift with our data → R4 closed-loop risk-controlled robot (demo).
- **Groups to borrow quality from:** Angelopoulos & Bates / Candès (conformal + Conformal
  Risk Control); Pointcept / PTv3 (Wu, Hengshuang Zhao); deel-ai (2D conformal seg, liftable).

---

## Open threads / decisions NOT yet made (discussed, not decided)
- Whether/when to make `/rslidar_points` `best_effort` (unblocks live FAST-LIO + live map +
  live dashboard, without back-pressure).
- Whether the first research task is segmentation (recommended) vs detection vs occupancy.
- Whether to build the demo as live (Foxglove over Tailscale) or recorded→replay (recommended).
- Phase-2 autonomy order: route memory (quick win) → localization → A* junction routing.

## Pointers
- Golden controller: `locomotion/corridor_center.py` (eyes-only, do NOT modify).
- Key docs: `ROBOT_CONTROL_LEARNINGS.md` (§9 = the eyes-only breakthrough), `RESUME_HERE.md`,
  `../02_fast_lio2/FINDINGS.md` (#8 = LiDAR UDP buffer; camera `device busy` gotcha).
- Framework diagram: `framework_2026-06-15.png` / `.gv` (this folder).
- Memory: `research-uncertainty-lidar-ml` (north star), `eyes-only-corridor-centering`,
  `autonomy-project03-goal`.

## Sources (research angle)
- Conformal image segmentation: https://arxiv.org/abs/2405.05145
- deel-ai conformal-segmentation: https://github.com/deel-ai-papers/conformal-segmentation
- LiDAR seg UQ / OOD: https://arxiv.org/pdf/2410.08687
- Conformal Risk Control: https://arxiv.org/abs/2208.02814
- Evidential 3D detection: https://arxiv.org/abs/2410.23910
- UQ-for-AV review (IEEE T-ITS 2025): https://dl.acm.org/doi/10.1109/TITS.2025.3532803
- Point Transformer V3 (CVPR'24): https://openaccess.thecvf.com/content/CVPR2024/papers/Wu_Point_Transformer_V3_Simpler_Faster_Stronger_CVPR_2024_paper.pdf
- Pointcept codebase: https://github.com/pointcept/pointcept
- Foxglove vs Rerun: https://foxglove.dev/robotics/rerun-vs-foxglove
