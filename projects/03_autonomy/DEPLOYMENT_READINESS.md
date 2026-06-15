# Teach-and-Repeat — Deployment Readiness (red-teamed 2026-06-15)

> Verdict from independent adversarial review (round 2): **NEEDS-FIX-FIRST.** The SLAM/map/
> route are verified sound; the *deployment plan* as first drafted has blockers. This doc is
> the corrected plan. Read with `SLAM_AND_DATA_VERIFIED.md`.

## TL;DR
- **SLAM + data + map: verified twice, solid.** Build ML/research on it freely.
- **Full figure-8 teach-and-repeat: do NOT attempt as first drafted.** Three blockers (below).
- **The fix is an architecture change:** drive on **route-memory (junction counting)**, NOT
  live FAST-LIO progress. That removes the worst blocker entirely.
- **First hardware test should be descoped to ONE rectangle** (no spine re-cross) — low risk,
  no live FAST-LIO, proves autonomy before the hard figure-8.

## Round-2 verification verdicts
| Agent | Scope | Verdict |
|---|---|---|
| A | reproduce data + closure with *different* metrics | **PASS** — voxel-IoU 4.4–5.5×, Chamfer 3.2–5.7×, double-floor 0.30→0.003 m; walls RMS 0.06–0.08 m; dedup lossless. All round-1 numbers reproduced. |
| B | taught-route extraction | **PASS** (with note) — 251.6 m / 1.25 m loop confirmed; but **12 turns, not 10** (spine entries are 3-lobe S-bends); **2 decision nodes** (top/bottom spine), each driven twice. |
| C | red-team the deployment | **NEEDS-FIX-FIRST** — blockers FM-1/2/4 below. |

## The blockers (and the fix)

### FM-1 (was BLOCKER) — live FAST-LIO back-pressures the eyes controller
Verified in source: rslidar publisher is reliable, FAST-LIO subscribes `KeepLast(10).reliable`,
**and the GOLDEN `corridor_center.py` subscribes RELIABLE (`:94`)**. A stalled FAST-LIO throttles
the 731 KB cloud for *all* readers. Making the publisher `best_effort` would make the RELIABLE
golden controller get **zero** data (QoS incompatibility) — and the golden file is do-not-modify.
This contradicts our own finding "FAST-LIO must stay OFF while driving."
- **FIX: don't run FAST-LIO in the driving loop at all.** Use **route-memory (junction count)**
  for progress instead of FAST-LIO position. Eliminates the blocker.

### FM-2 (was BLOCKER) — start-heading sensitivity
1° start-heading error = 0.92 m lateral at the far loop; 3° = 2.75 m (> the ±2 m slack) before
any drift. Hand-placing heading to the needed ~0.6° is impossible.
- **FIX:** route-memory doesn't use absolute progress, so this mostly evaporates. If any absolute
  progress is used, **measure-and-zero the start heading** by driving 3–5 m straight and reading
  the eyes' wall-angle `alpha` vs the recorded local heading — never trust hand-placement.

### FM-4 (was BLOCKER for full loop) — the spine is driven TWICE, same direction, identical scene
The mid-spine (x≈−3.5) is traversed at arc 100–116 m and again 230–246 m, both heading ~−90°,
identical LiDAR scene. The memoryless eyes can't tell the passes apart. The two passes are **130 m
apart on the route** and the **same physical node needs OPPOSITE turns** on each pass (bottom-center
exits WEST then EAST).
- **FIX:** **junction-count route memory** is the discriminator ("this is the 1st vs 2nd spine
  entry"), not scene and not absolute progress. Validate the full figure-8 in the **sim**
  (`sim/.../corridor_sim.py`, which loads this exact path) BEFORE hardware. If it can't be made
  deterministic offline, **descope to one rectangle**.

### FM-3 (MAJOR) — cavity false-junctions
The reactive TURN fires on `front<1.0` and picks the "more-open" side; lab-doorway cavities mis-fire
(confirmed in run logs: `TURN_RIGHT` fired in 3 episodes). 
- **FIX: the router does TWO jobs** — (1) inject the taught turn AT taught junctions; (2) **VETO the
  reactive TURN everywhere else** (force straight-through). Count a junction only when **both sides
  open, sustained over N scans** (a doorway opens one side only). Add a min-translation dwell before
  a second turn can fire (kills ping-pong). Reject the off-route **SE room/tail opening** at
  bottom-center (Agent B: 223 k off-route points hang SE; the robot never drove there).

### FM-5 (MAJOR) — `range_max=8 m` saturates the open-side heuristic at wide junctions
At a true junction both sides read ≥8 m → open-side choice is noise. At taught junctions the router
must **own** the decision and ignore `openL/openR`.

### FM-6 (MAJOR, operational) — reboot/hardware sequencing
Positive pre-flight checks required (abort if any fails): `candump` shows CAN-mode status; CAN TX
climbing; `/camera/camera/imu`≈198 Hz; `/rslidar_points`=10.0 Hz; `sysctl net.core.rmem_max`=26214400;
(if FAST-LIO used at all) `/lio/odometry`≈208 Hz. The D455 `device busy` needs a physical replug.

### FM-7 (MINOR) — no loop-complete detection
The eyes controller has no "I'm done" notion. The router (with junction count) declares completion
when all junctions are passed and progress wraps to ~start.

## Safety (the question that matters)
Even if routing fully fails, the eyes controller is **repel-only + hard overrides** (`hard_min=0.35`,
`crit=0.28`, `front<0.30`→stop) and has **never hit a wall in ~25 min / 8700 cycles** — it *wanders
safely*. **Correction to the docs:** the closest wall approach was **0.351 m** (run1) / 0.451 m
(run2), NOT the "0.47 m" previously written — still safe (body half-width 0.29 m → ~6 cm clearance),
but the headline was optimistic. **The router's injected turn must remain SUBORDINATE to the
crit/hard_min safety overrides** — never able to countermand them.

## The corrected plan (in order)
1. **Build the route-memory junction-router** (thin wrapper; golden core untouched): junction-count
   progress, inject taught turn at the Nth ambiguous junction, veto reactive turns elsewhere,
   subordinate to safety. **No live FAST-LIO.**
2. **Validate in the sim** on this exact path (figure-8, incl. both spine passes) — must be
   deterministic before hardware.
3. **First hardware test = ONE rectangle** (right loop, no spine re-cross): eyes-only + one initial
   turn. Proves autonomy with no blockers.
4. **Then the full figure-8** with the route-memory router, supervised, e-stop in hand.
5. Pre-flight hardware checklist (FM-6) before every run.

## Decision-junction reference (from Agent B, verified)
2 physical nodes (top-center y≈19, bottom-center y≈2), 4 decision events over the loop:
| arc (m) | node | pass | required turn |
|---|---|---|---|
| ~95 | TOP-CENTER | 1 (from right loop) | enter spine, go down |
| ~119 | BOTTOM-CENTER | 1 | turn **WEST** into left loop |
| ~227 | TOP-CENTER | 2 (from left loop) | enter spine, go down |
| ~249 | BOTTOM-CENTER | 2 | turn **EAST** back to START |

Passes are 130 m apart → trivially disambiguated by junction count. Artifacts:
`results/route_verified_junctions.png`, `results/central_junction_zoom.png`.
