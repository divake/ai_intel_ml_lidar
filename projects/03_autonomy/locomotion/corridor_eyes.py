"""Shared EYES-ONLY perception + control + route-memory router (sim == robot).

This mirrors the GOLDEN `corridor_center.py` control law EXACTLY (same constants, same
_act logic) so what we validate in the sim is what runs on the robot. `corridor_center.py`
stays untouched as the proven baseline; the route-memory ROBOT node (to be written) will
import THIS module, as does the sim.

What's added on top of the golden eyes: a RouteRouter that, at an AMBIGUOUS junction
(both sides genuinely open), injects the TAUGHT turn from a route memory and counts
junctions — instead of the eyes' "guess the more-open side". Simple corners (one side open)
and cavities (front never blocks) are left to the eyes. Safety overrides are never
countermanded (the router only sets the turn DIRECTION the corner SM uses).

Pure python (numpy). No ROS, no rendering here.
"""
import math
import numpy as np

DEG = math.pi / 180.0


def _clamp(x, lim):
    return max(-lim, min(lim, x))


class EyesParams:
    """Defaults identical to corridor_center.CorridorCenter.__init__."""
    def __init__(self, **kw):
        self.cruise = 0.10
        self.max_w = 0.35
        self.kp = 0.6
        self.kd = 0.8
        self.target = 0.82
        self.band = 0.10
        self.head_band = 0.10
        self.hard_min = 0.35
        self.crit = 0.28
        self.crawl = 0.05
        self.front_turn = 1.0
        self.front_clear = 1.6
        self.turn_w = 0.30
        self.phi_lp = 0.5
        self.phi_cap = 0.35
        self.beam_half = 6.0          # deg
        self.front_half = 12.0        # deg
        # --- route-memory router (NEW) ---
        self.j_open = 2.2             # m: a side counts as "open" for junction test
        self.j_front = 1.3            # m: front must be this close to consider a turn-decision
        self.j_sustain = 3            # consecutive scans of both-open before declaring a junction
        self.j_refractory = 6.0       # m of travel before another junction can be counted
        for k, v in kw.items():
            setattr(self, k, v)


# --------------------------------------------------------------- perception (eyes)
def beams_from_scan(ang, rng, p):
    """Compute the eyes-only features from a polar scan (already range/z filtered).

    EXACTLY mirrors corridor_center._on_cloud: two beams per side (abeam +-90, fwd +-45),
    wall angle alpha + perp distance D, heading phi, front-cone clearance, open-side ranges.
    Returns (D_left, D_right, phi, front, openL, openR).
    """
    bh = p.beam_half * DEG
    fh = p.front_half * DEG

    def beam(a0):
        s = (ang >= a0 - bh) & (ang <= a0 + bh)
        return float(np.median(rng[s])) if s.any() else None

    c45, s45 = math.cos(45 * DEG), math.sin(45 * DEG)
    bL, aL = beam(90 * DEG), beam(45 * DEG)
    bR, aR = beam(-90 * DEG), beam(-45 * DEG)

    D_left = alphaL = None
    if bL is not None and aL is not None:
        alphaL = math.atan2(aL * c45 - bL, aL * s45)
        D_left = bL * math.cos(alphaL)
    D_right = alphaR = None
    if bR is not None and aR is not None:
        alphaR = math.atan2(aR * c45 - bR, aR * s45)
        D_right = bR * math.cos(alphaR)

    if alphaL is not None and alphaR is not None:
        phi = (alphaL - alphaR) / 2.0
    elif alphaL is not None:
        phi = alphaL
    elif alphaR is not None:
        phi = -alphaR
    else:
        phi = 0.0

    fsel = (ang >= -fh) & (ang <= fh)
    front = float(np.percentile(rng[fsel], 15)) if int(fsel.sum()) >= 5 else 99.0

    lsel = (ang >= 35 * DEG) & (ang <= 110 * DEG)
    rsel = (ang >= -110 * DEG) & (ang <= -35 * DEG)
    openL = float(np.percentile(rng[lsel], 85)) if lsel.any() else 0.0
    openR = float(np.percentile(rng[rsel], 85)) if rsel.any() else 0.0
    return D_left, D_right, phi, front, openL, openR


# --------------------------------------------------------------- control + router
class EyesController:
    """Golden eyes-only control law + route-memory router.

    route: list of +1 (LEFT/CCW) / -1 (RIGHT/CW) — the taught turn at each ambiguous
    junction, in order. If the route is exhausted or None, falls back to the eyes' guess
    (more-open side) so behaviour degrades gracefully to the proven baseline.
    """
    def __init__(self, p=None, route=None):
        self.p = p or EyesParams()
        self.route = list(route) if route else []
        self._phi_f = None
        self._turning = False
        self._turn_dir = 0
        # router state
        self.jcount = 0
        self._both_open_run = 0
        self._last_j_s = -1e9
        self.events = []                 # (s, jcount, turn_dir, 'route'|'guess')

    def step(self, D_left, D_right, phi_raw, front, openL, openR, s_travel):
        p = self.p
        self._phi_f = phi_raw if self._phi_f is None else (
            p.phi_lp * phi_raw + (1 - p.phi_lp) * self._phi_f)
        phi = max(-p.phi_cap, min(p.phi_cap, self._phi_f))

        present = [d for d in (D_left, D_right) if d is not None]
        nearest = min(present) if present else None

        # ---- both-sides-open detector (router): sustained over N scans ----
        both_open = (openL > p.j_open) and (openR > p.j_open) and (front < p.j_front)
        self._both_open_run = self._both_open_run + 1 if both_open else 0

        # ---- reactive corner state machine (hysteresis on front), golden ----
        if self._turning and front > p.front_clear:
            self._turning = False
        if (not self._turning) and front < p.front_turn:
            self._turning = True
            # ROUTER: ambiguous junction (both sides open, sustained, past refractory)?
            is_junction = (self._both_open_run >= p.j_sustain
                           and (s_travel - self._last_j_s) > p.j_refractory)
            if is_junction:
                self.jcount += 1
                self._last_j_s = s_travel
                if self.jcount - 1 < len(self.route):
                    self._turn_dir = self.route[self.jcount - 1]
                    src = "route"
                else:
                    self._turn_dir = +1 if openL >= openR else -1
                    src = "guess(route-exhausted)"
                self.events.append((s_travel, self.jcount, self._turn_dir, src))
            else:
                # simple corner -> eyes pick the only open side
                self._turn_dir = +1 if openL >= openR else -1

        if self._turning:
            w = self._turn_dir * p.turn_w
            v = p.crawl if front > 0.45 else 0.0
            state = "TURN_LEFT" if self._turn_dir > 0 else "TURN_RIGHT"
        elif nearest is None:
            v, w, state = 0.0, 0.0, "NO_WALLS_STOP"
        else:
            if D_left is not None and D_right is not None:
                e, base = D_left - D_right, "CENTER"
            elif D_left is not None:
                e, base = D_left - p.target, "LEFT_ONLY"
            else:
                e, base = p.target - D_right, "RIGHT_ONLY"

            if abs(e) < p.band and abs(phi) < p.head_band:
                w, state = 0.0, "STRAIGHT"
            else:
                w, state = _clamp(p.kp * e + p.kd * phi, p.max_w), base

            v = p.cruise if abs(w) < 0.05 else p.cruise * 0.6
            if nearest < p.hard_min:
                v, state = p.crawl, "CRAWL_" + state
            if nearest < p.crit:                      # HARD safety override (never countermanded)
                if D_left is not None and (D_right is None or D_left <= D_right):
                    w = -p.max_w
                else:
                    w = p.max_w
                v, state = p.crawl, "CRIT_AVOID"

        if front < 0.30:
            v = 0.0
            if not self._turning:
                state = "FRONT_EMERG"
        return v, w, state
