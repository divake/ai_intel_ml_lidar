"""Shared corridor controller — the SINGLE source of truth used by BOTH the real
robot (path_follow.py) and the simulator (sim/corridor_sim.py).

Pure function, no ROS, no I/O. Given the robot pose, the path, the progress index, and
the nearest left/right wall distances, it returns the drive command. Keeping this shared
means whatever we validate in sim is literally what runs on the robot.

  direction = pure pursuit toward a lookahead point on the path (from the map)
  safety    = repel from a CLOSE side wall (repel-only => cavity-safe, SLAM-independent)
  corners   = ARC (slow down, never stop) so LiDAR-SLAM keeps translation to track
"""
import math


class Cfg:
    """Controller gains/limits. Same object passed in sim and on the robot."""
    def __init__(self, cruise=0.18, max_w=0.45, lookahead=0.8,
                 safe=0.5, k_repel=1.6, hard_min=0.35, min_v=0.06):
        self.cruise = cruise
        self.max_w = max_w
        self.Ld = lookahead
        self.safe = safe
        self.k_repel = k_repel
        self.hard_min = hard_min
        self.min_v = min_v


def _clamp(x, lim):
    return max(-lim, min(lim, x))


def plan(pose, path, idx, d_left, d_right, cfg):
    """pose=(x,y,yaw); path=Nx2 array; idx=current progress index;
    d_left/d_right = nearest wall on each side (m) or None.
    Returns (v, w, new_idx, reached, state, alpha)."""
    rx, ry, ryaw = pose

    # advance progress index along the path (monotonic, local forward window)
    lo = idx
    best, bj = 1e18, idx
    for k in range(lo, min(lo + 80, len(path))):
        dx, dy = path[k][0] - rx, path[k][1] - ry
        d2 = dx * dx + dy * dy
        if d2 < best:
            best, bj = d2, k
    idx = bj

    if idx >= len(path) - 2:
        return 0.0, 0.0, idx, True, "GOAL", 0.0

    # lookahead point ~Ld ahead along the path
    j = idx
    while j < len(path) - 1 and math.hypot(path[j][0] - rx, path[j][1] - ry) < cfg.Ld:
        j += 1
    dx, dy = path[j][0] - rx, path[j][1] - ry

    # heading error to the lookahead, in the robot frame
    tx = math.cos(ryaw) * dx + math.sin(ryaw) * dy
    ty = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
    alpha = math.atan2(ty, tx)

    # direction: pure pursuit
    w_path = 1.1 * alpha

    # safety: repel from a close side wall
    w_repel = 0.0
    if d_right is not None and d_right < cfg.safe:
        w_repel += cfg.k_repel * (cfg.safe - d_right)   # close RIGHT -> steer left (w>0)
    if d_left is not None and d_left < cfg.safe:
        w_repel -= cfg.k_repel * (cfg.safe - d_left)    # close LEFT  -> steer right (w<0)

    w = _clamp(w_path + w_repel, cfg.max_w)

    # speed: arc through turns (slow, never stop); extra slow very near a wall
    v = cfg.cruise * (1.0 - 0.6 * min(1.0, abs(alpha) / 0.9))
    nearest = min([d for d in (d_left, d_right) if d is not None], default=99.0)
    if nearest < cfg.hard_min:
        v = min(v, 0.08)
    v = max(v, cfg.min_v)

    return v, w, idx, False, ("TURN" if abs(alpha) > 0.5 else "FOLLOW"), alpha
