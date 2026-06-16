#!/usr/bin/env python3
"""HERO FILM - the corridor2.0 mission. The robot drives its real route; the map
accumulates live (dim, EDL-shaded) while a bright laser sweep flares around a
glowing robot marker over a subtle floor grid; at the end the camera pulls back
and orbits the finished figure-8. CUDA rasterizer. ~85 s premium timelapse.

  env_py311 python render_hero.py [palette] [secs] [--preview]
"""
import os, sys, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import common as C
from gpu_raster import Renderer

PAL = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else "intensity"
SECS = float(sys.argv[2]) if len(sys.argv) > 2 and not sys.argv[2].startswith("-") else 85.0
PREVIEW = "--preview" in sys.argv
W, H, SS, FPS = (1024, 576, 1, 30) if PREVIEW else (1920, 1080, 2, 30)
STRIDE = 5 if PREVIEW else 1
DRIVE_FRAC = 0.84

# ---- data ----
d = C.load_cache()
vox, inten, first, traj, nscans = d["vox"], d["inten"], d["first"], d["traj"], d["nscans"]
zlo, zhi = np.percentile(vox[:, 2], 2), np.percentile(vox[:, 2], 98)
o = np.argsort(first, kind="stable")
vox, inten, first = vox[o], inten[o], first[o]
rgb_map = (C.colorize(vox, inten, PAL, zlo=zlo, zhi=zhi) * 0.55).astype(np.float32)

L = np.load(os.path.join(C.CACHEDIR, "live_scans.npz"))
lpts, lint, loffs = L["pts"].astype(np.float32), L["inten"].astype(np.float32), L["offs"]
lpos, lfwd = L["pos"].astype(np.float32), L["fwd"].astype(np.float32)
nL = int(L["nscans"])

ctr = vox.mean(0)
ext = np.percentile(np.linalg.norm(vox[:, :2] - ctr[:2], axis=1), 99.5)
floor_z = float(np.percentile(vox[:, 2], 8))

# ---- subtle floor grid (Foxglove look) ----
def ground_grid(step=2.0, dens=0.10):
    xmin, xmax = vox[:, 0].min() - 2, vox[:, 0].max() + 2
    ymin, ymax = vox[:, 1].min() - 2, vox[:, 1].max() + 2
    out = []
    yy = np.arange(ymin, ymax, dens)
    for x in np.arange(xmin, xmax, step):
        out.append(np.stack([np.full_like(yy, x), yy, np.full_like(yy, floor_z)], 1))
    xx = np.arange(xmin, xmax, dens)
    for y in np.arange(ymin, ymax, step):
        out.append(np.stack([xx, np.full_like(xx, y), np.full_like(xx, floor_z)], 1))
    return np.concatenate(out).astype(np.float32)

grid_xyz = ground_grid()
grid_rgb = np.tile(np.array([0.10, 0.13, 0.18], np.float32), (len(grid_xyz), 1))

# ---- smoothed heading + camera path ----
def smooth(a, k):
    ker = np.ones(k) / k
    pad = k // 2
    ap = np.pad(a, ((pad, pad), (0, 0)), mode="edge")
    return np.stack([np.convolve(ap[:, i], ker, mode="valid")[:len(a)] for i in range(a.shape[1])], 1)

spos = smooth(lpos, 25)
vel = np.gradient(spos, axis=0); vel[:, 2] = 0
hn = np.linalg.norm(vel, axis=1, keepdims=True)
head = np.where(hn > 1e-3, vel / np.clip(hn, 1e-6, None), lfwd)
head[:, 2] = 0
head = head / np.clip(np.linalg.norm(head, axis=1, keepdims=True), 1e-6, None)

# high tilted drone-follow, precomputed per scan then smoothed (no jitter, never buries)
BACK, HEIGHT, LOOK = 7.0, 15.0, 6.0
eye_f = spos - head * BACK + np.array([0, 0, HEIGHT]); eye_f[:, 2] = spos[:, 2] + HEIGHT
tgt_f = spos + head * LOOK
eye_f = smooth(eye_f, 25); tgt_f = smooth(tgt_f, 25)

def hero_eye(th):
    elev = np.deg2rad(38.0); dist = 1.85 * ext
    return ctr + np.array([dist * np.cos(elev) * np.cos(th),
                           dist * np.cos(elev) * np.sin(th),
                           dist * np.sin(elev)], np.float32)

def live_union(s0, s1):
    s0 = max(s0, 0); s1 = min(max(s1, s0 + 1), nL)
    return lpts[loffs[s0]:loffs[s1]], lint[loffs[s0]:loffs[s1]]

def marker(p, h):
    rng = np.random.default_rng(0)
    # glowing core sphere
    v = rng.normal(size=(360, 3)); v /= np.linalg.norm(v, axis=1, keepdims=True)
    core = p + v * (0.42 * np.cbrt(rng.uniform(0, 1, 360))[:, None])
    # halo ring (horizontal)
    a = np.linspace(0, 2 * np.pi, 160)
    ring = p + np.stack([0.95 * np.cos(a), 0.95 * np.sin(a), np.zeros_like(a)], 1)
    # forward arrow
    rt = np.array([-h[1], h[0], 0.0])
    t = np.linspace(0, 1.4, 60)[:, None]
    arrow = p + t * h[None]
    head_tip = p + 1.4 * h
    bb = np.linspace(0, 1, 24)[:, None]
    barb1 = head_tip + bb * (-0.5 * h + 0.4 * rt)[None]
    barb2 = head_tip + bb * (-0.5 * h - 0.4 * rt)[None]
    return np.vstack([core, ring, arrow, barb1, barb2]).astype(np.float32)

# ---- render ----
out = os.path.join(C.OUTDIR, f"hero_{PAL}{'_preview' if PREVIEW else ''}.mp4")
vw = C.FFWriter(out, W, H, FPS)   # small PPT-friendly default (720p web crf)
R = Renderer(W, H, ssaa=SS)
mcol = np.array([1.0, 0.96, 0.88], np.float32)

F = int(SECS * FPS); drive_F = int(F * DRIVE_FRAC)
th0 = np.arctan2(eye_f[-1][1] - ctr[1], eye_f[-1][0] - ctr[0])
last = (eye_f[0].copy(), tgt_f[0].copy())
prev_s = 0
for fr in range(0, F, STRIDE):
    if fr < drive_F:
        t = fr / max(drive_F - 1, 1)
        s = int(round(t * (nscans - 1))); si = min(s, nL - 1)
        k = max(int(np.searchsorted(first, s, side="right")), 50)
        lp, li = live_union(prev_s, s + 1); prev_s = s
        lrgb = (np.clip(C.colorize(lp, li, PAL, zlo=zlo, zhi=zhi) * 1.5 + 0.45, 0, 1).astype(np.float32)
                if len(lp) else np.zeros((0, 3), np.float32))
        mk = marker(spos[si], head[si])
        xyz = np.vstack([grid_xyz, vox[:k], lp, mk])
        col = np.vstack([grid_rgb, rgb_map[:k], lrgb, np.tile(mcol, (len(mk), 1))])
        eye, tgt = eye_f[si], tgt_f[si]; last = (eye.copy(), tgt.copy())
    else:
        u = (fr - drive_F) / max(F - drive_F - 1, 1)
        e = C.ease(min(u / 0.32, 1.0))
        th = th0 + (0.15 + 0.85 * u) * np.pi
        oe, ot = hero_eye(th), ctr.astype(np.float32)
        eye = (1 - e) * last[0] + e * oe
        tgt = (1 - e) * last[1] + e * ot
        mk = marker(spos[nL - 1], head[nL - 1])
        xyz = np.vstack([grid_xyz, vox, mk])
        col = np.vstack([grid_rgb, rgb_map, np.tile(mcol, (len(mk), 1))])
    R.set_cloud(xyz, col)
    img = R.render(eye, tgt, up=(0, 0, 1), fov_deg=52, bg=tuple(C.BG[PAL]),
                   point_px=2.3, edl=0.8, bloom=0.95)
    vw.write(img)
    if fr % 300 == 0:
        print(f"  frame {fr}/{F}", flush=True)
vw.close()
print(f"DONE -> {out}")
