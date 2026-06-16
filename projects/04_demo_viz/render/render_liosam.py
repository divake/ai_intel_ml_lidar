"""LIO-SAM-style demo: a faithful re-creation of the look in their kitti-demo.gif,
applied to corridor2.0. Matches their RViz config — Intensity rainbow (blue=low,
green=mid, red=high), black bg, ~2px points, accumulating map, and an Orbit-locked
chase camera behind the robot. The live spinning-LiDAR scan sweeps out each frame.
RAW look: no floor grid, no robot marker, no bloom (RViz has none). CUDA rasterizer.

  env_py311 python render_liosam.py [secs] [--preview]
"""
import os, sys, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import common as C
from gpu_raster import Renderer

SECS = float([a for a in sys.argv[1:] if a.replace(".", "").isdigit()][0]) if any(
    a.replace(".", "").isdigit() for a in sys.argv[1:]) else 40.0
PREVIEW = "--preview" in sys.argv
W, H, SS, FPS = (1024, 576, 1, 30) if PREVIEW else (1920, 1080, 2, 30)
STRIDE = 5 if PREVIEW else 1
PAL = "rviz"

d = C.load_cache()
vox, inten, first, nscans = d["vox"], d["inten"], d["first"], d["nscans"]
zlo, zhi = np.percentile(vox[:, 2], 2), np.percentile(vox[:, 2], 98)
o = np.argsort(first, kind="stable")
vox, inten, first = vox[o], inten[o], first[o]
rgb_map = (C.colorize(vox, inten, PAL, zlo=zlo, zhi=zhi) * 0.70).astype(np.float32)  # accumulating map (alpha-ish)

L = np.load(os.path.join(C.CACHEDIR, "live_scans.npz"))
lpts, lint, loffs = L["pts"].astype(np.float32), L["inten"].astype(np.float32), L["offs"]
lpos, lfwd = L["pos"].astype(np.float32), L["fwd"].astype(np.float32)
nL = int(L["nscans"])

# smoothed chase camera locked behind the robot (RViz Orbit / target_frame=base_link)
def smooth(a, k):
    pad = k // 2
    ap = np.pad(a, ((pad, pad), (0, 0)), mode="edge")
    ker = np.ones(k) / k
    return np.stack([np.convolve(ap[:, i], ker, mode="valid")[:len(a)] for i in range(a.shape[1])], 1)

spos = smooth(lpos, 25)
vel = np.gradient(spos, axis=0); vel[:, 2] = 0
hn = np.linalg.norm(vel, axis=1, keepdims=True)
head = np.where(hn > 1e-3, vel / np.clip(hn, 1e-6, None), lfwd); head[:, 2] = 0
head = head / np.clip(np.linalg.norm(head, axis=1, keepdims=True), 1e-6, None)
BACK, HEIGHT, LOOK = 7.0, 15.0, 4.0                 # pitch ~ 56 deg behind-above
eye_f = spos - head * BACK; eye_f[:, 2] = spos[:, 2] + HEIGHT
tgt_f = spos + head * LOOK
eye_f, tgt_f = smooth(eye_f, 20), smooth(tgt_f, 20)

def live_union(s0, s1):
    s0 = max(s0, 0); s1 = min(max(s1, s0 + 1), nL)
    return lpts[loffs[s0]:loffs[s1]], lint[loffs[s0]:loffs[s1]]

out = os.path.join(C.OUTDIR, f"liosam_corridor{'_preview' if PREVIEW else ''}.mp4")
vw = C.FFWriter(out, W, H, FPS)
R = Renderer(W, H, ssaa=SS)
F = int(SECS * FPS); prev_s = 0
for fr in range(0, F, STRIDE):
    t = fr / max(F - 1, 1)
    s = int(round(t * (nscans - 1))); si = min(s, nL - 1)
    k = max(int(np.searchsorted(first, s, side="right")), 50)
    lp, li = live_union(prev_s, s + 1); prev_s = s
    lrgb = C.colorize(lp, li, PAL, zlo=zlo, zhi=zhi).astype(np.float32) if len(lp) else np.zeros((0, 3), np.float32)
    xyz = np.vstack([vox[:k], lp])
    col = np.vstack([rgb_map[:k], lrgb])     # live scan at full brightness over the dimmed map
    R.set_cloud(xyz, col)
    img = R.render(eye_f[si], tgt_f[si], up=(0, 0, 1), fov_deg=50, bg=tuple(C.BG[PAL]),
                   point_px=1.6, edl=0.25, bloom=0.0)   # flat RViz look: minimal EDL, no bloom
    vw.write(img)
    if fr % 300 == 0:
        print(f"  frame {fr}/{F}", flush=True)
vw.close()
print(f"DONE -> {out}")
