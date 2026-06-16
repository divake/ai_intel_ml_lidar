#!/usr/bin/env python3
"""Shared helpers for the corridor2.0 premium demo renders.

Palettes, bloom post-processing, ffmpeg piping, cache loading. Kept dependency-light
(numpy + PIL + matplotlib colormaps); Open3D/PyVista are imported only by the renderers.

Run everything with the lidar_viz env python:
  /home/divake/miniconda3/envs/lidar_viz/bin/python ...
"""
import os, subprocess, numpy as np
from matplotlib import cm
from matplotlib.colors import LinearSegmentedColormap
from PIL import Image, ImageFilter

REPO = "/ssd_4TB/divake/ml_lidar"
CACHE = os.path.join(REPO, "projects/03_autonomy/results/video/cache.npz")
OUTDIR = os.path.join(REPO, "projects/04_demo_viz/out")
CACHEDIR = os.path.join(REPO, "projects/04_demo_viz/cache")
FFMPEG = "/home/divake/miniconda3/envs/env_cu121/bin/ffmpeg"  # the one ffmpeg present on this box

# ---- background colours per palette (near-black, tinted) ----
BG = {
    "turbo":     np.array([8, 9, 14],  np.float32) / 255.0,
    "intensity": np.array([10, 7, 5],  np.float32) / 255.0,
    "ice":       np.array([5, 7, 12],  np.float32) / 255.0,
    "rviz":      np.array([0, 0, 0],   np.float32) / 255.0,   # LIO-SAM/RViz: pure black
}

# custom ice ramp: deep blue -> cyan -> white
_ICE = LinearSegmentedColormap.from_list("ice", [
    (0.00, (0.04, 0.07, 0.16)),
    (0.35, (0.05, 0.35, 0.62)),
    (0.65, (0.25, 0.75, 0.92)),
    (1.00, (0.95, 0.99, 1.00)),
])


def _norm(v, lo_pct=2, hi_pct=98, lo=None, hi=None):
    lo = np.percentile(v, lo_pct) if lo is None else lo
    hi = np.percentile(v, hi_pct) if hi is None else hi
    return np.clip((v - lo) / max(hi - lo, 1e-6), 0, 1)


def colorize(xyz, inten, palette, zlo=None, zhi=None):
    """(N,3) rgb in 0..1 for the chosen palette. Consistent across frames if zlo/zhi fixed."""
    z = xyz[:, 2]
    if palette == "turbo":
        n = _norm(z, lo=zlo, hi=zhi)
        return cm.turbo(n)[:, :3].astype(np.float32)
    if palette == "intensity":
        # intensity is heavily skewed (median ~27, max 255) -> sqrt compress then inferno
        n = _norm(np.sqrt(np.clip(inten, 0, None)), lo_pct=3, hi_pct=99)
        return cm.inferno(n)[:, :3].astype(np.float32)
    if palette == "ice":
        n = _norm(z, lo=zlo, hi=zhi)
        return _ICE(n)[:, :3].astype(np.float32)
    if palette == "rviz":
        # match RViz "Intensity" rainbow: blue=low, green=mid, red=high (jet).
        # sqrt-compress the skewed intensity so the bulk lands green, retro-reflectors red.
        n = _norm(np.sqrt(np.clip(inten, 0, None)), lo_pct=2, hi_pct=86)
        return cm.jet(n)[:, :3].astype(np.float32)
    raise ValueError(palette)


def load_cache():
    d = np.load(CACHE)
    return dict(vox=d["vox"].astype(np.float32), inten=d["inten"].astype(np.float32),
                first=d["first"].astype(np.int32), traj=d["traj"].astype(np.float32),
                nscans=int(d["nscans"]))


def bloom(rgb_u8, thresh=90, radius=6, strength=0.85):
    """Bright-pass gaussian bloom, screen-blended. rgb_u8: (H,W,3) uint8 -> uint8."""
    lum = rgb_u8.max(2)
    bright = (rgb_u8 * (lum[..., None] > thresh)).astype(np.uint8)
    bl = np.asarray(Image.fromarray(bright).filter(ImageFilter.GaussianBlur(radius)), np.float32) * strength
    a = rgb_u8.astype(np.float32)
    screen = 255.0 - (255.0 - a) * (255.0 - bl) / 255.0
    return np.clip(screen, 0, 255).astype(np.uint8)


def vignette(rgb_u8, amount=0.35):
    H, W = rgb_u8.shape[:2]
    yy, xx = np.mgrid[0:H, 0:W]
    cx, cy = W / 2, H / 2
    r = np.sqrt(((xx - cx) / cx) ** 2 + ((yy - cy) / cy) ** 2)
    mask = np.clip(1.0 - amount * np.clip(r - 0.5, 0, None) ** 1.5, 0, 1)[..., None]
    return (rgb_u8.astype(np.float32) * mask).astype(np.uint8)


class FFWriter:
    """Pipe raw rgb24 frames to ffmpeg -> SMALL, PPT-friendly mp4.

    Point-cloud footage is high-entropy, so we render at full res (SSAA quality)
    but ENCODE downscaled (scale_h) at a web CRF. Keeps visual quality high while
    keeping files a few MB (an 85 s 720p/crf32 clip is ~10 MB, not 350 MB).
    Pass scale_h=None to keep full resolution for a one-off high-res master.
    """
    def __init__(self, path, w, h, fps=30, crf=32, scale_h=720):
        ff = FFMPEG if os.path.exists(FFMPEG) else _imageio_ffmpeg()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        vf = ["-vf", f"scale=-2:{scale_h}"] if (scale_h and scale_h < h) else []
        cmd = [ff, "-y", "-hide_banner", "-loglevel", "error",
               "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", f"{w}x{h}",
               "-framerate", str(fps), "-i", "-", *vf,
               "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", str(crf),
               "-preset", "slow", "-movflags", "+faststart", "-an", path]
        self.p = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        self.path = path

    def write(self, rgb_u8):
        self.p.stdin.write(np.ascontiguousarray(rgb_u8).tobytes())

    def close(self):
        self.p.stdin.close(); self.p.wait()


def _imageio_ffmpeg():
    import imageio_ffmpeg
    return imageio_ffmpeg.get_ffmpeg_exe()


def ease(t):
    """smoothstep ease-in-out on t in [0,1]."""
    t = np.clip(t, 0, 1)
    return t * t * (3 - 2 * t)
