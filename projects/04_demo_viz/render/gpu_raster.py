#!/usr/bin/env python3
"""CUDA point-cloud rasterizer (PyTorch) -- premium look without OpenGL.

This box is a compute node: CUDA works, but /dev/dri is locked so every GL/EGL
renderer (Open3D, PyVista, Blender-EGL) segfaults. So we rasterize ourselves on
the GPU with torch: perspective projection -> per-pixel z-buffer (scatter-amin) ->
eye-dome lighting (screen-space depth edges) -> bloom -> supersampled downscale.

Run with the torch env:  /home/divake/miniconda3/envs/env_py311/bin/python

  R = Renderer(W=1920, H=1080, ssaa=2)
  R.set_cloud(xyz_np, rgb_np)                  # rgb in 0..1
  frame = R.render(eye, target, up=(0,0,1), fov_deg=45, bg=(0.03,0.035,0.055),
                   point_px=2.2, edl=0.9, bloom=0.85)   # -> (H,W,3) uint8
"""
import numpy as np, torch
import torch.nn.functional as F

DEV = "cuda" if torch.cuda.is_available() else "cpu"


def _norm(v):
    return v / (torch.linalg.norm(v) + 1e-9)


def _gauss1d(sigma, dev):
    r = max(1, int(round(3 * sigma)))
    x = torch.arange(-r, r + 1, device=dev, dtype=torch.float32)
    k = torch.exp(-(x ** 2) / (2 * sigma ** 2)); return (k / k.sum())


class Renderer:
    def __init__(self, W=1920, H=1080, ssaa=2, near=0.2):
        self.W, self.H, self.ss, self.near = W, H, ssaa, near
        self.Wb, self.Hb = W * ssaa, H * ssaa
        self.xyz = None; self.rgb = None
        # disc splat offsets in supersampled px (radius ~ss) with weights
        offs = []
        rad = ssaa
        for ox in range(-rad, rad + 1):
            for oy in range(-rad, rad + 1):
                d2 = ox * ox + oy * oy
                if d2 <= rad * rad + 0.5:
                    offs.append((ox, oy, float(np.exp(-d2 / (2 * (0.7 * rad + 0.5) ** 2)))))
        self.offs = offs

    def set_cloud(self, xyz_np, rgb_np):
        self.xyz = torch.as_tensor(np.ascontiguousarray(xyz_np), dtype=torch.float32, device=DEV)
        self.rgb = torch.as_tensor(np.ascontiguousarray(rgb_np), dtype=torch.float32, device=DEV).clamp(0, 1)

    def add_cloud(self, xyz_np, rgb_np):  # convenience: a second emissive cloud (trail/scan)
        x = torch.as_tensor(np.ascontiguousarray(xyz_np), dtype=torch.float32, device=DEV)
        c = torch.as_tensor(np.ascontiguousarray(rgb_np), dtype=torch.float32, device=DEV).clamp(0, 1)
        self.xyz = torch.cat([self.xyz, x]); self.rgb = torch.cat([self.rgb, c])

    @torch.no_grad()
    def render(self, eye, target, up=(0, 0, 1), fov_deg=45.0, bg=(0.03, 0.035, 0.055),
               point_px=2.2, edl=0.9, bloom=0.85, glow=0.6, gamma=1.0):
        Wb, Hb, dev = self.Wb, self.Hb, DEV
        eye = torch.tensor(eye, dtype=torch.float32, device=dev)
        tgt = torch.tensor(target, dtype=torch.float32, device=dev)
        up = torch.tensor(up, dtype=torch.float32, device=dev)
        f = _norm(tgt - eye); r = _norm(torch.linalg.cross(f, up)); u = torch.linalg.cross(r, f)

        rel = self.xyz - eye
        xc = rel @ r; yc = rel @ u; zc = rel @ f
        front = zc > self.near
        focal = (Wb / 2) / np.tan(np.deg2rad(fov_deg) / 2)
        sx = Wb / 2 + focal * xc / zc
        sy = Hb / 2 - focal * yc / zc

        npix = Wb * Hb
        zbuf = torch.full((npix,), float("inf"), device=dev)
        # z-buffer (nearest) over all disc samples
        flats, zcs, cols = [], [], []
        for ox, oy, w in self.offs:
            px = (sx + ox).round().long(); py = (sy + oy).round().long()
            ok = front & (px >= 0) & (px < Wb) & (py >= 0) & (py < Hb)
            flats.append((py * Wb + px)[ok]); zcs.append(zc[ok]); cols.append(self.rgb[ok])
        flat = torch.cat(flats); zca = torch.cat(zcs); cola = torch.cat(cols)
        zbuf.scatter_reduce_(0, flat, zca, "amin", include_self=True)
        # nearest-surface color (within tolerance of zbuf); nearest sample wins
        order = torch.argsort(zca, descending=True)          # far->near so near overwrites
        flat_s, col_s = flat[order], cola[order]
        colbuf = torch.zeros((npix, 3), device=dev)
        colbuf[flat_s] = col_s

        depth = zbuf.view(Hb, Wb)
        col = colbuf.view(Hb, Wb, 3)
        filled = torch.isfinite(depth)

        # ---- eye-dome lighting: darken pixels that sit behind their neighbours ----
        if edl > 0:
            ld = torch.where(filled, torch.log(depth.clamp(min=1e-3)), torch.zeros_like(depth))
            resp = torch.zeros_like(depth)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-2, 0), (2, 0), (0, -2), (0, 2)]:
                nb = torch.roll(ld, shifts=(dy, dx), dims=(0, 1))
                nbf = torch.roll(filled, shifts=(dy, dx), dims=(0, 1))
                resp += torch.where(filled & nbf, (ld - nb).clamp(min=0), torch.zeros_like(depth))
            shade = torch.exp(-edl * 40.0 * resp).clamp(0.25, 1.0)
            col = col * shade[..., None]

        # composite onto background
        bgt = torch.tensor(bg, dtype=torch.float32, device=dev)
        img = torch.where(filled[..., None], col, bgt.view(1, 1, 3).expand(Hb, Wb, 3))

        # ---- downsample (area average = AA) ----
        img = img.permute(2, 0, 1).unsqueeze(0)
        img = F.avg_pool2d(img, self.ss).squeeze(0).permute(1, 2, 0)  # (H,W,3)

        # ---- bloom (bright-pass gaussian, screen blend) ----
        if bloom > 0:
            lum = img.amax(-1)
            bright = img * (lum > 0.45)[..., None].float()
            k = _gauss1d(self.W / 240.0, dev)
            b = bright.permute(2, 0, 1).unsqueeze(0)
            b = F.conv2d(b, k.view(1, 1, 1, -1).expand(3, 1, 1, -1), padding=(0, len(k) // 2), groups=3)
            b = F.conv2d(b, k.view(1, 1, -1, 1).expand(3, 1, -1, 1), padding=(len(k) // 2, 0), groups=3)
            b = b.squeeze(0).permute(1, 2, 0) * bloom
            img = 1.0 - (1.0 - img.clamp(0, 1)) * (1.0 - b.clamp(0, 1))

        if gamma != 1.0:
            img = img.clamp(0, 1) ** (1.0 / gamma)
        out = (img.clamp(0, 1) * 255).to(torch.uint8).cpu().numpy()
        return out
