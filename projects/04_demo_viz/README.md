# 04_demo_viz — premium corridor2.0 demo renders

Cinematic, pre-rendered demo videos of the `corridor2.0` LiDAR map (project 02).
Built **on the server** — this is a headless compute node with no OpenGL/display
(`/dev/dri` is locked), so every GL renderer (Open3D, PyVista, Blender-EGL) segfaults.
Instead we rasterize on the GPU **via CUDA/PyTorch**: `render/gpu_raster.py` does
perspective projection → per-pixel z-buffer → eye-dome lighting → bloom → SSAA.

## Run (use the torch env; ffmpeg comes from env_cu121)
```bash
PY=/home/divake/miniconda3/envs/env_py311/bin/python
$PY render/gpu_still.py                      # 3 hero stills, all palettes
$PY render/render_orbit.py 6                  # orbit beauty, all palettes
$PY render/render_reveal.py 6                 # "building draws itself", all palettes
# hero film (reads the bag once for the live laser sweep):
/home/divake/miniconda3/envs/lidar_viz/bin/python render/extract_live.py   # -> out/live_scans.npz
$PY render/render_hero.py intensity 85        # full hero film  (add --preview for a fast check)
```

## Palettes
`turbo` (height) · `intensity` (reflectivity, amber) · `ice` (white/cyan).

## Data sources
- `projects/03_autonomy/results/video/cache.npz` — 1.04M-voxel accumulated map + per-voxel
  intensity + first-seen scan index + 7,619 trajectory poses (built on the NUC).
- `out/live_scans.npz` — per-scan live LiDAR points (the bright sweep), extracted from the
  `corridor2.0` mcap here on the server.

## Outputs (`out/`, gitignored)
`hero_intensity.mp4` (the film), `orbit_*.mp4`, `reveal_*.mp4`, `still_*.png`, `CONTACT_SHEET.png`.
Pull them to your laptop with `pull_to_laptop.sh` (run it on the laptop; fill in the two blanks).
