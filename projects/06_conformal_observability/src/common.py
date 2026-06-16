"""Shared paths, constants, and helpers for project 06 (Conformalized Observability).

Data source: corridor2.0 ONLY (project 02, FAST-LIO2). See ../PLAN.md.
Env: env_py311 (torch 2.12+cu130, 2x RTX 6000 Ada, sklearn, scipy, mcap).
"""
from __future__ import annotations
import os
# Pin BLAS to 1 thread/process BEFORE numpy import — we parallelize with multiprocessing,
# so per-process BLAS threading would oversubscribe (24 procs x 16 threads = load 700+).
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
import numpy as np

# ---------------------------------------------------------------- paths
ROOT = "/ssd_4TB/divake/ml_lidar"
PROJ = os.path.join(ROOT, "projects/06_conformal_observability")
CACHE = os.path.join(PROJ, "cache")
RESULTS = os.path.join(PROJ, "results")
FIGS = os.path.join(PROJ, "figs")

BAG = os.path.join(ROOT, "projects/02_fast_lio2/results/corridor2.0/corridor2.0_0.mcap")
NPZ_LOOPCLOSED = os.path.join(ROOT, "projects/03_autonomy/results/corridor2_loopclosed_fastlio.npz")
VOX_MAP_CACHE = os.path.join(ROOT, "projects/03_autonomy/results/video/cache.npz")  # 1.04M voxel map + traj

for _d in (CACHE, RESULTS, FIGS):
    os.makedirs(_d, exist_ok=True)

# ---------------------------------------------------------------- sensor
# RoboSense Helios 16P, organized 1800x16, XYZIRT, point_step = 26 bytes.
PT_XYZIRT = np.dtype({
    "names":   ["x", "y", "z", "intensity", "ring", "timestamp"],
    "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f8"],
    "offsets": [0, 4, 8, 12, 16, 18],
    "itemsize": 26,
})
N_RINGS = 16
TOPIC_LIDAR = "/rslidar_points"
TOPIC_ODOM = "/lio/odometry"
TOPIC_IMU = "/camera/camera/imu"
N_SCANS_EXPECTED = 7619

SEED = 0

# ---------------------------------------------------------------- helpers
def set_seed(seed: int = SEED):
    import random
    random.seed(seed)
    np.random.seed(seed)
    try:
        import torch
        torch.manual_seed(seed)
        torch.cuda.manual_seed_all(seed)
    except Exception:
        pass


def device():
    import torch
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


def parse_pointcloud2(raw_bytes) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (xyz (N,3) f32, intensity (N,) f32, ring (N,) u8) for VALID points only.

    Masks NaN and zero-range padding (organized cloud is is_dense=false).
    """
    a = np.frombuffer(bytes(raw_bytes), dtype=PT_XYZIRT)
    xyz = np.stack([a["x"], a["y"], a["z"]], axis=1).astype(np.float32)
    valid = np.isfinite(xyz).all(1) & (np.abs(xyz).sum(1) > 0)
    return xyz[valid], a["intensity"][valid].astype(np.float32), a["ring"][valid].astype(np.uint8)


def voxel_downsample(xyz: np.ndarray, voxel: float = 0.10) -> np.ndarray:
    """One representative point per occupied voxel (first-in-voxel). Preserves wall
    geometry while shrinking ~28k -> ~few-k points for fast normal estimation."""
    if len(xyz) == 0:
        return xyz
    keys = np.floor(xyz / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return xyz[idx]


def quat_to_yaw(qx, qy, qz, qw) -> np.ndarray:
    """Yaw (rad) about +z from a quaternion (REP-103)."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(siny, cosy)


def save_npz(path: str, **arrays):
    np.savez_compressed(path, **arrays)
    mb = os.path.getsize(path) / 1e6
    print(f"  wrote {os.path.relpath(path, PROJ)}  ({mb:.1f} MB)")
