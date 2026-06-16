"""Shared geometry: normals, the point-to-plane constraint matrix (= observability Hessian),
and a point-to-plane ICP. Used by error_target.py (injected-GT recovery) and observability.py.

Constraint matrix:  C = Σ Jᵢ Jᵢᵀ ,  Jᵢ = [ aᵢ × nᵢ ; nᵢ ]  (6-vector)
where aᵢ is a point (in the working frame) and nᵢ its surface normal. eig(C):
 - small λ  → poorly-constrained DoF;  its eigenvector = the blind direction.
 - this is the Zhang-2016 / Gelfand-2003 / X-ICP degeneracy quantity.
"""
from __future__ import annotations
import numpy as np
from scipy.spatial import cKDTree


# ----------------------------------------------------------------- SE3
def pose_to_T(pos, quat) -> np.ndarray:
    """quat = (x,y,z,w). Returns 4x4."""
    x, y, z, w = quat
    R = np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-z*w),     2*(x*z+y*w)],
        [2*(x*y+z*w),     1 - 2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),     2*(y*z+x*w),     1 - 2*(x*x+y*y)],
    ], dtype=np.float64)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    return T


def transform(pts, T):
    return pts @ T[:3, :3].T + T[:3, 3]


def _expSO3(w):
    th = np.linalg.norm(w)
    if th < 1e-9:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th)*K + (1-np.cos(th))*(K @ K)


# ----------------------------------------------------------------- normals
def estimate_normals(pts, k=20, kdtree=None, workers=-1):
    """Per-point unit normal via PCA of the k-NN (smallest-eigenvector). CPU/scipy.
    workers=-1 uses all cores for the query (set workers=1 inside a multiprocessing pool)."""
    n = len(pts)
    if n < k + 1:
        return np.zeros_like(pts)
    tree = kdtree if kdtree is not None else cKDTree(pts)
    _, idx = tree.query(pts, k=k, workers=workers)
    nb = pts[idx]                                  # (n,k,3)
    nb = nb - nb.mean(axis=1, keepdims=True)
    cov = np.einsum('nki,nkj->nij', nb, nb) / k    # (n,3,3)
    w, v = np.linalg.eigh(cov)                      # ascending
    nrm = v[:, :, 0]                               # smallest-eigenvector
    return nrm / (np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12)


# ----------------------------------------------------------------- constraint matrix
def constraint_matrix(a, n):
    """C = Σ [a×n ; n][a×n ; n]ᵀ  (6x6). a,n: (M,3)."""
    cr = np.cross(a, n)
    J = np.concatenate([cr, n], axis=1)            # (M,6)
    return J.T @ J


def constraint_eig(a, n):
    C = constraint_matrix(a, n)
    w, v = np.linalg.eigh(C)                        # ascending eigenvalues
    return w, v, C


# ----------------------------------------------------------------- reference map
def build_reference_map(scan_indices, pts_concat, offsets, R_all, t_all, voxel=0.10, knn=20):
    """Accumulate the chosen scans (transformed to map frame), voxel-downsample, normals, KDTree."""
    chunks = []
    for i in scan_indices:
        p = pts_concat[offsets[i]:offsets[i+1]]
        chunks.append(p @ R_all[i].T + t_all[i])
    M = np.concatenate(chunks, axis=0)
    keys = np.floor(M / voxel).astype(np.int64)
    _, u = np.unique(keys, axis=0, return_index=True)
    M = M[u].astype(np.float64)
    tree = cKDTree(M)
    nrm = estimate_normals(M, k=knn, kdtree=tree)
    return M, nrm, tree


# ----------------------------------------------------------------- point-to-plane ICP
def point_to_plane_icp(src, T_init, map_pts, map_nrm, tree,
                       iters=25, max_corr=1.0, damp=1e-6, tol=1e-5):
    """Register src (Nx3, sensor frame) to the map by point-to-plane Gauss-Newton.

    Incremental left-perturbation about the origin:  a' = a + ω×a + v.
    Returns (T, info) with info = {resid (RMS plane dist), ninl, C (final 6x6 constraint)}.
    """
    T = T_init.copy().astype(np.float64)
    resid, ninl = np.nan, 0
    C = np.zeros((6, 6))
    for _ in range(iters):
        a = transform(src, T)
        d, idx = tree.query(a)
        m = d < max_corr
        ninl = int(m.sum())
        if ninl < 30:
            break
        p, q, nn = a[m], map_pts[idx[m]], map_nrm[idx[m]]
        r = np.sum((p - q) * nn, axis=1)            # point-to-plane residual
        cr = np.cross(p, nn)
        J = np.concatenate([cr, nn], axis=1)        # (M,6) = [ω, v]
        C = J.T @ J
        g = J.T @ r
        try:
            delta = -np.linalg.solve(C + damp*np.eye(6), g)
        except np.linalg.LinAlgError:
            break
        dR = _expSO3(delta[:3])
        T[:3, :3] = dR @ T[:3, :3]
        T[:3, 3] = dR @ T[:3, 3] + delta[3:]
        resid = float(np.sqrt(np.mean(r * r)))
        if np.linalg.norm(delta) < tol:
            break
    return T, dict(resid=resid, ninl=ninl, C=C)
