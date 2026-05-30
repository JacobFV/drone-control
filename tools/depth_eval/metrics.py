"""Depth metrics + visualization (EVAL ONLY)."""

from __future__ import annotations

import numpy as np


def affine_align(est: np.ndarray, gt: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """Best-fit ``a*est + b`` to gt over valid pixels (forgives scale/shift).

    Monocular relative depth is only defined up to an affine transform; aligning
    before scoring is the charitable comparison. Metric front-ends should not
    *need* this (a≈1, b≈0), which is itself a signal.
    """
    e = est[mask].astype(np.float64)
    g = gt[mask].astype(np.float64)
    if e.size < 8 or np.std(e) < 1e-9:
        return est
    A = np.column_stack([e, np.ones_like(e)])
    sol, *_ = np.linalg.lstsq(A, g, rcond=None)
    a, b = sol
    return a * est + b


def metrics(est: np.ndarray, gt: np.ndarray, *, align: bool = False) -> dict:
    """Standard monocular-depth metrics over pixels where both are valid."""
    mask = np.isfinite(gt) & np.isfinite(est) & (gt > 1e-3)
    out = {"coverage": float(mask.mean()), "n": int(mask.sum())}
    if mask.sum() < 8:
        out.update(absRel=float("nan"), rmse=float("nan"), delta1=float("nan"),
                   corr=float("nan"), a=float("nan"), b=float("nan"))
        return out
    e = est.copy()
    a, b = 1.0, 0.0
    if align:
        ee = est[mask].astype(np.float64)
        gg = gt[mask].astype(np.float64)
        A = np.column_stack([ee, np.ones_like(ee)])
        sol, *_ = np.linalg.lstsq(A, gg, rcond=None)
        a, b = float(sol[0]), float(sol[1])
        e = a * est + b
    ev = e[mask].astype(np.float64)
    gv = gt[mask].astype(np.float64)
    ev = np.clip(ev, 1e-3, None)
    abs_rel = float(np.mean(np.abs(ev - gv) / gv))
    rmse = float(np.sqrt(np.mean((ev - gv) ** 2)))
    ratio = np.maximum(ev / gv, gv / ev)
    delta1 = float(np.mean(ratio < 1.25))
    corr = float(np.corrcoef(est[mask].astype(np.float64), gv)[0, 1])
    out.update(absRel=abs_rel, rmse=rmse, delta1=delta1, corr=corr, a=a, b=b)
    return out


# -- visualization ---------------------------------------------------------

_CMAP = np.array(
    [[48, 18, 130], [33, 144, 200], [60, 200, 120], [240, 220, 60], [230, 60, 40]],
    dtype=np.float64,
)


def colorize(depth: np.ndarray, vmin: float, vmax: float, invalid=(20, 20, 24)) -> np.ndarray:
    """Depth -> RGB (near = warm). Invalid (NaN) pixels get a flat dark colour."""
    d = np.nan_to_num(np.asarray(depth, np.float64), nan=vmax)
    norm = (d - vmin) / max(1e-6, vmax - vmin)
    norm = 1.0 - np.clip(norm, 0.0, 1.0)  # near = warm end
    x = norm * (len(_CMAP) - 1)
    lo = np.floor(x).astype(int)
    hi = np.clip(lo + 1, 0, len(_CMAP) - 1)
    frac = (x - lo)[..., None]
    rgb = (_CMAP[lo] * (1 - frac) + _CMAP[hi] * frac).astype(np.uint8)
    bad = ~np.isfinite(depth)
    rgb[bad] = np.array(invalid, dtype=np.uint8)
    return rgb


def error_map(est: np.ndarray, gt: np.ndarray, cap: float = 1.0) -> np.ndarray:
    """absRel error per pixel -> grayscale->red ramp (black=0, red=>=cap)."""
    mask = np.isfinite(gt) & np.isfinite(est) & (gt > 1e-3)
    rel = np.full(gt.shape, np.nan)
    rel[mask] = np.abs(est[mask] - gt[mask]) / gt[mask]
    out = np.zeros((*gt.shape, 3), dtype=np.uint8)
    v = np.clip(rel / cap, 0, 1)
    out[..., 0] = np.where(mask, (v * 255), 0).astype(np.uint8)
    out[..., 1] = np.where(mask, ((1 - v) * 120), 0).astype(np.uint8)
    return out
