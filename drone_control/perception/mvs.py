"""
Dense plane-sweep multi-view stereo (the depth workhorse).

The sparse feature front-end (``slam.py``) is the structural backbone, but on the
real input — ~128px frames, heavy cheap-CMOS noise, and intensely repetitive
geometry — discrete descriptor matching collapses (every feature has a near
twin, so the ratio test rejects almost everything). With the camera poses known
and accurate, the robust formulation is **plane-sweep stereo**: sweep a stack of
fronto-parallel depth planes through the reference camera; for each plane warp
every windowed source view into the reference by the plane-induced homography;
score photo-consistency with a noise-robust patch **NCC**; and pick, per pixel,
the depth whose plane best agrees across the views.

Why this beats sparse matching here:
  * No discrete correspondences — every pixel gets evidence, including smooth /
    blurry regions ORB can't corner-detect.
  * NCC over a patch is invariant to the per-frame brightness / white-balance
    drift the sim's sensor model injects.
  * Aggregating cost across views of *different* baselines breaks the periodic
    ambiguity of repeated shelves: a coincidental match lines up at one
    baseline's disparity but not the others, so only the true depth wins the
    aggregate.

Output is a metric z-depth map in the reference camera plus a confidence mask
(low uniqueness / high cost / too few supporting views are dropped). Everything
is derived from frames + calibrated poses only — no sim ground truth.
"""

from __future__ import annotations

import numpy as np


def _box(img: np.ndarray, k: int, cv2) -> np.ndarray:
    return cv2.boxFilter(img, ddepth=-1, ksize=(k, k), normalize=True, borderType=cv2.BORDER_REFLECT)


def _guided_filter(guide: np.ndarray, src: np.ndarray, radius: int, eps: float, cv2) -> np.ndarray:
    """Edge-aware filter (He et al.): smooth ``src`` while respecting edges in the
    grayscale ``guide``. Built from box filters so it needs no opencv-contrib."""
    k = 2 * radius + 1
    mean_i = _box(guide, k, cv2)
    mean_p = _box(src, k, cv2)
    corr_i = _box(guide * guide, k, cv2)
    corr_ip = _box(guide * src, k, cv2)
    var_i = corr_i - mean_i * mean_i
    cov_ip = corr_ip - mean_i * mean_p
    a = cov_ip / (var_i + eps)
    b = mean_p - a * mean_i
    return _box(a, k, cv2) * guide + _box(b, k, cv2)


def _ncc_cost(ref: np.ndarray, warp: np.ndarray, valid: np.ndarray, k: int, cv2) -> np.ndarray:
    """1 - normalized cross-correlation over a k×k patch, per pixel.

    Returns cost in [0, 2] (0 = perfect match); invalid pixels -> +inf.
    """
    a = ref
    b = warp
    mu_a = _box(a, k, cv2)
    mu_b = _box(b, k, cv2)
    aa = _box(a * a, k, cv2) - mu_a * mu_a
    bb = _box(b * b, k, cv2) - mu_b * mu_b
    ab = _box(a * b, k, cv2) - mu_a * mu_b
    denom = np.sqrt(np.maximum(aa, 1e-6) * np.maximum(bb, 1e-6))
    ncc = ab / denom
    cost = 1.0 - np.clip(ncc, -1.0, 1.0)
    cost[~valid] = np.inf
    return cost


def _census(img: np.ndarray, radius: int = 2) -> np.ndarray:
    """Census transform: per pixel, the sign of each neighbour vs the centre,
    as a bit-stack [B,H,W] (bool). Invariant to any monotonic intensity change
    (white-balance / vignette drift) — the right cost for this noisy sensor."""
    h, w = img.shape
    pad = np.pad(img, radius, mode="reflect")
    bits = []
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dy == 0 and dx == 0:
                continue
            shifted = pad[radius + dy:radius + dy + h, radius + dx:radius + dx + w]
            bits.append(shifted >= img)
    return np.stack(bits, axis=0)


def _homography(K, Kinv, R_ref, C_ref, R_src, C_src, z) -> np.ndarray:
    """Homography mapping reference pixels -> source pixels for the fronto-parallel
    plane at reference-camera depth ``z`` (plane normal = camera forward)."""
    R_sr = R_src.T @ R_ref                    # ref-cam -> src-cam rotation
    t_sr = R_src.T @ (C_ref - C_src)          # ref-cam -> src-cam translation
    n = np.array([0.0, 0.0, 1.0])
    # X_src = (R_sr + t_sr n^T / z) X_ref for points on the plane n^T X_ref = z.
    H = K @ (R_sr + np.outer(t_sr, n) / z) @ Kinv
    return H


def plane_sweep(
    views: list[tuple[np.ndarray, np.ndarray, np.ndarray]],
    ref_idx: int,
    K: np.ndarray,
    *,
    near: float = 0.4,
    far: float = 25.0,
    n_depths: int = 48,
    patch: int = 7,
    min_views: int = 2,
    uniqueness: float = 0.92,
    max_cost: float = 0.6,
    aggregate: int = 9,
    cost_mode: str = "census",
    census_radius: int = 3,
) -> tuple[np.ndarray, np.ndarray]:
    """Dense MVS for one reference view.

    ``views`` is a list of (gray_float[H,W], R cam->world, C[3]); ``ref_idx``
    selects the reference. ``cost_mode`` is "census" (radiometric-invariant, the
    default — best on this noisy sensor) or "ncc". Returns (z_depth[H,W],
    confidence[H,W]); masked pixels are NaN in the depth map.
    """
    import cv2

    H_img, W_img = views[ref_idx][0].shape
    Kinv = np.linalg.inv(K)
    ref_gray, R_ref, C_ref = views[ref_idx]
    sources = [v for i, v in enumerate(views) if i != ref_idx]
    if not sources:
        return np.full((H_img, W_img), np.nan), np.zeros((H_img, W_img))

    ref_census = _census(ref_gray, census_radius) if cost_mode == "census" else None
    census_bits = ref_census.shape[0] if ref_census is not None else 1

    inv_d = np.linspace(1.0 / far, 1.0 / near, n_depths)
    depths = 1.0 / inv_d

    # Per-depth aggregated cost volume, occlusion-robust: at each pixel/plane we
    # average the *best half* of the per-source costs so a source that doesn't
    # see the surface (occluded / out of frame) can't poison the score.
    cost_vol = np.full((n_depths, H_img, W_img), np.inf, dtype=np.float32)
    count_vol = np.zeros((n_depths, H_img, W_img), dtype=np.int16)

    for di, z in enumerate(depths):
        per_src = []
        for (g_src, R_src, C_src) in sources:
            Hmat = _homography(K, Kinv, R_ref, C_ref, R_src, C_src, z)
            warp = cv2.warpPerspective(
                g_src, Hmat, (W_img, H_img),
                flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP, borderValue=-1.0,
            )
            valid = warp >= 0.0
            if cost_mode == "census":
                wc = _census(np.where(valid, warp, 0.0), census_radius)
                ham = (np.count_nonzero(wc != ref_census, axis=0).astype(np.float32)
                       / census_bits)
                # Box-aggregate the Hamming cost over a support window (ignoring
                # invalid pixels) — census is per-pixel and needs spatial support.
                vf = valid.astype(np.float32)
                num = _box(ham * vf, patch, cv2)
                den = _box(vf, patch, cv2)
                cost = np.where(den > 1e-3, num / np.maximum(den, 1e-6), np.inf)
                cost[~valid] = np.inf
            else:
                cost = _ncc_cost(ref_gray, np.where(valid, warp, 0.0), valid, patch, cv2)
            per_src.append(cost)
        stack = np.stack(per_src, axis=0)            # [S,H,W]
        valid_mask = np.isfinite(stack)
        nvalid = valid_mask.sum(axis=0)
        # best-half mean (robust); fall back to all valid when few sources.
        big = np.where(valid_mask, stack, np.inf)
        big.sort(axis=0)
        keep = np.maximum(1, (nvalid + 1) // 2)
        csum = np.cumsum(np.where(np.isfinite(big), big, 0.0), axis=0)
        idx = np.clip(keep - 1, 0, big.shape[0] - 1)
        agg = np.take_along_axis(csum, idx[None], axis=0)[0] / np.maximum(keep, 1)
        agg = np.where(nvalid >= min_views, agg, np.inf)
        cost_vol[di] = agg.astype(np.float32)
        count_vol[di] = nvalid.astype(np.int16)

    # Cost-volume filtering: per-pixel winner-take-all is speckle. Aggregating
    # each depth slice spatially — edge-aware on the reference image (guided
    # filter) when available, else a box filter — enforces local depth
    # smoothness and is the single biggest quality lever here. Invalid entries
    # are capped (NCC=0 ⇒ cost 1) so the filter stays well-defined.
    if aggregate and aggregate > 1:
        cap = 1.0
        ref_guide = (ref_gray / 255.0).astype(np.float32)
        radius = max(1, aggregate // 2)
        for di in range(n_depths):
            slc = np.where(np.isfinite(cost_vol[di]), cost_vol[di], cap).astype(np.float32)
            cost_vol[di] = _guided_filter(ref_guide, slc, radius, 1e-3, cv2)

    # Winner-take-all with sub-pixel (parabola in inverse-depth) refinement.
    # The cost volume holds +inf for pixels/planes with too few valid views, so
    # arithmetic below can touch inf — ignore the resulting numpy warnings; the
    # `good` mask drops those pixels anyway.
    with np.errstate(invalid="ignore", divide="ignore"):
        best = np.argmin(cost_vol, axis=0)
        best_cost = np.take_along_axis(cost_vol, best[None], axis=0)[0]

        # Uniqueness: ratio of best to the best cost OUTSIDE a ±1 neighbourhood.
        masked = cost_vol.copy()
        for off in (-1, 0, 1):
            bi = np.clip(best + off, 0, n_depths - 1)
            np.put_along_axis(masked, bi[None], np.inf, axis=0)
        second = np.min(masked, axis=0)
        ratio = best_cost / np.maximum(second, 1e-6)

        inv_best = inv_d[best]
        # Parabolic refinement on inverse-depth using the cost neighbours.
        bi0 = np.clip(best - 1, 0, n_depths - 1)
        bi2 = np.clip(best + 1, 0, n_depths - 1)
        c0 = np.take_along_axis(cost_vol, bi0[None], axis=0)[0]
        c1 = best_cost
        c2 = np.take_along_axis(cost_vol, bi2[None], axis=0)[0]
        denom = (c0 - 2 * c1 + c2)
        delta = np.where(np.abs(denom) > 1e-9, 0.5 * (c0 - c2) / denom, 0.0)
        delta = np.clip(delta, -1.0, 1.0)
        step = np.gradient(inv_d).mean()
        inv_ref = inv_best + delta * step
        z_depth = 1.0 / np.clip(inv_ref, 1.0 / far, 1.0 / near)

    confidence = np.clip(1.0 - ratio, 0.0, 1.0) * np.clip(1.0 - best_cost, 0.0, 1.0)
    good = (
        np.isfinite(best_cost)
        & (best_cost < max_cost)
        & (ratio < uniqueness)
        & (count_vol[best, np.arange(H_img)[:, None], np.arange(W_img)[None, :]] >= min_views)
        & (best > 0) & (best < n_depths - 1)
    )
    z_depth = np.where(good, z_depth, np.nan)
    confidence = np.where(good, confidence, 0.0)
    return z_depth.astype(np.float64), confidence.astype(np.float64)


def densify(z_depth: np.ndarray, ref_gray: np.ndarray, *, near: float = 0.4,
            far: float = 25.0, radius: int = 4, eps: float = 1e-3,
            iters: int = 12) -> np.ndarray:
    """Edge-aware fill of the confident-but-holey plane-sweep depth.

    Works in inverse-depth (linear in disparity). Holes are first seeded with the
    nearest confident anchor (a bounded, sane prior — a plain harmonic fill drifts
    toward zero inverse-depth in big holes and explodes the depth), then refined
    by repeated guided filtering against the reference image so depth propagates
    across smooth surfaces but stops at intensity edges. Known pixels are held
    fixed and every iterate is clamped to the valid depth range.
    """
    import cv2
    from scipy import ndimage

    known = np.isfinite(z_depth)
    if known.sum() < 16:
        return np.full_like(z_depth, np.nan)
    guide = (ref_gray / 255.0).astype(np.float32)
    inv_known = np.zeros_like(guide)
    inv_known[known] = 1.0 / z_depth[known]
    inv_hi, inv_lo = 1.0 / near, 1.0 / far

    # Nearest-anchor seed: bounded everywhere, no zero-inverse-depth blow-ups.
    idx = ndimage.distance_transform_edt(~known, return_distances=False, return_indices=True)
    cur = inv_known[tuple(idx)].astype(np.float32)
    m = known.astype(np.float32)
    for _ in range(iters):
        filled = _guided_filter(guide, cur, radius, eps, cv2)
        cur = m * inv_known + (1.0 - m) * filled
        cur = np.clip(cur, inv_lo, inv_hi)
    return 1.0 / cur


def zdepth_to_euclidean(z_depth: np.ndarray, K: np.ndarray) -> np.ndarray:
    """Convert reference-camera z-depth to Euclidean ray length (matches the
    monocular path + oracle convention)."""
    h, w = z_depth.shape
    xs = np.arange(w); ys = np.arange(h)
    gx, gy = np.meshgrid(xs, ys)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    scale = np.sqrt(((gx - cx) / fx) ** 2 + ((gy - cy) / fy) ** 2 + 1.0)
    return z_depth * scale


def backproject_zdepth(z_depth: np.ndarray, rgb: np.ndarray, K: np.ndarray,
                       R: np.ndarray, C: np.ndarray, stride: int = 1):
    """World points + colours from a z-depth map (R = cam->world, C = centre)."""
    h, w = z_depth.shape
    xs = np.arange(0, w, stride); ys = np.arange(0, h, stride)
    gx, gy = np.meshgrid(xs, ys)
    z = z_depth[gy, gx]
    valid = np.isfinite(z)
    gx, gy, z = gx[valid], gy[valid], z[valid]
    if gx.size == 0:
        return np.zeros((0, 3)), np.zeros((0, 3), np.uint8)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    cam = np.stack([(gx - cx) / fx * z, (gy - cy) / fy * z, z], axis=1)
    world = cam @ R.T + C
    colors = rgb[gy, gx, :]
    return world, colors.reshape(-1, 3)
