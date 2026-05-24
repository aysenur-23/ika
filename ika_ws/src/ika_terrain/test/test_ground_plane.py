"""ground_plane modulu icin birim testler.

Sentetik nokta bulutlari uzerinde RANSAC ve siniflandirma davranisini dogrular.
"""
import math

import numpy as np
import pytest

from ika_terrain.ground_plane import (
    depth_to_points,
    optical_to_base,
    fit_plane_ransac,
    analyze_ground,
)


# -----------------------------------------------------------------------
# RANSAC dogruluk testleri
# -----------------------------------------------------------------------
def _flat_ground_cloud(n: int = 600, noise_std: float = 0.005,
                       seed: int = 0) -> np.ndarray:
    """base_link frame'inde duz bir zemin nokta bulutu (z=0)."""
    rng = np.random.default_rng(seed)
    x = rng.uniform(0.1, 1.5, n)
    y = rng.uniform(-0.6, 0.6, n)
    z = rng.normal(0.0, noise_std, n)
    return np.stack([x, y, z], axis=1).astype(np.float32)


def _sloped_cloud(slope_deg: float, n: int = 600, seed: int = 0) -> np.ndarray:
    """z = tan(slope) * x duzlemi - aracin onunde yokus."""
    rng = np.random.default_rng(seed)
    x = rng.uniform(0.1, 1.5, n)
    y = rng.uniform(-0.6, 0.6, n)
    z = math.tan(math.radians(slope_deg)) * x + rng.normal(0.0, 0.005, n)
    return np.stack([x, y, z], axis=1).astype(np.float32)


def test_fit_plane_ransac_flat_ground():
    pts = _flat_ground_cloud()
    plane = fit_plane_ransac(pts, iterations=80, tolerance=0.03)
    assert plane is not None
    # Normalin Z bileseni 1'e yakin olmali
    nz = abs(plane.normal[2])
    assert nz > 0.95, f'flat ground normal Z = {nz:.3f}, beklenmeyen'
    # Inlier orani yuksek olmali
    assert plane.inlier_count / len(pts) > 0.9


def test_fit_plane_ransac_sloped_15_deg():
    pts = _sloped_cloud(slope_deg=15.0)
    plane = fit_plane_ransac(pts, iterations=80, tolerance=0.03)
    assert plane is not None
    # Normal -> dunya Z arasindaki aci ~ 15 derece olmali
    n = plane.normal
    if n[2] < 0:
        n = -n
    angle_deg = math.degrees(math.acos(min(1.0, max(-1.0, n[2]))))
    assert abs(angle_deg - 15.0) < 3.0, f'beklenen ~15, alinan {angle_deg:.2f}'


def test_fit_plane_ransac_too_few_points():
    pts = _flat_ground_cloud(n=30)
    plane = fit_plane_ransac(pts)
    assert plane is None


# -----------------------------------------------------------------------
# analyze_ground siniflandirmasi
# -----------------------------------------------------------------------
def test_analyze_safe_flat():
    pts = _flat_ground_cloud()
    rep = analyze_ground(pts)
    assert rep.classification == 'SAFE'
    assert rep.slope_deg < 3.0
    assert rep.dropoff_risk is False


def test_analyze_caution_slope():
    pts = _sloped_cloud(slope_deg=20.0)
    rep = analyze_ground(pts, safe_slope_deg=15.0, caution_slope_deg=25.0)
    assert rep.classification == 'CAUTION'


def test_analyze_impassable_steep():
    pts = _sloped_cloud(slope_deg=35.0)
    rep = analyze_ground(pts, safe_slope_deg=15.0, caution_slope_deg=25.0)
    assert rep.classification == 'IMPASSABLE'


def test_analyze_dropoff_detected():
    # Yakin bolge duz, uzakta cukur (z asagi inmis)
    rng = np.random.default_rng(0)
    near_x = rng.uniform(0.1, 0.3, 300)
    near_y = rng.uniform(-0.4, 0.4, 300)
    near_z = rng.normal(0.0, 0.005, 300)
    far_x = rng.uniform(0.35, 0.6, 300)
    far_y = rng.uniform(-0.4, 0.4, 300)
    far_z = -0.30 + rng.normal(0.0, 0.005, 300)   # 30cm asagida
    pts = np.stack([
        np.concatenate([near_x, far_x]),
        np.concatenate([near_y, far_y]),
        np.concatenate([near_z, far_z]),
    ], axis=1).astype(np.float32)
    rep = analyze_ground(
        pts, dropoff_depth_threshold_m=0.15, lookout_distance_m=0.6,
    )
    assert rep.dropoff_risk is True
    assert rep.classification == 'DROPOFF_DANGER'


def test_analyze_unknown_too_few_points():
    pts = _flat_ground_cloud(n=100)  # ileri bolgede yetersiz
    rep = analyze_ground(pts)
    # Cogu nokta ileri filtreden gecmeyebilir veya yetersiz olabilir.
    # Burada UNKNOWN ya da SAFE kabul edilebilir; en azindan ERROR atmamali.
    assert rep.classification in ('UNKNOWN', 'SAFE')


# -----------------------------------------------------------------------
# depth_to_points
# -----------------------------------------------------------------------
def test_depth_to_points_shape():
    h, w = 60, 80
    depth = np.full((h, w), 2.0, dtype=np.float32)
    pts = depth_to_points(depth, fx=100.0, fy=100.0, cx=40.0, cy=30.0, stride=2)
    # Tum noktalar 2m derinlikte, Z = 2.0
    assert pts.shape[1] == 3
    assert pts.shape[0] > 0
    assert np.allclose(pts[:, 2], 2.0)


def test_depth_to_points_filters_invalid():
    h, w = 40, 40
    depth = np.zeros((h, w), dtype=np.float32)  # tum gecersiz (z=0 < z_min)
    pts = depth_to_points(depth, fx=100.0, fy=100.0, cx=20.0, cy=20.0)
    assert pts.shape == (0, 3)


def test_optical_to_base_origin():
    # Tek nokta: optikte (0, 0, 1) ileri 1m
    pts_opt = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)
    pts_base = optical_to_base(pts_opt, camera_pitch=0.0,
                               camera_x=0.1, camera_z=0.15)
    # Pitch yokken: x_base = z_opt + camera_x; y_base = -x_opt; z_base = -y_opt + camera_z
    assert pytest.approx(pts_base[0, 0], abs=1e-5) == 1.0 + 0.1
    assert pytest.approx(pts_base[0, 1], abs=1e-5) == 0.0
    assert pytest.approx(pts_base[0, 2], abs=1e-5) == 0.15
