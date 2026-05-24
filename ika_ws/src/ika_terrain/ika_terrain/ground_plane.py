"""Zemin duzlemi tahmini ve terrain siniflandirma yardimcilari.

Saf-Python + NumPy. ROS bagimliligi yoktur; unit test edilebilir.

Akis:
  1. depth_to_points()      - depth goruntusu -> optical frame'de Nx3 nokta bulutu
  2. optical_to_base()      - optical frame -> base_link frame (statik kamera pose)
  3. fit_plane_ransac()     - RANSAC ile zemin duzlemi (n*p + d = 0)
  4. analyze_ground()       - dropoff, slope_deg, max_step, confidence dondurur
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


# -----------------------------------------------------------------------
# Depth -> point cloud
# -----------------------------------------------------------------------
def depth_to_points(
    depth_m: np.ndarray,
    fx: float, fy: float, cx: float, cy: float,
    *,
    z_min: float = 0.2,
    z_max: float = 6.0,
    stride: int = 4,
) -> np.ndarray:
    """16UC1 depth goruntusunden optical frame'de Nx3 nokta bulutu.

    Optical frame (REP-103): x=sag, y=asagi, z=ileri.
    """
    h, w = depth_m.shape
    vs = np.arange(0, h, stride)
    us = np.arange(0, w, stride)
    uu, vv = np.meshgrid(us, vs)
    zz = depth_m[vv, uu]

    mask = (zz > z_min) & (zz < z_max)
    if not np.any(mask):
        return np.empty((0, 3), dtype=np.float32)

    z = zz[mask]
    u = uu[mask]
    v = vv[mask]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.stack([x, y, z], axis=1).astype(np.float32)


def optical_to_base(
    pts_opt: np.ndarray,
    *,
    camera_pitch: float,
    camera_x: float,
    camera_z: float,
) -> np.ndarray:
    """Optical frame noktalarini base_link frame'e tasi.

    Kamera URDF'te `<origin xyz="camera_x 0 camera_z" rpy="0 camera_pitch 0">`
    seklinde monte edilmis kabul edilir. Yani:
      1) optical -> camera_frame: standart REP-103 rotasyonu
      2) camera_frame -> base_link: Y ekseni etrafinda camera_pitch + cevirme
    """
    if pts_opt.size == 0:
        return pts_opt
    # optical -> camera_frame (ROS REP-103): x_cam = z_opt, y_cam = -x_opt, z_cam = -y_opt
    x_c = pts_opt[:, 2]
    y_c = -pts_opt[:, 0]
    z_c = -pts_opt[:, 1]

    # camera_frame -> base_link: pitch (Y ekseninde dondur), sonra translate
    cp = math.cos(camera_pitch)
    sp = math.sin(camera_pitch)
    x_b = cp * x_c + sp * z_c + camera_x
    y_b = y_c
    z_b = -sp * x_c + cp * z_c + camera_z

    return np.stack([x_b, y_b, z_b], axis=1).astype(np.float32)


# -----------------------------------------------------------------------
# RANSAC duzlem uydurma
# -----------------------------------------------------------------------
@dataclass
class Plane:
    """Duzlem: n . p + d = 0, ||n||=1. n = (a, b, c), d skaler."""
    normal: np.ndarray   # shape (3,)
    d: float
    inlier_count: int
    inlier_mask: np.ndarray  # shape (N,) boolean


def fit_plane_ransac(
    points: np.ndarray,
    *,
    iterations: int = 60,
    tolerance: float = 0.04,
    rng: Optional[np.random.Generator] = None,
) -> Optional[Plane]:
    """RANSAC ile en cok inlier'a sahip duzlemi bul.

    `tolerance` metre cinsinden bir noktanin duzleme uzakliginin esiklenmesi.
    En az 50 nokta ve geçerli bir duzlem bulunmazsa None doner.
    """
    n = points.shape[0]
    if n < 50:
        return None
    if rng is None:
        rng = np.random.default_rng(seed=42)

    best: Optional[Plane] = None

    for _ in range(iterations):
        idx = rng.choice(n, size=3, replace=False)
        p0, p1, p2 = points[idx]
        v1 = p1 - p0
        v2 = p2 - p0
        normal = np.cross(v1, v2)
        nrm = float(np.linalg.norm(normal))
        if nrm < 1e-6:
            continue
        normal = normal / nrm
        d = -float(np.dot(normal, p0))

        distances = np.abs(points @ normal + d)
        inliers = distances < tolerance
        count = int(inliers.sum())
        if best is None or count > best.inlier_count:
            best = Plane(normal=normal, d=d, inlier_count=count, inlier_mask=inliers)

    return best


# -----------------------------------------------------------------------
# Yuksek seviye analiz
# -----------------------------------------------------------------------
@dataclass
class TerrainReport:
    classification: str          # SAFE / CAUTION / IMPASSABLE / DROPOFF_DANGER / UNKNOWN
    slope_deg: float
    dropoff_risk: bool
    max_step_height_m: float
    confidence: float


def analyze_ground(
    pts_base: np.ndarray,
    *,
    tolerance: float = 0.04,
    safe_slope_deg: float = 15.0,
    caution_slope_deg: float = 25.0,
    dropoff_depth_threshold_m: float = 0.15,
    lookout_distance_m: float = 0.6,
    max_step_height_m: float = 0.04,
    confidence_threshold: float = 0.6,
) -> TerrainReport:
    """base_link frame'deki noktalardan terrain durumunu sinifla.

    Cikti:
      - classification: SAFE / CAUTION / IMPASSABLE / DROPOFF_DANGER / UNKNOWN
      - slope_deg: zemin duzleminin normal vektoru ile dunya Z'si arasi aci
      - dropoff_risk: aracin onunde zemin altinda bos alan tespit edildi mi
      - max_step_height_m: zemin uzerinde tespit edilen en yuksek bilinen engel
      - confidence: 0-1 arasi guven (inlier orani)
    """
    if pts_base.size < 200:
        return TerrainReport('UNKNOWN', 0.0, False, 0.0, 0.0)

    # Yalniz onumuzdeki bolgeye odaklan: 0.1 < x < lookout, |y| < 0.5
    ahead = (
        (pts_base[:, 0] > 0.1) &
        (pts_base[:, 0] < lookout_distance_m) &
        (np.abs(pts_base[:, 1]) < 0.5)
    )
    front = pts_base[ahead]
    if front.shape[0] < 100:
        return TerrainReport('UNKNOWN', 0.0, False, 0.0, 0.3)

    # Plane uydurmasini YAKIN bolgeye yap - bu "aracin altinda kabul edilen
    # zemin"dir. Sonra UZAK bolgeyi bu plane'e gore degerlendirir.
    mid = lookout_distance_m * 0.5
    near = front[front[:, 0] < mid]
    far = front[front[:, 0] >= mid]

    fit_pts = near if near.shape[0] >= 80 else front
    plane = fit_plane_ransac(fit_pts, tolerance=tolerance)
    if plane is None:
        return TerrainReport('UNKNOWN', 0.0, False, 0.0, 0.2)

    confidence = float(plane.inlier_count / max(1, fit_pts.shape[0]))

    # Normal vektorunu yukari yonelt (z>=0). Plane denklemi: n.p + d = 0.
    # Cevirirken d isaretini de cevirmek gerekir.
    if plane.normal[2] < 0:
        n_up = -plane.normal
        d_up = -plane.d
    else:
        n_up = plane.normal
        d_up = plane.d

    slope_rad = math.acos(max(-1.0, min(1.0, n_up[2])))
    slope_deg = math.degrees(slope_rad)

    # Cukur: yakin plane'e gore uzak noktalar asagi mi dusmus?
    # signed = n_up.p + d_up.  >0 plane uzerinde, <0 altinda.
    dropoff_risk = False
    if near.shape[0] > 20 and far.shape[0] > 20:
        signed_far = far @ n_up + d_up
        if np.median(signed_far) < -dropoff_depth_threshold_m:
            dropoff_risk = True
    elif near.shape[0] > 20 and far.shape[0] < 5:
        dropoff_risk = True
        confidence = max(0.3, confidence * 0.7)

    # En yuksek engel: front'taki pozitif sapma
    signed = front @ n_up + d_up
    above = signed[signed > 0]
    max_step = float(above.max()) if above.size > 0 else 0.0

    # Siniflandirma
    if dropoff_risk:
        cls = 'DROPOFF_DANGER'
    elif confidence < confidence_threshold:
        cls = 'UNKNOWN'
    elif slope_deg <= safe_slope_deg and max_step <= max_step_height_m:
        cls = 'SAFE'
    elif slope_deg <= caution_slope_deg:
        cls = 'CAUTION'
    else:
        cls = 'IMPASSABLE'

    return TerrainReport(
        classification=cls,
        slope_deg=round(slope_deg, 2),
        dropoff_risk=dropoff_risk,
        max_step_height_m=round(max_step, 3),
        confidence=round(confidence, 3),
    )
