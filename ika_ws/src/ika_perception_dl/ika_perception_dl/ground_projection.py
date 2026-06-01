"""IKA - Yer-duzlemi projeksiyon (IPM) cekirdegi (ROS'suz, test edilebilir).

Tek RGB kamera (Pi Camera CSI) icin: derinlik yok, ama nesne ZEMINE deger
varsayimi altinda 2B kutudan metrik (x, y) konumu hesaplanabilir.

Yontem:
  1. Pixel (u, v) -> kamera optical frame'inde isin (ray)
  2. Isini base_link frame'ine donustur (mevcut optical->base konvansiyonu ile)
  3. Isini zemin duzlemi z=0 ile kes -> (x_b, y_b)

Varsayim: base_link orijini zeminde (z=0). Kamera (camera_x, 0, camera_z) konumunda,
Y ekseninde camera_pitch kadar dondurulmus (pozitif pitch = nose-down).
Ucan/asili nesneler (drone, raf) icin GECERSIZ; insan/arac/sandalye icin dogru.

Output birimleri metre. Sky/ufuk pixel'leri (zemine kesmeyen) None doner.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple


def pixel_to_base_ground(
    u: float, v: float,
    fx: float, fy: float, cx: float, cy: float,
    *,
    camera_pitch: float,
    camera_x: float,
    camera_z: float,
) -> Optional[Tuple[float, float]]:
    """Pixel (u, v) icin kamera-isin -> zemin kesisim noktasi (base_link x, y).

    camera_z kameranin zeminden yuksekligidir (base_link orijini zeminde varsayilir).
    Donus: (x_b, y_b) m, veya None (isin yukari/ufka gidiyor, zemine basmiyor).
    """
    # Pixel -> optical frame ray (REP-103: x sag, y asagi, z ileri)
    rx_opt = (u - cx) / fx
    ry_opt = (v - cy) / fy
    rz_opt = 1.0

    # Optical -> camera_frame (ground_plane.optical_to_base ile ayni konvansiyon):
    #   x_c = z_o, y_c = -x_o, z_c = -y_o
    rx_c = rz_opt
    ry_c = -rx_opt
    rz_c = -ry_opt

    # camera_frame -> base_link rotasyonu (Y ekseni etrafinda pitch)
    cp = math.cos(camera_pitch)
    sp = math.sin(camera_pitch)
    dx = cp * rx_c + sp * rz_c
    dy = ry_c
    dz = -sp * rx_c + cp * rz_c

    # Isin orijini: kamera konumu (camera_x, 0, camera_z) - base_link frame'de
    # Zemin z=0; kesim t cozumu: camera_z + t * dz = 0 -> t = -camera_z / dz
    # dz >= 0 ise isin yukari/ufka gider, zemine basmaz.
    if dz >= -1e-9:
        return None
    t = -camera_z / dz
    if t <= 0:
        return None
    x_b = camera_x + t * dx
    y_b = t * dy
    return x_b, y_b


def bbox_bottom_to_ground(
    bbox_norm: Tuple[float, float, float, float],
    image_w: int, image_h: int,
    fx: float, fy: float, cx: float, cy: float,
    *,
    camera_pitch: float,
    camera_x: float,
    camera_z: float,
) -> Optional[Tuple[float, float]]:
    """Normalize 2B kutu (xmin, ymin, xmax, ymax in 0-1) -> zemin (x, y).

    Kutunun alt-orta noktasi zemine yansitilir (nesnenin ayagi/zemine degdigi yer).
    """
    xmin, _ymin, xmax, ymax = bbox_norm
    u = (xmin + xmax) / 2.0 * image_w
    v = ymax * image_h
    return pixel_to_base_ground(
        u, v, fx, fy, cx, cy,
        camera_pitch=camera_pitch, camera_x=camera_x, camera_z=camera_z,
    )


def range_from_ground_point(point: Optional[Tuple[float, float]]) -> float:
    """(x, y) -> base_link XY duzleminde mesafe; None ise inf."""
    if point is None:
        return float('inf')
    x, y = point
    return math.hypot(x, y)
