"""ground_projection (IPM) cekirdegi icin birim testler (ROS'suz)."""
import math

from ika_perception_dl.ground_projection import (
    bbox_bottom_to_ground,
    pixel_to_base_ground,
    range_from_ground_point,
)


# Pi Camera v3 kabaca: 1280x720, FOV ~66 dikey - tipik intrinsics
W, H = 1280, 720
FX, FY = 900.0, 900.0
CX, CY = W / 2.0, H / 2.0

# Standart montaj (mevcut sistem ile uyumlu)
PITCH = 0.15      # ~8.6 derece nose-down
CAM_X = 0.10
CAM_Z = 0.15


# -----------------------------------------------------------------------
# Ufuk / sky pixel'leri
# -----------------------------------------------------------------------
def test_center_pixel_no_pitch_is_horizon():
    # Yatay kamera + tam merkez pixel -> isin ufka gider, zemine basmaz
    p = pixel_to_base_ground(CX, CY, FX, FY, CX, CY,
                             camera_pitch=0.0, camera_x=0.0, camera_z=CAM_Z)
    assert p is None

def test_above_horizon_pixel_returns_none():
    # Resmin ust yarisinda + pitch=0 -> ufuk uzeri, isin yukari -> None
    p = pixel_to_base_ground(CX, CY * 0.5, FX, FY, CX, CY,
                             camera_pitch=0.0, camera_x=0.0, camera_z=CAM_Z)
    assert p is None


# -----------------------------------------------------------------------
# Zemine basan pixel'ler - mesafe artisinin yonu
# -----------------------------------------------------------------------
def test_bottom_pixel_hits_close_ground():
    # Resmin tam altinda, pitch=0 -> en yakin zemin
    p = pixel_to_base_ground(CX, H - 1, FX, FY, CX, CY,
                             camera_pitch=0.0, camera_x=0.0, camera_z=CAM_Z)
    assert p is not None
    x, y = p
    assert x > 0 and abs(y) < 1e-6

def test_higher_v_yields_closer_distance():
    # Daha asagidaki pixel daha yakin zemini gosterir (perspektif)
    p_low = pixel_to_base_ground(CX, H - 1, FX, FY, CX, CY,
                                 camera_pitch=PITCH, camera_x=CAM_X, camera_z=CAM_Z)
    p_mid = pixel_to_base_ground(CX, H - 200, FX, FY, CX, CY,
                                 camera_pitch=PITCH, camera_x=CAM_X, camera_z=CAM_Z)
    assert p_low is not None and p_mid is not None
    assert p_low[0] < p_mid[0]   # alt pixel daha yakin

def test_pitch_down_brings_ground_closer():
    # Ayni pixel, pitch artarsa zemin yaklasir
    p_flat = pixel_to_base_ground(CX, CY, FX, FY, CX, CY,
                                  camera_pitch=0.10, camera_x=0.0, camera_z=CAM_Z)
    p_tilt = pixel_to_base_ground(CX, CY, FX, FY, CX, CY,
                                  camera_pitch=0.30, camera_x=0.0, camera_z=CAM_Z)
    assert p_flat is not None and p_tilt is not None
    assert p_tilt[0] < p_flat[0]


# -----------------------------------------------------------------------
# Yan/yatay simetri
# -----------------------------------------------------------------------
def test_left_pixel_gives_positive_y_base():
    # u < cx (resimde sol) -> base_link y > 0 (sol)
    p = pixel_to_base_ground(CX - 200, H - 50, FX, FY, CX, CY,
                             camera_pitch=PITCH, camera_x=CAM_X, camera_z=CAM_Z)
    assert p is not None
    assert p[1] > 0

def test_right_pixel_gives_negative_y_base():
    p = pixel_to_base_ground(CX + 200, H - 50, FX, FY, CX, CY,
                             camera_pitch=PITCH, camera_x=CAM_X, camera_z=CAM_Z)
    assert p is not None
    assert p[1] < 0


# -----------------------------------------------------------------------
# bbox_bottom_to_ground
# -----------------------------------------------------------------------
def test_bbox_bottom_matches_pixel_call():
    # Tam onde duran nesne: bbox merkez x, alt y resmin altinda
    bbox = (0.45, 0.40, 0.55, 0.95)  # normalized
    p_bbox = bbox_bottom_to_ground(
        bbox, W, H, FX, FY, CX, CY,
        camera_pitch=PITCH, camera_x=CAM_X, camera_z=CAM_Z)
    # Esdeger pixel cagrisi
    u = (bbox[0] + bbox[2]) / 2.0 * W
    v = bbox[3] * H
    p_px = pixel_to_base_ground(u, v, FX, FY, CX, CY,
                                camera_pitch=PITCH, camera_x=CAM_X, camera_z=CAM_Z)
    assert p_bbox is not None and p_px is not None
    assert abs(p_bbox[0] - p_px[0]) < 1e-9
    assert abs(p_bbox[1] - p_px[1]) < 1e-9

def test_bbox_top_half_returns_none_for_sky():
    # Sadece resmin ust yarisinda kalan bbox + pitch=0 -> alt v ufuk uzeri -> None
    bbox = (0.40, 0.10, 0.60, 0.40)  # ymax=0.40 < 0.5 (resmin yarisindan asagi gelmiyor)
    p = bbox_bottom_to_ground(
        bbox, W, H, FX, FY, CX, CY,
        camera_pitch=0.0, camera_x=0.0, camera_z=CAM_Z)
    assert p is None


# -----------------------------------------------------------------------
# range_from_ground_point
# -----------------------------------------------------------------------
def test_range_for_none_is_inf():
    assert math.isinf(range_from_ground_point(None))

def test_range_basic():
    assert abs(range_from_ground_point((3.0, 4.0)) - 5.0) < 1e-9
