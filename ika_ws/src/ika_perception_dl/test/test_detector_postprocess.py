"""detector_postprocess cekirdegi icin birim testler.

Sentetik spatial tespitler uzerinde filtre, koordinat donusumu ve
hazard siniflandirma davranisini dogrular. ROS / depthai gerektirmez.
"""
import math

from ika_perception_dl.detector_postprocess import (
    RawSpatialDetection,
    camera_optical_to_base,
    classify_hazard,
    label_for,
    process_detections,
    summarize,
)


LABELS = ['background', 'person', 'bicycle', 'car', 'chair']
DYNAMIC = ['person', 'bicycle', 'car']


# -----------------------------------------------------------------------
# Koordinat donusumu
# -----------------------------------------------------------------------
def test_camera_to_base_straight_ahead():
    # Optical frame: tam onde 2 m (x=0 sag, y=0 asagi, z=2 ileri)
    x_b, y_b, z_b = camera_optical_to_base(
        0.0, 0.0, 2.0, camera_pitch=0.0, camera_x=0.10, camera_z=0.15)
    assert abs(x_b - 2.10) < 1e-6   # ileri = z + camera_x
    assert abs(y_b - 0.0) < 1e-6
    assert abs(z_b - 0.15) < 1e-6   # camera_z

def test_camera_to_base_object_to_right_is_negative_y():
    # Optical x>0 (sag) -> base_link y<0 (sag = negatif sol)
    _, y_b, _ = camera_optical_to_base(
        0.5, 0.0, 2.0, camera_pitch=0.0, camera_x=0.0, camera_z=0.0)
    assert y_b < 0.0

def test_camera_to_base_matches_ground_plane_convention():
    # ika_terrain.optical_to_base ile ayni sonucu vermeli (vektor vs tek nokta)
    import numpy as np
    from ika_terrain.ground_plane import optical_to_base
    pt = np.array([[0.3, -0.1, 1.5]], dtype=np.float32)
    ref = optical_to_base(pt, camera_pitch=0.15, camera_x=0.10, camera_z=0.15)[0]
    got = camera_optical_to_base(
        0.3, -0.1, 1.5, camera_pitch=0.15, camera_x=0.10, camera_z=0.15)
    assert all(abs(float(r) - g) < 1e-5 for r, g in zip(ref, got))


# -----------------------------------------------------------------------
# Siniflandirma
# -----------------------------------------------------------------------
def test_classify_dynamic_takes_priority():
    assert classify_hazard('person', DYNAMIC, ['chair']) == 'DYNAMIC'

def test_classify_static():
    assert classify_hazard('chair', DYNAMIC, ['chair']) == 'STATIC'

def test_classify_unknown_uses_default():
    assert classify_hazard('zebra', DYNAMIC, ['chair'], default='STATIC') == 'STATIC'

def test_label_for_out_of_range():
    assert label_for(99, LABELS) == '99'
    assert label_for(1, LABELS) == 'person'


# -----------------------------------------------------------------------
# process_detections - filtreler
# -----------------------------------------------------------------------
def test_confidence_filter_drops_low():
    raw = [RawSpatialDetection(1, 0.3, 0.0, 0.0, 2.0)]
    out = process_detections(raw, LABELS, dynamic_labels=DYNAMIC,
                             confidence_threshold=0.5)
    assert out == []

def test_range_filter_drops_far():
    raw = [RawSpatialDetection(1, 0.9, 0.0, 0.0, 10.0)]
    out = process_detections(raw, LABELS, dynamic_labels=DYNAMIC,
                             max_range_m=6.0)
    assert out == []

def test_range_filter_drops_too_close():
    raw = [RawSpatialDetection(1, 0.9, 0.0, 0.0, 0.05)]
    out = process_detections(raw, LABELS, dynamic_labels=DYNAMIC,
                             min_range_m=0.2)
    assert out == []

def test_ignore_label_dropped():
    raw = [RawSpatialDetection(0, 0.9, 0.0, 0.0, 2.0)]  # 'background'
    out = process_detections(raw, LABELS, dynamic_labels=DYNAMIC,
                             ignore_labels=['background'])
    assert out == []

def test_person_detection_passes_and_is_dynamic():
    raw = [RawSpatialDetection(1, 0.9, 0.0, 0.0, 2.0)]
    out = process_detections(
        raw, LABELS, dynamic_labels=DYNAMIC,
        camera_x=0.10, camera_z=0.15)
    assert len(out) == 1
    d = out[0]
    assert d.label == 'person'
    assert d.hazard == 'DYNAMIC'
    assert abs(d.x - 2.10) < 1e-3
    assert abs(d.range_m - 2.10) < 1e-3

def test_multiple_detections_mixed():
    raw = [
        RawSpatialDetection(1, 0.9, 0.0, 0.0, 2.0),    # person, dynamic
        RawSpatialDetection(4, 0.8, 0.0, 0.0, 1.5),    # chair, static
        RawSpatialDetection(3, 0.2, 0.0, 0.0, 1.0),    # car, low conf -> drop
    ]
    out = process_detections(
        raw, LABELS, dynamic_labels=DYNAMIC, static_labels=['chair'],
        confidence_threshold=0.5)
    assert len(out) == 2
    hazards = {d.label: d.hazard for d in out}
    assert hazards == {'person': 'DYNAMIC', 'chair': 'STATIC'}


# -----------------------------------------------------------------------
# summarize
# -----------------------------------------------------------------------
def test_summarize_empty():
    s = summarize([])
    assert s['count'] == 0
    assert s['dynamic_count'] == 0
    assert s['nearest_range_m'] is None
    assert s['nearest_dynamic_range_m'] is None

def test_summarize_nearest_dynamic():
    raw = [
        RawSpatialDetection(1, 0.9, 0.0, 0.0, 3.0),    # person @ ~3m dynamic
        RawSpatialDetection(4, 0.9, 0.0, 0.0, 1.0),    # chair @ ~1m static
    ]
    out = process_detections(
        raw, LABELS, dynamic_labels=DYNAMIC, static_labels=['chair'])
    s = summarize(out)
    assert s['count'] == 2
    assert s['dynamic_count'] == 1
    # en yakin nesne chair (~1m), en yakin DINAMIK person (~3m)
    assert s['nearest_range_m'] < s['nearest_dynamic_range_m']
