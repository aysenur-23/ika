"""hazard_fusion cekirdegi icin birim testler.

Terrain + dinamik nesne fuzyon karari ve detection->grid donusumu dogrulanir.
ROS gerektirmez.
"""
from ika_fusion.hazard_fusion import (
    DetectedObject,
    decision_payload,
    detections_to_grid,
    fuse_hazard,
)


def _person(range_m, x=None):
    x = range_m if x is None else x
    return DetectedObject(x=x, y=0.0, hazard='DYNAMIC', label='person',
                          confidence=0.9, range_m=range_m)


def _chair(range_m):
    return DetectedObject(x=range_m, y=0.0, hazard='STATIC', label='chair',
                          confidence=0.9, range_m=range_m)


# -----------------------------------------------------------------------
# fuse_hazard - terrain
# -----------------------------------------------------------------------
def test_clear_when_nothing():
    d = fuse_hazard('SAFE', [])
    assert d.action == 'CLEAR'
    assert d.sources == []

def test_terrain_stop():
    d = fuse_hazard('DROPOFF_DANGER', [])
    assert d.action == 'STOP'
    assert 'terrain' in d.sources

def test_terrain_slow():
    d = fuse_hazard('CAUTION', [])
    assert d.action == 'SLOW'
    assert 'terrain' in d.sources


# -----------------------------------------------------------------------
# fuse_hazard - dinamik nesne
# -----------------------------------------------------------------------
def test_dynamic_close_stops():
    d = fuse_hazard('SAFE', [_person(0.5)], dynamic_stop_range_m=0.8)
    assert d.action == 'STOP'
    assert 'dynamic_object' in d.sources
    assert d.dynamic_count == 1

def test_dynamic_mid_slows():
    d = fuse_hazard('SAFE', [_person(1.5)],
                    dynamic_stop_range_m=0.8, dynamic_slow_range_m=2.0)
    assert d.action == 'SLOW'

def test_dynamic_far_clear():
    d = fuse_hazard('SAFE', [_person(5.0)],
                    dynamic_stop_range_m=0.8, dynamic_slow_range_m=2.0)
    assert d.action == 'CLEAR'

def test_static_object_does_not_trigger_dynamic_rule():
    # Statik nesne yakin olsa bile dinamik kural tetiklenmez (costmap isi)
    d = fuse_hazard('SAFE', [_chair(0.3)], dynamic_stop_range_m=0.8)
    assert d.action == 'CLEAR'
    assert d.dynamic_count == 0


# -----------------------------------------------------------------------
# fuse_hazard - kombinasyon / oncelik
# -----------------------------------------------------------------------
def test_stop_dominates_slow():
    # terrain SLOW + dinamik STOP -> STOP
    d = fuse_hazard('CAUTION', [_person(0.5)], dynamic_stop_range_m=0.8)
    assert d.action == 'STOP'
    assert set(d.sources) == {'terrain', 'dynamic_object'}

def test_nearest_dynamic_reported():
    d = fuse_hazard('SAFE', [_person(3.0), _person(1.2)],
                    dynamic_slow_range_m=2.0)
    assert abs(d.nearest_dynamic_range_m - 1.2) < 1e-6
    assert d.action == 'SLOW'


# -----------------------------------------------------------------------
# decision_payload
# -----------------------------------------------------------------------
def test_payload_no_dynamic_is_none():
    p = decision_payload(fuse_hazard('SAFE', []))
    assert p['action'] == 'CLEAR'
    assert p['nearest_dynamic_range_m'] is None

def test_payload_serializable():
    import json
    p = decision_payload(fuse_hazard('CAUTION', [_person(0.5)]))
    json.dumps(p)  # raise etmemeli
    assert p['action'] == 'STOP'


# -----------------------------------------------------------------------
# detections_to_grid
# -----------------------------------------------------------------------
def test_grid_dimensions():
    data, meta = detections_to_grid([], size_cells=40, resolution=0.05)
    assert len(data) == 40 * 40
    assert meta['size_cells'] == 40
    assert abs(meta['origin_x'] - (-1.0)) < 1e-9   # -(40*0.05)/2

def test_grid_empty_is_all_zero():
    data, _ = detections_to_grid([], size_cells=20)
    assert set(data) == {0}

def test_grid_marks_detection_cell():
    # base_link'te tam onde 1 m bir person; grid merkezli, 0.15 m disk
    det = _person(1.0)
    data, meta = detections_to_grid([det], size_cells=80, resolution=0.05,
                                    obstacle_radius_m=0.15, dynamic_cost=100)
    n = meta['size_cells']
    res = meta['resolution']
    col = int((det.x - meta['origin_x']) / res)
    row = int((det.y - meta['origin_y']) / res)
    assert data[row * n + col] == 100
    # En az birkac komsu hucre de isaretlenmeli (disk)
    assert sum(1 for v in data if v == 100) > 1

def test_grid_out_of_range_ignored():
    far = DetectedObject(x=50.0, y=0.0, hazard='DYNAMIC', range_m=50.0)
    data, _ = detections_to_grid([far], size_cells=40, resolution=0.05)
    assert set(data) == {0}
