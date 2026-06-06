"""local_costmap unit testleri — saf-Python, ROS bağımsız."""
from __future__ import annotations

import math

from ika_local_planner.local_costmap import (
    CostmapConfig, Detection, LocalCostmap,
    build_costmap_from_scan, overlay_detections, query_cost,
    is_occupied, find_free_lateral_gaps, summarize_costmap,
)


# Standart test config: 4m × 4m, 10cm çözünürlük
_CFG = CostmapConfig(width_m=4.0, height_m=4.0, resolution_m=0.10,
                     inflation_radius_m=0.20)


def _scan_360(n: int = 360, dist: float = 10.0):
    return [dist] * n


# ─── Boş scan ────────────────────────────────────────────────────────────

def test_empty_scan_returns_empty_grid():
    cm = build_costmap_from_scan([], -math.pi, math.radians(1.0), _CFG)
    assert cm.nx == 40 and cm.ny == 40
    assert cm.max_cost() == 0.0


def test_all_inf_scan_low_costmap():
    # Tüm ışın ufukta → grid içine düşen hücre yok
    rs = [10.0] * 360  # 10m, grid 4m × 4m sınırını aşar
    cm = build_costmap_from_scan(rs, -math.pi, math.radians(1.0), _CFG)
    assert cm.max_cost() == 0.0


# ─── Tek engel + inflation ──────────────────────────────────────────────

def test_single_obstacle_high_cost_at_location():
    # 1m ileri, 0° → x=1, y=0
    rs = [10.0] * 360
    idx_forward = int(round((0.0 - (-math.pi)) / math.radians(1.0)))
    rs[idx_forward] = 1.0
    cm = build_costmap_from_scan(rs, -math.pi, math.radians(1.0), _CFG)
    assert is_occupied(cm, 1.0, 0.0, threshold=0.9)
    # Engel olmayan uzak nokta düşük olmalı
    assert query_cost(cm, 3.5, 1.5) < 0.5


def test_inflation_spreads_around_obstacle():
    rs = [10.0] * 360
    idx_forward = int(round((0.0 - (-math.pi)) / math.radians(1.0)))
    rs[idx_forward] = 1.5  # x=1.5m, y=0
    cm = build_costmap_from_scan(rs, -math.pi, math.radians(1.0), _CFG)
    # 0.10m yan = inflation içinde, hâlâ cost > 0
    cost_near = query_cost(cm, 1.5, 0.10)
    cost_far = query_cost(cm, 1.5, 0.80)
    assert cost_near > 0.0
    assert cost_near > cost_far


# ─── Invalid range filtreleme ───────────────────────────────────────────

def test_invalid_ranges_are_filtered():
    rs = [10.0] * 360
    # Geçersiz değerleri ön sektöre koy — costmap'e işlenmemeli
    idx_forward = 180
    rs[idx_forward - 2] = 0.0
    rs[idx_forward - 1] = -1.0
    rs[idx_forward] = float('inf')
    rs[idx_forward + 1] = float('nan')
    cm = build_costmap_from_scan(rs, -math.pi, math.radians(1.0), _CFG)
    assert cm.max_cost() == 0.0


# ─── Detection overlay ─────────────────────────────────────────────────

def test_detection_overlay_increases_cost():
    cm = build_costmap_from_scan([], -math.pi, math.radians(1.0), _CFG)
    assert cm.max_cost() == 0.0
    dets = [Detection(class_id='person', x=1.0, y=0.5, confidence=1.0)]
    cm2 = overlay_detections(cm, dets, semantic_weights={'person': 0.9})
    assert query_cost(cm2, 1.0, 0.5) >= 0.8


def test_unknown_detection_uses_default_unknown_cost():
    cm = build_costmap_from_scan([], -math.pi, math.radians(1.0), _CFG)
    dets = [Detection(class_id='alien_widget', x=1.5, y=-0.5)]
    overlay_detections(cm, dets, semantic_weights={'person': 0.9})
    # alien_widget known weight yok → unknown_cost (default 0.5) civarı
    c = query_cost(cm, 1.5, -0.5)
    assert c >= 0.4 and c <= 0.6


def test_overlay_skips_out_of_grid_detections():
    cm = build_costmap_from_scan([], -math.pi, math.radians(1.0), _CFG)
    # Grid 4m × 4m → (5,0) dışarıda
    dets = [Detection(class_id='box', x=5.0, y=0.0)]
    overlay_detections(cm, dets, semantic_weights={'box': 1.0})
    assert cm.max_cost() == 0.0


def test_overlay_filters_non_finite_coords():
    cm = build_costmap_from_scan([], -math.pi, math.radians(1.0), _CFG)
    dets = [
        Detection(class_id='box', x=float('nan'), y=0.0),
        Detection(class_id='box', x=1.0, y=float('inf')),
    ]
    overlay_detections(cm, dets, semantic_weights={'box': 1.0})
    assert cm.max_cost() == 0.0


# ─── Free lateral gaps ─────────────────────────────────────────────────

def test_free_gaps_full_open_returns_single_wide_gap():
    cm = build_costmap_from_scan([], -math.pi, math.radians(1.0), _CFG)
    gaps = find_free_lateral_gaps(cm, lookahead_x=1.5,
                                   corridor_width=0.4)
    assert len(gaps) == 1
    y0, y1 = gaps[0]
    assert (y1 - y0) >= 3.0  # ~4m'lik tam genişlik


def test_free_gaps_central_obstacle_splits():
    # x=1.5'te merkez engeli (y≈0) → iki gap
    rs = [10.0] * 360
    inc = math.radians(1.0)
    idx_forward = int(round((0.0 - (-math.pi)) / inc))
    rs[idx_forward] = 1.5
    cm = build_costmap_from_scan(rs, -math.pi, inc, _CFG)
    gaps = find_free_lateral_gaps(cm, lookahead_x=1.5,
                                   corridor_width=0.4)
    assert len(gaps) >= 2  # sol ve sağ


def test_summarize_costmap_detects_front_block():
    rs = [10.0] * 360
    inc = math.radians(1.0)
    idx_forward = int(round((0.0 - (-math.pi)) / inc))
    rs[idx_forward] = 1.0
    cm = build_costmap_from_scan(rs, -math.pi, inc, _CFG)
    s = summarize_costmap(cm, front_window_x=1.5, side_window_y=1.0)
    assert s['front_blocked'] is True
    assert s['min_obs_dist'] > 0.0 and s['min_obs_dist'] < 1.5


def test_summarize_costmap_open():
    cm = build_costmap_from_scan([], -math.pi, math.radians(1.0), _CFG)
    s = summarize_costmap(cm)
    assert s['front_blocked'] is False
    assert s['left_blocked'] is False
    assert s['right_blocked'] is False
    assert s['min_obs_dist'] == -1.0
