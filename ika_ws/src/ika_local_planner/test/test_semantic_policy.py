"""semantic_policy unit testleri."""
from __future__ import annotations

from ika_local_planner.local_costmap import Detection
from ika_local_planner.semantic_policy import (
    BehaviorMode, BehaviorDecision, select_behavior,
)


def _summary(front=False, left=False, right=False, min_d=-1.0):
    return {
        'front_blocked': front, 'left_blocked': left,
        'right_blocked': right, 'min_obs_dist': min_d,
    }


# ─── Defansif sınıflar ────────────────────────────────────────────────

def test_person_yields_stop_and_bypass():
    dets = [Detection(class_id='person', x=1.0, y=0.0)]
    d = select_behavior(dets, _summary())
    assert d.mode == BehaviorMode.STOP_AND_BYPASS
    assert d.speed_scale < 0.6
    assert d.hold_time_s > 0.0
    assert d.target_class == 'person'


def test_pedestrian_class_normalized_to_stop_and_bypass():
    dets = [Detection(class_id='pedestrian:DYNAMIC', x=1.0, y=0.0)]
    d = select_behavior(dets, _summary())
    assert d.mode == BehaviorMode.STOP_AND_BYPASS


def test_pothole_yields_slow_check():
    dets = [Detection(class_id='pothole', x=1.0, y=0.0)]
    d = select_behavior(dets, _summary())
    assert d.mode == BehaviorMode.SLOW_CHECK_AND_BYPASS
    assert d.speed_scale < 0.7


def test_ramp_yields_ramp_align():
    dets = [Detection(class_id='ramp', x=1.0, y=0.0)]
    d = select_behavior(dets, _summary())
    assert d.mode == BehaviorMode.RAMP_ALIGN


def test_corridor_yields_corridor_follow():
    dets = [Detection(class_id='wall', x=1.0, y=1.0),
            Detection(class_id='wall', x=1.0, y=-1.0)]
    d = select_behavior(dets, _summary())
    assert d.mode == BehaviorMode.CORRIDOR_FOLLOW


# ─── Bilinmeyen + sadece costmap ───────────────────────────────────────

def test_unknown_class_yields_generic_bypass():
    dets = [Detection(class_id='alien_widget', x=1.0, y=0.0)]
    d = select_behavior(dets, _summary())
    assert d.mode == BehaviorMode.GENERIC_BYPASS
    assert d.target_class == 'alien_widget'


def test_no_detection_but_costmap_front_blocked_generic_bypass():
    d = select_behavior(detections=[], costmap_summary=_summary(front=True))
    assert d.mode == BehaviorMode.GENERIC_BYPASS
    assert d.target_class == 'unknown'


def test_no_detection_costmap_open_drive():
    d = select_behavior(detections=[], costmap_summary=_summary())
    assert d.mode == BehaviorMode.DRIVE
    assert d.speed_scale == 1.0


# ─── Çok kaynaklı önceliklendirme ──────────────────────────────────────

def test_priority_defensive_wins():
    dets = [
        Detection(class_id='box', x=1.0, y=0.5),         # GENERIC_BYPASS
        Detection(class_id='pothole', x=1.5, y=0.0),     # SLOW_CHECK
        Detection(class_id='person', x=2.0, y=-0.5),     # STOP_AND_BYPASS
    ]
    d = select_behavior(dets, _summary())
    assert d.mode == BehaviorMode.STOP_AND_BYPASS


# ─── Tüm yan + ön kapalı → HOLD ────────────────────────────────────────

def test_all_blocked_yields_hold():
    d = select_behavior([], _summary(front=True, left=True, right=True))
    assert d.mode == BehaviorMode.HOLD
    assert d.speed_scale == 0.0


# ─── Tercih edilen yan ────────────────────────────────────────────────

def test_preferred_side_right_when_left_blocked():
    dets = [Detection(class_id='box', x=1.0, y=0.0)]
    d = select_behavior(dets, _summary(left=True))
    assert d.preferred_side == 'right'


def test_preferred_side_left_when_right_blocked():
    dets = [Detection(class_id='box', x=1.0, y=0.0)]
    d = select_behavior(dets, _summary(right=True))
    assert d.preferred_side == 'left'


def test_preferred_side_auto_when_both_open():
    dets = [Detection(class_id='box', x=1.0, y=0.0)]
    d = select_behavior(dets, _summary())
    assert d.preferred_side == 'auto'


def test_ground_patch_does_not_alarm():
    dets = [Detection(class_id='ground_patch', x=1.0, y=0.0)]
    d = select_behavior(dets, _summary())
    # Sadece DRIVE bekleniyor (yer örtüsü uyarmamalı)
    assert d.mode == BehaviorMode.DRIVE


def test_empty_class_id_yields_generic_bypass():
    class FakeDet:
        class_id = ''
        x, y = 1.0, 0.0
    d = select_behavior([FakeDet()], _summary())
    assert d.mode == BehaviorMode.GENERIC_BYPASS
