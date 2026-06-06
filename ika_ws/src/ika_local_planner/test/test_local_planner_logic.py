"""local_planner_logic unit testleri."""
from __future__ import annotations

import math

from ika_local_planner.local_costmap import (
    CostmapConfig, Detection, build_costmap_from_scan, overlay_detections,
)
from ika_local_planner.semantic_policy import (
    BehaviorMode, BehaviorDecision, select_behavior,
)
from ika_local_planner.local_planner_logic import (
    Pose2D, Waypoint, PlannerConfig, LocalPlan,
    plan_local_waypoint, score_candidate_corridors, choose_bypass_side,
    is_path_blocked,
)


_CFG_CM = CostmapConfig(width_m=4.0, height_m=4.0, resolution_m=0.10,
                        inflation_radius_m=0.20)
_CFG_PL = PlannerConfig(lookahead_m=1.2,
                        lateral_offsets=(-1.2, -0.6, 0.0, 0.6, 1.2))


def _empty_cm():
    return build_costmap_from_scan([], -math.pi, math.radians(1.0), _CFG_CM)


def _scan_with_obstacle(at_x: float, at_y: float = 0.0,
                        rng: float = None) -> list:
    """Tek ışın engelli: at_x, at_y'ye karşılık gelen ışın."""
    rs = [10.0] * 360
    inc = math.radians(1.0)
    ang = math.atan2(at_y, at_x)
    if rng is None:
        rng = math.hypot(at_x, at_y)
    idx = int(round((ang - (-math.pi)) / inc)) % 360
    rs[idx] = rng
    return rs


# ─── Açık yol → düz local waypoint ────────────────────────────────────

def test_drive_clear_path_returns_forward_waypoint():
    cm = _empty_cm()
    pose = Pose2D(0.0, 0.0, 0.0)
    target = Waypoint(5.0, 0.0)
    decision = BehaviorDecision(mode=BehaviorMode.DRIVE, speed_scale=1.0)
    plan = plan_local_waypoint(pose, target, cm, decision, _CFG_PL)
    assert plan.success is True
    assert plan.mode == BehaviorMode.DRIVE.value
    assert plan.local_waypoint is not None
    assert plan.local_waypoint.x > 0.5
    assert abs(plan.local_waypoint.y) < 0.1
    assert plan.speed_mps > 0.15


# ─── Önde engel + preferred='left' → sol bypass ─────────────────────

def test_front_obstacle_left_preference_bypass_left():
    # Tam önde engel — merkez candidate blocked, ikisi de tarafta açık
    rs = _scan_with_obstacle(at_x=1.0, at_y=0.0)
    cm = build_costmap_from_scan(rs, -math.pi, math.radians(1.0), _CFG_CM)
    pose = Pose2D(0.0, 0.0, 0.0)
    target = Waypoint(5.0, 0.0)
    decision = BehaviorDecision(mode=BehaviorMode.GENERIC_BYPASS,
                                preferred_side='left', speed_scale=0.8)
    plan = plan_local_waypoint(pose, target, cm, decision, _CFG_PL)
    assert plan.success
    # Robot frame'de y > 0 olmalı (sol). pose.yaw=0 → world.y == robot.y
    assert plan.local_waypoint.y > 0.05


# ─── Önde engel + preferred='right' → sağ bypass ────────────────────

def test_front_obstacle_right_preference_bypass_right():
    rs = _scan_with_obstacle(at_x=1.0, at_y=0.0)
    cm = build_costmap_from_scan(rs, -math.pi, math.radians(1.0), _CFG_CM)
    pose = Pose2D(0.0, 0.0, 0.0)
    target = Waypoint(5.0, 0.0)
    decision = BehaviorDecision(mode=BehaviorMode.GENERIC_BYPASS,
                                preferred_side='right', speed_scale=0.8)
    plan = plan_local_waypoint(pose, target, cm, decision, _CFG_PL)
    assert plan.success
    assert plan.local_waypoint.y < -0.05


# ─── Tüm yönler kapalı → HOLD ────────────────────────────────────────

def test_all_blocked_yields_hold():
    # Hem ön hem yanlar engelli — birçok ışın
    rs = [10.0] * 360
    inc = math.radians(1.0)
    # -45..+45 derece arası geniş cephe yakın engel
    for d in range(-50, 51, 2):
        idx = int(round((math.radians(d) - (-math.pi)) / inc)) % 360
        rs[idx] = 0.8
    # Yanlar da 1m'de engelli
    for d in list(range(60, 121, 2)) + list(range(-120, -59, 2)):
        idx = int(round((math.radians(d) - (-math.pi)) / inc)) % 360
        rs[idx] = 1.0
    cm = build_costmap_from_scan(rs, -math.pi, inc, _CFG_CM)
    pose = Pose2D(0.0, 0.0, 0.0)
    target = Waypoint(5.0, 0.0)
    decision = BehaviorDecision(mode=BehaviorMode.GENERIC_BYPASS)
    plan = plan_local_waypoint(pose, target, cm, decision, _CFG_PL)
    assert plan.success is False
    assert plan.mode == BehaviorMode.HOLD.value
    assert plan.speed_mps == 0.0


# ─── HOLD davranışı doğrudan ────────────────────────────────────────

def test_hold_decision_returns_zero_plan():
    cm = _empty_cm()
    pose = Pose2D(0.0, 0.0, 0.0)
    decision = BehaviorDecision(mode=BehaviorMode.HOLD, speed_scale=0.0)
    plan = plan_local_waypoint(pose, Waypoint(5.0, 0.0), cm,
                                decision, _CFG_PL)
    assert plan.success is False
    assert plan.speed_mps == 0.0


# ─── Pedestrian (STOP_AND_BYPASS) → düşük hız ──────────────────────

def test_pedestrian_behavior_yields_low_speed():
    rs = _scan_with_obstacle(at_x=1.0, at_y=-0.3)
    cm = build_costmap_from_scan(rs, -math.pi, math.radians(1.0), _CFG_CM)
    dets = [Detection(class_id='person', x=1.0, y=-0.3)]
    overlay_detections(cm, dets, semantic_weights={'person': 0.9})
    pose = Pose2D(0.0, 0.0, 0.0)
    decision = BehaviorDecision(mode=BehaviorMode.STOP_AND_BYPASS,
                                speed_scale=0.4, hold_time_s=1.5)
    plan = plan_local_waypoint(pose, Waypoint(5.0, 0.0), cm,
                                decision, _CFG_PL)
    assert plan.success
    assert plan.speed_mps <= _CFG_PL.slow_speed_mps + 1e-6


# ─── Unknown obstacle generic bypass ───────────────────────────────

def test_unknown_obstacle_generic_bypass():
    cm = _empty_cm()
    dets = [Detection(class_id='alien', x=1.0, y=0.0)]
    overlay_detections(cm, dets, semantic_weights={})
    decision = BehaviorDecision(mode=BehaviorMode.GENERIC_BYPASS,
                                speed_scale=0.8)
    pose = Pose2D(0.0, 0.0, 0.0)
    plan = plan_local_waypoint(pose, Waypoint(5.0, 0.0), cm,
                                decision, _CFG_PL)
    assert plan.success
    assert plan.mode == BehaviorMode.GENERIC_BYPASS.value


# ─── Local waypoint güvenli bölgede ────────────────────────────────

def test_chosen_waypoint_below_safety_threshold():
    rs = _scan_with_obstacle(at_x=1.0, at_y=0.0)
    cm = build_costmap_from_scan(rs, -math.pi, math.radians(1.0), _CFG_CM)
    decision = BehaviorDecision(mode=BehaviorMode.GENERIC_BYPASS,
                                preferred_side='auto', speed_scale=0.8)
    pose = Pose2D(0.0, 0.0, 0.0)
    plan = plan_local_waypoint(pose, Waypoint(5.0, 0.0), cm,
                                decision, _CFG_PL)
    if plan.success:
        from ika_local_planner.local_costmap import query_cost
        # local waypoint dünya = robot (pose 0)
        c = query_cost(cm, plan.local_waypoint.x, plan.local_waypoint.y)
        assert c < _CFG_PL.safety_cost_threshold


# ─── is_path_blocked sanity ────────────────────────────────────────

def test_is_path_blocked_clear_returns_false():
    cm = _empty_cm()
    assert is_path_blocked(cm, 0.0, 0.0, 1.0, 0.0) is False


def test_is_path_blocked_through_obstacle_returns_true():
    rs = _scan_with_obstacle(at_x=1.0, at_y=0.0)
    cm = build_costmap_from_scan(rs, -math.pi, math.radians(1.0), _CFG_CM)
    assert is_path_blocked(cm, 0.0, 0.0, 1.5, 0.0) is True


# ─── score_candidate_corridors sanity ───────────────────────────────

def test_score_candidates_blocked_flag():
    rs = _scan_with_obstacle(at_x=1.2, at_y=0.0)
    cm = build_costmap_from_scan(rs, -math.pi, math.radians(1.0), _CFG_CM)
    scored = score_candidate_corridors(
        cm, lookahead=1.2,
        lateral_offsets=(-1.0, 0.0, 1.0),
        target_offset_y=0.0,
        config=_CFG_PL,
    )
    # Ortadaki (0.0) blocked olmalı; uçlar değil
    by_off = {off: (s, mx, b) for off, s, mx, b in scored}
    assert by_off[0.0][2] is True
    assert by_off[1.0][2] is False or by_off[-1.0][2] is False


def test_choose_bypass_side_returns_none_when_all_blocked():
    candidates = [(-0.5, 5.0, 1.0, True),
                  (0.0, 4.0, 1.0, True),
                  (0.5, 6.0, 1.0, True)]
    assert choose_bypass_side(candidates) is None


def test_choose_bypass_side_left_preference():
    candidates = [(-0.5, 1.0, 0.0, False),  # sağ, skor 1
                  (0.5, 1.0, 0.0, False)]   # sol, skor 1
    best = choose_bypass_side(candidates, preferred_side='left')
    assert best is not None and best[0] == 0.5


# ─── TASK-4B-2: hysteresis ────────────────────────────────────────────

def test_hysteresis_bonus_for_same_offset():
    from ika_local_planner.local_planner_logic import score_candidate_corridors
    cm = _empty_cm()
    cfg = PlannerConfig(
        lateral_offsets=(-1.0, 0.0, 1.0),
        hysteresis_bonus=0.5, hysteresis_distance_penalty=0.0,
    )
    scored_no_hist = score_candidate_corridors(
        cm, lookahead=1.0,
        lateral_offsets=cfg.lateral_offsets,
        target_offset_y=0.0, config=cfg, last_offset_y=None,
    )
    scored_hist = score_candidate_corridors(
        cm, lookahead=1.0,
        lateral_offsets=cfg.lateral_offsets,
        target_offset_y=0.0, config=cfg, last_offset_y=1.0,
    )
    # Hysteresis altında off=1.0 skoru 0.5 düşmüş olmalı
    by_off_no = {c[0]: c[1] for c in scored_no_hist}
    by_off = {c[0]: c[1] for c in scored_hist}
    assert by_off[1.0] < by_off_no[1.0] - 0.4


def test_hysteresis_distance_penalty_for_far_offset():
    from ika_local_planner.local_planner_logic import score_candidate_corridors
    cm = _empty_cm()
    cfg = PlannerConfig(
        lateral_offsets=(-1.0, 0.0, 1.0),
        hysteresis_bonus=0.0, hysteresis_distance_penalty=0.5,
    )
    scored = score_candidate_corridors(
        cm, lookahead=1.0,
        lateral_offsets=cfg.lateral_offsets,
        target_offset_y=0.0, config=cfg, last_offset_y=1.0,
    )
    by_off = {c[0]: c[1] for c in scored}
    # Uzaktaki off=-1.0, |off - last| = 2 → penalty 1.0 ekleniyor
    assert by_off[-1.0] > by_off[1.0] + 0.9


def test_switch_margin_keeps_last_when_marginally_better():
    """Yeni best, last_offset'ten switch_margin kadar daha iyi DEĞİLse last kalır.

    Kural: best.score + switch_margin > last.score → KEEP last.
    Yani yeni best, last'tan en az `switch_margin` kadar daha iyi olmalı.
    """
    cm = _empty_cm()
    cfg = PlannerConfig(
        lateral_offsets=(-0.6, 0.0, 0.6),
        hysteresis_bonus=0.0, hysteresis_distance_penalty=0.0,
        switch_margin=0.50,  # büyük margin → 0.5 fark gerekir
    )
    decision = BehaviorDecision(mode=BehaviorMode.GENERIC_BYPASS,
                                preferred_side='auto', speed_scale=0.8)
    pose = Pose2D(0.0, 0.0, 0.0)
    # Boş costmap → off=0 score=0, off=±0.6 score=0.3 (alignment 0.5*0.6).
    # Yeni best=off=0 last_c=off=0.6 (score=0.3); margin 0.5 → 0+0.5>0.3 True
    # → KEEP last_c (off=0.6).
    plan = plan_local_waypoint(
        pose, Waypoint(5.0, 0.0), cm, decision, cfg,
        last_offset_y=0.6,
    )
    assert plan.success
    assert plan.local_waypoint.y > 0.4  # ≈ last offset (0.6)


def test_switch_margin_switches_when_clearly_better():
    """Yeni best, switch_margin'den fazla daha iyiyse switch olur."""
    cm = _empty_cm()
    cfg = PlannerConfig(
        lateral_offsets=(-0.6, 0.0, 0.6),
        hysteresis_bonus=0.0, hysteresis_distance_penalty=0.0,
        switch_margin=0.10,  # küçük margin
    )
    decision = BehaviorDecision(mode=BehaviorMode.GENERIC_BYPASS,
                                preferred_side='auto', speed_scale=0.8)
    pose = Pose2D(0.0, 0.0, 0.0)
    # off=0 score=0, off=0.6 score=0.3. Margin 0.1 → 0+0.1>0.3 False → SWITCH
    plan = plan_local_waypoint(
        pose, Waypoint(5.0, 0.0), cm, decision, cfg,
        last_offset_y=0.6,
    )
    assert plan.success
    assert abs(plan.local_waypoint.y) < 0.1  # ≈ off=0


def test_forced_side_overrides_preferred():
    """forced_side verildiğinde decision.preferred_side ignore edilir."""
    # Sağ tarafa engel koyalım, ortayı blokla → sağ candidate blocked
    rs = _scan_with_obstacle(at_x=1.0, at_y=0.0)
    cm = build_costmap_from_scan(rs, -math.pi, math.radians(1.0), _CFG_CM)
    decision = BehaviorDecision(mode=BehaviorMode.GENERIC_BYPASS,
                                preferred_side='left', speed_scale=0.8)
    pose = Pose2D(0.0, 0.0, 0.0)
    plan_left = plan_local_waypoint(
        pose, Waypoint(5.0, 0.0), cm, decision, _CFG_PL,
        forced_side='right',
    )
    assert plan_left.success
    assert plan_left.local_waypoint.y < -0.05  # forced right'a uydu
