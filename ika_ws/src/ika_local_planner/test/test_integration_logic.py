"""Integration testleri — modüllerin uçtan-uca çalıştığını doğrula."""
from __future__ import annotations

import math

from ika_local_planner.local_costmap import (
    CostmapConfig, Detection,
    build_costmap_from_scan, overlay_detections, summarize_costmap,
)
from ika_local_planner.semantic_policy import (
    BehaviorMode, select_behavior,
)
from ika_local_planner.local_planner_logic import (
    Pose2D, Waypoint, PlannerConfig, plan_local_waypoint,
)
from ika_local_planner.path_rejoin import (
    RejoinConfig, compute_rejoin_command,
)


_CFG_CM = CostmapConfig(width_m=4.0, height_m=4.0, resolution_m=0.10,
                        inflation_radius_m=0.20)
_CFG_PL = PlannerConfig(lookahead_m=1.2,
                        lateral_offsets=(-1.2, -0.6, 0.0, 0.6, 1.2))


def _scan_with_obstacle(at_x: float, at_y: float = 0.0) -> list:
    rs = [10.0] * 360
    inc = math.radians(1.0)
    ang = math.atan2(at_y, at_x)
    idx = int(round((ang - (-math.pi)) / inc)) % 360
    rs[idx] = math.hypot(at_x, at_y)
    return rs


# ─── Senaryo 1: lidar engel + unknown detection → generic bypass ────

def test_scenario_lidar_obstacle_plus_unknown_detection_generic_bypass():
    rs = _scan_with_obstacle(at_x=1.0, at_y=0.0)
    cm = build_costmap_from_scan(rs, -math.pi, math.radians(1.0), _CFG_CM)
    overlay_detections(cm, [Detection(class_id='alien_widget',
                                       x=1.0, y=0.0)],
                       semantic_weights={})
    summary = summarize_costmap(cm)
    assert summary['front_blocked'] is True

    decision = select_behavior(
        detections=[Detection(class_id='alien_widget', x=1.0, y=0.0)],
        costmap_summary=summary,
    )
    assert decision.mode == BehaviorMode.GENERIC_BYPASS

    pose = Pose2D(0.0, 0.0, 0.0)
    plan = plan_local_waypoint(pose, Waypoint(5.0, 0.0), cm,
                                decision, _CFG_PL)
    assert plan.success
    # Robot sapacak (lateral offset ≠ 0)
    assert abs(plan.local_waypoint.y) > 0.1
    assert plan.rejoin_required is True


# ─── Senaryo 2: pedestrian → stop_and_bypass + düşük hız ────────────

def test_scenario_pedestrian_low_speed_bypass():
    rs = _scan_with_obstacle(at_x=1.2, at_y=-0.3)
    cm = build_costmap_from_scan(rs, -math.pi, math.radians(1.0), _CFG_CM)
    dets = [Detection(class_id='person', x=1.2, y=-0.3)]
    overlay_detections(cm, dets, semantic_weights={'person': 0.9})
    summary = summarize_costmap(cm)
    decision = select_behavior(detections=dets, costmap_summary=summary)
    assert decision.mode == BehaviorMode.STOP_AND_BYPASS
    assert decision.speed_scale <= 0.5

    pose = Pose2D(0.0, 0.0, 0.0)
    plan = plan_local_waypoint(pose, Waypoint(5.0, 0.0), cm,
                                decision, _CFG_PL)
    assert plan.success
    assert plan.speed_mps <= _CFG_PL.slow_speed_mps + 1e-6


# ─── Senaryo 3: corridor detection → corridor follow ────────────────

def test_scenario_corridor_follow():
    # Çift duvar — y=+1 ve y=-1'de duvarlar
    rs = [10.0] * 360
    inc = math.radians(1.0)
    for d_deg in range(45, 80):
        idx = int(round((math.radians(d_deg) - (-math.pi)) / inc)) % 360
        rs[idx] = 1.0
    for d_deg in range(-80, -44):
        idx = int(round((math.radians(d_deg) - (-math.pi)) / inc)) % 360
        rs[idx] = 1.0
    cm = build_costmap_from_scan(rs, -math.pi, inc, _CFG_CM)
    dets = [Detection(class_id='wall', x=0.5, y=1.0),
            Detection(class_id='wall', x=0.5, y=-1.0)]
    summary = summarize_costmap(cm)
    decision = select_behavior(detections=dets, costmap_summary=summary)
    assert decision.mode == BehaviorMode.CORRIDOR_FOLLOW

    pose = Pose2D(0.0, 0.0, 0.0)
    plan = plan_local_waypoint(pose, Waypoint(5.0, 0.0), cm,
                                decision, _CFG_PL)
    assert plan.success
    # Koridor merkezini takip et (y ≈ 0)
    assert abs(plan.local_waypoint.y) < 0.1


# ─── Senaryo 4: bypass sonrası rejoin ana hatta döndürür ──────────

def test_scenario_bypass_then_rejoin():
    cm = build_costmap_from_scan([], -math.pi, math.radians(1.0), _CFG_CM)
    pose_after_bypass = Pose2D(2.0, 0.6, 0.05)  # robot 60cm sola sapmış
    cmd = compute_rejoin_command(pose_after_bypass,
                                  path_y=0.0, target_heading=0.0)
    assert cmd.done is False
    # path y=0, robot y=0.6 → y_err = -0.6 → sağa dönmeli (negatif angular)
    assert cmd.angular_z < 0.0
    # Bir sonraki adımda robot y=0.1'e gelmiş varsayalım
    pose_close = Pose2D(2.5, 0.05, 0.03)
    cmd2 = compute_rejoin_command(pose_close,
                                   path_y=0.0, target_heading=0.0)
    assert cmd2.done is True


# ─── Senaryo 5: tüm path kapalı → HOLD ─────────────────────────────

def test_scenario_all_blocked_returns_hold():
    rs = [10.0] * 360
    inc = math.radians(1.0)
    # Geniş cephe yakın engel
    for d in range(-90, 91, 2):
        idx = int(round((math.radians(d) - (-math.pi)) / inc)) % 360
        rs[idx] = 0.6
    cm = build_costmap_from_scan(rs, -math.pi, inc, _CFG_CM)
    summary = summarize_costmap(cm)
    decision = select_behavior(detections=[], costmap_summary=summary)
    # Tüm yanlar bloklu → HOLD ya da generic_bypass; HOLD ise zaten beklenti
    assert decision.mode in (BehaviorMode.HOLD, BehaviorMode.GENERIC_BYPASS)

    pose = Pose2D(0.0, 0.0, 0.0)
    plan = plan_local_waypoint(pose, Waypoint(5.0, 0.0), cm,
                                decision, _CFG_PL)
    assert plan.success is False
    assert plan.mode == BehaviorMode.HOLD.value
    assert plan.speed_mps == 0.0
