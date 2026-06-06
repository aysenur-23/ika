"""Goal-Aware Avoider state machine birim testleri (saf-Python, ROS yok).

Defense-in-depth Katman 2 (`docs/avoidance_architecture.md`).

State machine:
    DRIVING (goal heading'i takip et)
        -> AVOIDING (engel görünce, goal'a yakın yön)
        -> PASSING (yan geç)
        -> REALIGNING (goal heading'e dön — DİNAMİK PLAN REVİZYONU)
        -> DRIVING (tekrar goal'a yönlü)

Test kategorileri:
    1. DRIVING durumu + goal heading takibi
    2. DRIVING → AVOIDING geçişleri (lidar + camera)
    3. AVOIDING + hysteresis
    4. PASSING + defansif
    5. REALIGNING — goal heading'e dönüş
    6. DONE terminal
    7. Yardımcı fonksiyonlar
    8. Hazard ignored (CLAUDE.md kararı)
    9. Uçtan-uca senaryo
"""
from __future__ import annotations

import math
from typing import List

from ika_mission.avoider_logic import (
    AvoiderConfig, AvoiderState, AvoiderPhase,
    decide, pick_avoid_direction, pick_avoid_direction_goal_aware,
    front_min_range, side_minima, wrap_pi, HAZARD_BLOCKING,
)


# ═══════════════════════════════════════════════════════════════════════
# TASK-1: side_minima telemetri helper
# ═══════════════════════════════════════════════════════════════════════

# Standart RPLIDAR benzeri: angle_min=-pi, 1° artış, 360 ışın.
_AMIN = -math.pi
_AINC = math.radians(1.0)
_FRONT = math.radians(60.0)


def _idx_for_angle(deg: float) -> int:
    """deg derecesine en yakın index (angle_min=-pi, 1°/ışın)."""
    return int(round((math.radians(deg) - _AMIN) / _AINC)) % 360


def test_side_minima_open_scan_all_finite_equal():
    rs = [5.0] * 360
    f, l, r = side_minima(rs, _AMIN, _AINC, _FRONT)
    assert math.isclose(f, 5.0)
    assert math.isclose(l, 5.0)
    assert math.isclose(r, 5.0)


def test_side_minima_negative_angle_is_right():
    # -20°..-5° (negatif açı) = SAĞ
    rs = [5.0] * 360
    for d in range(-20, -4):
        rs[_idx_for_angle(d)] = 0.4
    f, l, r = side_minima(rs, _AMIN, _AINC, _FRONT)
    assert math.isclose(r, 0.4), "negatif açı sağa karşılık gelmeli"
    assert l > 1.0, "sol açık olmalı"
    assert math.isclose(f, 0.4)


def test_side_minima_positive_angle_is_left():
    # +5°..+20° (pozitif açı) = SOL
    rs = [5.0] * 360
    for d in range(5, 21):
        rs[_idx_for_angle(d)] = 0.3
    f, l, r = side_minima(rs, _AMIN, _AINC, _FRONT)
    assert math.isclose(l, 0.3), "pozitif açı sola karşılık gelmeli"
    assert r > 1.0, "sağ açık olmalı"
    assert math.isclose(f, 0.3)


def test_side_minima_excludes_outside_front_arc():
    # 60° ön ark → ±30°. 45° = ark DIŞI, etki etmemeli.
    rs = [5.0] * 360
    rs[_idx_for_angle(45)] = 0.1
    rs[_idx_for_angle(-45)] = 0.1
    f, l, r = side_minima(rs, _AMIN, _AINC, _FRONT)
    assert f == 5.0 and l == 5.0 and r == 5.0


def test_side_minima_filters_invalid_ranges():
    rs = [5.0] * 360
    # SAĞ tarafa (negatif açı) geçersiz + tek geçerli
    rs[_idx_for_angle(-10)] = 0.0
    rs[_idx_for_angle(-11)] = -1.0
    rs[_idx_for_angle(-12)] = float('inf')
    rs[_idx_for_angle(-13)] = float('nan')
    rs[_idx_for_angle(-14)] = 0.5
    f, l, r = side_minima(rs, _AMIN, _AINC, _FRONT)
    assert math.isclose(r, 0.5)
    assert l > 1.0


def test_side_minima_empty_returns_inf():
    f, l, r = side_minima([], _AMIN, _AINC, _FRONT)
    assert f == float('inf') and l == float('inf') and r == float('inf')


def test_side_minima_all_invalid_returns_inf():
    rs = [float('nan')] * 360
    f, l, r = side_minima(rs, _AMIN, _AINC, _FRONT)
    assert f == float('inf') and l == float('inf') and r == float('inf')


# TASK-3.1: pure-Python decide() auto_start gating'den etkilenmemeli
def test_decide_still_drives_when_clear_after_task31():
    """auto_start node-level; decide() çekirdeği değişmemiş olmalı."""
    cfg = AvoiderConfig()
    state = AvoiderState(phase=AvoiderPhase.DRIVING, goal_heading_rad=0.0)
    cmd = decide(state, [5.0] * 360, math.radians(360),
                 hazard_action="CLEAR", current_yaw=0.0,
                 odom_delta_m=0.0, cfg=cfg,
                 camera_obstacle_distance_m=float('inf'))
    assert math.isclose(cmd.linear_x, cfg.forward_speed_mps)
    assert cmd.next_state.phase == AvoiderPhase.DRIVING


def test_side_minima_wraps_pi():
    # angle_min = 0, 360 ışın, 1°/ışın → ön sektör hem 0° (sol) hem ~359° (sağ)
    rs = [5.0] * 360
    rs[_idx_for_angle(355) % 360 if False else int(round((math.radians(355) - 0.0) / _AINC))] = 0.7  # 355° → -5° wrap, SAĞ
    rs[int(round((math.radians(10) - 0.0) / _AINC))] = 0.8  # 10° SOL
    f, l, r = side_minima(rs, 0.0, _AINC, _FRONT)
    assert math.isclose(r, 0.7), "wrap_pi sonrası 355° sağ olmalı"
    assert math.isclose(l, 0.8), "10° sol olmalı"
    assert math.isclose(f, 0.7)



# ═══════════════════════════════════════════════════════════════════════
# YARDIMCILAR
# ═══════════════════════════════════════════════════════════════════════

def _open_scan(n: int = 360, dist: float = 5.0) -> List[float]:
    return [dist] * n


def _blocked_scan_front(n: int = 360, blocked_at_m: float = 0.25,
                        open_at_m: float = 5.0,
                        block_width: int = 30) -> List[float]:
    """Default 0.25 m (< 0.35 obstacle_distance threshold)."""
    rs = [open_at_m] * n
    mid = n // 2
    half = block_width // 2
    for i in range(mid - half, mid + half):
        rs[i] = blocked_at_m
    return rs


def _cfg(**kw) -> AvoiderConfig:
    return AvoiderConfig(**kw)


# ═══════════════════════════════════════════════════════════════════════
# 1. DRIVING + goal heading takibi
# ═══════════════════════════════════════════════════════════════════════

def test_driving_empty_world_drives_forward():
    s = AvoiderState(goal_heading_rad=0.0)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.linear_x > 0.0
    # yaw_err = 0, angular_z ≈ 0
    assert abs(c.angular_z) < 0.01
    assert c.next_state.phase == AvoiderPhase.DRIVING


def test_driving_no_heading_correction_in_default_config():
    """KULLANICI ISTEGI: DRIVING'de heading correction kapatildi
    (heading_kp=0). Robot saçma sapma yapmasin diye. Heading sadece
    REALIGNING'de düzeltilir."""
    s = AvoiderState(goal_heading_rad=0.0)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR",
               current_yaw=0.2, odom_delta_m=0.0, cfg=_cfg())
    # heading_kp=0 ve max_correction=0 -> angular_z = 0
    assert math.isclose(c.angular_z, 0.0)
    assert c.linear_x > 0  # ileri sürüyor


def test_driving_distance_accumulates():
    s = AvoiderState(goal_heading_rad=0.0)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.10, _cfg())
    assert math.isclose(c.next_state.distance_clear_m, 0.10)


def test_driving_to_done_after_target_distance():
    s = AvoiderState(goal_heading_rad=0.0, distance_clear_m=1.95)
    cfg = _cfg(target_distance_m=2.0)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.10, cfg)
    assert c.next_state.phase == AvoiderPhase.DONE


def test_driving_negative_odom_delta_ignored():
    s = AvoiderState(goal_heading_rad=0.0, distance_clear_m=0.5)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, -0.10, _cfg())
    assert math.isclose(c.next_state.distance_clear_m, 0.5)


# ═══════════════════════════════════════════════════════════════════════
# 2. DRIVING → AVOIDING (lidar + camera)
# ═══════════════════════════════════════════════════════════════════════

def test_driving_lidar_obstacle_triggers_avoiding():
    s = AvoiderState(goal_heading_rad=0.0)
    scan = _blocked_scan_front(blocked_at_m=0.25)  # < 0.35
    c = decide(s, scan, math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.AVOIDING
    assert c.linear_x == 0.0
    assert abs(c.angular_z) > 0.0


def test_driving_camera_obstacle_triggers_avoiding():
    """Camera DL detection 0.40m'de (< 0.50m threshold) → AVOIDING."""
    s = AvoiderState(goal_heading_rad=0.0)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.0, _cfg(),
               camera_obstacle_distance_m=0.40)
    assert c.next_state.phase == AvoiderPhase.AVOIDING


def test_driving_lidar_outside_threshold_no_trigger():
    """0.40 m > 0.35 m threshold → trigger yok."""
    s = AvoiderState(goal_heading_rad=0.0)
    scan = _blocked_scan_front(blocked_at_m=0.40)
    c = decide(s, scan, math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.DRIVING


def test_driving_hazard_does_NOT_trigger_anymore():
    """KRITIK: HAZARD_BLOCKING boş. STOP/SLOW avoider'ı tetikleMEZ."""
    s = AvoiderState(goal_heading_rad=0.0)
    c = decide(s, _open_scan(), math.radians(360), "STOP", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.DRIVING  # hala DRIVING

    c = decide(s, _open_scan(), math.radians(360), "SLOW", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.DRIVING


# ═══════════════════════════════════════════════════════════════════════
# 3. AVOIDING + hysteresis
# ═══════════════════════════════════════════════════════════════════════

def test_avoiding_keeps_turning_while_blocked():
    s = AvoiderState(phase=AvoiderPhase.AVOIDING, avoid_direction=1,
                     goal_heading_rad=0.0)
    scan = _blocked_scan_front(blocked_at_m=0.25)
    c = decide(s, scan, math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.AVOIDING
    assert c.angular_z > 0.0


def test_avoiding_to_passing_when_front_clear():
    s = AvoiderState(phase=AvoiderPhase.AVOIDING, avoid_direction=1,
                     goal_heading_rad=0.0)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.PASSING
    assert c.linear_x > 0.0


def test_avoiding_release_distance_hysteresis():
    """Hysteresis: 0.35 girer, 0.60 cikar → 0.45 hala AVOIDING."""
    s = AvoiderState(phase=AvoiderPhase.AVOIDING, avoid_direction=1,
                     goal_heading_rad=0.0)
    scan = _blocked_scan_front(blocked_at_m=0.45)
    c = decide(s, scan, math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.AVOIDING


# ═══════════════════════════════════════════════════════════════════════
# 4. PASSING + defansif
# ═══════════════════════════════════════════════════════════════════════

def test_passing_drives_forward_and_accumulates():
    s = AvoiderState(phase=AvoiderPhase.PASSING, pass_distance_m=0.0,
                     distance_clear_m=1.0, goal_heading_rad=0.0)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.05, _cfg())
    assert c.next_state.phase == AvoiderPhase.PASSING
    assert c.linear_x > 0.0
    assert math.isclose(c.next_state.pass_distance_m, 0.05)


def test_passing_to_realigning_after_clear_distance():
    cfg = _cfg(pass_clear_distance_m=0.40)
    s = AvoiderState(phase=AvoiderPhase.PASSING, pass_distance_m=0.38,
                     distance_clear_m=1.0, goal_heading_rad=0.0)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.05, cfg)
    assert c.next_state.phase == AvoiderPhase.REALIGNING


def test_passing_to_done_when_target_distance_reached():
    cfg = _cfg(target_distance_m=2.0, pass_clear_distance_m=0.50)
    s = AvoiderState(phase=AvoiderPhase.PASSING, pass_distance_m=0.10,
                     distance_clear_m=1.97, goal_heading_rad=0.0)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.05, cfg)
    assert c.next_state.phase == AvoiderPhase.DONE


def test_passing_defensive_lidar_returns_to_avoiding():
    s = AvoiderState(phase=AvoiderPhase.PASSING, pass_distance_m=0.20,
                     avoid_direction=1, goal_heading_rad=0.0)
    scan = _blocked_scan_front(blocked_at_m=0.25)
    c = decide(s, scan, math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.AVOIDING


def test_passing_hazard_does_NOT_trigger():
    """Hazard artık avoider'a etki etmez."""
    s = AvoiderState(phase=AvoiderPhase.PASSING, pass_distance_m=0.20,
                     goal_heading_rad=0.0)
    c = decide(s, _open_scan(), math.radians(360), "STOP", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.PASSING


def test_passing_camera_close_triggers_avoiding():
    """Camera < 0.50m → defansif AVOIDING."""
    s = AvoiderState(phase=AvoiderPhase.PASSING, pass_distance_m=0.20,
                     goal_heading_rad=0.0)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.0, _cfg(),
               camera_obstacle_distance_m=0.40)
    assert c.next_state.phase == AvoiderPhase.AVOIDING


# ═══════════════════════════════════════════════════════════════════════
# 5. REALIGNING → goal heading'e dönüş
# ═══════════════════════════════════════════════════════════════════════

def test_realigning_to_driving_when_yaw_aligned():
    s = AvoiderState(phase=AvoiderPhase.REALIGNING, goal_heading_rad=0.0,
                     distance_clear_m=0.7)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR",
               current_yaw=0.05, odom_delta_m=0.0, cfg=_cfg())
    # yaw_err = -0.05; |0.05| < 0.15 tolerance → DRIVING
    assert c.next_state.phase == AvoiderPhase.DRIVING
    assert math.isclose(c.next_state.distance_clear_m, 0.7)


def test_realigning_keeps_turning_if_yaw_off():
    s = AvoiderState(phase=AvoiderPhase.REALIGNING, goal_heading_rad=0.0)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR",
               current_yaw=0.5, odom_delta_m=0.0, cfg=_cfg())
    assert c.next_state.phase == AvoiderPhase.REALIGNING
    assert c.linear_x == 0.0
    assert c.angular_z != 0.0


def test_realigning_correct_turn_direction():
    """Goal 0, current -0.5 → angular > 0 (sola)."""
    cfg = _cfg()
    s = AvoiderState(phase=AvoiderPhase.REALIGNING, goal_heading_rad=0.0)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR",
               current_yaw=-0.5, odom_delta_m=0.0, cfg=cfg)
    assert c.angular_z > 0
    c = decide(s, _open_scan(), math.radians(360), "CLEAR",
               current_yaw=0.5, odom_delta_m=0.0, cfg=cfg)
    assert c.angular_z < 0


def test_realigning_defensive_obstacle_returns_to_avoiding():
    s = AvoiderState(phase=AvoiderPhase.REALIGNING, goal_heading_rad=0.0)
    scan = _blocked_scan_front(blocked_at_m=0.25)
    c = decide(s, scan, math.radians(360), "CLEAR",
               current_yaw=0.5, odom_delta_m=0.0, cfg=_cfg())
    assert c.next_state.phase == AvoiderPhase.AVOIDING


# ═══════════════════════════════════════════════════════════════════════
# 6. DONE terminal
# ═══════════════════════════════════════════════════════════════════════

def test_done_phase_stays_stopped_even_under_obstacle():
    s = AvoiderState(phase=AvoiderPhase.DONE, goal_heading_rad=0.0)
    c = decide(s, _blocked_scan_front(), math.radians(360), "STOP",
               0.0, 0.0, _cfg())
    assert c.linear_x == 0.0
    assert c.angular_z == 0.0
    assert c.next_state.phase == AvoiderPhase.DONE


# ═══════════════════════════════════════════════════════════════════════
# 7. Yardımcı fonksiyonlar — goal-aware direction
# ═══════════════════════════════════════════════════════════════════════

def test_pick_direction_goal_left_prefers_left_when_clear():
    """Goal sola (yaw_err=0.5), her iki taraf yakın eşit → goal yönü kazanır."""
    n = 60
    front = [1.0] * n  # ikisi de yakın eşit
    d = pick_avoid_direction_goal_aware(front, current_yaw=0.0,
                                        goal_heading=0.5)
    assert d == 1  # sola


def test_pick_direction_goal_right_prefers_right():
    n = 60
    front = [1.0] * n
    d = pick_avoid_direction_goal_aware(front, current_yaw=0.0,
                                        goal_heading=-0.5)
    assert d == -1


def test_pick_direction_lidar_dominant_when_gap_big():
    """Sol çok daha boş, goal sağda → lidar kazanır (sola dön)."""
    n = 60
    front = [0.5] * (n // 2) + [5.0] * (n - n // 2)
    # Burada front[0:30]=0.5, front[30:60]=5.0
    # mid=30, left=[0.5]*30 max=0.5, right=[5.0]*30 max=5.0
    # right_max çok daha büyük → return -1 (saga)
    d = pick_avoid_direction_goal_aware(front, current_yaw=0.0,
                                        goal_heading=0.5)  # goal sola
    assert d == -1  # saga (lidar dominant)


def test_pick_avoid_direction_backward_compat():
    """Eski API hala çalışır."""
    front = [0.5, 0.5, 5.0, 5.0]
    d = pick_avoid_direction(front)
    assert d in (-1, 1)


def test_wrap_pi_basics():
    assert math.isclose(wrap_pi(0.0), 0.0)
    assert math.isclose(wrap_pi(2 * math.pi), 0.0, abs_tol=1e-9)


def test_front_min_range_finds_nearest():
    n = 360
    rs = [10.0] * n
    rs[180] = 1.5
    rs[0] = 0.5  # arka
    min_r, sector = front_min_range(rs, math.radians(360), math.radians(60))
    assert math.isclose(min_r, 1.5)
    assert 0.5 not in sector


def test_front_min_range_empty():
    rs = [float('inf')] * 100
    min_r, sector = front_min_range(rs, math.radians(360), math.radians(60))
    assert min_r == float('inf')
    assert sector == []


# ═══════════════════════════════════════════════════════════════════════
# 8. HAZARD KAPATILDI — kararı doğrula
# ═══════════════════════════════════════════════════════════════════════

def test_hazard_blocking_is_empty_set():
    """KRITIK TASARIM KARARI: HAZARD_BLOCKING boş.
    Sebep: CLAUDE.md — terrain_perception yanlış sınıflandırma yapıyor.
    """
    assert HAZARD_BLOCKING == set()
    assert "STOP" not in HAZARD_BLOCKING
    assert "SLOW" not in HAZARD_BLOCKING


# ═══════════════════════════════════════════════════════════════════════
# 9. UÇTAN-UCA — tezdeki demo
# ═══════════════════════════════════════════════════════════════════════

def test_full_cycle_obstacle_pass_realign_continue():
    """Tezdeki ana senaryo: goal 0, engel önümde, sap, geç, geri dön."""
    cfg = AvoiderConfig(target_distance_m=100.0)
    s = AvoiderState(goal_heading_rad=0.0)

    # 1) 1 m engelsiz ilerle
    for _ in range(50):
        s = decide(s, _open_scan(), math.radians(360), "CLEAR",
                   0.0, 0.02, cfg).next_state
    assert s.phase == AvoiderPhase.DRIVING

    # 2) Engel çıktı
    s = decide(s, _blocked_scan_front(blocked_at_m=0.25),
               math.radians(360), "CLEAR", 0.0, 0.0, cfg).next_state
    assert s.phase == AvoiderPhase.AVOIDING

    # 3) Döndü, ön temiz
    s = decide(s, _open_scan(), math.radians(360), "CLEAR",
               0.0, 0.0, cfg).next_state
    assert s.phase == AvoiderPhase.PASSING

    # 4) PASSING (40 cm)
    for _ in range(20):
        s = decide(s, _open_scan(), math.radians(360), "CLEAR",
                   current_yaw=0.5, odom_delta_m=0.025, cfg=cfg).next_state
    assert s.phase == AvoiderPhase.REALIGNING

    # 5) REALIGNING — goal heading 0'a dön
    s = decide(s, _open_scan(), math.radians(360), "CLEAR",
               current_yaw=0.05, odom_delta_m=0.0, cfg=cfg).next_state
    assert s.phase == AvoiderPhase.DRIVING


def test_state_does_not_mutate_original():
    original = AvoiderState(distance_clear_m=0.5, goal_heading_rad=0.0)
    decide(original, _open_scan(), math.radians(360), "CLEAR",
           0.0, 0.10, _cfg())
    assert math.isclose(original.distance_clear_m, 0.5)


def test_lidar_with_60_rays():
    n = 60
    rs = [5.0] * n
    rs[30] = 0.3  # tam ön, threshold (0.35) altı
    min_r, _ = front_min_range(rs, math.radians(360), math.radians(50))
    assert math.isclose(min_r, 0.3)


def test_negative_inf_in_scan_ignored():
    n = 360
    rs = [5.0] * n
    rs[180] = 0.0
    rs[181] = -float('inf')
    rs[182] = 0.3
    min_r, _ = front_min_range(rs, math.radians(360), math.radians(60))
    assert math.isclose(min_r, 0.3)
