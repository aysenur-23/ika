"""Avoider mantik testleri (saf-Python, ROS yok).

Hedef:
    DRIVING dumduz hareket + mesafe sayar
    Engel -> AVOIDING (dogru yon secimi)
    Engel temizlenince -> REALIGNING -> DRIVING (distance_clear KORUNUR)
    2 m engelsiz -> DONE
    Hazard state STOP/SLOW da tetikler
"""
from __future__ import annotations

import math
from typing import List

from ika_mission.avoider_logic import (
    AvoiderConfig, AvoiderState, AvoiderPhase,
    decide, pick_avoid_direction, front_min_range, wrap_pi,
    HAZARD_BLOCKING,
)


def _open_scan(n: int = 360, dist: float = 5.0) -> List[float]:
    """Tum yonleri X m acik tarayici."""
    return [dist] * n


def _blocked_scan_front(n: int = 360, blocked_at_m: float = 0.5,
                        open_at_m: float = 5.0) -> List[float]:
    """Sadece on yaridaki 30 dilimi engelli, kalan acik."""
    rs = [open_at_m] * n
    mid = n // 2
    for i in range(mid - 15, mid + 15):
        rs[i] = blocked_at_m
    return rs


def _cfg() -> AvoiderConfig:
    return AvoiderConfig()  # tum default'lar


def test_driving_empty_world_no_obstacle_drives_forward():
    s = AvoiderState()
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.linear_x > 0.0
    assert c.angular_z == 0.0
    assert c.next_state.phase == AvoiderPhase.DRIVING


def test_distance_clear_accumulates():
    s = AvoiderState()
    cfg = _cfg()
    # 10 cm odom delta sonra
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.10, cfg)
    assert math.isclose(c.next_state.distance_clear_m, 0.10)


def test_driving_to_done_after_2m():
    s = AvoiderState(distance_clear_m=1.95)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.10, _cfg())
    # 1.95 + 0.10 = 2.05 >= 2.0 -> DONE
    assert c.next_state.phase == AvoiderPhase.DONE
    assert c.linear_x == 0.0
    assert c.angular_z == 0.0


def test_obstacle_ahead_triggers_avoiding():
    s = AvoiderState()
    scan = _blocked_scan_front(blocked_at_m=0.5)
    c = decide(s, scan, math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.AVOIDING
    assert c.linear_x == 0.0
    assert abs(c.angular_z) > 0.0  # donus komutu var


def test_hazard_stop_triggers_avoiding_even_without_lidar_block():
    s = AvoiderState()
    c = decide(s, _open_scan(), math.radians(360), "STOP", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.AVOIDING


def test_hazard_slow_also_triggers():
    s = AvoiderState()
    c = decide(s, _open_scan(), math.radians(360), "SLOW", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.AVOIDING


def test_hazard_clear_does_not_trigger():
    s = AvoiderState()
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.DRIVING


def test_avoiding_to_realigning_when_clear():
    s = AvoiderState(phase=AvoiderPhase.AVOIDING, avoid_direction=1)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.REALIGNING


def test_realigning_to_driving_when_yaw_close_to_home():
    s = AvoiderState(phase=AvoiderPhase.REALIGNING, home_yaw=0.0,
                     distance_clear_m=0.7)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR",
               current_yaw=0.01, odom_delta_m=0.0, cfg=_cfg())
    assert c.next_state.phase == AvoiderPhase.DRIVING
    # mesafe sifirlanmiyor — kullanici spec'i: 2 m TOPLAM engelsiz
    assert math.isclose(c.next_state.distance_clear_m, 0.7)


def test_realigning_keeps_turning_if_yaw_off():
    s = AvoiderState(phase=AvoiderPhase.REALIGNING, home_yaw=0.0)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR",
               current_yaw=0.5, odom_delta_m=0.0, cfg=_cfg())
    assert c.next_state.phase == AvoiderPhase.REALIGNING
    assert c.linear_x == 0.0
    assert c.angular_z != 0.0


def test_done_phase_stays_stopped():
    s = AvoiderState(phase=AvoiderPhase.DONE)
    c = decide(s, _blocked_scan_front(), math.radians(360), "STOP",
               0.0, 0.0, _cfg())
    assert c.linear_x == 0.0
    assert c.angular_z == 0.0
    assert c.next_state.phase == AvoiderPhase.DONE


def test_pick_avoid_direction_prefers_more_open_side():
    # sol yari bos, sag yari engelli
    n = 60
    front = [0.5] * n
    for i in range(0, n // 2):
        front[i] = 5.0
    assert pick_avoid_direction(front) == 1  # sola don


def test_pick_avoid_direction_empty_returns_right():
    assert pick_avoid_direction([]) == -1


def test_wrap_pi_basics():
    assert math.isclose(wrap_pi(0.0), 0.0)
    assert math.isclose(wrap_pi(math.pi), math.pi) or math.isclose(wrap_pi(math.pi), -math.pi)
    assert math.isclose(wrap_pi(2 * math.pi), 0.0, abs_tol=1e-9)
    assert math.isclose(wrap_pi(-math.pi - 0.1), math.pi - 0.1, abs_tol=1e-9)


def test_front_min_range_finds_nearest_in_sector():
    # tum aciklik 360, on sektor 60 derece
    n = 360
    rs = [10.0] * n
    rs[180] = 1.5   # tam on
    rs[0] = 0.5     # arka (sektor disinda)
    min_r, sector = front_min_range(rs, math.radians(360), math.radians(60))
    assert math.isclose(min_r, 1.5)
    # arkadaki 0.5 ON SEKTORDE OLMAMALI
    assert 0.5 not in sector


def test_full_cycle_obstacle_then_continue_then_stop():
    """End-to-end: dur, engel, kac, donus, devam, 2m, dur."""
    cfg = AvoiderConfig(forward_speed_mps=0.2, target_distance_m=2.0)
    s = AvoiderState()
    # 1) 1m engelsiz ilerle
    for _ in range(50):
        s = decide(s, _open_scan(), math.radians(360), "CLEAR",
                   0.0, 0.02, cfg).next_state
    assert s.phase == AvoiderPhase.DRIVING
    assert s.distance_clear_m >= 1.0

    # 2) Engel cikti
    s = decide(s, _blocked_scan_front(blocked_at_m=0.5), math.radians(360),
               "CLEAR", 0.0, 0.0, cfg).next_state
    assert s.phase == AvoiderPhase.AVOIDING

    # 3) Donerek engelden kurtuldu (artik temiz)
    s = decide(s, _open_scan(), math.radians(360), "CLEAR",
               0.0, 0.0, cfg).next_state
    assert s.phase == AvoiderPhase.REALIGNING

    # 4) Ev yonune dondu
    s = decide(s, _open_scan(), math.radians(360), "CLEAR",
               current_yaw=0.0, odom_delta_m=0.0, cfg=cfg).next_state
    assert s.phase == AvoiderPhase.DRIVING
    # mesafe sayaci korunmali (kullanici spec'i)
    assert s.distance_clear_m >= 1.0

    # 5) Yeterli engelsiz mesafe -> DONE
    for _ in range(60):
        s = decide(s, _open_scan(), math.radians(360), "CLEAR",
                   0.0, 0.02, cfg).next_state
    assert s.phase == AvoiderPhase.DONE


def test_hazard_blocking_set_contains_expected_values():
    assert "STOP" in HAZARD_BLOCKING
    assert "SLOW" in HAZARD_BLOCKING
    assert "CLEAR" not in HAZARD_BLOCKING
