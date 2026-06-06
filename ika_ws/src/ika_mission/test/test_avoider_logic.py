"""Avoider state machine birim testleri (saf-Python, ROS yok).

Defense-in-depth Katman 2 (`docs/avoidance_architecture.md`).

State machine:
    DRIVING -> AVOIDING -> PASSING -> REALIGNING -> DRIVING
    Her durumda defansif: engel ON sektordeyse AVOIDING'e geri.

Test kategorileri:
    1. Durum gecisleri (her gecis icin pozitif + negatif test)
    2. Defansif güvenlik (PASSING/REALIGNING'da engel cikinca dön)
    3. distance_clear_m bütünlügü (DONE şartı için)
    4. Yardimci fonksiyonlar (pick_avoid_direction, wrap_pi, front_min_range)
    5. Uçtan-uca senaryolar (DRIVING -> ... -> DONE)
"""
from __future__ import annotations

import math
from typing import List

from ika_mission.avoider_logic import (
    AvoiderConfig, AvoiderState, AvoiderPhase,
    decide, pick_avoid_direction, front_min_range, wrap_pi,
    HAZARD_BLOCKING,
)


# ═══════════════════════════════════════════════════════════════════════
# YARDIMCILAR
# ═══════════════════════════════════════════════════════════════════════

def _open_scan(n: int = 360, dist: float = 5.0) -> List[float]:
    """Tum yonleri X m acik tarayici (engelsiz dunya)."""
    return [dist] * n


def _blocked_scan_front(n: int = 360, blocked_at_m: float = 0.5,
                        open_at_m: float = 5.0,
                        block_width: int = 30) -> List[float]:
    """Sadece on +/-15 dilimi (60 deg arc) engelli."""
    rs = [open_at_m] * n
    mid = n // 2
    half = block_width // 2
    for i in range(mid - half, mid + half):
        rs[i] = blocked_at_m
    return rs


def _cfg(**kw) -> AvoiderConfig:
    """Default config + opsiyonel override."""
    return AvoiderConfig(**kw)


# ═══════════════════════════════════════════════════════════════════════
# 1. DRIVING durumu testleri
# ═══════════════════════════════════════════════════════════════════════

def test_driving_empty_world_drives_forward():
    s = AvoiderState()
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.linear_x > 0.0
    assert c.angular_z == 0.0
    assert c.next_state.phase == AvoiderPhase.DRIVING


def test_driving_distance_accumulates():
    s = AvoiderState()
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.10, _cfg())
    assert math.isclose(c.next_state.distance_clear_m, 0.10)


def test_driving_to_done_after_target_distance():
    s = AvoiderState(distance_clear_m=1.95)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.10, _cfg())
    assert c.next_state.phase == AvoiderPhase.DONE
    assert c.linear_x == 0.0
    assert c.angular_z == 0.0


def test_driving_negative_odom_delta_ignored():
    """Geri kayma odom_delta < 0 distance'a eklenmez (mesafe sadece artar)."""
    s = AvoiderState(distance_clear_m=0.5)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, -0.10, _cfg())
    assert math.isclose(c.next_state.distance_clear_m, 0.5)


# ═══════════════════════════════════════════════════════════════════════
# 2. DRIVING -> AVOIDING gecisleri
# ═══════════════════════════════════════════════════════════════════════

def test_driving_obstacle_triggers_avoiding():
    s = AvoiderState()
    scan = _blocked_scan_front(blocked_at_m=0.5)
    c = decide(s, scan, math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.AVOIDING
    assert c.linear_x == 0.0
    assert abs(c.angular_z) > 0.0
    assert c.next_state.avoid_direction in (-1, 1)


def test_driving_hazard_stop_triggers_avoiding():
    s = AvoiderState()
    c = decide(s, _open_scan(), math.radians(360), "STOP", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.AVOIDING


def test_driving_hazard_slow_triggers_avoiding():
    s = AvoiderState()
    c = decide(s, _open_scan(), math.radians(360), "SLOW", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.AVOIDING


def test_driving_hazard_clear_does_not_trigger():
    s = AvoiderState()
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.DRIVING


def test_driving_obstacle_just_outside_threshold_no_trigger():
    """0.80 m'den biraz fazla -> trigger yok."""
    s = AvoiderState()
    scan = _blocked_scan_front(blocked_at_m=0.85)  # cfg default 0.80
    c = decide(s, scan, math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.DRIVING


# ═══════════════════════════════════════════════════════════════════════
# 3. AVOIDING durumu + AVOIDING -> PASSING gecisi
# ═══════════════════════════════════════════════════════════════════════

def test_avoiding_keeps_turning_while_blocked():
    s = AvoiderState(phase=AvoiderPhase.AVOIDING, avoid_direction=1)
    scan = _blocked_scan_front(blocked_at_m=0.5)
    c = decide(s, scan, math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.AVOIDING
    assert c.linear_x == 0.0
    assert c.angular_z > 0.0  # sola don (dir=1)


def test_avoiding_to_passing_when_front_clear():
    """On sektor temizlenince PASSING'e gec."""
    s = AvoiderState(phase=AvoiderPhase.AVOIDING, avoid_direction=1)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.PASSING
    assert c.linear_x > 0.0  # PASSING ileri sürer
    assert c.angular_z == 0.0


def test_avoiding_release_distance_hysteresis():
    """Hysteresis: 0.80 girer ama 1.00 cikar (chattering yok)."""
    # 0.85 m engel: blocked_enter False (0.80 esik), blocked_exit True (1.00 esik)
    s = AvoiderState(phase=AvoiderPhase.AVOIDING, avoid_direction=1)
    scan = _blocked_scan_front(blocked_at_m=0.85)
    c = decide(s, scan, math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    # AVOIDING'den cikmak icin release_distance lazim -> hala AVOIDING
    assert c.next_state.phase == AvoiderPhase.AVOIDING


# ═══════════════════════════════════════════════════════════════════════
# 4. PASSING durumu + PASSING -> REALIGNING gecisi
# ═══════════════════════════════════════════════════════════════════════

def test_passing_drives_forward_and_accumulates():
    s = AvoiderState(phase=AvoiderPhase.PASSING, pass_distance_m=0.0,
                     distance_clear_m=1.0)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.05, _cfg())
    assert c.next_state.phase == AvoiderPhase.PASSING
    assert c.linear_x > 0.0
    assert math.isclose(c.next_state.pass_distance_m, 0.05)
    # distance_clear da artar (DONE icin)
    assert math.isclose(c.next_state.distance_clear_m, 1.05)


def test_passing_to_realigning_after_clear_distance():
    """PASSING fazinda pass_clear_distance_m kat edilince REALIGNING'e gec."""
    cfg = _cfg(pass_clear_distance_m=0.50)
    s = AvoiderState(phase=AvoiderPhase.PASSING, pass_distance_m=0.48,
                     distance_clear_m=1.0)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.05, cfg)
    # 0.48 + 0.05 = 0.53 >= 0.50 -> REALIGNING
    assert c.next_state.phase == AvoiderPhase.REALIGNING


def test_passing_to_done_when_target_distance_reached():
    """PASSING fazinda toplam distance_clear_m >= target -> DONE."""
    cfg = _cfg(target_distance_m=2.0, pass_clear_distance_m=0.50)
    s = AvoiderState(phase=AvoiderPhase.PASSING, pass_distance_m=0.10,
                     distance_clear_m=1.97)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR", 0.0, 0.05, cfg)
    # 1.97 + 0.05 = 2.02 >= 2.0 -> DONE
    assert c.next_state.phase == AvoiderPhase.DONE
    assert c.linear_x == 0.0


def test_passing_defensive_obstacle_returns_to_avoiding():
    """KRITIK: PASSING sirasinda ON sektorde engel cikarsa AVOIDING'e geri."""
    s = AvoiderState(phase=AvoiderPhase.PASSING, pass_distance_m=0.20,
                     avoid_direction=1)
    scan = _blocked_scan_front(blocked_at_m=0.5)
    c = decide(s, scan, math.radians(360), "CLEAR", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.AVOIDING
    assert c.linear_x == 0.0  # surus durur, don baslar
    assert abs(c.angular_z) > 0.0
    # pass_distance_m sifirlanir (yeniden saymak gerekecek)
    assert c.next_state.pass_distance_m == 0.0


def test_passing_defensive_hazard_returns_to_avoiding():
    """Hazard STOP da PASSING'i kesir."""
    s = AvoiderState(phase=AvoiderPhase.PASSING, pass_distance_m=0.20)
    c = decide(s, _open_scan(), math.radians(360), "STOP", 0.0, 0.0, _cfg())
    assert c.next_state.phase == AvoiderPhase.AVOIDING


# ═══════════════════════════════════════════════════════════════════════
# 5. REALIGNING durumu + REALIGNING -> DRIVING gecisi
# ═══════════════════════════════════════════════════════════════════════

def test_realigning_to_driving_when_yaw_aligned():
    """Ev yonune yakinsadi (yaw_err < yaw_tolerance) -> DRIVING."""
    s = AvoiderState(phase=AvoiderPhase.REALIGNING, home_yaw=0.0,
                     distance_clear_m=0.7)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR",
               current_yaw=0.05, odom_delta_m=0.0, cfg=_cfg())
    # yaw_err = 0.05 < 0.10 (default tolerance) -> DRIVING
    assert c.next_state.phase == AvoiderPhase.DRIVING
    # mesafe sayaci KORUNUR (kullanici spec: 2 m TOPLAM engelsiz)
    assert math.isclose(c.next_state.distance_clear_m, 0.7)


def test_realigning_keeps_turning_if_yaw_off():
    s = AvoiderState(phase=AvoiderPhase.REALIGNING, home_yaw=0.0)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR",
               current_yaw=0.5, odom_delta_m=0.0, cfg=_cfg())
    assert c.next_state.phase == AvoiderPhase.REALIGNING
    assert c.linear_x == 0.0
    assert c.angular_z != 0.0


def test_realigning_correct_turn_direction():
    """Yaw_err > 0 -> +angular (sola), yaw_err < 0 -> -angular (saga)."""
    cfg = _cfg()
    # home_yaw 0, current_yaw -0.5 (saga donmus) -> sola don gerek
    s = AvoiderState(phase=AvoiderPhase.REALIGNING, home_yaw=0.0)
    c = decide(s, _open_scan(), math.radians(360), "CLEAR",
               current_yaw=-0.5, odom_delta_m=0.0, cfg=cfg)
    assert c.angular_z > 0  # sola
    # tersi
    c = decide(s, _open_scan(), math.radians(360), "CLEAR",
               current_yaw=0.5, odom_delta_m=0.0, cfg=cfg)
    assert c.angular_z < 0  # saga


def test_realigning_defensive_obstacle_returns_to_avoiding():
    """REALIGNING sirasinda engel cikinca AVOIDING'e geri."""
    s = AvoiderState(phase=AvoiderPhase.REALIGNING, home_yaw=0.0)
    scan = _blocked_scan_front(blocked_at_m=0.5)
    c = decide(s, scan, math.radians(360), "CLEAR",
               current_yaw=0.5, odom_delta_m=0.0, cfg=_cfg())
    assert c.next_state.phase == AvoiderPhase.AVOIDING


# ═══════════════════════════════════════════════════════════════════════
# 6. DONE durumu (terminal)
# ═══════════════════════════════════════════════════════════════════════

def test_done_phase_stays_stopped_even_under_obstacle():
    """DONE ASLA cikis yok — engel veya hazard etkilemez."""
    s = AvoiderState(phase=AvoiderPhase.DONE)
    c = decide(s, _blocked_scan_front(), math.radians(360), "STOP",
               0.0, 0.0, _cfg())
    assert c.linear_x == 0.0
    assert c.angular_z == 0.0
    assert c.next_state.phase == AvoiderPhase.DONE


# ═══════════════════════════════════════════════════════════════════════
# 7. Yardimci fonksiyonlar
# ═══════════════════════════════════════════════════════════════════════

def test_pick_avoid_direction_prefers_more_open_side():
    n = 60
    front = [0.5] * n
    for i in range(0, n // 2):
        front[i] = 5.0  # sol yari bos
    assert pick_avoid_direction(front) == 1  # sola don


def test_pick_avoid_direction_right_side_more_open():
    n = 60
    front = [0.5] * n
    for i in range(n // 2, n):
        front[i] = 5.0  # sag yari bos
    assert pick_avoid_direction(front) == -1  # saga don


def test_pick_avoid_direction_empty_returns_right():
    assert pick_avoid_direction([]) == -1


def test_wrap_pi_basics():
    assert math.isclose(wrap_pi(0.0), 0.0)
    assert math.isclose(wrap_pi(math.pi), math.pi) or math.isclose(wrap_pi(math.pi), -math.pi)
    assert math.isclose(wrap_pi(2 * math.pi), 0.0, abs_tol=1e-9)
    assert math.isclose(wrap_pi(-math.pi - 0.1), math.pi - 0.1, abs_tol=1e-9)


def test_wrap_pi_negative_outside_range():
    # -3.5 rad -> -3.5 + 2pi ≈ 2.78
    result = wrap_pi(-3.5)
    assert -math.pi <= result <= math.pi


def test_front_min_range_finds_nearest_in_sector():
    n = 360
    rs = [10.0] * n
    rs[180] = 1.5  # tam on
    rs[0] = 0.5    # arka (sektor disinda)
    min_r, sector = front_min_range(rs, math.radians(360), math.radians(60))
    assert math.isclose(min_r, 1.5)
    assert 0.5 not in sector  # arka sektor disi


def test_front_min_range_empty_returns_inf():
    """Hicbir nokta yoksa min inf, sektor bos."""
    n = 360
    rs = [float('inf')] * n
    min_r, sector = front_min_range(rs, math.radians(360), math.radians(60))
    assert min_r == float('inf')
    assert sector == []


def test_hazard_blocking_set_contains_expected_values():
    assert "STOP" in HAZARD_BLOCKING
    assert "SLOW" in HAZARD_BLOCKING
    assert "CLEAR" not in HAZARD_BLOCKING


# ═══════════════════════════════════════════════════════════════════════
# 8. UÇTAN-UCA SENARYOLAR (tezdeki "engel kaçınma demosu")
# ═══════════════════════════════════════════════════════════════════════

def test_full_cycle_obstacle_pass_realign_continue_done():
    """Tezdeki ana senaryo: dur, engel, kac, gec, donus, devam, 2m, dur."""
    cfg = AvoiderConfig(
        forward_speed_mps=0.2,
        target_distance_m=2.0,
        pass_clear_distance_m=0.40,
        yaw_tolerance_rad=0.10,
    )
    s = AvoiderState()

    # 1) 1 m engelsiz ilerle (50 tik × 2 cm odom = 1 m)
    for _ in range(50):
        s = decide(s, _open_scan(), math.radians(360), "CLEAR",
                   0.0, 0.02, cfg).next_state
    assert s.phase == AvoiderPhase.DRIVING
    assert s.distance_clear_m >= 1.0

    # 2) Engel cikti -> AVOIDING
    s = decide(s, _blocked_scan_front(blocked_at_m=0.5),
               math.radians(360), "CLEAR", 0.0, 0.0, cfg).next_state
    assert s.phase == AvoiderPhase.AVOIDING

    # 3) Donerek engelden kurtuldu -> PASSING
    s = decide(s, _open_scan(), math.radians(360), "CLEAR",
               0.0, 0.0, cfg).next_state
    assert s.phase == AvoiderPhase.PASSING

    # 4) PASSING fazinda yan tarafa surdu (20 tik × 2.5 cm = 0.50 m > 0.40)
    for _ in range(20):
        s = decide(s, _open_scan(), math.radians(360), "CLEAR",
                   current_yaw=0.5,  # robot donmus halde
                   odom_delta_m=0.025, cfg=cfg).next_state
    assert s.phase == AvoiderPhase.REALIGNING

    # 5) REALIGNING -> ev yonune don
    s = decide(s, _open_scan(), math.radians(360), "CLEAR",
               current_yaw=0.05,  # ev yonune yakin
               odom_delta_m=0.0, cfg=cfg).next_state
    assert s.phase == AvoiderPhase.DRIVING
    # mesafe sayaci korunmali
    assert s.distance_clear_m > 1.0

    # 6) Yeterli engelsiz mesafe -> DONE
    for _ in range(100):
        s = decide(s, _open_scan(), math.radians(360), "CLEAR",
                   0.0, 0.02, cfg).next_state
    assert s.phase == AvoiderPhase.DONE


def test_defensive_re_avoidance_during_passing():
    """KRITIK GUVENLIK: PASSING'da yeni engel cikinca AVOIDING'e geri don."""
    cfg = _cfg()
    s = AvoiderState(phase=AvoiderPhase.PASSING, pass_distance_m=0.20,
                     avoid_direction=1)
    # Yeni engel onumde
    s = decide(s, _blocked_scan_front(blocked_at_m=0.5),
               math.radians(360), "CLEAR", 0.0, 0.0, cfg).next_state
    assert s.phase == AvoiderPhase.AVOIDING
    # On sektor tekrar temizlenir
    s = decide(s, _open_scan(), math.radians(360), "CLEAR",
               0.0, 0.0, cfg).next_state
    assert s.phase == AvoiderPhase.PASSING
    # pass_distance_m sifirdan baslar (gercekce: yeni engel sebebiyle)
    assert s.pass_distance_m == 0.0


def test_distance_clear_preserved_through_full_cycle():
    """distance_clear_m TUM cycle boyunca artar, ASLA sifirlanmaz."""
    cfg = _cfg(target_distance_m=10.0)  # buyuk hedef -> DONE'a girmesin
    s = AvoiderState(distance_clear_m=0.0)
    # DRIVING'de 0.5 m
    for _ in range(25):
        s = decide(s, _open_scan(), math.radians(360), "CLEAR",
                   0.0, 0.02, cfg).next_state
    d_after_driving = s.distance_clear_m
    assert d_after_driving >= 0.5

    # engel + AVOIDING (mesafe değişmez)
    s = decide(s, _blocked_scan_front(blocked_at_m=0.5),
               math.radians(360), "CLEAR", 0.0, 0.0, cfg).next_state
    assert math.isclose(s.distance_clear_m, d_after_driving)

    # AVOIDING -> PASSING (yine değişmez bu tikte odom 0)
    s = decide(s, _open_scan(), math.radians(360), "CLEAR",
               0.0, 0.0, cfg).next_state
    assert math.isclose(s.distance_clear_m, d_after_driving)

    # PASSING'da 0.5 m daha (yan surus)
    for _ in range(25):
        s = decide(s, _open_scan(), math.radians(360), "CLEAR",
                   0.0, 0.02, cfg).next_state
    # Cycle bir an REALIGNING'a gecmis olabilir; ya orada ya devam'da
    # ama distance_clear ASLA sifirdan baslamadi
    assert s.distance_clear_m >= d_after_driving + 0.4


def test_three_consecutive_obstacles():
    """3 engelli senaryo: kacinma sonrasi yeni engel cikiyor."""
    cfg = AvoiderConfig(target_distance_m=10.0)  # DONE'a gitmesin
    s = AvoiderState()
    # Engel 1: tetikle + kacin
    s = decide(s, _blocked_scan_front(),
               math.radians(360), "CLEAR", 0.0, 0.0, cfg).next_state
    assert s.phase == AvoiderPhase.AVOIDING
    s = decide(s, _open_scan(), math.radians(360), "CLEAR",
               0.0, 0.0, cfg).next_state
    assert s.phase == AvoiderPhase.PASSING

    # Engel 2: PASSING sirasinda cikti -> AVOIDING (defansif)
    s = decide(s, _blocked_scan_front(),
               math.radians(360), "CLEAR", 0.0, 0.0, cfg).next_state
    assert s.phase == AvoiderPhase.AVOIDING

    # Tekrar temiz -> PASSING
    s = decide(s, _open_scan(), math.radians(360), "CLEAR",
               0.0, 0.0, cfg).next_state
    assert s.phase == AvoiderPhase.PASSING

    # Engel 3: REALIGNING'da olur (PASSING done first)
    for _ in range(20):
        s = decide(s, _open_scan(), math.radians(360), "CLEAR",
                   current_yaw=0.5, odom_delta_m=0.025, cfg=cfg).next_state
    # Asama REALIGNING'a gecti
    assert s.phase == AvoiderPhase.REALIGNING

    # Engel 3 onumde
    s = decide(s, _blocked_scan_front(),
               math.radians(360), "CLEAR", current_yaw=0.5,
               odom_delta_m=0.0, cfg=cfg).next_state
    assert s.phase == AvoiderPhase.AVOIDING  # defansif tekrar


# ═══════════════════════════════════════════════════════════════════════
# 9. Edge case'ler
# ═══════════════════════════════════════════════════════════════════════

def test_negative_inf_in_scan_ignored():
    """Lidar bazen 0 veya -inf gonderir; filtre etsin."""
    n = 360
    rs = [5.0] * n
    rs[180] = 0.0  # gecersiz
    rs[181] = -float('inf')
    rs[182] = 1.5  # gercek engel
    min_r, _ = front_min_range(rs, math.radians(360), math.radians(60))
    assert math.isclose(min_r, 1.5)


def test_lidar_with_60_rays_not_360():
    """360 isin yok, 60 isinla calistir (RPLIDAR C1 dusuk cozunurluk modu)."""
    n = 60
    rs = [5.0] * n
    rs[30] = 0.5  # tam on
    min_r, _ = front_min_range(rs, math.radians(360), math.radians(60))
    assert math.isclose(min_r, 0.5)


def test_state_replace_does_not_mutate_original():
    """decide() pure: gelen state degismez."""
    original = AvoiderState(distance_clear_m=0.5)
    decide(original, _open_scan(), math.radians(360), "CLEAR",
           0.0, 0.10, _cfg())
    # original distance_clear hala 0.5
    assert math.isclose(original.distance_clear_m, 0.5)
