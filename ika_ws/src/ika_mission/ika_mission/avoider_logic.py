"""IKA — Tam otonom reaktif engel kacinma cekirdegi (ROS'suz).

Davranis (kullanici spec, tezdeki "obstacle-avoiding wanderer"):

    Acilis -> dumduz ileri suruyor.
    Onunde gecemeyecegi engel veya dusme riski -> sola/saga doniyor.
    Engel bitti -> ev yonune (baslangic yaw) geri doniyor, devam ediyor.
    Toplam 2 m engelsiz mesafe -> kendiliginden duruyor.

Tasarim:
    - Saf-Python, ROS yok -> `pytest` ile Windows'ta dahi kosar.
    - Karar = state + (scan_front + hazard_action + yaw + odom_delta) -> Twist.
    - Durum makinesi (`AvoiderPhase`): DRIVING / AVOIDING / REALIGNING / DONE.

Engel tanimi (OR):
    - Lidar `front_arc_deg` sektorunde min range < `obstacle_distance_m`.
    - VEYA hazard_action `STOP` / `SLOW` (terrain dropoff, dinamik nesne...).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Sequence, Tuple


class AvoiderPhase(str, Enum):
    DRIVING = "DRIVING"
    AVOIDING = "AVOIDING"
    REALIGNING = "REALIGNING"
    DONE = "DONE"


@dataclass(frozen=True)
class AvoiderConfig:
    forward_speed_mps: float = 0.20      # 0.25 hiz limiti altinda guvenli marj
    turn_speed_rps: float = 0.5
    obstacle_distance_m: float = 0.80    # bu mesafede engel sayilir
    front_arc_deg: float = 60.0          # +/-30 derece on sektoru
    target_distance_m: float = 2.0       # bu kadar engelsiz mesafe -> dur
    yaw_tolerance_rad: float = 0.05      # ev yonune yakinsama esiği


@dataclass
class AvoiderState:
    phase: AvoiderPhase = AvoiderPhase.DRIVING
    home_yaw: float = 0.0                # baslangic yon (radyan)
    distance_clear_m: float = 0.0        # birikmis engelsiz mesafe
    avoid_direction: int = 0             # +1 sol, -1 sag, 0 yok


# Hazard action degerlerini fusion node uretir; bunlar kacisi tetikler.
HAZARD_BLOCKING = {"STOP", "SLOW"}


def wrap_pi(angle: float) -> float:
    """Aciyi [-pi, pi] araligina sar."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _front_sector_indices(num_rays: int, total_fov_rad: float,
                          front_arc_rad: float) -> Tuple[int, int]:
    """Lidar mesafe dizisinden on sektorun indeks araligini hesapla.

    LaserScan kabulu: indeks 0 -> -fov/2 yonu, indeks N-1 -> +fov/2 yonu.
    On (yaw=0) ortada. Bu kullanim icin tipik 360 derece tarayicili lidar:
    indeks (N/2 - arc/2) ile (N/2 + arc/2) arasi ileri sektor.
    """
    if num_rays <= 1:
        return (0, num_rays)
    half_arc = front_arc_rad / 2.0
    angle_per_ray = total_fov_rad / max(num_rays - 1, 1)
    half_count = max(int(round(half_arc / angle_per_ray)), 1)
    mid = num_rays // 2
    lo = max(mid - half_count, 0)
    hi = min(mid + half_count + 1, num_rays)
    return (lo, hi)


def front_min_range(scan_ranges: Sequence[float], total_fov_rad: float,
                    front_arc_rad: float) -> Tuple[float, Sequence[float]]:
    """On sektorun min menzilini + ilgili dilimi don."""
    lo, hi = _front_sector_indices(len(scan_ranges), total_fov_rad, front_arc_rad)
    sector = [r for r in scan_ranges[lo:hi]
              if r > 0.0 and math.isfinite(r)]
    if not sector:
        return (float('inf'), [])
    return (min(sector), sector)


def pick_avoid_direction(front_sector: Sequence[float]) -> int:
    """On sektorun sol/sag yarisindan daha bos olanin yonunu sec.

    Returns +1 (sol) veya -1 (sag). Bilgi yoksa sag (-1) dondur.
    """
    if not front_sector:
        return -1
    mid = len(front_sector) // 2
    left = [r for r in front_sector[:mid] if math.isfinite(r)]
    right = [r for r in front_sector[mid:] if math.isfinite(r)]
    left_max = max(left) if left else 0.0
    right_max = max(right) if right else 0.0
    return 1 if left_max > right_max else -1


@dataclass
class AvoiderCommand:
    linear_x: float
    angular_z: float
    next_state: AvoiderState
    reason: str = ""

    def as_tuple(self) -> Tuple[float, float, AvoiderState, str]:
        return (self.linear_x, self.angular_z, self.next_state, self.reason)


def decide(state: AvoiderState,
           scan_ranges: Sequence[float],
           scan_fov_rad: float,
           hazard_action: str,
           current_yaw: float,
           odom_delta_m: float,
           cfg: AvoiderConfig) -> AvoiderCommand:
    """Tek-tik karar fonksiyonu. State'i guncellemez (yeni state doner)."""
    front_arc_rad = math.radians(cfg.front_arc_deg)
    min_r, sector = front_min_range(scan_ranges, scan_fov_rad, front_arc_rad)
    obstacle_close = min_r < cfg.obstacle_distance_m
    hazard_blocked = (hazard_action or "").upper() in HAZARD_BLOCKING
    blocked = obstacle_close or hazard_blocked

    new = replace(state)

    if state.phase == AvoiderPhase.DRIVING:
        if blocked:
            new.avoid_direction = pick_avoid_direction(sector)
            new.phase = AvoiderPhase.AVOIDING
            reason = (f"DRIVING->AVOIDING (min_r={min_r:.2f}m, "
                      f"hazard={hazard_action}, dir={new.avoid_direction:+d})")
            return AvoiderCommand(0.0, cfg.turn_speed_rps * new.avoid_direction,
                                  new, reason)
        # Engelsiz: hizla ilerle, mesafe say
        new.distance_clear_m = state.distance_clear_m + max(odom_delta_m, 0.0)
        if new.distance_clear_m >= cfg.target_distance_m:
            new.phase = AvoiderPhase.DONE
            return AvoiderCommand(0.0, 0.0, new,
                                  f"DRIVING->DONE (d={new.distance_clear_m:.2f}m)")
        return AvoiderCommand(cfg.forward_speed_mps, 0.0, new,
                              f"DRIVING (d={new.distance_clear_m:.2f}m)")

    if state.phase == AvoiderPhase.AVOIDING:
        if blocked:
            # Engel hala onumde -> donmeye devam
            return AvoiderCommand(0.0, cfg.turn_speed_rps * state.avoid_direction,
                                  new, "AVOIDING (still blocked)")
        # On temiz -> ev yonune dogrultmaya gec
        new.phase = AvoiderPhase.REALIGNING
        return AvoiderCommand(0.0, 0.0, new, "AVOIDING->REALIGNING")

    if state.phase == AvoiderPhase.REALIGNING:
        yaw_err = wrap_pi(state.home_yaw - current_yaw)
        if abs(yaw_err) < cfg.yaw_tolerance_rad:
            new.phase = AvoiderPhase.DRIVING
            new.avoid_direction = 0
            # NOT: distance_clear_m sifirlanmaz; toplam engelsiz mesafe sayar
            return AvoiderCommand(cfg.forward_speed_mps, 0.0, new,
                                  "REALIGNING->DRIVING (aligned)")
        # Engel cikabilir mi? Engel yeniden cikarsa AVOIDING'e geri don
        if blocked:
            new.phase = AvoiderPhase.AVOIDING
            new.avoid_direction = pick_avoid_direction(sector)
            return AvoiderCommand(0.0, cfg.turn_speed_rps * new.avoid_direction,
                                  new, "REALIGNING->AVOIDING (new obstacle)")
        # Ev yonune dogru don (kucuk aci hatasini kapatmak icin yon isareti)
        sign = 1 if yaw_err > 0 else -1
        return AvoiderCommand(0.0, cfg.turn_speed_rps * sign, new,
                              f"REALIGNING (yaw_err={yaw_err:.3f})")

    # DONE
    return AvoiderCommand(0.0, 0.0, state, "DONE")
