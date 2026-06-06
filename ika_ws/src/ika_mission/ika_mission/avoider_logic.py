"""IKA — Tam otonom reaktif engel kacinma cekirdegi (ROS'suz).

Defense-in-depth mimarisinin Katman 2'si (`docs/avoidance_architecture.md`).
Bu modul saf-Python; ROS yok -> pytest ile Windows'ta dahi kosar.

Davranis (tezde "engelden dolanip yola donen reaktif kacinici"):

    Acilis -> dumduz ileri suruyor (DRIVING).
    Onunde gecemeyecegi engel veya tehlike -> donmek icin durur (AVOIDING).
    On sektor temiz -> yan tarafa surup engelin otesine gecer (PASSING).
    Engel artik solda/sagda -> ev yonune geri doner (REALIGNING).
    Ev yonune yakinsadi -> tekrar DRIVING.
    Toplam target_distance_m engelsiz mesafe -> DONE.

State machine (4 calisma fazi + DONE):

      ┌──────────────┐
      │   DRIVING    │◄──────────────────────────────┐
      └──────┬───────┘                               │
             │ engel(min_r < obstacle_distance_m)    │ heading ≈ home_yaw
             │ VEYA hazard ∈ HAZARD_BLOCKING         │
             ▼                                       │
      ┌──────────────┐                               │
      │  AVOIDING    │ yerinde don (yön: bos olan)   │
      └──────┬───────┘                               │
             │ on sektor temiz (min_r > release_m)   │
             ▼                                       │
      ┌──────────────┐                               │
      │   PASSING    │ duz ilerle, engel-yan-mesafe sayar
      └──────┬───────┘                               │
             │ engel artik on sektorde degil VE      │
             │ pass_clear_distance_m kat edildi      │
             ▼                                       │
      ┌──────────────┐                               │
      │ REALIGNING   │ ev yonune don                 │
      └──────┬───────┘                               │
             │ |yaw_err| < yaw_tolerance_rad         │
             └───────────────────────────────────────┘

      DONE: terminal, asla cikis yok.

Defansiflik:
  Her durumda, on sektorde engel goründügünde, anında AVOIDING'e geri
  donulur. PASSING ve REALIGNING bu güvenlik kontrolune sahiptir.

Engel tanimi (OR):
  - Lidar `front_arc_deg` sektorunde min range < obstacle_distance_m
  - VEYA hazard_action ∈ HAZARD_BLOCKING (terrain STOP, dl detection STOP)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Sequence, Tuple


class AvoiderPhase(str, Enum):
    DRIVING = "DRIVING"
    AVOIDING = "AVOIDING"
    PASSING = "PASSING"
    REALIGNING = "REALIGNING"
    DONE = "DONE"


@dataclass(frozen=True)
class AvoiderConfig:
    """Avoider parametreleri. Tezdeki "robot 0.25 m/s" sinirina uyumlu."""
    forward_speed_mps: float = 0.20      # 0.25 hiz limiti altinda guvenli marj
    turn_speed_rps: float = 0.5
    obstacle_distance_m: float = 0.80    # bu mesafede engel sayilir
    release_distance_m: float = 1.00     # AVOIDING'den cikmak icin daha gevsek esik
                                          # (chattering'i onler: 0.8 girer, 1.0 cikar)
    front_arc_deg: float = 60.0          # +/-30 derece on sektoru
    pass_clear_distance_m: float = 0.50  # PASSING fazinda yan tarafta ne kadar suruluyor
                                          # (engel ~0.40 m kutu varsayilan + 10 cm margin)
    target_distance_m: float = 2.0       # bu kadar engelsiz mesafe -> dur
    yaw_tolerance_rad: float = 0.10      # ev yonune yakinsama esigi


@dataclass
class AvoiderState:
    """Cekirdegin canli durumu. Immutable degil; her tikte `replace` ile kopyalanir."""
    phase: AvoiderPhase = AvoiderPhase.DRIVING
    home_yaw: float = 0.0                # baslangic yon (radyan)
    distance_clear_m: float = 0.0        # toplam engelsiz mesafe (DONE icin)
    avoid_direction: int = 0             # +1 sol, -1 sag, 0 yok
    pass_distance_m: float = 0.0         # PASSING fazinda kat edilen mesafe


# Faz 3'te lidar-only yeterli oldugundan terrain hazard kapatildi;
# fakat sim/terrain testleri icin geri acilabilir set olarak kalsin.
# Tezdeki katman analizi: terrain_layer global costmap'te ayri marker olarak.
HAZARD_BLOCKING = {"STOP", "SLOW"}


def wrap_pi(angle: float) -> float:
    """Aciyi [-pi, pi] araligina sar."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _front_sector_indices(num_rays: int, total_fov_rad: float,
                          front_arc_rad: float) -> Tuple[int, int]:
    """Lidar mesafe dizisinden on sektorun indeks araligini hesapla.

    LaserScan kabulu: indeks 0 -> -fov/2 yonu, indeks N-1 -> +fov/2 yonu.
    On (yaw=0) ortada. 360 derece lidarlar icin tipik:
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

    Returns +1 (sol) veya -1 (sag). Bilgi yoksa sag (-1) dondur (deterministik).
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
    """Cekirdek karari: tek tik icin hiz komutu + yeni state + insan icin reason."""
    linear_x: float
    angular_z: float
    next_state: AvoiderState
    reason: str = ""

    def as_tuple(self) -> Tuple[float, float, AvoiderState, str]:
        return (self.linear_x, self.angular_z, self.next_state, self.reason)


def _is_blocked(min_range: float, hazard_action: str,
                threshold_m: float) -> bool:
    """Defansif engel kontrolu: lidar yakin VEYA hazard STOP/SLOW."""
    obstacle_close = min_range < threshold_m
    hazard_blocked = (hazard_action or "").upper() in HAZARD_BLOCKING
    return obstacle_close or hazard_blocked


def decide(state: AvoiderState,
           scan_ranges: Sequence[float],
           scan_fov_rad: float,
           hazard_action: str,
           current_yaw: float,
           odom_delta_m: float,
           cfg: AvoiderConfig) -> AvoiderCommand:
    """Tek-tik karar fonksiyonu. State'i guncellemez (yeni state doner).

    Args:
        state         : mevcut AvoiderState
        scan_ranges   : LaserScan.ranges (m)
        scan_fov_rad  : tarayicinin toplam FOV'u (radyan, 360 icin 2*pi)
        hazard_action : "CLEAR" | "SLOW" | "STOP" (ika_fusion'dan)
        current_yaw   : odom/EKF yaw (radyan)
        odom_delta_m  : bu tikteki kat edilen lineer mesafe (m)
        cfg           : AvoiderConfig

    Returns:
        AvoiderCommand (linear, angular, next_state, reason).
    """
    front_arc_rad = math.radians(cfg.front_arc_deg)
    min_r, sector = front_min_range(scan_ranges, scan_fov_rad, front_arc_rad)
    blocked_enter = _is_blocked(min_r, hazard_action, cfg.obstacle_distance_m)
    blocked_exit = _is_blocked(min_r, hazard_action, cfg.release_distance_m)

    # ─── DRIVING ────────────────────────────────────────────────────────
    if state.phase == AvoiderPhase.DRIVING:
        if blocked_enter:
            new = replace(state)
            new.avoid_direction = pick_avoid_direction(sector)
            new.phase = AvoiderPhase.AVOIDING
            new.pass_distance_m = 0.0
            return AvoiderCommand(
                0.0, cfg.turn_speed_rps * new.avoid_direction, new,
                f"DRIVING->AVOIDING (min_r={min_r:.2f}m, "
                f"hazard={hazard_action}, dir={new.avoid_direction:+d})",
            )
        # Engel yok: ilerle, mesafe say
        new = replace(state)
        new.distance_clear_m = state.distance_clear_m + max(odom_delta_m, 0.0)
        if new.distance_clear_m >= cfg.target_distance_m:
            new.phase = AvoiderPhase.DONE
            return AvoiderCommand(0.0, 0.0, new,
                                  f"DRIVING->DONE (d={new.distance_clear_m:.2f}m)")
        return AvoiderCommand(cfg.forward_speed_mps, 0.0, new,
                              f"DRIVING (d={new.distance_clear_m:.2f}m)")

    # ─── AVOIDING ──────────────────────────────────────────────────────
    if state.phase == AvoiderPhase.AVOIDING:
        if blocked_exit:
            # Hala engelliyim, donmeye devam
            return AvoiderCommand(
                0.0, cfg.turn_speed_rps * state.avoid_direction,
                replace(state), "AVOIDING (still blocked)",
            )
        # On sektor temizlendi: PASSING'e gec (engelin yanindan suruyoruz)
        new = replace(state)
        new.phase = AvoiderPhase.PASSING
        new.pass_distance_m = 0.0
        return AvoiderCommand(
            cfg.forward_speed_mps, 0.0, new,
            f"AVOIDING->PASSING (cleared, min_r={min_r:.2f}m)",
        )

    # ─── PASSING ───────────────────────────────────────────────────────
    if state.phase == AvoiderPhase.PASSING:
        # Defansif: ON sektorde tekrar engel cikarsa AVOIDING'e geri don
        if blocked_enter:
            new = replace(state)
            new.phase = AvoiderPhase.AVOIDING
            new.avoid_direction = pick_avoid_direction(sector)
            new.pass_distance_m = 0.0
            return AvoiderCommand(
                0.0, cfg.turn_speed_rps * new.avoid_direction, new,
                f"PASSING->AVOIDING (new obstacle min_r={min_r:.2f}m)",
            )
        # Yeterince ilerledim mi? engel artik yanimda kaldi
        new = replace(state)
        new.pass_distance_m = state.pass_distance_m + max(odom_delta_m, 0.0)
        new.distance_clear_m = state.distance_clear_m + max(odom_delta_m, 0.0)
        if new.distance_clear_m >= cfg.target_distance_m:
            new.phase = AvoiderPhase.DONE
            return AvoiderCommand(
                0.0, 0.0, new,
                f"PASSING->DONE (d={new.distance_clear_m:.2f}m)",
            )
        if new.pass_distance_m >= cfg.pass_clear_distance_m:
            new.phase = AvoiderPhase.REALIGNING
            return AvoiderCommand(
                cfg.forward_speed_mps, 0.0, new,
                f"PASSING->REALIGNING (passed {new.pass_distance_m:.2f}m)",
            )
        return AvoiderCommand(
            cfg.forward_speed_mps, 0.0, new,
            f"PASSING (passed {new.pass_distance_m:.2f}m)",
        )

    # ─── REALIGNING ────────────────────────────────────────────────────
    if state.phase == AvoiderPhase.REALIGNING:
        # Defansif: ON sektorde engel cikarsa AVOIDING'e geri don
        if blocked_enter:
            new = replace(state)
            new.phase = AvoiderPhase.AVOIDING
            new.avoid_direction = pick_avoid_direction(sector)
            new.pass_distance_m = 0.0
            return AvoiderCommand(
                0.0, cfg.turn_speed_rps * new.avoid_direction, new,
                f"REALIGNING->AVOIDING (new obstacle min_r={min_r:.2f}m)",
            )
        yaw_err = wrap_pi(state.home_yaw - current_yaw)
        if abs(yaw_err) < cfg.yaw_tolerance_rad:
            new = replace(state)
            new.phase = AvoiderPhase.DRIVING
            new.avoid_direction = 0
            new.pass_distance_m = 0.0
            return AvoiderCommand(
                cfg.forward_speed_mps, 0.0, new,
                "REALIGNING->DRIVING (aligned)",
            )
        # Ev yonune dogru kuçuk açi ile don (linear küçük, angular dominant)
        sign = 1 if yaw_err > 0 else -1
        return AvoiderCommand(
            0.0, cfg.turn_speed_rps * sign, replace(state),
            f"REALIGNING (yaw_err={yaw_err:.3f})",
        )

    # ─── DONE ──────────────────────────────────────────────────────────
    return AvoiderCommand(0.0, 0.0, replace(state), "DONE")
