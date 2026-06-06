"""IKA — Goal-Aware Reaktif Engel Kaçınma Çekirdeği (ROS'suz, saf-Python).

Defense-in-depth Katman 2 (`docs/avoidance_architecture.md`).

PROFESYONEL TASARIM:
    - Goal-direction awareness (hedefe yönlü, random değil)
    - Multi-sensor: lidar + camera DL (vision_msgs/Detection3DArray)
    - Initial planning: start'ta heading'i goal'a göre belirle
    - Dynamic re-planning: engel görünce yan geç, sonra heading'i goal'a güncelle
    - Hysteresis (chattering yok)
    - Defansif: her durumda yakın engel → AVOIDING

State machine (4 calisma fazi + DONE):

    DRIVING (goal heading'i takip et)
        |
        | engel(lidar) VEYA detection(camera)
        v
    AVOIDING (yerinde dön — goal'a yakın boş yönü seç)
        |
        | front clear (hysteresis)
        v
    PASSING (engelin yanından ileri sür)
        |
        | yan mesafe yeterli (pass_clear_distance_m)
        v
    REALIGNING (goal heading'e geri dön — DİNAMİK PLAN REVİZYONU)
        |
        | yaw yakın goal_heading
        v
    DRIVING (yeniden goal'a yönlü ilerle)

Her durumda lidar/camera engel görünce AVOIDING'e atlama önceliği.

TASARIM KARARI — hazard_state KAPATILDI (lidar+camera only):
    Sim'de terrain_perception yanlış sınıflandırma yapıyor (SAFE alanı
    SLOW sayıyor). Avoider yanlış tepki veriyordu. Çözüm: hazard_state
    avoider'a etki etmesin. Sadece kesin sensör verisi (lidar mesafesi +
    DL detection 3D bbox) tetikleyici.

    Hazard fusion safety_supervisor tarafından tüketilir (ayrı katman).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import List, Optional, Sequence, Tuple


class AvoiderPhase(str, Enum):
    DRIVING = "DRIVING"
    AVOIDING = "AVOIDING"
    PASSING = "PASSING"
    REALIGNING = "REALIGNING"
    DONE = "DONE"


@dataclass(frozen=True)
class AvoiderConfig:
    """Avoider parametreleri."""
    forward_speed_mps: float = 0.25
    turn_speed_rps: float = 0.5
    obstacle_distance_m: float = 0.35       # lidar tetikleyici eşik
    release_distance_m: float = 0.60        # AVOIDING'den çıkış (hysteresis)
    # KULLANICI: "Engele yaklaşmadan erken dönüyor"
    # Camera threshold lidar'dan biraz daha geniş tutuldu (0.50m).
    # Daha uzaktan tetiklemez. Lidar 0.35m birincil.
    camera_detection_distance_m: float = 0.50
    front_arc_deg: float = 50.0
    pass_clear_distance_m: float = 0.40
    target_distance_m: float = 10000.0      # mission mode: sonsuz
    yaw_tolerance_rad: float = 0.15
    # Heading correction DRIVING'de KAPATILDI.
    # Sebep: yaw noise (EKF, odom drift) sürekli minik düzeltmelere yol
    # açıyordu — robot "saçma sapan" tepkiler veriyordu. DRIVING düz gider.
    # Heading düzeltmesi sadece REALIGNING fazında (engel sonrası).
    heading_kp: float = 0.0                 # 0 = devre dışı
    max_heading_correction_rps: float = 0.0
    heading_critical_err_rad: float = math.inf  # asla tetiklenmez


@dataclass
class AvoiderState:
    """Cekirdegin canli durumu."""
    phase: AvoiderPhase = AvoiderPhase.DRIVING
    goal_heading_rad: float = 0.0           # ASIL goal yönü (radyan, world frame)
    distance_clear_m: float = 0.0
    avoid_direction: int = 0
    pass_distance_m: float = 0.0


# hazard_state KAPATILDI — bkz. modül docstring.
# Avoider sadece doğrudan sensör verisi (lidar + camera DL) ile çalışır.
HAZARD_BLOCKING = set()


def wrap_pi(angle: float) -> float:
    """Aciyi [-pi, pi] araligina sar."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _front_sector_indices(num_rays: int, total_fov_rad: float,
                          front_arc_rad: float) -> Tuple[int, int]:
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
    lo, hi = _front_sector_indices(len(scan_ranges), total_fov_rad, front_arc_rad)
    sector = [r for r in scan_ranges[lo:hi]
              if r > 0.0 and math.isfinite(r)]
    if not sector:
        return (float('inf'), [])
    return (min(sector), sector)


def pick_avoid_direction_goal_aware(front_sector: Sequence[float],
                                     current_yaw: float,
                                     goal_heading: float) -> int:
    """Engelden kaçınma yönü: hem boş taraf hem goal'a yakın taraf.

    Eğer goal sağdaysa (yaw_err > 0) → sağa eğilim
    Eğer goal soldaysa (yaw_err < 0) → sola eğilim
    Ek: lidar'da daha boş tarafı tercih et (kararlı algoritma).

    Returns: +1 (sol) veya -1 (sağ).
    """
    if not front_sector:
        # Bilgi yoksa goal yönüne göre seç
        yaw_err = wrap_pi(goal_heading - current_yaw)
        return 1 if yaw_err > 0 else -1

    mid = len(front_sector) // 2
    left = [r for r in front_sector[:mid] if math.isfinite(r)]
    right = [r for r in front_sector[mid:] if math.isfinite(r)]
    left_max = max(left) if left else 0.0
    right_max = max(right) if right else 0.0

    # Lidar boşluk farkı belirgin değilse (örneğin %20'den az) → goal yönü kazanır
    if abs(left_max - right_max) < 0.20 * max(left_max, right_max, 1.0):
        yaw_err = wrap_pi(goal_heading - current_yaw)
        return 1 if yaw_err > 0 else -1

    return 1 if left_max > right_max else -1


# Backward compat: eski testler için
def pick_avoid_direction(front_sector: Sequence[float]) -> int:
    return pick_avoid_direction_goal_aware(front_sector, 0.0, 0.0)


@dataclass
class AvoiderCommand:
    linear_x: float
    angular_z: float
    next_state: AvoiderState
    reason: str = ""

    def as_tuple(self) -> Tuple[float, float, AvoiderState, str]:
        return (self.linear_x, self.angular_z, self.next_state, self.reason)


def _is_blocked(min_range: float, hazard_action: str,
                threshold_m: float,
                camera_detection_close: bool = False) -> bool:
    """Engel tespiti: lidar yakın VEYA camera close detection.
    hazard_action varsayılan olarak göz ardı edilir (HAZARD_BLOCKING boş).
    """
    obstacle_close = min_range < threshold_m
    hazard_blocked = (hazard_action or "").upper() in HAZARD_BLOCKING
    return obstacle_close or hazard_blocked or camera_detection_close


def decide(state: AvoiderState,
           scan_ranges: Sequence[float],
           scan_fov_rad: float,
           hazard_action: str,
           current_yaw: float,
           odom_delta_m: float,
           cfg: AvoiderConfig,
           camera_obstacle_distance_m: float = float('inf')) -> AvoiderCommand:
    """Goal-aware reactive obstacle avoidance — tek tik karar.

    Args:
        state         : mevcut AvoiderState
        scan_ranges   : LaserScan.ranges (m)
        scan_fov_rad  : tarayicinin toplam FOV'u (radyan)
        hazard_action : (ignored, geriye uyumluluk için tutulur)
        current_yaw   : robot mevcut yaw (radyan, world frame)
        odom_delta_m  : bu tikteki kat edilen mesafe (m)
        cfg           : AvoiderConfig
        camera_obstacle_distance_m: en yakın DL detection mesafesi
                                    (inf ise yok)

    Goal heading state.goal_heading_rad'da. start_delay sonra node bunu
    current_yaw olarak set eder (robot başlangıçta hangi yöne bakıyorsa
    onu hedef yapar).

    Returns:
        AvoiderCommand
    """
    front_arc_rad = math.radians(cfg.front_arc_deg)
    min_r, sector = front_min_range(scan_ranges, scan_fov_rad, front_arc_rad)

    # Multi-sensor: lidar VEYA camera detection yakın
    camera_close = camera_obstacle_distance_m < cfg.camera_detection_distance_m

    blocked_enter = _is_blocked(min_r, hazard_action,
                                 cfg.obstacle_distance_m, camera_close)
    blocked_exit = _is_blocked(min_r, hazard_action,
                                cfg.release_distance_m, camera_close)

    # Goal heading hata (hedeften ne kadar sapmış)
    yaw_err_to_goal = wrap_pi(state.goal_heading_rad - current_yaw)

    # ─── DRIVING ────────────────────────────────────────────────────────
    if state.phase == AvoiderPhase.DRIVING:
        if blocked_enter:
            new = replace(state)
            new.avoid_direction = pick_avoid_direction_goal_aware(
                sector, current_yaw, state.goal_heading_rad)
            new.phase = AvoiderPhase.AVOIDING
            new.pass_distance_m = 0.0
            return AvoiderCommand(
                0.0, cfg.turn_speed_rps * new.avoid_direction, new,
                f"DRIVING->AVOIDING (lidar_min={min_r:.2f}m, "
                f"cam={camera_obstacle_distance_m:.2f}m, "
                f"dir={new.avoid_direction:+d})",
            )

        new = replace(state)
        new.distance_clear_m = state.distance_clear_m + max(odom_delta_m, 0.0)
        if new.distance_clear_m >= cfg.target_distance_m:
            new.phase = AvoiderPhase.DONE
            return AvoiderCommand(0.0, 0.0, new,
                                  f"DRIVING->DONE (d={new.distance_clear_m:.2f}m)")

        # Goal heading'e doğru sürekli minik düzeltme — yol planı revizyonu
        if abs(yaw_err_to_goal) > cfg.heading_critical_err_rad:
            # Çok sapmışız, dur ve dön
            sign = 1 if yaw_err_to_goal > 0 else -1
            return AvoiderCommand(
                0.0, cfg.turn_speed_rps * sign, new,
                f"DRIVING (heading correction, err={yaw_err_to_goal:.2f})",
            )
        # Normal sürüş — minik heading düzeltme + ileri
        angular_corr = max(-cfg.max_heading_correction_rps,
                           min(cfg.max_heading_correction_rps,
                               cfg.heading_kp * yaw_err_to_goal))
        return AvoiderCommand(
            cfg.forward_speed_mps, angular_corr, new,
            f"DRIVING (d={new.distance_clear_m:.2f}m, "
            f"yaw_err={yaw_err_to_goal:.2f})",
        )

    # ─── AVOIDING ──────────────────────────────────────────────────────
    if state.phase == AvoiderPhase.AVOIDING:
        if blocked_exit:
            return AvoiderCommand(
                0.0, cfg.turn_speed_rps * state.avoid_direction,
                replace(state),
                f"AVOIDING (still blocked, lidar={min_r:.2f}m)",
            )
        new = replace(state)
        new.phase = AvoiderPhase.PASSING
        new.pass_distance_m = 0.0
        return AvoiderCommand(
            cfg.forward_speed_mps, 0.0, new,
            f"AVOIDING->PASSING (cleared, lidar={min_r:.2f}m)",
        )

    # ─── PASSING ───────────────────────────────────────────────────────
    if state.phase == AvoiderPhase.PASSING:
        if blocked_enter:
            new = replace(state)
            new.phase = AvoiderPhase.AVOIDING
            new.avoid_direction = pick_avoid_direction_goal_aware(
                sector, current_yaw, state.goal_heading_rad)
            new.pass_distance_m = 0.0
            return AvoiderCommand(
                0.0, cfg.turn_speed_rps * new.avoid_direction, new,
                f"PASSING->AVOIDING (new obstacle, lidar={min_r:.2f}m)",
            )

        new = replace(state)
        new.pass_distance_m = state.pass_distance_m + max(odom_delta_m, 0.0)
        new.distance_clear_m = state.distance_clear_m + max(odom_delta_m, 0.0)
        if new.distance_clear_m >= cfg.target_distance_m:
            new.phase = AvoiderPhase.DONE
            return AvoiderCommand(
                0.0, 0.0, new, f"PASSING->DONE (d={new.distance_clear_m:.2f}m)",
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
        if blocked_enter:
            new = replace(state)
            new.phase = AvoiderPhase.AVOIDING
            new.avoid_direction = pick_avoid_direction_goal_aware(
                sector, current_yaw, state.goal_heading_rad)
            new.pass_distance_m = 0.0
            return AvoiderCommand(
                0.0, cfg.turn_speed_rps * new.avoid_direction, new,
                f"REALIGNING->AVOIDING (new obstacle, lidar={min_r:.2f}m)",
            )

        # Goal heading'e dön — bu DİNAMİK PLAN REVİZYONU
        if abs(yaw_err_to_goal) < cfg.yaw_tolerance_rad:
            new = replace(state)
            new.phase = AvoiderPhase.DRIVING
            new.avoid_direction = 0
            new.pass_distance_m = 0.0
            return AvoiderCommand(
                cfg.forward_speed_mps, 0.0, new,
                f"REALIGNING->DRIVING (aligned, yaw_err={yaw_err_to_goal:.2f})",
            )
        sign = 1 if yaw_err_to_goal > 0 else -1
        return AvoiderCommand(
            0.0, cfg.turn_speed_rps * sign, replace(state),
            f"REALIGNING (yaw_err={yaw_err_to_goal:.2f})",
        )

    # ─── DONE ──────────────────────────────────────────────────────────
    return AvoiderCommand(0.0, 0.0, replace(state), "DONE")
