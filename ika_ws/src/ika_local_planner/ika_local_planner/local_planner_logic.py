"""Local planner core — candidate corridor scoring + local waypoint seçimi.

Robot frame (REP-103): +x ileri, +y sol. Costmap aynı frame'de.
`pose` ve `target_waypoint` dünya frame'inde — robot frame'e çevirilir.

Strateji (TASK-4A): A* / DWA değil, basit yet ölçeklenebilir
candidate-corridor scoring. İleride aynı arayüzle daha gelişmiş planlayıcı
takılabilir.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from ika_local_planner.local_costmap import (
    LocalCostmap, query_cost, is_occupied,
)
from ika_local_planner.semantic_policy import (
    BehaviorDecision, BehaviorMode,
)


# ════════════════════════════════════════════════════════════════════════
# Veri sınıfları
# ════════════════════════════════════════════════════════════════════════

@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class Waypoint:
    x: float
    y: float


@dataclass(frozen=True)
class PlannerConfig:
    lookahead_m: float = 1.2
    lateral_offsets: Tuple[float, ...] = (-1.2, -0.8, -0.4, 0.0, 0.4, 0.8, 1.2)
    safety_cost_threshold: float = 0.65
    default_speed_mps: float = 0.22
    slow_speed_mps: float = 0.12
    # Skor: candidate_cost + alpha * abs(offset - target_offset)
    goal_alignment_weight: float = 0.5
    # Bir candidate "boş" sayılması için ray boyunca max cost eşiği
    ray_clear_cost_threshold: float = 0.65


@dataclass
class LocalPlan:
    success: bool
    mode: str
    local_waypoint: Optional[Waypoint]
    speed_mps: float
    reason: str
    rejoin_required: bool = False


# ════════════════════════════════════════════════════════════════════════
# Geometri yardımcıları
# ════════════════════════════════════════════════════════════════════════

def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _world_to_robot(pose: Pose2D, wx: float, wy: float) -> Tuple[float, float]:
    """Dünya pozunu robot frame'ine çevir."""
    dx = wx - pose.x
    dy = wy - pose.y
    c, s = math.cos(-pose.yaw), math.sin(-pose.yaw)
    rx = dx * c - dy * s
    ry = dx * s + dy * c
    return (rx, ry)


def _robot_to_world(pose: Pose2D, rx: float, ry: float) -> Tuple[float, float]:
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    wx = pose.x + rx * c - ry * s
    wy = pose.y + rx * s + ry * c
    return (wx, wy)


# ════════════════════════════════════════════════════════════════════════
# Düşük seviye API
# ════════════════════════════════════════════════════════════════════════

def is_path_blocked(
    costmap: LocalCostmap,
    rx_start: float,
    ry_start: float,
    rx_end: float,
    ry_end: float,
    threshold: float = 0.65,
    step_m: float = 0.10,
) -> bool:
    """İki robot-frame nokta arasında düz çizgi engelli mi?"""
    dx = rx_end - rx_start
    dy = ry_end - ry_start
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return is_occupied(costmap, rx_start, ry_start, threshold)
    steps = max(int(dist / step_m), 1)
    for k in range(steps + 1):
        t = k / steps
        x = rx_start + t * dx
        y = ry_start + t * dy
        if query_cost(costmap, x, y) >= threshold:
            return True
    return False


def _ray_cost_sum(
    costmap: LocalCostmap,
    rx_end: float,
    ry_end: float,
    step_m: float = 0.10,
) -> Tuple[float, float]:
    """Robot'tan (0,0)'dan candidate'a ray boyunca (sum, max) cost."""
    dist = math.hypot(rx_end, ry_end)
    if dist < 1e-6:
        c = query_cost(costmap, 0.0, 0.0)
        return (c, c)
    steps = max(int(dist / step_m), 1)
    csum = 0.0
    cmax = 0.0
    for k in range(steps + 1):
        t = k / steps
        c = query_cost(costmap, t * rx_end, t * ry_end)
        csum += c
        if c > cmax:
            cmax = c
    return (csum, cmax)


def score_candidate_corridors(
    costmap: LocalCostmap,
    lookahead: float,
    lateral_offsets: Sequence[float],
    target_offset_y: float,
    config: PlannerConfig,
) -> List[Tuple[float, float, float, bool]]:
    """Her candidate için (offset_y, score, max_cost, blocked) listesi.

    score düşük = iyi. Blocked candidate'lar listeye dahil edilir ama
    çağıran filtre edebilir.
    """
    out: List[Tuple[float, float, float, bool]] = []
    for off in lateral_offsets:
        csum, cmax = _ray_cost_sum(costmap, lookahead, off)
        blocked = cmax >= config.ray_clear_cost_threshold
        alignment_pen = abs(off - target_offset_y) * config.goal_alignment_weight
        score = csum + alignment_pen + (1.0 if blocked else 0.0) * 10.0
        out.append((off, score, cmax, blocked))
    return out


def choose_bypass_side(
    candidates: Sequence[Tuple[float, float, float, bool]],
    preferred_side: str = 'auto',
) -> Optional[Tuple[float, float, float, bool]]:
    """Skorlanmış candidate listesinden en iyi free olanı seç.

    preferred_side: 'left' → off>0 önceliği; 'right' → off<0 önceliği.
    Free yoksa None döner.
    """
    free = [c for c in candidates if not c[3]]
    if not free:
        return None
    # preferred side önceliği: skoru +/- küçük bonus ile etkile
    def _eff_score(c):
        off, s, cmax, _ = c
        bias = 0.0
        if preferred_side == 'left' and off > 0:
            bias = -0.1
        elif preferred_side == 'right' and off < 0:
            bias = -0.1
        return s + bias
    free_sorted = sorted(free, key=_eff_score)
    return free_sorted[0]


# ════════════════════════════════════════════════════════════════════════
# Yüksek seviyeli API
# ════════════════════════════════════════════════════════════════════════

def plan_local_waypoint(
    pose: Pose2D,
    target_waypoint: Waypoint,
    costmap: LocalCostmap,
    behavior_decision: BehaviorDecision,
    config: Optional[PlannerConfig] = None,
) -> LocalPlan:
    """Costmap + hedef + davranış → lokal waypoint + hız.

    - DRIVE: doğrudan hedefe yönelmiş lookahead waypoint.
    - GENERIC_BYPASS / BYPASS_*: candidate corridor scoring.
    - STOP_AND_BYPASS / SLOW_CHECK_AND_BYPASS: aynı + düşük hız.
    - CORRIDOR_FOLLOW: lateral=0 öncelik, hız düşük.
    - HOLD: success=False.
    - RAMP_ALIGN: y=0 lookahead + düşük hız.
    """
    cfg = config or PlannerConfig()
    mode = behavior_decision.mode

    # HOLD modu — durduralım
    if mode == BehaviorMode.HOLD:
        return LocalPlan(
            success=False, mode=mode.value, local_waypoint=None,
            speed_mps=0.0, reason="HOLD: all sides blocked",
        )

    # Hedef robot frame'inde
    tgt_rx, tgt_ry = _world_to_robot(pose, target_waypoint.x, target_waypoint.y)
    # Eğer hedef arkada veya çok yakındaysa — düz forward ile lookahead
    if tgt_rx <= 0.0:
        tgt_rx = cfg.lookahead_m

    # Hız ölçeği: davranış profilinden
    speed = cfg.default_speed_mps * behavior_decision.speed_scale
    if mode in (BehaviorMode.STOP_AND_BYPASS,
                BehaviorMode.SLOW_CHECK_AND_BYPASS,
                BehaviorMode.RAMP_ALIGN):
        speed = min(speed, cfg.slow_speed_mps)

    # DRIVE: hedef doğrudan açıksa düz ilerle
    lookahead = cfg.lookahead_m
    # Hedefe doğru y-offset
    if tgt_rx > 0:
        target_y_at_lookahead = tgt_ry * (lookahead / max(tgt_rx, 1e-3))
        # Lateral offsets aralığında tut
        max_off = max(abs(o) for o in cfg.lateral_offsets)
        target_y_at_lookahead = max(-max_off, min(max_off, target_y_at_lookahead))
    else:
        target_y_at_lookahead = 0.0

    if mode == BehaviorMode.DRIVE:
        # Düz forward — ama tedbiren önü kontrol et
        if not is_path_blocked(costmap, 0.0, 0.0,
                               lookahead, target_y_at_lookahead,
                               threshold=cfg.safety_cost_threshold):
            wx, wy = _robot_to_world(pose, lookahead, target_y_at_lookahead)
            return LocalPlan(
                success=True, mode=mode.value,
                local_waypoint=Waypoint(wx, wy),
                speed_mps=speed,
                reason="DRIVE clear",
            )
        # Önü bloklu → generic bypass'a düş
        mode = BehaviorMode.GENERIC_BYPASS

    # CORRIDOR_FOLLOW: lateral=0 öncelikli, gap analizi
    if mode == BehaviorMode.CORRIDOR_FOLLOW:
        # Önü açıksa düz ileri
        if not is_path_blocked(costmap, 0.0, 0.0, lookahead, 0.0,
                               threshold=cfg.safety_cost_threshold):
            wx, wy = _robot_to_world(pose, lookahead, 0.0)
            return LocalPlan(
                success=True, mode=mode.value,
                local_waypoint=Waypoint(wx, wy),
                speed_mps=speed,
                reason="CORRIDOR_FOLLOW center",
            )
        # Aksi halde candidate scoring'e düş

    # Candidate-corridor scoring (BYPASS / GENERIC / RAMP_ALIGN / SLOW_*)
    candidates = score_candidate_corridors(
        costmap, lookahead, cfg.lateral_offsets,
        target_offset_y=target_y_at_lookahead, config=cfg,
    )
    # preferred_side: davranış kararından veya BYPASS_LEFT/RIGHT modundan
    pref = behavior_decision.preferred_side
    if mode == BehaviorMode.BYPASS_LEFT:
        pref = 'left'
    elif mode == BehaviorMode.BYPASS_RIGHT:
        pref = 'right'

    best = choose_bypass_side(candidates, preferred_side=pref)
    if best is None:
        return LocalPlan(
            success=False, mode=BehaviorMode.HOLD.value,
            local_waypoint=None, speed_mps=0.0,
            reason="no free candidate corridor — HOLD",
        )
    off, score, cmax, _ = best
    wx, wy = _robot_to_world(pose, lookahead, off)
    return LocalPlan(
        success=True, mode=mode.value,
        local_waypoint=Waypoint(wx, wy),
        speed_mps=speed,
        reason=(f"{mode.value}: off={off:+.2f} score={score:.2f} "
                f"max_cost={cmax:.2f}"),
        rejoin_required=(abs(off) > 0.05),
    )
