"""Semantic obstacle policy — sınıf → davranış kararı.

Bilinen sınıf taksonomisi (CLAUDE.md §"Tez Engel Taksonomisi"):
    box, pole, cone, pothole, threshold, ramp,
    pedestrian/person, corridor, ground_patch
Bilinmeyenler → GENERIC_BYPASS (generic obstacle policy).
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Sequence


class BehaviorMode(str, Enum):
    DRIVE = "drive"
    GENERIC_BYPASS = "generic_bypass"
    BYPASS_LEFT = "bypass_left"
    BYPASS_RIGHT = "bypass_right"
    STOP_AND_BYPASS = "stop_and_bypass"
    SLOW_CHECK_AND_BYPASS = "slow_check_and_bypass"
    CORRIDOR_FOLLOW = "corridor_follow"
    RAMP_ALIGN = "ramp_align"
    HOLD = "hold"


# ════════════════════════════════════════════════════════════════════════
# Class → kategori eşlemesi
# ════════════════════════════════════════════════════════════════════════

# Class id'leri sim_detection_node'dan ham gelir; "label:hazard" formatı
# olabilir (örn. "person:DYNAMIC"). Sadece baş kısmı normalize edilir.
_KNOWN_CLASSES: Dict[str, BehaviorMode] = {
    'box': BehaviorMode.GENERIC_BYPASS,
    'obstacle': BehaviorMode.GENERIC_BYPASS,
    'pole': BehaviorMode.GENERIC_BYPASS,
    'thin_pole': BehaviorMode.GENERIC_BYPASS,
    'cone': BehaviorMode.GENERIC_BYPASS,
    'pothole': BehaviorMode.SLOW_CHECK_AND_BYPASS,
    'pit': BehaviorMode.SLOW_CHECK_AND_BYPASS,
    'trench': BehaviorMode.SLOW_CHECK_AND_BYPASS,
    'threshold': BehaviorMode.SLOW_CHECK_AND_BYPASS,
    'kerb': BehaviorMode.SLOW_CHECK_AND_BYPASS,
    'ramp': BehaviorMode.RAMP_ALIGN,
    'pedestrian': BehaviorMode.STOP_AND_BYPASS,
    'person': BehaviorMode.STOP_AND_BYPASS,
    'corridor': BehaviorMode.CORRIDOR_FOLLOW,
    'wall': BehaviorMode.CORRIDOR_FOLLOW,
    'ground_patch': BehaviorMode.DRIVE,
    'surface': BehaviorMode.DRIVE,
}

# Davranış başına default profil
_PROFILE: Dict[BehaviorMode, Dict[str, float]] = {
    BehaviorMode.DRIVE:                 {'speed_scale': 1.0, 'hold_time_s': 0.0},
    BehaviorMode.GENERIC_BYPASS:        {'speed_scale': 0.8, 'hold_time_s': 0.0},
    BehaviorMode.BYPASS_LEFT:           {'speed_scale': 0.8, 'hold_time_s': 0.0},
    BehaviorMode.BYPASS_RIGHT:          {'speed_scale': 0.8, 'hold_time_s': 0.0},
    BehaviorMode.STOP_AND_BYPASS:       {'speed_scale': 0.4, 'hold_time_s': 1.5},
    BehaviorMode.SLOW_CHECK_AND_BYPASS: {'speed_scale': 0.5, 'hold_time_s': 0.5},
    BehaviorMode.CORRIDOR_FOLLOW:       {'speed_scale': 0.7, 'hold_time_s': 0.0},
    BehaviorMode.RAMP_ALIGN:            {'speed_scale': 0.5, 'hold_time_s': 0.0},
    BehaviorMode.HOLD:                  {'speed_scale': 0.0, 'hold_time_s': 0.0},
}

# Davranış önceliği — birden fazla sınıf varsa daha defansif kazanır
_PRIORITY: Dict[BehaviorMode, int] = {
    BehaviorMode.DRIVE: 0,
    BehaviorMode.GENERIC_BYPASS: 1,
    BehaviorMode.BYPASS_LEFT: 1,
    BehaviorMode.BYPASS_RIGHT: 1,
    BehaviorMode.CORRIDOR_FOLLOW: 2,
    BehaviorMode.RAMP_ALIGN: 2,
    BehaviorMode.SLOW_CHECK_AND_BYPASS: 3,
    BehaviorMode.STOP_AND_BYPASS: 4,
    BehaviorMode.HOLD: 5,
}


@dataclass
class BehaviorDecision:
    mode: BehaviorMode
    target_class: str = "none"
    preferred_side: str = "auto"   # 'left' | 'right' | 'auto'
    speed_scale: float = 1.0
    hold_time_s: float = 0.0
    reason: str = ""


# ════════════════════════════════════════════════════════════════════════
# Yardımcılar
# ════════════════════════════════════════════════════════════════════════

def _normalize_class(class_id: str) -> str:
    """'person:DYNAMIC' → 'person'. Boş/None → 'unknown'."""
    if not class_id:
        return 'unknown'
    head = str(class_id).split(':', 1)[0].strip().lower()
    return head or 'unknown'


def _mode_for_class(class_id: str) -> BehaviorMode:
    head = _normalize_class(class_id)
    return _KNOWN_CLASSES.get(head, BehaviorMode.GENERIC_BYPASS)


def _pick_preferred_side(summary: Dict[str, float]) -> str:
    """Costmap özetine göre tercih edilen yan kaçış yönü."""
    left = bool(summary.get('left_blocked', False))
    right = bool(summary.get('right_blocked', False))
    if left and not right:
        return 'right'
    if right and not left:
        return 'left'
    return 'auto'


# ════════════════════════════════════════════════════════════════════════
# Yüksek seviyeli API
# ════════════════════════════════════════════════════════════════════════

def select_behavior(
    detections: Sequence,
    costmap_summary: Optional[Dict[str, float]] = None,
    mission_context: Optional[Dict[str, float]] = None,
) -> BehaviorDecision:
    """Sınıf-aware + costmap-aware davranış kararı.

    detections      : ika_local_planner.local_costmap.Detection veya
                      `.class_id` özniteliği olan herhangi obje listesi.
    costmap_summary : `summarize_costmap` çıktısı (dict).
    mission_context : opsiyonel — şu an kullanılmıyor, gelecek için.
    """
    summary = costmap_summary or {}
    _ = mission_context  # şimdilik kullanılmıyor

    # 1) Detection bazlı en defansif modu seç
    chosen_mode = BehaviorMode.DRIVE
    chosen_class = "none"
    for det in detections or []:
        cls_raw = getattr(det, 'class_id', None)
        if cls_raw is None:
            continue
        m = _mode_for_class(cls_raw)
        if _PRIORITY[m] > _PRIORITY[chosen_mode]:
            chosen_mode = m
            chosen_class = _normalize_class(cls_raw)

    # 2) Detection yok / sadece DRIVE ama costmap engelliyse → generic
    front_blocked = bool(summary.get('front_blocked', False))
    if chosen_mode == BehaviorMode.DRIVE and front_blocked:
        chosen_mode = BehaviorMode.GENERIC_BYPASS
        chosen_class = 'unknown'

    # 3) Tüm yan ve önler kapalı → HOLD
    if (front_blocked
            and bool(summary.get('left_blocked', False))
            and bool(summary.get('right_blocked', False))):
        chosen_mode = BehaviorMode.HOLD
        return BehaviorDecision(
            mode=chosen_mode, target_class=chosen_class,
            preferred_side='auto', speed_scale=0.0, hold_time_s=0.0,
            reason="all sides blocked",
        )

    # 4) Profil + tercih edilen yan
    profile = _PROFILE[chosen_mode]
    side = _pick_preferred_side(summary)

    reason_bits = [f"mode={chosen_mode.value}"]
    if chosen_class != 'none':
        reason_bits.append(f"class={chosen_class}")
    if front_blocked:
        reason_bits.append("front_blocked")
    if side != 'auto':
        reason_bits.append(f"prefer_side={side}")

    return BehaviorDecision(
        mode=chosen_mode,
        target_class=chosen_class,
        preferred_side=side,
        speed_scale=float(profile['speed_scale']),
        hold_time_s=float(profile['hold_time_s']),
        reason=", ".join(reason_bits),
    )
