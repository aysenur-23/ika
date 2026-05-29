"""IKA - Hibrit hazard fuzyon cekirdegi (ROS'suz, test edilebilir).

DL nesne tespiti ("bu ne / dinamik mi?") ile RANSAC terrain ("zemin gecilebilir
mi?") kararlarini tek bir hazard kararinda birlestirir:

  - fuse_hazard()        -> CLEAR / SLOW / STOP + gerekce  (safety supervisor)
  - detections_to_grid() -> base_link OccupancyGrid hucreleri (costmap layer)

ROS bagimliligi yoktur; ground_plane.py / detector_postprocess.py ile ayni
desende unit test edilir.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Sequence

import numpy as np


# Aksiyon onceligi: buyuk sayi daha kisitlayici.
_ACTION_RANK = {'CLEAR': 0, 'SLOW': 1, 'STOP': 2}


@dataclass
class DetectedObject:
    """base_link frame'inde tespit edilmis nesne (fusion node'un parse ettigi)."""
    x: float
    y: float
    hazard: str          # DYNAMIC / STATIC
    label: str = ''
    confidence: float = 0.0
    range_m: float = field(default=0.0)


@dataclass
class HazardDecision:
    action: str                      # CLEAR / SLOW / STOP
    sources: list                    # ['terrain', 'dynamic_object'] gibi
    terrain_class: str
    dynamic_count: int
    nearest_dynamic_range_m: float   # inf = dinamik nesne yok
    reasons: list


def _stricter(a: str, b: str) -> str:
    return a if _ACTION_RANK[a] >= _ACTION_RANK[b] else b


def fuse_hazard(
    terrain_class: str,
    detections: Iterable[DetectedObject],
    *,
    terrain_stop_classes: Iterable[str] = ('DROPOFF_DANGER', 'IMPASSABLE'),
    terrain_slow_classes: Iterable[str] = ('CAUTION', 'UNKNOWN'),
    dynamic_stop_range_m: float = 0.8,
    dynamic_slow_range_m: float = 2.0,
) -> HazardDecision:
    """Terrain sinifi + dinamik nesneleri birlesik bir aksiyona indir.

    STOP > SLOW > CLEAR onceligi. Hem terrain hem nesne ayni anda kisitlayici
    olabilir; en kisitlayici aksiyon kazanir, tum gerekceler raporlanir.
    """
    stop_set = set(terrain_stop_classes)
    slow_set = set(terrain_slow_classes)

    action = 'CLEAR'
    sources: list = []
    reasons: list = []

    # Terrain katkisi
    if terrain_class in stop_set:
        action = _stricter(action, 'STOP')
        sources.append('terrain')
        reasons.append(f'terrain={terrain_class}')
    elif terrain_class in slow_set:
        action = _stricter(action, 'SLOW')
        sources.append('terrain')
        reasons.append(f'terrain={terrain_class}')

    # Dinamik nesne katkisi
    dets = list(detections)
    dynamic = [d for d in dets if d.hazard == 'DYNAMIC']
    nearest_dyn = min(
        (d.range_m for d in dynamic), default=float('inf'))

    if dynamic:
        if nearest_dyn <= dynamic_stop_range_m:
            action = _stricter(action, 'STOP')
            if 'dynamic_object' not in sources:
                sources.append('dynamic_object')
            reasons.append(f'dynamic@{nearest_dyn:.2f}m')
        elif nearest_dyn <= dynamic_slow_range_m:
            action = _stricter(action, 'SLOW')
            if 'dynamic_object' not in sources:
                sources.append('dynamic_object')
            reasons.append(f'dynamic@{nearest_dyn:.2f}m')

    return HazardDecision(
        action=action,
        sources=sources,
        terrain_class=terrain_class,
        dynamic_count=len(dynamic),
        nearest_dynamic_range_m=nearest_dyn,
        reasons=reasons,
    )


def decision_payload(d: HazardDecision) -> dict:
    """HazardDecision -> JSON-serializable dict (/hazard_state)."""
    nearest = d.nearest_dynamic_range_m
    return {
        'action': d.action,
        'sources': d.sources,
        'terrain_class': d.terrain_class,
        'dynamic_count': d.dynamic_count,
        'nearest_dynamic_range_m':
            None if math.isinf(nearest) else round(nearest, 3),
        'reasons': d.reasons,
    }


def detections_to_grid(
    detections: Iterable[DetectedObject],
    *,
    resolution: float = 0.05,
    size_cells: int = 80,
    obstacle_radius_m: float = 0.15,
    dynamic_cost: int = 100,
    static_cost: int = 100,
):
    """Tespitleri base_link merkezli bir occupancy grid'e isle.

    Grid base_link'te ortalanir: origin = (-size/2, -size/2). Her tespit,
    `obstacle_radius_m` yaricapinda bir disk olarak isaretlenir (kameranin
    nokta tahmininin belirsizligini + nesne genisligini kapsar).

    Donus: (data, meta)
      data  : list[int8], satir-oncelikli (row-major), uzunluk size*size
      meta  : dict(resolution, size_cells, origin_x, origin_y)
    """
    n = int(size_cells)
    res = float(resolution)
    grid = np.zeros((n, n), dtype=np.int16)

    origin_x = -(n * res) / 2.0
    origin_y = -(n * res) / 2.0
    rad_cells = max(0, int(round(obstacle_radius_m / res)))

    for d in detections:
        col = int((d.x - origin_x) / res)
        row = int((d.y - origin_y) / res)
        if not (0 <= col < n and 0 <= row < n):
            continue
        cost = dynamic_cost if d.hazard == 'DYNAMIC' else static_cost
        r0 = max(0, row - rad_cells)
        r1 = min(n, row + rad_cells + 1)
        c0 = max(0, col - rad_cells)
        c1 = min(n, col + rad_cells + 1)
        # Disk: hucre merkezinden uzaklik <= yaricap
        for rr in range(r0, r1):
            for cc in range(c0, c1):
                if (rr - row) ** 2 + (cc - col) ** 2 <= rad_cells ** 2:
                    if cost > grid[rr, cc]:
                        grid[rr, cc] = cost

    meta = {
        'resolution': res,
        'size_cells': n,
        'origin_x': origin_x,
        'origin_y': origin_y,
    }
    return grid.astype(np.int8).flatten().tolist(), meta
