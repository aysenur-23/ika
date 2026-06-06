"""Local costmap — pure-Python 2D occupancy/cost grid.

Robot frame (REP-103):
    +x = forward, +y = left, yaw CCW positive.

Grid covers x ∈ [0, width_m] (forward only) and
            y ∈ [-height_m/2, +height_m/2] (lateral around robot).
Resolution `resolution_m` per cell.

Costs ∈ [0.0, 1.0]:
    0.0  = free / unknown free
    high = obstacle / inflated zone
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple


# ════════════════════════════════════════════════════════════════════════
# Veri sınıfları
# ════════════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class CostmapConfig:
    width_m: float = 4.0         # ileri menzil
    height_m: float = 4.0        # yanal toplam
    resolution_m: float = 0.10
    obstacle_cost: float = 1.0
    inflation_radius_m: float = 0.35
    inflation_decay: float = 0.6  # her hücre artışında çarpan
    unknown_cost: float = 0.5    # bilinmeyen detection ağırlığı


@dataclass
class Detection:
    """Semantic detection in robot frame (x ileri, y sol)."""
    class_id: str
    x: float
    y: float
    confidence: float = 1.0


@dataclass
class LocalCostmap:
    """2D cost grid. cells[gy][gx] ∈ [0, 1]."""
    config: CostmapConfig
    nx: int
    ny: int
    cells: List[List[float]] = field(default_factory=list)

    # Konvansiyon: dünya koord (robot frame) → hücre indexi
    def world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        cfg = self.config
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        gx = int(x / cfg.resolution_m)
        gy = int((y + cfg.height_m / 2.0) / cfg.resolution_m)
        if 0 <= gx < self.nx and 0 <= gy < self.ny:
            return (gx, gy)
        return None

    def cell_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        cfg = self.config
        x = (gx + 0.5) * cfg.resolution_m
        y = (gy + 0.5) * cfg.resolution_m - cfg.height_m / 2.0
        return (x, y)

    def max_cost(self) -> float:
        m = 0.0
        for row in self.cells:
            rm = max(row) if row else 0.0
            if rm > m:
                m = rm
        return m


# ════════════════════════════════════════════════════════════════════════
# Yardımcılar
# ════════════════════════════════════════════════════════════════════════

def _empty_grid(cfg: CostmapConfig) -> LocalCostmap:
    nx = max(int(round(cfg.width_m / cfg.resolution_m)), 1)
    ny = max(int(round(cfg.height_m / cfg.resolution_m)), 1)
    cells = [[0.0 for _ in range(nx)] for _ in range(ny)]
    return LocalCostmap(config=cfg, nx=nx, ny=ny, cells=cells)


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _stamp_with_inflation(cm: LocalCostmap, gx: int, gy: int,
                          base_cost: float) -> None:
    """Bir engel hücresi + etrafına decay'li inflation stamp et."""
    cfg = cm.config
    if not (0 <= gx < cm.nx and 0 <= gy < cm.ny):
        return
    # Merkez hücre
    if cm.cells[gy][gx] < base_cost:
        cm.cells[gy][gx] = base_cost
    # Inflation halkaları
    radius_cells = max(int(round(cfg.inflation_radius_m / cfg.resolution_m)), 0)
    if radius_cells == 0:
        return
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx == 0 and dy == 0:
                continue
            d = math.hypot(dx, dy)
            if d > radius_cells:
                continue
            ix, iy = gx + dx, gy + dy
            if not (0 <= ix < cm.nx and 0 <= iy < cm.ny):
                continue
            # Decay: dış halkalarda azalan cost
            decay_steps = max(int(round(d)), 1)
            cost = base_cost * (cfg.inflation_decay ** decay_steps)
            if cost > cm.cells[iy][ix]:
                cm.cells[iy][ix] = cost


# ════════════════════════════════════════════════════════════════════════
# Yüksek seviyeli API
# ════════════════════════════════════════════════════════════════════════

def build_costmap_from_scan(
    ranges: Sequence[float],
    angle_min_rad: float,
    angle_increment_rad: float,
    config: Optional[CostmapConfig] = None,
) -> LocalCostmap:
    """Lidar scan → cost grid.

    - Geçersiz range (0/negatif/NaN/inf) elenir.
    - Her ışın için (x,y) = (r cosθ, r sinθ) hücresine obstacle_cost basılır.
    - Inflation uygulanır.
    """
    cfg = config or CostmapConfig()
    cm = _empty_grid(cfg)
    if not ranges:
        return cm
    for i, r in enumerate(ranges):
        try:
            rf = float(r)
        except (TypeError, ValueError):
            continue
        if rf <= 0.0 or not math.isfinite(rf):
            continue
        ang = _wrap_pi(angle_min_rad + i * angle_increment_rad)
        x = rf * math.cos(ang)
        y = rf * math.sin(ang)
        cell = cm.world_to_cell(x, y)
        if cell is None:
            continue
        gx, gy = cell
        _stamp_with_inflation(cm, gx, gy, cfg.obstacle_cost)
    return cm


def overlay_detections(
    costmap: LocalCostmap,
    detections: Sequence[Detection],
    semantic_weights: Optional[Dict[str, float]] = None,
) -> LocalCostmap:
    """Detection listesini cost grid'e ağırlıklı bas (in-place + return).

    semantic_weights: class_id → cost in [0,1]. Bulunamazsa cfg.unknown_cost.
    """
    cfg = costmap.config
    weights = semantic_weights or {}
    for det in detections:
        if det is None:
            continue
        try:
            x, y = float(det.x), float(det.y)
        except (TypeError, ValueError):
            continue
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        w = weights.get(det.class_id, cfg.unknown_cost)
        w = max(0.0, min(1.0, float(w))) * max(0.0, min(1.0, float(det.confidence)))
        cell = costmap.world_to_cell(x, y)
        if cell is None:
            continue
        gx, gy = cell
        _stamp_with_inflation(costmap, gx, gy, w)
    return costmap


def query_cost(costmap: LocalCostmap, x: float, y: float) -> float:
    """Verilen dünya koord → hücre cost. Grid dışı = 0.0."""
    cell = costmap.world_to_cell(x, y)
    if cell is None:
        return 0.0
    gx, gy = cell
    return costmap.cells[gy][gx]


def is_occupied(costmap: LocalCostmap, x: float, y: float,
                threshold: float = 0.7) -> bool:
    return query_cost(costmap, x, y) >= threshold


def find_free_lateral_gaps(
    costmap: LocalCostmap,
    lookahead_x: float,
    corridor_width: float,
    occupied_threshold: float = 0.65,
) -> List[Tuple[float, float]]:
    """`lookahead_x` mesafesindeki y dilimi üzerinde boş kanal aralıkları.

    Returns: [(y_min, y_max), ...] sıralı, her biri >= corridor_width.
    """
    if lookahead_x <= 0:
        return []
    cfg = costmap.config
    gx = int(lookahead_x / cfg.resolution_m)
    if gx < 0 or gx >= costmap.nx:
        return []
    free_runs: List[Tuple[float, float]] = []
    run_start_gy: Optional[int] = None
    for gy in range(costmap.ny):
        occupied = costmap.cells[gy][gx] >= occupied_threshold
        if not occupied:
            if run_start_gy is None:
                run_start_gy = gy
        else:
            if run_start_gy is not None:
                y0 = costmap.cell_to_world(gx, run_start_gy)[1] \
                    - cfg.resolution_m / 2.0
                y1 = costmap.cell_to_world(gx, gy - 1)[1] \
                    + cfg.resolution_m / 2.0
                if (y1 - y0) >= corridor_width:
                    free_runs.append((y0, y1))
                run_start_gy = None
    if run_start_gy is not None:
        y0 = costmap.cell_to_world(gx, run_start_gy)[1] \
            - cfg.resolution_m / 2.0
        y1 = costmap.cell_to_world(gx, costmap.ny - 1)[1] \
            + cfg.resolution_m / 2.0
        if (y1 - y0) >= corridor_width:
            free_runs.append((y0, y1))
    return free_runs


def summarize_costmap(
    costmap: LocalCostmap,
    front_window_x: float = 1.5,
    side_window_y: float = 1.0,
    occupied_threshold: float = 0.65,
) -> Dict[str, float]:
    """Planner / policy için hızlı özet."""
    cfg = costmap.config
    front_blocked = False
    left_blocked = False
    right_blocked = False
    min_obs = float('inf')
    nx_lim = min(int(front_window_x / cfg.resolution_m), costmap.nx)
    for gx in range(nx_lim):
        for gy in range(costmap.ny):
            c = costmap.cells[gy][gx]
            if c < occupied_threshold:
                continue
            wx, wy = costmap.cell_to_world(gx, gy)
            d = math.hypot(wx, wy)
            if d < min_obs:
                min_obs = d
            if abs(wy) <= 0.35:
                front_blocked = True
            if 0.0 < wy <= side_window_y:
                left_blocked = True
            if -side_window_y <= wy < 0.0:
                right_blocked = True
    return {
        'front_blocked': front_blocked,
        'left_blocked': left_blocked,
        'right_blocked': right_blocked,
        'min_obs_dist': min_obs if math.isfinite(min_obs) else -1.0,
    }
