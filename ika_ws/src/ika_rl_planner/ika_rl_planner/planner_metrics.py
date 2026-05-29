"""IKA - Yerel planlayici karsilastirma metrikleri (ROS'suz, test edilebilir).

DWB (klasik) ile MPPI (optimizasyon tabanli) - ve ileride ogrenilmis politika -
sim kosumlarini ayni olcutlerle karsilastirmak icin. metrics_recorder_node
robotun yorungesini/hedefini/engellerini toplar, bu cekirdek olcer.

Olcutler:
  - success            : hedefe ulasildi mi (tolerans icinde)
  - path_length_m      : kat edilen toplam yol
  - duration_s         : sure
  - min_clearance_m    : yorunge boyunca en yakin engele mesafe (guvenlik)
  - mean_abs_curvature : yon degisimi / birim yol (rad/m) - puruzsuzluk
  - avg_speed_mps      : ortalama hiz
"""
from __future__ import annotations

import math
from dataclasses import dataclass, asdict
from typing import Sequence, Tuple

Point = Tuple[float, float]


@dataclass
class RunResult:
    success: bool
    duration_s: float
    path_length_m: float
    min_clearance_m: float
    mean_abs_curvature: float
    avg_speed_mps: float
    num_points: int

    def as_dict(self) -> dict:
        return asdict(self)


def path_length(points: Sequence[Point]) -> float:
    """Ardisik noktalar arasi Oklid mesafelerinin toplami."""
    total = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def goal_reached(final_xy: Point, goal_xy: Point, tolerance_m: float) -> bool:
    return math.hypot(final_xy[0] - goal_xy[0],
                      final_xy[1] - goal_xy[1]) <= tolerance_m


def _point_to_segment(p: Point, a: Point, b: Point) -> float:
    """p noktasinin [a, b] segmentine en kisa mesafesi."""
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    if seg2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / seg2
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def min_clearance(points: Sequence[Point],
                  obstacles: Sequence[Point]) -> float:
    """Yorunge (segmentleri) boyunca en yakin engele min mesafesi.

    Nokta-segment mesafesi kullanilir; seyrek ornek alinmis yorungelerde de
    dogru (sadece kayit noktalarini degil, aralarini da sayar). Engel yoksa inf.
    """
    if not obstacles or not points:
        return float('inf')
    best = float('inf')
    for ox, oy in obstacles:
        if len(points) == 1:
            d = math.hypot(points[0][0] - ox, points[0][1] - oy)
            best = min(best, d)
            continue
        for a, b in zip(points, points[1:]):
            d = _point_to_segment((ox, oy), a, b)
            if d < best:
                best = d
    return best


def mean_abs_curvature(points: Sequence[Point]) -> float:
    """Birim yol basina ortalama mutlak yon degisimi (rad/m).

    Dusuk = puruzsuz/duz; yuksek = cok donus/zikzak. Cok kisa segmentler
    (gurultu) atlanir.
    """
    if len(points) < 3:
        return 0.0
    headings = []
    seg_lengths = []
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        dx, dy = x1 - x0, y1 - y0
        seg = math.hypot(dx, dy)
        if seg < 1e-4:
            continue
        headings.append(math.atan2(dy, dx))
        seg_lengths.append(seg)
    if len(headings) < 2:
        return 0.0
    total_turn = 0.0
    total_len = 0.0
    for i in range(1, len(headings)):
        dtheta = _wrap_angle(headings[i] - headings[i - 1])
        total_turn += abs(dtheta)
        total_len += seg_lengths[i]
    if total_len < 1e-6:
        return 0.0
    return total_turn / total_len


def _wrap_angle(a: float) -> float:
    """Aciyi [-pi, pi] araligina sar."""
    return math.atan2(math.sin(a), math.cos(a))


def summarize_run(
    points: Sequence[Point],
    *,
    duration_s: float,
    goal_xy: Point,
    obstacles: Sequence[Point] = (),
    goal_tolerance_m: float = 0.25,
) -> RunResult:
    """Bir kosumun ham verisinden RunResult uret."""
    n = len(points)
    plen = path_length(points)
    success = bool(n > 0 and goal_reached(points[-1], goal_xy, goal_tolerance_m))
    clearance = min_clearance(points, obstacles)
    curv = mean_abs_curvature(points)
    avg_speed = (plen / duration_s) if duration_s > 1e-6 else 0.0
    return RunResult(
        success=success,
        duration_s=round(duration_s, 3),
        path_length_m=round(plen, 3),
        min_clearance_m=(round(clearance, 3)
                         if not math.isinf(clearance) else float('inf')),
        mean_abs_curvature=round(curv, 4),
        avg_speed_mps=round(avg_speed, 3),
        num_points=n,
    )
