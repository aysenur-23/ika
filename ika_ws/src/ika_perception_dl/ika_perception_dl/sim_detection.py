"""IKA - Sim icin sentetik nesne tespiti cekirdegi (ROS'suz, test edilebilir).

Gazebo'da fiziksel VPU olmadigindan, sim'de DL yolunu (fusion -> costmap ->
safety -> planlayici tepkisi) deterministik test etmek icin kullanilir.
Bilinen/scripted engellerin yer-gercegi konumundan, kameranin FOV + menziline
gore "kusursuz tespit" uretir ve gercek dl_perception_node ile AYNI cikti
kontratini (Detection3DArray, class_id="label:hazard") doldurur.

Modelin gercek dunya dogrulugu burada test EDILMEZ; o, gercek araçla saha
testinde sinanir. Burada sadece tespit -> tepki zinciri sinanir.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple


@dataclass
class SimObstacle:
    """Sim engeli: t=0'da (x0,y0) world, sabit (vx,vy) ile dogrusal hareket."""
    label: str
    hazard: str          # DYNAMIC / STATIC
    x0: float
    y0: float
    vx: float = 0.0
    vy: float = 0.0


@dataclass
class SimDetection:
    label: str
    hazard: str
    x: float             # base_link: ileri+
    y: float             # base_link: sol+
    z: float             # base_link: yukari+
    confidence: float
    range_m: float


def obstacle_position_at(obs: SimObstacle, t: float) -> Tuple[float, float]:
    """Engelin t aninda world konumu (dogrusal hareket)."""
    return (obs.x0 + obs.vx * t, obs.y0 + obs.vy * t)


def world_to_base(ox: float, oy: float,
                  rx: float, ry: float, ryaw: float) -> Tuple[float, float]:
    """World noktasini robot base_link frame'ine tasi (2B).

    base_link: x ileri, y sol. ryaw robotun world'deki yaw'i.
    """
    dx = ox - rx
    dy = oy - ry
    c = math.cos(ryaw)
    s = math.sin(ryaw)
    x_b = c * dx + s * dy
    y_b = -s * dx + c * dy
    return x_b, y_b


def in_fov(x_b: float, y_b: float, *,
           hfov_rad: float, min_range_m: float, max_range_m: float) -> bool:
    """Nokta ileri-bakan kameranin FOV + menzili icinde mi?

    Kamera +x'e bakar; yatay yari-aci hfov_rad/2.
    """
    if x_b <= 0.0:
        return False
    rng = math.hypot(x_b, y_b)
    if rng < min_range_m or rng > max_range_m:
        return False
    angle = math.atan2(y_b, x_b)
    return abs(angle) <= hfov_rad / 2.0


def simulate_detections(
    obstacles: Iterable[SimObstacle],
    t: float,
    robot_x: float, robot_y: float, robot_yaw: float,
    *,
    hfov_rad: float = 1.20,         # ~69 derece (OAK-D Lite RGB yatay FOV)
    min_range_m: float = 0.2,
    max_range_m: float = 6.0,
    nominal_z: float = 0.3,
    confidence: float = 0.95,
) -> list:
    """Robot pozuna ve t anina gore gorunur engellerin tespitlerini uret."""
    out: list = []
    for obs in obstacles:
        ox, oy = obstacle_position_at(obs, t)
        x_b, y_b = world_to_base(ox, oy, robot_x, robot_y, robot_yaw)
        if not in_fov(x_b, y_b, hfov_rad=hfov_rad,
                      min_range_m=min_range_m, max_range_m=max_range_m):
            continue
        out.append(SimDetection(
            label=obs.label,
            hazard=obs.hazard,
            x=round(x_b, 3),
            y=round(y_b, 3),
            z=nominal_z,
            confidence=confidence,
            range_m=round(math.hypot(x_b, y_b), 3),
        ))
    return out
