"""IKA - DL nesne tespiti post-process cekirdegi (ROS'suz, test edilebilir).

OAK-D Lite VPU'sundaki SpatialDetectionNetwork ham ciktisini alir ve:
  - guven esigi + menzil filtresi uygular
  - label -> hazard sinifi atar (DYNAMIC / STATIC)
  - kamera optical frame -> base_link 3B donusum yapar

ROS bagimliligi yoktur; ika_terrain/ground_plane.py ile ayni desende unit
test edilebilir. Agir NN cikarimi kameranin VPU'sunda kosar, bu modul yalniz
hafif son-isleme yapar (Pi'yi yormaz).

NOT - eksen konvansiyonu: depthai spatial koordinatlari pinhole projeksiyonla
uretildigi icin REP-103 OPTICAL frame ile ayni kabul edilir:
  x = sag, y = asagi, z = ileri (kameraya gore).
Bu varsayim gercek cihazda dogrulanmali: tam onde duran bir nesne base_link'te
x>0, y~0 vermeli. Sapma varsa eksen isaretleri burada tek noktadan duzeltilir.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Sequence


@dataclass
class RawSpatialDetection:
    """SpatialDetectionNetwork ham ciktisinin saf-Python karsiligi.

    Konum kamera OPTICAL frame'inde, METRE cinsinden (depthai mm -> m donusumu
    cagirandan once yapilir). bbox normalize (0-1): (xmin, ymin, xmax, ymax).
    """
    label_id: int
    confidence: float
    x: float
    y: float
    z: float
    bbox: tuple = (0.0, 0.0, 0.0, 0.0)


@dataclass
class FusedDetection:
    """base_link frame'e tasinmis, hazard sinifi atanmis tespit."""
    label_id: int
    label: str
    hazard: str          # DYNAMIC / STATIC
    confidence: float
    x: float             # base_link: ileri+
    y: float             # base_link: sol+
    z: float             # base_link: yukari+
    range_m: float       # base_link XY duzleminde mesafe
    bbox: tuple = field(default=(0.0, 0.0, 0.0, 0.0))


def classify_hazard(
    label: str,
    dynamic_labels: Iterable[str],
    static_labels: Iterable[str],
    *,
    default: str = 'STATIC',
) -> str:
    """Label adini hazard sinifina cevir.

    dynamic_labels oncelikli (insan/arac gibi hareketli nesneler). Listede
    olmayan ama static_labels'ta da olmayan label'lar `default` alir.
    """
    if label in dynamic_labels:
        return 'DYNAMIC'
    if label in static_labels:
        return 'STATIC'
    return default


def camera_optical_to_base(
    x: float, y: float, z: float,
    *,
    camera_pitch: float,
    camera_x: float,
    camera_z: float,
) -> tuple:
    """Tek bir optical-frame noktasini base_link'e tasi.

    ground_plane.optical_to_base ile ayni donusum (tek nokta surumu):
      optical (x sag, y asagi, z ileri) -> camera_frame (REP-103)
      -> Y ekseninde camera_pitch dondur -> (camera_x, camera_z) ile otele.
    """
    # optical -> camera_frame
    x_c = z
    y_c = -x
    z_c = -y
    # camera_frame -> base_link
    cp = math.cos(camera_pitch)
    sp = math.sin(camera_pitch)
    x_b = cp * x_c + sp * z_c + camera_x
    y_b = y_c
    z_b = -sp * x_c + cp * z_c + camera_z
    return x_b, y_b, z_b


def label_for(label_id: int, label_names: Sequence[str]) -> str:
    """Index -> label adi. Listede yoksa string index dondur (guvenli)."""
    if 0 <= label_id < len(label_names):
        return label_names[label_id]
    return str(label_id)


def process_detections(
    raw: Iterable[RawSpatialDetection],
    label_names: Sequence[str],
    *,
    dynamic_labels: Iterable[str] = (),
    static_labels: Iterable[str] = (),
    ignore_labels: Iterable[str] = (),
    confidence_threshold: float = 0.5,
    min_range_m: float = 0.2,
    max_range_m: float = 6.0,
    camera_pitch: float = 0.0,
    camera_x: float = 0.0,
    camera_z: float = 0.0,
    default_hazard: str = 'STATIC',
) -> list:
    """Ham spatial tespitleri filtreleyip base_link'e tasinmis FusedDetection
    listesine cevir.

    Sirasiyla: guven esigi -> ignore label -> koordinat donusumu ->
    menzil filtresi -> hazard siniflandirma.
    """
    dynamic_set = set(dynamic_labels)
    static_set = set(static_labels)
    ignore_set = set(ignore_labels)

    out: list = []
    for det in raw:
        if det.confidence < confidence_threshold:
            continue
        label = label_for(det.label_id, label_names)
        if label in ignore_set:
            continue

        x_b, y_b, z_b = camera_optical_to_base(
            det.x, det.y, det.z,
            camera_pitch=camera_pitch, camera_x=camera_x, camera_z=camera_z,
        )
        rng = math.hypot(x_b, y_b)
        if rng < min_range_m or rng > max_range_m:
            continue

        hazard = classify_hazard(
            label, dynamic_set, static_set, default=default_hazard)
        out.append(FusedDetection(
            label_id=det.label_id,
            label=label,
            hazard=hazard,
            confidence=det.confidence,
            x=round(x_b, 3),
            y=round(y_b, 3),
            z=round(z_b, 3),
            range_m=round(rng, 3),
            bbox=det.bbox,
        ))
    return out


def summarize(detections: Sequence[FusedDetection]) -> dict:
    """JSON-serializable ozet (diagnostics / fusion node icin)."""
    dynamic = [d for d in detections if d.hazard == 'DYNAMIC']
    nearest = min((d.range_m for d in detections), default=float('inf'))
    nearest_dynamic = min(
        (d.range_m for d in dynamic), default=float('inf'))
    return {
        'count': len(detections),
        'dynamic_count': len(dynamic),
        'nearest_range_m': None if math.isinf(nearest) else round(nearest, 3),
        'nearest_dynamic_range_m':
            None if math.isinf(nearest_dynamic) else round(nearest_dynamic, 3),
        'labels': [d.label for d in detections],
    }
