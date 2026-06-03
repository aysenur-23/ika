# İKA Parkur Haritası

`test_world.sdf` içinde **18 m uzun × 6 m geniş** doğrusal parkur. Robot
**Start (0, 0)** noktasından **+X yönüne** bakar, 8 istasyondan sırayla geçip
**Goal (18, 0)** noktasındaki bitiş kemerine ulaşır.

## Üst-Görünüş Şeması

```
                           y = +3  ╔═══════════════════════════════════════════════════════════════╗  KUZEY DUVAR
                                   ║                                                               ║
                                   ║  ▮1   ▮2     ▮3   ▮4    ▮5    ▮6   ▮7      ▮8                ║  (▮ = istasyon levhasi)
                                   ║                                                               ║
                                   ║        □  □                                                   ║
                                   ║   ╱╲     □                ⊕              ↳━━┓                 ║
   y = 0   START ══►   ▰▰▰▰        ║  RMP  □  □   ║║   │  ⬛   ⨉    ◽◽◽   ┃   ┃   ▓▒░   ══► FINISH
                                   ║   ╱╲          ║║         o     ◽       ┃   ┃                  ║
                                   ║                                          ┗━━━┛                 ║
                                   ║                                                               ║
                                   ║                                                               ║
                           y = -3  ╚═══════════════════════════════════════════════════════════════╝  GUNEY DUVAR
                                   x=0   2    4    6    8    9    11   13     15     17    18

         Start          1            2          3        4        5          6          7            8         Finish
         GATE           RAMPA      SLALOM      DAR     INCE+    DINAMIK   NEGATIF    L-KOR     YUZEY GRAD     GATE
         (yesil)       (Sinif 4)  (Sinif 1)  GECIT  ESIK+POLE  YAYA      ENGEL                (S/C/R)        (kirmizi)
                                            (S 6)   (S 1+4)   (S 5)    +CUKUR(S3)  (S 6 ileri) (Sinif 7)
```

Lejant: `▰` rampa  ·  `□` kutu  ·  `║║` duvar  ·  `│` ince direk  ·  `⨉` esik  ·  `⬛` yaya  ·  `◽` koni  ·  `⊕` platform  ·  `o` cukur  ·  `▓▒░` yuzey gradyani

## İstasyonlar — Engel Taksonomisi Eşlemesi

| İst | x (m) | Sınıf | Sim modelleri | Beklenen davranış | Tezdeki figür adı |
|----:|------:|:---:|---|---|---|
| **1** | 2.0 | **4** Yükselti | `ramp_safe` (~5.7°, ana hat), `ramp_caution` (~17°, yan, isteğe bağlı) | Düz tırmanır, çıkar; eğim algılama `SAFE` döner | F-1: Eğim algılama (terrain) |
| **2** | 4.0 – 5.6 | **1** Sabit engel | `obstacle_box_1/2/3` (alternatif sol/sağ) | Slalom zigzag; lidar costmap + DWB/MPPI yörünge | F-2: Lokal planlayıcı yörüngesi |
| **3** | 7.0 | **6** Dar geçit | `wall_left + wall_right` (0.8 m kapı) | Inflation çevresi sıkı; orta hattan geçer | F-3: Costmap inflation + dar geçit |
| **4** | 9.0 – 9.3 | **1+4** İnce/Eşik | `thin_pole_1` (Ø 8 cm), `kerb_step_1` (8 cm yüksek) | Direk lidar'da nokta olarak; eşik `CAUTION` (border) | F-4: İnce engel çözünürlüğü + eşik |
| **5** | 11.0 | **5** Dinamik | `person_static_1` (sim_detection_node tarafından `person:DYNAMIC`) | Yaya yakına gelince `SLOW`, çok yakında `STOP` | F-5: DL detection + füzyon stop |
| **6** | 12.3 – 13.6 | **3** Negatif | `pit_platform_forward`, `pothole_visual_{1,2}`, `trench_visual_1`, `hazard_cone_{1,2}` | Platform üstüne çıkar → ön kenarda `DROPOFF_DANGER` | F-6: Terrain RANSAC dropoff |
| **7** | 15.0 – 15.8 | **6** ileri | `l_corridor_wall_a/b` (L şekli, kuzey tarafı açık) | İçeri girer, kapalı yüzeyi görür, geri ve kuzeyden dolanır | F-7: SLAM tabanlı oklüzyon + replan |
| **8** | 16.6 – 17.8 | **7** Riskli zemin | `surface_patch_{safe,caution,risky}` (yeşil→sarı→kırmızı şerit) | Şu an: geometrik etki yok (görsel). Faz 2: RGB classifier `RISKY` → SLOW | F-8: Yüzey sınıflandırma yamaları |

## Spawn / Start / Goal Koordinatları

```yaml
spawn:        {x: 0.0,  y: 0.0,  z: 0.1, yaw: 0.0}    # +X'e bakar
finish_line:  {x: 18.0, y: 0.0}                       # kirmizi kemer
course_bbox:  {x_min: -1.0, x_max: 19.0, y_min: -3.0, y_max: 3.0}
```

Manuel test komutu:

```bash
# Bitis kemerine git
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header:{frame_id: "map"}, pose:{position:{x: 18.0, y: 0.0}, orientation:{w: 1.0}}}'

# Ara hedef (Istasyon 5 onu, dinamik testi tetikler)
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header:{frame_id: "map"}, pose:{position:{x: 11.0, y: 0.0}, orientation:{w: 1.0}}}'
```

## Tez Görseli için Kamera Açıları

Tezde aşağıdaki açılardan screenshot önerilir:

1. **Üst-açı kuş bakışı** — `gz sim` GUI'sinde sağ-tık → "Move To" → `(9, 0, 18)` bakış aşağı. Tüm parkur görünür. **Sistem mimarisi + parkur tanıtım figürü.**
2. **Eş-açı (35° yukarı, 45° yan)** — start'a yakın, robotun arkasından. **Robot + sensör yerleşimi figürü.**
3. **Her istasyona kuşbakışı close-up** — `(istasyon_x, istasyon_y, 5)` → 8 ayrı figür.
4. **RViz birinci-kişi görünüm** — robot kamerası (`/oak/image`) + LaserScan üst-bindirme. **Algılama figürü.**

## Boyut Tablosu (tezde "tablo: parkur fiziksel ölçüler")

| Eleman | Boyut (m) |
|---|---|
| Parkur uzunluğu (start–finish ekseni) | 18.0 |
| Parkur genişliği (kuzey duvar–güney duvar) | 6.0 |
| Sınır duvar yüksekliği | 0.50 |
| Slalom kutu kenarı | 0.40 (orta), 0.50 (büyük) |
| Dar geçit aralığı | 0.80 |
| Eşik yüksekliği | 0.08 |
| Yaya boyu (silindir + küre) | 1.18 |
| Negatif platform yüksekliği | 0.30 |
| Rampa eğimi (SAFE) | 5.7° (%10) |
| Rampa eğimi (CAUTION) | 17° (%30) |
