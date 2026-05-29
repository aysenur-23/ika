# İKA — Test Senaryoları ve Saha Test Protokolü

Bu doküman tez kapsamındaki genişletmenin (hibrit DL+RANSAC algılama, hazard
füzyon, detection_layer, dinamik nesne güvenliği, DWB↔MPPI planlayıcı
karşılaştırması) doğrulama planını içerir.

**Doğrulama felsefesi (bkz. proje belleği):** Her davranış önce simülasyonda
doğrulanır, sonra gerçek araçta sınırlı sahada. Birim test edilebilir mantık
(DL post-process, füzyon, güvenlik kararı, sim tespit, planlayıcı metrikleri)
ROS'suz saf-Python çekirdeklerde tutulur ve `colcon test` / `pytest` ile
makinede koşar.

---

## 0. Birim testler (donanımsız, her zaman koşar)

```bash
cd ~/ika_ws
colcon test --packages-select \
  ika_terrain ika_perception_dl ika_fusion ika_safety ika_rl_planner
colcon test-result --verbose
```

Kapsam:
- `ika_terrain/test/test_ground_plane.py` — RANSAC + terrain sınıflandırma
- `ika_perception_dl/test/test_detector_postprocess.py` — DL post-process, koordinat dönüşümü, label→hazard, filtre
- `ika_perception_dl/test/test_sim_detection.py` — sim sentetik tespit (world→base, FOV/menzil)
- `ika_fusion/test/test_hazard_fusion.py` — terrain+nesne füzyon kararı, detection grid
- `ika_safety/test/test_safety_logic.py` — hazard→cmd_vel aksiyon eşleme + e-stop önceliği
- `ika_rl_planner/test/test_planner_metrics.py` — yol uzunluğu, clearance, pürüzsüzlük, başarı

---

## 1. Senaryo A — DL dinamik engel (simülasyon)

**Amaç:** DL tespiti → füzyon (DYNAMIC) → costmap detection_layer + safety
SLOW/STOP → planlayıcı tepkisi zincirinin sim'de doğrulanması. Gazebo'da fiziksel
VPU olmadığından `sim_detection_node` yer-gerçeği pozundan kusursuz tespit üretir
(çıktı kontratı gerçek `dl_perception_node` ile aynı: `/detected_objects`).

**Çalıştırma (iki terminal):**

```bash
# T1 — tam sim stack (Gazebo + nav + terrain + fusion + safety)
ros2 launch ika_bringup sim_full.launch.py

# T2 — sentetik tespit (önümüzden geçen yaya + sabit sandalye)
ros2 launch ika_perception_dl sim_detection.launch.py
```

Sonra RViz'den veya CLI'dan bir hedef ver:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0}, orientation: {w: 1.0}}}"
```

**Beklenen davranış / geçme kriterleri:**
1. `/detected_objects` yaya `person:DYNAMIC` içerir (yaya FOV'a girince).
2. `/hazard_state.action` yaya menzile girince `SLOW`, çok yaklaşınca `STOP` olur
   (`dynamic_slow_range_m=2.0`, `dynamic_stop_range_m=0.8`).
3. `/cmd_vel_safe` STOP anında sıfırlanır (araç durur), yaya geçince devam eder.
4. `/detection_obstacles` costmap detection_layer'da yaya/sandalye konumunda
   maliyet gösterir; Nav2 bunların etrafından planlar.

**İzleme:**
```bash
ros2 topic echo /hazard_state
ros2 topic echo /safety_status
ros2 run rqt_console rqt_console   # "Hazard DUR" uyarıları
```

**Not — eksen doğrulaması:** Bu senaryo `sim_detection`'ın world→base dönüşümünü
de dolaylı doğrular. Gerçek OAK-D'de `detector_postprocess`'in depthai spatial
eksen varsayımı ayrıca sınanmalı (bkz. Bölüm 3).

---

## 2. Senaryo B — DWB ↔ MPPI planlayıcı karşılaştırması (simülasyon)

**Amaç:** Klasik DWB ile örnekleme/optimizasyon tabanlı MPPI'yi aynı dünyada,
aynı hedeflerle, aynı ölçütlerle karşılaştırmak.

**Çalıştırma:**

```bash
# DWB (varsayılan)
ros2 launch ika_bringup sim_full.launch.py
# MPPI
ros2 launch ika_bringup sim_full.launch.py local_planner:=mppi
```
> `local_planner` argümanı `navigation.launch.py`'de tanımlı; `sim_full` onu
> include ediyor. MPPI seçilince `mppi_controller.yaml`, `nav2_params.yaml`
> üzerine yüklenip `controller_server/FollowPath` bloğunu DWB→MPPI ezer.

**Metrik kaydı (ayrı terminal, her koşumdan önce `planner_label` ayarla):**

```bash
# DWB koşumları
ros2 run ika_rl_planner metrics_recorder_node --ros-args \
  --params-file $(ros2 pkg prefix ika_rl_planner)/share/ika_rl_planner/config/metrics_params.yaml \
  -p planner_label:=dwb
# MPPI koşumları için aynı komutta -p planner_label:=mppi
```

Her `/goal_pose` bir koşum başlatır; hedefe varınca (veya `run_timeout_s`)
`planner_comparison.csv`'ye bir satır eklenir.

**Karşılaştırma ölçütleri (CSV sütunları):**

| Ölçüt | Anlam | Yorum |
|---|---|---|
| `success` | hedefe ulaşıldı mı | temel başarı |
| `duration_s` | süre | düşük = hızlı |
| `path_length_m` | kat edilen yol | düşük = verimli |
| `min_clearance_m` | engele min mesafe | yüksek = güvenli |
| `mean_abs_curvature` | rad/m, pürüzsüzlük | düşük = yumuşak |
| `avg_speed_mps` | ortalama hız | — |

**Önerilen protokol:** Aynı 3–5 hedef için her planlayıcıyı ≥5 kez koş (toplam
≥25 koşum/planlayıcı). Ortalama ± std raporla. Aynı `test_world.sdf` engelleri
(3 kutu, dar geçit, rampalar) kullanılır; `metrics_params.yaml`'daki `obstacles`
kutu merkezlerine ayarlı (clearance için).

---

## 3. Gerçek araç — sınırlı saha testi protokolü

> **UYARI:** Gerçek araç testi yalnızca tüm güvenlik ön koşulları sağlandığında
> ve fiziksel E-Stop elde tutularak yapılır. Maksimum hız encoder eklenene kadar
> **0.25 m/s** ile sınırlı (bkz. proje belleği).

### 3.1 Ön koşullar (checklist)
- [ ] Fiziksel E-Stop anahtarı çalışıyor (basınca motorlar kesiliyor).
- [ ] Arduino watchdog aktif; seri bağlantı koparsa motor durur.
- [ ] `safety_supervisor` sensör watchdog'u test edildi: `/scan`, `/depth`,
      `/imu` birini kes → `/e_stop` true → araç durur.
- [ ] `hazard_fusion` ayakta; `/hazard_state` yayınlıyor (yoksa supervisor
      başlangıçta SLOW'da kalır — güvenli ama yavaş).
- [ ] OAK-D Lite bağlı; `dl_perception` aktive oldu (`/detected_objects` akıyor).
- [ ] **Eksen doğrulaması:** Kameranın ~2 m önüne bir kişi koy → `/detected_objects`
      içinde `person:DYNAMIC`, base_link konumu **x≈2, y≈0** olmalı. Sapma varsa
      `detector_postprocess.camera_optical_to_base` eksen işaretleri düzeltilir.
- [ ] Açık, düz, engelsiz bir saha; çevrede gözlemci/yardımcı.
- [ ] Hız sınırı YAML'da 0.25 m/s doğrulandı (`nav2_params` max_vel_x, base controller).

### 3.2 Test adımları
1. **Statik engel:** Aracın önüne sabit kutu. Beklenen: lidar + DL costmap'e
   basar, Nav2 etrafından planlar; çok yakınsa collision_monitor/safety durdurur.
2. **Dinamik engel (kişi):** Bir kişi aracın yoluna girer. Beklenen:
   `/hazard_state=SLOW` sonra `STOP`; kişi çekilince devam. (En kritik test.)
3. **Negatif engel / çukur kenarı:** Terrain `DROPOFF_DANGER` → STOP (lidar
   göremez, depth RANSAC yakalar).
4. **Sensör kaybı:** Koşum sırasında kamerayı çek → watchdog E-Stop → durur.

### 3.3 Kaydedilecek veri
- `ros2 bag record /scan /oak/depth/image_raw /detected_objects /hazard_state /safety_status /cmd_vel_safe /odometry/filtered /e_stop`
- Her senaryo için: tepki mesafesi (engelle araç arası durma anındaki mesafe),
  yanlış pozitif/negatif tespit sayısı, müdahale (E-Stop) gerekti mi.

### 3.4 İptal kriterleri
- Araç beklenmedik hareket → E-Stop.
- `/hazard_state` veya `/safety_status` akışı durursa → E-Stop, koşumu bitir.
- Tespit gecikmesi tehlikeli (yaya çok yaklaşana kadar STOP yok) → hız düşür,
  `dynamic_stop_range_m` artır, tekrar.

---

## 4. Öğrenilmiş politika (PPO/SAC) — yol haritası (sonraki faz)

MPPI karşılaştırması tamamlandıktan ve zaman kaldıkça eklenecek. `ika_rl_planner`
paketi metrik harness'ı ile bu fazın temelini şimdiden sağlıyor (aynı ölçütlerle
karşılaştırılabilir).

**Planlanan yapı:**
1. **Gymnasium ortamı** — Gazebo'yu saran `ika-nav-v0`:
   - *Gözlem:* yerel costmap penceresi (veya lidar demeti) + hedef göreli konumu
     + mevcut hız.
   - *Aksiyon:* (v, ω) sürekli; `[-0.25,0.25]×[-0.8,0.8]` ile sınırlı.
   - *Ödül:* hedefe yaklaşma + (varış bonusu) − (çarpışma cezası) − (zaman) −
     (sarsıntı/curvature). `planner_metrics` ölçütleriyle hizalı.
2. **Eğitim** — stable-baselines3 PPO/SAC, headless Gazebo (`headless:=true`),
   sim hızlandırma. Checkpoint + TensorBoard.
3. **Entegrasyon** — eğitilmiş politika ya Nav2 controller plugin'i olarak ya da
   `/cmd_vel_nav` yayınlayan ayrı bir çıkarım node'u olarak; güvenlik zinciri
   (collision_monitor + safety_supervisor) DEĞİŞMEDEN üstte kalır.
4. **Karşılaştırma** — aynı `metrics_recorder_node`, `planner_label:=rl`; DWB,
   MPPI, RL üçlü tablo.

**Riskler/notlar:** Eğitim sim-saati ve ödül tasarımı maliyetli. Pi5'te çıkarım
küçük politika ağıyla uygulanabilir; eğitim PC/sunucuda yapılır. Güvenlik
zincirinin politikadan bağımsızlığı korunur (öğrenilmiş politika asla doğrudan
motora yazmaz, hep safety_supervisor'dan geçer).
