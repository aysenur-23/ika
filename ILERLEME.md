# İKA — İlerleme Kaydı

> Bu dosya çalışma seansları boyunca **canlı tutulur**. Her dönüm noktasında
> Claude güncellesin. Eski oturumların özet zaman çizelgesi + şu an aktif iş +
> sıradakiler bir arada.
>
> **Yeni Claude oturumu / hesap geçişi:** Önce `CLAUDE.md` (kalıcı bağlam),
> sonra bu dosya (canlı durum). Hesap-özel hafıza (Anthropic memory)
> taşınmaz, ama bu iki md projeyle birlikte git'te → her hesapta sorunsuz.

**Son güncelleme:** 2026-06-02

---

## 🎯 Tez Hedefi (kısaca)

ROS 2 Jazzy tabanlı otonom kara aracı **İKA** için hibrit DL+RANSAC algılama +
hazard füzyon + Nav2 (DWB ↔ MPPI) ve simülasyon tabanlı doğrulama. Pi 5 +
Arduino + RPLIDAR C1 + Pi Camera donanımı; sim önce, sonra sınırlı saha testi.

Daha geniş kapsam: `TEZ.md`, `TEST_PROTOKOLU.md`, proje belleği.

---

## ✅ Bugün Yapılanlar (2026-06-02)

### Simülasyon WSL'de canlandı

**Konum:** `~/ika` (Ubuntu-24.04 WSL distrosunda ayrı klon).

Build + launch akışı `scripts/deploy_sim.sh` ile çalışıyor; topic hz'leri Pi
beklentisiyle uyumlu (/imu/data ≈ 90 Hz, /scan ≈ 9 Hz, /odom_truth ≈ 27 Hz).

### Üç kritik düzeltme

1. **`scripts/deploy_sim.sh`** — `set -u` + ROS 2 `setup.bash`
   (`AMENT_TRACE_SETUP_FILES` tanımsız) çatışması. ROS sourcing'i
   `set +u; source ...; set -u` ile sarıldı.

2. **`ika_simulation/launch/simulation.launch.py`** — `render_engine`
   launch arg eklendi (`ogre|ogre2`, default `ogre2`). Gazebo Sensors render
   thread fallback'i için.

3. **`ika_description/urdf/ika.urdf.xacro`** — **KRİTİK BUG**: `xacro
   use_sim:=true` arg'i Python `True` boolean'a coerce ediliyordu; eski koşul
   `${use_sim == 'true' or use_sim == 'True'}` daima False döndüğü için
   URDF'e **hiç `<gazebo>` tag'i girmiyordu** → DiffDrive plugin, sensörler,
   IMU, lidar, kamera **HİÇBİRİ yüklenmiyordu**. Sim'de spawn olan ika modeli
   plugin'siz iskeletti. Düzeltme: `<xacro:if value="${use_sim}">` (Python
   truthy). Bu hata Pi'de de aktifti — Pi'de yapılan tüm önceki sim
   doğrulamalarının da sensörsüz koştuğunu varsaymak doğru olur.
   Bellek: `project_ika_xacro_use_sim_gotcha.md`.

### WSL pencere görünürlük araştırması

WSL2/WSLg'de Gazebo GUI + RViz + sensör render surface'ları aynı anda
açılınca shared surface pool taşıyor → main pencere `[WARN:COPY MODE]`'a
düşüp Windows tarafına aktarılamıyor (taskbar'da görünür, tıklayınca öne
gelmez). Çözüm:

- **Geliştirme/test için**: `headless:=true rviz:=true` → tek pencere RViz;
  Gazebo offscreen.
- **Tez screenshot'u için**: `headless:=false rviz:=false render_engine:=ogre`
  → tek pencere Gazebo (OGRE1, daha az surface). **Test edildi, çalışıyor.**

Bellek: `project_ika_wsl_simulation.md`.

### Memory dosyaları güncellendi

- `project_ika_xacro_use_sim_gotcha.md` (yeni) — xacro bool coerce kapanı
- `project_ika_wsl_simulation.md` (yeni) — WSL akışı
- `MEMORY.md` indexi yenilendi

---

## 🔄 Şu An Aktif

**Tez için iki paralel iş hattı kuruluyor:**

### Hat 1 — Tez görseli (Gazebo + RViz screenshot)

- OGRE1 + Gazebo GUI tek başına ✅ çalıştığı doğrulandı
- ⏳ **Sıradaki test:** OGRE1 + Gazebo + RViz aynı anda açılıyor mu? (`render_engine:=ogre rviz:=true`)
  - Çalışırsa: tek koşumda hem Gazebo sahnesi hem RViz görseli alınabilir
  - Çalışmazsa: ayrı koşumlar, screenshot'lar birleştirilir
- ⏳ Screenshot scenaryolarının listesi (her tez bölümü için 1-2 görsel)

### Hat 2 — Benchmark / karşılaştırma altyapısı

- Mevcut: `ika_rl_planner/metrics_recorder_node` tek-run CSV satırı yazıyor
- ⏳ `benchmarks/` dizini açılacak (proje kökünde)
- ⏳ `benchmarks/scenarios.yaml` — 7 engel sınıfı senaryoları
- ⏳ `benchmarks/run_benchmark.py` — N-trial wrapper
- ⏳ `benchmarks/render_tables.py` — CSV → md/LaTeX + matplotlib plot

---

## 📋 Yapılacaklar (Sıralı)

### Faz 1 — Tez görseli protokolü (1-2 saat)

- [ ] **(test)** OGRE1 + Gazebo + RViz birlikte açılıyor mu? Komut:
  ```bash
  ros2 launch ika_simulation simulation.launch.py headless:=false rviz:=true render_engine:=ogre
  ```
- [ ] Screenshot çekim listesi (`docs/thesis_figures.md`?) — her tez
      bölümünün hangi şekle/sahneye ihtiyacı var:
  - Sistem mimarisi (block diagram — Gazebo screenshot değil, ayrı)
  - Sim ortamı genel görünüm (test_world.sdf üst-açı)
  - 7 engel sınıfı (her birine 1 görsel)
  - DL detection bbox + base_link projeksiyon (RViz Marker)
  - Hazard fusion `/hazard_state` durum çıktıları (kod + RViz)
  - Nav2 plan path + costmap (RViz)
  - DWB vs MPPI yörünge karşılaştırma (RViz overlay)
  - Safety supervisor STOP/SLOW olay zaman çizgisi (matplotlib)
- [ ] Tekrar üretilebilir screenshot scripti — `scripts/capture_thesis_figs.sh`
      her senaryoyu açar, X süre bekler, Win+Shift+S yerine `import` veya
      `gnome-screenshot` ile WSL'de PNG kaydeder
- [ ] Çıktı dizini: `docs/figures/`

### Faz 2 — Benchmark altyapısı (3-4 saat)

- [ ] `benchmarks/scenarios.yaml` taslağı:
  ```yaml
  scenarios:
    - id: s1_static_corridor
      world: test_world.sdf
      start: {x: 0.0, y: 0.0, yaw: 0.0}
      goal:  {x: 6.0, y: 0.0}
      obstacles_for_clearance: [[2.0,1.0],[4.0,-1.0]]
      pass_criteria: {success: true, max_duration_s: 30}
    # ... 7 sınıf, her biri için 1 senaryo
  ```
- [ ] Launch arg'leri eklenecek (yoksa):
  - `ika_bringup/sim_full.launch.py` → `fusion_enabled:=true|false`
  - `ika_bringup/sim_full.launch.py` → `safety_enabled:=true|false`
  - `ika_bringup/sim_full.launch.py` → `perception_mode:=terrain|detection|fusion`
- [ ] `benchmarks/run_benchmark.py`:
  - Kombinasyon iterator (planner × fusion × safety × scenario × trial)
  - Her trial: sim aç → goal pub → metrics_recorder dinle → CSV satır →
    sim kapat
  - Trial başlangıcında ufak random jitter (start poz x,y ±0.1)
  - N trial default 5
- [ ] `benchmarks/render_tables.py`:
  - pandas ile groupby (planner, scenario)
  - mean ± std, success_rate
  - Markdown çıktı: `results/summary.md`
  - LaTeX çıktı: `results/summary.tex` (`\input{}` ile tezde kullanılır)
  - matplotlib: boxplot success rate, scatter clearance/curvature
- [ ] `benchmarks/README.md` — kullanım kılavuzu

### Faz 3 — Karşılaştırma eksenlerinin tek tek devreye girmesi

İlk önce DWB/MPPI, sonra fusion on/off, sonra safety on/off, sonra perception
mode. **Her eksen eklendikten sonra ILERLEME.md ve sonuç tabloları
güncellenecek.**

- [ ] Eksen 1: **Planner** (DWB vs MPPI) — en hızlı, mevcut altyapı destekliyor
- [ ] Eksen 2: **Fusion on/off** — `ika_fusion` launch arg gerekli
- [ ] Eksen 3: **Safety on/off** — `ika_safety` launch arg gerekli
- [ ] Eksen 4: **Perception mode** — terrain-only / detection-only / fusion

### Faz 4 — Tez yazımı entegrasyonu

- [ ] `TEZ.md`'ye sonuç bölümü taslağı (table'lar `\input` ile)
- [ ] Sistem mimarisi figürü (block diagram, Graphviz veya draw.io)
- [ ] Sonuç paragrafları — tablolardan otomatik özet
- [ ] Bibliography stub (Nav2, MPPI, ROS 2 makaleleri)

---

## 🧭 Karar Notları

- **Karşılaştırma kapsamı:** Tüm eksenler (planner + fusion + safety +
  perception), sırayla devreye alınacak. Tüm 7 engel sınıfı senaryo seti.
- **Tez görseli:** Gazebo GUI screenshot **öncelikli** (OGRE1 ile WSL'de
  çalışıyor). Fallback olarak offscreen camera + RViz.
- **Donanım değişikliği (2026-06-01):** OAK-D → Pi Camera. Sim'de RGBD
  modellenmeye devam (terrain RANSAC sim testi için), gerçek robotta DL
  detector + IPM (yer-düzlemi projeksiyonu).

---

## ⚠️ Açık Riskler

- Pi'deki önceki sim koşumları **xacro bug yüzünden sensörsüz** çalışmış
  olmalı; o ortamda elde edilen "doğrulamalar" yeniden yapılmalı (en azından
  topic akışı sanity).
- WSL → Pi geçişinde xacro fix'i Pi'ye pull/sync'leme adımı atlanırsa Pi'de
  hâlâ aynı bug devam eder. Commit + push şart.
- Benchmark trial sayısı 5×4 senaryo×8 kombinasyon = 160 koşum/eksen seti.
  Sim hızlandırma (`<real_time_factor>5</real_time_factor>` SDF'te) olmazsa
  saatler sürer.

---

## 📁 İlgili Dosyalar

- `IKA_ROS2_System_Reference.md` — sistem referansı
- `TEZ.md` — tez şablonu
- `TEST_PROTOKOLU.md` — saha test protokolü, senaryo karşılığı
- `DENEMELER.md` — adım adım deneme rehberi
- `KURULUM.md` — Pi kurulumu
- `ika_ws/src/ika_rl_planner/` — metrik harness'in başlangıcı

## 🔗 İlgili Hafıza

- `project_ika.md`, `project_ika_workspace.md`
- `project_ika_thesis_extension.md`
- `project_ika_wsl_simulation.md` ⭐ yeni
- `project_ika_xacro_use_sim_gotcha.md` ⭐ yeni
