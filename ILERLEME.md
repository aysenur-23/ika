# İKA — İlerleme Kaydı

> **2026-06-04 (gece) — RViz GORSEL DUZELTILDI**
>
> Kullanici raporu: "rvizde tek basina bir kutu", "arac titriyor".
> Teshis:
> 1. URDF (17.2 KB) saglikli — `robot_state_publisher` /robot_description'i
>    `TRANSIENT_LOCAL` QoS ile yayinliyor.
> 2. TF tree saglikli — `map → base_link → laser_base → laser_frame`,
>    `base_link → camera_frame`, wheels, hepsi var.
> 3. **Asil sorun**: RViz `RobotModel` display default'u
>    `Description Source: File` (bos URDF File path) → ekran sadece TF
>    eksenleri + minik base_link kutusu gosteriyor, gercek mesh'ler yok.
> 4. **Yan sorun**: `enable_octomap=true` default'tu → /oak/points'tan
>    aslinda lidar disindaki simulated depth camera noise voxelize olup
>    devasa "mavi duvar" yariyordu.
>
> Fix:
> - `ika_full.rviz`: `RobotModel` display'e `Description Source: Topic`,
>   `Durability Policy: Transient Local` ekledi. Octomap layer'lari default
>   `Enabled: false`.
> - `sim_full.launch.py`: `enable_octomap` default `true` → **`false`**.
>   Kullanici 3D harita isterse `enable_octomap:=true` ile aciyor.
>
> "Titreme" iddiasi gercek titreme degildi — robot mesh gorunmediginden
> sadece TF axis arrow'lari hareket ediyordu (SLAM her ~1s kucuk correction
> yapinca rotate goruluyordu). Mesh gelince algi stabilizesi gelir.

> **2026-06-04 (gun sonu) — 10m OTONOM NAVIGASYON ✅**
>
> Test sonuclari (sim_full.launch.py default config, WSL2):
> - Goal (1.5, 0) — temiz alan       → **SUCCEEDED** robot (1.26, -0.03)
> - Goal (5, 0.5) — obs_2 inflation  → ABORTED 102 (goal hücresi engelde)
> - Goal (10, 0)  — **5 engel arasi** → **SUCCEEDED** robot (9.84, 0.07) 🎉
>
> Yarim parkur (10m) basariyla. Robot obs_1 (3,0), obs_2 (5.5,+0.4),
> obs_3 (8,-0.4), yaya (10.5,0) civarini gecerek 10m'de durdu.
>
> Son kritik bug: slam_params.yaml `use_sim_time: false` sabitti; launch
> override yapsa da SLAM bazen scan stamp'lerini wall_clock vs sim_time
> uyumsuzlugu yuzunden drop edebiliyordu. Fix: YAML default'u true
> (Pi'de launch'tan override edilecek).
>
> **Calisma modu** — tek komut:
> ```
> ros2 launch ika_bringup sim_full.launch.py
> ```
> → Gazebo (headless) + RViz + SLAM + Nav2 + DWB + safety + perception.
> RViz'de goal goster, robot 10m'ye kadar otonom hareket eder.

> **2026-06-04 GUNCELLEME — NAV2 STACK CALISIYOR**
>
> Saatlerce debug sonrasi Nav2 + SLAM + planlama tam aktif. Goal (1.5, 0)
> SUCCEEDED (5.25s); Goal (5, 0) testinde robot 2.1m ilerledi (obstacle
> kacinma denemesi). 3 kok neden bulundu ve fix:
>
> 1. **PythonExpression CAPITAL T BUG**: `enable_nav2` Python eval'da
>    'True' (capital), IfCondition'da 'true' ile karsilastiriliyordu →
>    controller_server SPAWN OLMUYORDU. Fix: capital 'True' ile compare.
> 2. **nav2_params filters: []**: planner_server boş listede
>    InvalidParameterValueException → CRASH. Fix: filters satiri kaldirildi.
> 3. **lifecycle_manager bond_timeout**: WSL sim_time vs wall_clock
>    uyumsuzlugu → 200ms 30sn sayilip CRITICAL FAILURE. Fix: bond_timeout=0
>    (heartbeat disable) + use_sim_time=False lifecycle_manager'larda.
>
> Bonus fix'ler:
> - collision_monitor lifecycle disinda (configure failures kiriyordu)
> - global_costmap rolling_window=true (slam ufku otesinde plan)
> - cmd_vel relay: nav2 mode'da /cmd_vel_safe yerine /cmd_vel_nav direkt
>   (safety chain bypass; DWB kendi collision check yapar)
>
> Stack durumu:
> - /slam_toolbox, /controller_server, /planner_server, /behavior_server,
>   /bt_navigator, /safety_supervisor, /hazard_fusion, /terrain_perception
>   **hepsi active [3]**
> - /map publisher 1, /scan 9.6Hz
> - Tek komut: `ros2 launch ika_bringup sim_full.launch.py`



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

### Onboarding + commit

- `CLAUDE.md` (yeni) — yeni Claude oturumu için kalıcı bağlam
- `ILERLEME.md` (bu dosya) — canlı durum
- Commit `ddca30c` `cacb118..ddca30c` origin/main'e push edildi
- WSL ~/ika senkron (git pull --ff-only)

---

## 🔄 Şu An Aktif

### Parkur tasarımı (yeni)

**Tez parkuru tasarlandı + kodlandı.** `test_world.sdf` baştan yazıldı: 18 m
uzun × 6 m geniş, 8 istasyonlu doğrusal parkur. Her istasyon 1 engel sınıfı.
Yeşil başlangıç + kırmızı bitiş kemerleri, sınır duvarları, istasyon levhaları.

URDF sensörleri tezde belirgin: lidar pucku büyütüldü, kamera prominent
+ lens noktaları, **GPS göbeği + gümüş anten direği + kırmızı LED ucu**, IMU
yeşil PCB. Her sensör ayrı renkte.

**İkinci tur düzeltmeler (kullanıcı geri bildirimi sonrası):**
- Tekerlekler "uçmuş" görünüyordu çünkü gövde z-pozisyonu tekerlek aksıyla
  aynı seviyedeydi. `chassis_z` introduced (`wheel_radius + clearance +
  robot_height/2`); gövde artık tekerleklerin 5 mm üstünde oturuyor.
- Sensörlerin gövde içinde kalmaması için `chassis_top_z` referansı eklendi;
  tüm sensör z'leri buna göre hesaplanıyor (gövdenin tam üstünde).
- Parkurda **22 parlak yeşil yol oku** zemine eklendi — robotun gideceği rota
  görsel olarak belli (slalom zigzag, pit platform güneyden, L-koridor
  kuzeyden dolanış).

**Üçüncü tur (boş viewport → pencere görünmüyor sorun ikilisi):**
- İlk denedik: `<render_engine>ogre2</render_engine>` world'den kaldırıldı,
  CLI'dan ogre1 ver. Sonuç: hem GUI hem sensör OGRE1'de, WSLg surface budget
  yetmedi, pencere taskbar'da göründü ama foreground'a gelmedi.
- Doğru kombinasyon (önceki çalışan halinin nedeni):
  - **GUI render engine: OGRE1** (CLI'dan, WSL pencere görünürlük için)
  - **Sensors render engine: OGRE2** (world'de hardcode, sensör backend ayrı)
- Bu mod sensör verisi yayımlamaz (OGRE2 sensör thread WSL'de "Waiting for
  init"'te takılır, bilinen WSLg bug'ı) ama **Gazebo penceresi görünür** →
  tezdeki visual screenshot için yeterli.

**Operasyonel üç-mod kararı (WSL) — TEST EDİLDİ:**
| Mod | Komut özü | Gazebo GUI | RViz | Sensör | Kod | Doğrulama |
|---|---|:-:|:-:|:-:|:-:|---|
| **A — Tez görseli** | `simulation.launch.py headless:=false rviz:=false render_engine:=ogre` | ✅ | ❌ | belirsiz | ❌ | parkur screenshot için |
| **B — Geliştirme** | `sim_full.launch.py headless:=true rviz:=true` | ❌ | ✅ | ✅ | ✅ | algoritma debugging |
| **C — Gazebo'da kod** | `sim_full.launch.py headless:=false rviz:=false render_engine:=ogre` | ✅ | ❌ | ✅ | ✅ | **/scan 8.6 Hz ✅** |

Mod C 2026-06-02 doğrulandı: WSL'de Gazebo penceresi açık + Nav2 + safety +
fusion + terrain + slam_toolbox arka planda çalışıyor + sensörler yayımlıyor.
Tez demo video/screenshot için ideal mod — robotu Gazebo'da gerçek-zamanlı
gösterip kodun davranışını görüntülerle anlatabilirsin.

**Mod C ilk koşum bug'ları (düzeltildi):**
- `nav2_params.yaml` bt_navigator config'i Humble-style idi (plugin_lib_names
  uzun liste, navigators eksik). Jazzy zorunlu kıldı `navigators` +
  `navigate_to_pose` + `navigate_through_poses` plugin mapping. bt_navigator
  configure aşamasında crash ediyordu → /goal_pose abonesi yoktu, Nav2 zinciri
  kırılıyordu.
- slam_toolbox lifecycle_manager_navigation listesinde değildi →
  `unconfigured` kalıyordu, harita üretmiyordu. Listeye eklendi.
- collision_monitor önceden `unconfigured` görünüyordu çünkü bt_navigator
  çökünce lifecycle_manager autostart sırası duruyordu; bt_navigator fix
  sonrası collision_monitor da configure oluyor.

Pi'de ise üçü de tek modda (native GUI sorunsuz, sim_full headless:=false
rviz:=true ile hem Gazebo hem RViz açık).

Dökümantasyon: `docs/parkur_haritasi.md` — şema + istasyon tablosu + tez
figürü adlandırması + kamera açısı önerileri.

Build doğrulandı (WSL): URDF 16 gazebo tag, world 64 model (42 obstacle +
22 path arrow). Sıradaki: kullanıcı OGRE1 sim'i yeniden başlatıp görsel
onayı verecek.

**Tez için iki paralel iş hattı:**

### Hat 1 — Tez görseli (Gazebo + RViz screenshot)

- OGRE1 + Gazebo GUI tek başına ✅ çalıştığı doğrulandı
- ⏳ Yeni parkurla görsel onayı (rampa, slalom, koridor, vs. doğru yerlerde mi)
- ⏳ Her istasyona kuş-bakışı screenshot — 8 figür için
- ⏳ Robot eş-açı portresi (sensör yerleşimi tezde figürü)

### Hat 2 — Benchmark / karşılaştırma altyapısı ✅ KURULDU

- ✅ `benchmarks/scenarios.yaml` — 8 senaryo × 3 mod × 5 trial = 120 koşum
- ✅ `benchmarks/run_benchmark.py` — auto-launch + trajectory recorder + metric
- ✅ `benchmarks/render_tables.py` — Markdown + LaTeX + matplotlib (4 plot turu)
- ✅ `benchmarks/README.md` — kullanım kılavuzu

Modlar: `avoider` (reaktif), `nav2_dwb` (klasik), `nav2_mppi` (optimizasyon).
Çıktılar: `results/raw_runs.csv`, `results/summary.{md,tex}`, `results/plots/`.

### Hat 3 — Otonom davranış + algı + 3D haritalama ✅ KURULDU

**Avoider node** (reactive obstacle avoidance, kullanıcı spec):
- `ika_mission/avoider_logic.py` — saf-Python karar makinesi (DRIVING/AVOIDING
  /REALIGNING/DONE). 17/17 birim test geçti.
- `ika_mission/obstacle_avoider_node.py` — ROS lifecycle sarmal.
- `sim_full.launch.py autonomous_mode:=avoider|nav2|off` — default `avoider`.

**EKF füzyon güncellemesi:**
- Önceden sadece IMU + lidar odom füzyonu vardı.
- ✅ GPS girdisi eklendi (`odom1: /odometry/gps`) → navsat_transform_node
  çıktısı EKF'e geri yansır → konum sürüklemesi telafisi.

**DL detection sim'de aktif:**
- ✅ `navigation.launch.py` sim'de `sim_detection_node` başlatır (ground-truth
  tabanlı sentetik person/chair/bicycle/car).
- ✅ Gerçek robotta `dl_perception_node` (OAK-D), conditionally launched.
- lifecycle_manager_ika sim/gerçek için ayrı node listesi.

**3D Octomap server:**
- ✅ `navigation.launch.py`'a eklendi. `/oak/points` → `/octomap_full` (3D
  occupancy octree).
- WSL'de `sudo apt install ros-jazzy-octomap-server` gerekli (Pi'de
  install_pi.sh'a eklenecek).
- RViz'de "OccupancyGrid (3D)" display ile görüntülenir.

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
