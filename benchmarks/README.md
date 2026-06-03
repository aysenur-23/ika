# IKA Tez Benchmark Altyapısı

Tezdeki **karşılaştırma tabloları + plotlar + harita görsellerini** otomatik
üretir. 3 mod (avoider, nav2_dwb, nav2_mppi) × 8 senaryo × 5 trial = 120 koşum.

## Hazırlık

```bash
# WSL veya Pi:
source /opt/ros/jazzy/setup.bash
source ~/ika/ika_ws/install/setup.bash
pip install pyyaml numpy matplotlib
```

## Kullanım

### Tam benchmark (uzun — 4-6 saat)

```bash
cd ~/ika
python3 benchmarks/run_benchmark.py
```

Çıktı:
- `benchmarks/results/raw_runs.csv` — her satır 1 koşum
- `benchmarks/results/trajectories/<id>.npz` — (T, 3) yörünge
- Periyodik ara kaydetme; çökerse partial sonuç korunur

### Hızlı test — tek senaryo, tek mod

```bash
python3 benchmarks/run_benchmark.py \
  --scenarios s2_ramp_climb \
  --modes avoider \
  --trials 1
```

### Tabloları + plotları üret

```bash
python3 benchmarks/render_tables.py
```

Çıktı:
- `results/summary.md` — Markdown tablo (tezin ekine yapıştır)
- `results/summary.tex` — LaTeX longtable (`\input{benchmarks/results/summary.tex}`)
- `results/plots/traj_<id>.png` — her senaryo için tüm modların yörüngeleri
- `results/plots/success_heatmap.png` — başarı yüzdesi ısı haritası
- `results/plots/duration_boxplot.png` — süre dağılımı

## Mimari

```
run_benchmark.py
    │
    ├─ scenarios.yaml oku
    ├─ her (senaryo × mod × trial) icin:
    │    ├─ sim_full.launch.py baslat (mod arg ile)
    │    ├─ 45 sn bringup bekle
    │    ├─ Recorder node /odometry/filtered ve /avoider_state dinle
    │    ├─ nav2 ise /goal_pose yayinla; avoider ise auto-start
    │    ├─ DONE state veya timeout
    │    ├─ metrik hesapla (path_length, clearance, curvature, speed)
    │    └─ CSV + npz kaydet
    │
    └─ Sim'i SIGINT + SIGKILL ile temiz kapat (gz sim dahil)

render_tables.py
    │
    ├─ raw_runs.csv oku
    ├─ Senaryo × Mod gruplama
    ├─ summary.md (Markdown tablo)
    ├─ summary.tex (LaTeX longtable)
    └─ plots/ (matplotlib png'ler)
```

## Tez Çıktı Akışı

1. `python3 benchmarks/run_benchmark.py` (gece bırak)
2. `python3 benchmarks/render_tables.py`
3. Tezdeki bölümlere:
   - **Tablolar:** `results/summary.md`'den kopyala
   - **LaTeX:** `\input{benchmarks/results/summary.tex}`
   - **Grafikler:** `results/plots/*.png` → tez figürleri
4. **Trajectory animasyonu (bonus):** `results/trajectories/*.npz` numpy ile
   yükle, matplotlib FuncAnimation ile gif üret.

## Senaryolar (`scenarios.yaml`)

7 obstacle taxonomy sınıfından + tam parkur = 8 senaryo. Her biri:
- start pose, goal
- obstacles_for_clearance (clearance metriği için sabit nokta listesi)
- pass_criteria (max_duration_s, min_clearance_m)

Senaryo ekleme: yaml'a yeni blok ekle, run_benchmark.py otomatik kullanır.

## Modlar (`scenarios.yaml`)

```yaml
modes:
  - id: avoider       # Reaktif (ika_mission/obstacle_avoider)
  - id: nav2_dwb      # Nav2 + DWB klasik
  - id: nav2_mppi     # Nav2 + MPPI
```

İleride RL planner eklenince:
```yaml
  - id: rl_planner
    launch_args: {autonomous_mode: rl}
```

## Bilinen Sınırlamalar

- **WSL'de full benchmark uzun sürer** — Pi yerine WSL kullanıyorsan
  expected ~30 dk/koşum. Pi'de daha hızlı.
- **Çakışan rf2o**: önceki sim'den artakalan node'lar çakışabilir. Script
  her trial sonunda `pkill gz sim` temizler ama nadir durumda manuel temizleme
  gerekebilir.
- **3D harita (octomap)**: bu benchmark'ta kaydedilmiyor. Ayrı bir `rosbag
  record /octomap_full` ile yakalanır.
