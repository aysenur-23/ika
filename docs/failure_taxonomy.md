# İKA Faz 1 — Başarısızlık Modu Taksonomisi

> **Amaç:** "Robot engele takılıyor" şikayetini ölçülebilir kategorilere bölmek.
> Her FAIL koşumu **tek bir kategoride** yer alır. Kategoriler katmanlara
> karşılık gelir (algı → planlama → kontrol → cmd_vel zinciri → tf/clock →
> recovery loop).
>
> Veri kaynağı: `benchmarks/debug_scenario/results/baseline_*.csv` +
> `/tmp/ika_trial_*.log` (her trial'ın launch log'u).

---

## Kategori Tanımları

### A — Algı geç (perception_late)
**Belirti:** local/global costmap'te engel hiç görünmüyor veya robot ona
çok yaklaşana kadar görünmüyor. DWB engel görmediği için içine yönelir.

**Ölçüm:** Engel x=1.5 ise, robot x=1.3'te (20 cm önce) costmap engeli
görmüş olmalı (inflation 0.55 → engel 0.95'te bile dolulukla işaretlenir).
`/local_costmap/costmap` topic'inde ilk dolu hücre stamp'ı vs robot pozu.

**CSV imzası:** `FAIL_COLL` + `min_obs_dist ≤ 0.25` + duration < 8 s
(robot tam hızla engele dalmış: ~0.25 m/s × 4 s = 1 m).

### B — Planlama yanlış (planner_through_obstacle)
**Belirti:** `/plan` topic'i engel hücresinden geçiyor. NavfnPlanner /
SmacPlannerHybrid inflation'ı görmüyor veya tolere ediyor.

**Ölçüm:** `/plan` PoseStamped array'i çek, engel x±inflation içinde nokta var mı?

**CSV imzası:** `FAIL_COLL` + duration > 10 s (yavaş yavaş engele girmiş,
DWB de path'i takip etmiş).

### C — Controller takip etmiyor (controller_ignores_plan)
**Belirti:** Path doğru (engelin etrafından), ama `/cmd_vel_nav` engele
doğru hız komutu üretiyor. DWB cost ağırlıkları yanlış (PathAlign >>
BaseObstacle).

**Ölçüm:** `/plan` ve `/cmd_vel_nav` ters yönde, robot path'i terk ediyor.

**CSV imzası:** `FAIL_COLL` + dist_to_goal büyük + duration normal.

### D — cmd_vel bypass (relay_bypass)
**Belirti:** controller_server dur diyor (`/cmd_vel_nav = 0`) ama tekerlek
hâlâ dönüyor. relay zinciri yanlış.

**Ölçüm:** `/cmd_vel_nav` ve `/cmd_vel_collision` ve `/cmd_vel` üçünü
karşılaştır.

**CSV imzası:** `FAIL_COLL` + ani çok yakın (min_obs_dist çok küçük).

### E — TF / saat tutarsızlığı (tf_clock_drift)
**Belirti:** EKF "Failed to meet update rate", controller "Transform data
too old", Nav2 lifecycle "bond timeout". WSL2'ye özgü sim_time/wall_clock
kayması.

**Ölçüm:** `/diagnostics` topic'inde TF freq, EKF rate; launch log'da
"transform extrapolation" hataları.

**CSV imzası:** `FAIL_NAV` + nav2_result='ABORTED' veya `FAIL_TIMEOUT` +
final_x küçük (robot hiç hareket etmemiş).

### F — Recovery loop (recovery_spin_forever)
**Belirti:** BT Spin → BackUp → Wait → Spin döngüsünde, replanlama
hep aynı blocked-path'i veriyor. 3 deneme sonra abort.

**Ölçüm:** BT log'da `<recovery>` tag'ı tekrar eden çağrılar.

**CSV imzası:** `FAIL_NAV` + duration uzun + final_x engele yakın.

### G — Sim altyapısı / stack init (sim_infra_fail)
**Belirti:** Nav2 action server READY_WAIT (45 s) sonra hâlâ yanıt vermez;
veya goal accept eder ama hemen ABORTED. WSL'de DDS keşfi gecikmesi,
costmap ilk update'i gecikmesi.

**CSV imzası:** `FAIL_NAV` + nav2_result='goal_send_timeout' veya 'ABORTED'
+ duration < 3 s + final_x ≤ 0.5 (robot hiç hareket etmedi).

---

## Baseline N=10 Sınıflandırması — 2026-06-05 13:45

CSV: `benchmarks/debug_scenario/results/baseline_n10.csv`

| Trial | Status | min_obs | dur | final_x | Kat. | Not |
|---:|---|---:|---:|---:|:-:|---|
| 1 | FAIL_COLL | 0.243 | 4.40 | 1.058 | **A** | Engele 0.25 m/s tam hızla daldı |
| 2 | FAIL_COLL | 0.195 | 3.53 | 1.105 | **A** | Daha derin girdi (-0.05 m içeride) |
| 3 | FAIL_NAV | 1.320 | 3.62 | -0.026 | **G** | goal_send_timeout, Nav2 cevap vermedi |
| 4 | FAIL_NAV | 1.207 | 0.34 | 0.093 | **G** | ABORTED 0.3 s'de — costmap ready değildi |
| 5 | FAIL_NAV | 0.261 | 32.63 | 1.003 | **F** | Recovery 32 s döndü, sonra abort |
| 6 | FAIL_NAV | 0.884 | 2.32 | 0.416 | **G** | ABORTED 2.3 s — 40 cm ilerledi |
| 7 | FAIL_COLL | 0.233 | 3.21 | 1.067 | **A** | Çarptı |
| 8 | FAIL_NAV | 1.255 | 1.06 | 0.045 | **G** | ABORTED 1 s — robot hiç hareket etmedi |
| 9 | FAIL_COLL | 0.244 | 3.35 | 1.057 | **A** | Çarptı |
| 10 | FAIL_NAV | 1.285 | 1.06 | 0.004 | **G** | ABORTED 1 s — robot hiç hareket etmedi |

---

## Kategori Dağılımı

| Kategori | Sayı | % | Yorum |
|---|---:|---:|---|
| **A** — Algı geç | **4** | 40% | Tutarlı tek mod: robot x=1.05'te durur, min_obs ≈ 0.24 |
| **B** — Planlama yanlış | 0 | 0% | (gözlenmedi — `/plan` daha incelenmedi) |
| **C** — Controller bypass | 0 | 0% | (B ile birlikte ileri analiz) |
| **D** — cmd_vel relay bypass | 0 | 0% | (collision_monitor son savunma çalışıyor, /cmd_vel_collision aktif) |
| **E** — TF/clock drift | 0 | 0% | (G ile karışabilir — log analizi gerekli) |
| **F** — Recovery loop | **1** | 10% | Trial 5: robot ulaştı, 32s döndü, abort |
| **G** — Sim altyapısı / stack init | **5** | 50% | 5 trial'da robot hiç hareket etmedi veya 2s içinde abort |
| **PASS** | **0** | **0%** | — |

**Toplam:** 10 / 10 FAIL. PASS rate: **%0**.

---

## Kritik Gözlemler

### Gözlem 1 — İki net mod, ilgisiz nedenler
Tüm FAIL'ler iki kümeye düşüyor:
- **Robot 1m gitti + çarptı** (4 trial, A): perception/control çöktü
- **Robot hiç gitmedi** (5 trial, G): stack ready değil

Bu **bağımsız** iki problem. Birini çözmek diğerini çözmez.

### Gözlem 2 — "Çarpan" trial'lar çok tutarlı (A)
final_x = 1.05–1.11, min_obs = 0.20–0.25, duration = 3.2–4.4 s, hep aynı
yörünge. Tam hızla (0.25 m/s) düz çizgi, engelde dur (collision_monitor
StopZone). Yani **collision_monitor çalışıyor** — son savunma tutuyor.
Ama tam zamanında: 24 cm. DWB / costmap erken hiçbir kaçınma sinyali
üretmemiş. Bu da algı katmanının (Faz 2 ablation hedef #1) çöktüğünü
gösterir.

### Gözlem 3 — "Hareket etmeyen" trial'lar varyanslı (G)
- Bazen Nav2 goal'i accept etmez (goal_send_timeout)
- Bazen accept eder, anında ABORTED
- Bazen 2 s sonra ABORTED, robot 40 cm gider
- Bazen 1 s'de ABORTED, robot hiç gitmez

Tek bir kök neden yok — **stack init süresinin yüksek varyansı**. READY_WAIT
45 s yetersiz **bazı koşumlarda**. Trial sayısının arttırılması (N=20+)
varyansı daha net gösterir.

### Gözlem 4 — ILERLEME.md v18 "PASS" iddiası tekrar üretilemedi
ILERLEME.md (2026-06-04, 5 katmanli savunma bölümü) "v18 - en stabil tur:
Robot 2.5 m otonom ilerleme + obs_1 önünde 50 cm güvenli durus,
CARPISMA SIFIR" diyor. Aynı parametrelerle bugün N=10 trial koşuldu,
**hiçbiri 1.1 m'den ileri gidemedi**. Bu:
- Tek bir başarılı koşumun cherry-pick olduğunu, veya
- Parametrelerin sonradan kaybolduğunu (`mppi_controller.yaml` stash'ledim
  — `transform_tolerance: 0.2→1.0` değişikliği gözden kaçmış olabilir), veya
- WSL'nin doğası gereği güvenilmez ölçüm verdiğini gösterir.

Faz 3 (Pi vs WSL kararı) bu sorunu cevaplamalı.

---

## Faz 2 Önceliklendirme

Kategori A (%40) ve G (%50) birlikte FAIL'lerin %90'ını oluşturuyor.

**Faz 2 ablation #1 (A için):** Algı freq + DWB scale + inflation
ablation'ı. "Robot neden engeli geç görüyor?" cevaplanmalı.

**Faz 2 ablation #2 (G için):** Stack init süresinin sertleştirilmesi.
READY_WAIT 45 → 60 s, ya da daha iyisi: **aktif hazırlık probu**
(Nav2 action server ready VE local costmap ilk update'i geldi VE TF
map→base_link mevcut) `run_trial.py` içinde beklenmeli.

Ablation matrisi `benchmarks/debug_scenario/ablation_results.md`'de
tutulacak.
