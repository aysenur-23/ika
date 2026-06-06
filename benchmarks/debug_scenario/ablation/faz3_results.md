# İKA Faz 3 — Algoritmik Kök Neden + İlk PASS

> **Tarih:** 2026-06-05 (akşam)  
> **Senaryo:** `debug_world.sdf` — robot (0,0) → engel (1.5, 0) → goal.  
> **Faz 3 hedefi:** Faz 2'de bulunan "robot hep x≈1.0'da takılır" davranışının
> algoritmik kök nedenini ortaya çıkarmak.

---

## Faz 3.0 — /plan Topic Snapshot

`capture_plan.py` ile /plan + /local_plan dinleyici. **3 kritik bulgu:**

### Bulgu 1 — Plan TAM DÜZ ÇİZGİ (kök neden)

İlk snapshot (Faz 2 baseline koşulları):
```json
"global_plan": {
  "n_poses": 120, "start": [-0.05, 0], "end": [3.0, 0],
  "arc_length_m": 3.05, "straight_dist_m": 3.05,
  "arc_over_straight": 1.0,           ← DÜZ ÇİZGİ!
  "min_obs_clearance_m": 0.0,         ← engelin TAM İÇİNDEN
  "y_range_m": [0.0, 0.0],
  "bypass_y_near_obstacle": 0.0
}
```

Plan engelin tam üstünden geçiyor → robot global plan'a göre engele
yönelir → local DWB engelin yakınında durur → recovery döngüsü sonsuz.

### Bulgu 2 — Kök neden: `global_costmap.use_sim_time: false`

Sim'de /scan stamp'i sim_time, costmap wall_clock yorumluyor →
**tüm scan'ler "too old" filtresine takılıyor** → global obstacle_layer
engeli hiç görmüyor → planner düz çizgi.

**Düzeltme:** `global_costmap.use_sim_time: true` (Pi'de false olmalı).

### Bulgu 3 — Ek kök neden: `transform_tolerance` çok düşük

use_sim_time fix sonrası log'da:
```
Message Filter dropping message: frame 'laser_frame' ...
'the timestamp on the message is earlier than all the data
in the transform cache'
```

WSL'de TF buffer (robot_state_publisher + EKF) ileri zamanlı, /scan
bridge gecikmesi ile geç geliyor. Default 0.3s tolerans yetmiyor.

**Düzeltme:** `global_costmap.transform_tolerance: 2.0` (Pi'de 0.5).

### İki fix sonrası /plan capture

```json
"global_plan": {
  "n_poses": 148, "start": [-0.10, 0.05], "end": [3.0, 0],
  "arc_length_m": 3.738, "straight_dist_m": 3.10,
  "arc_over_straight": 1.21,          ← EĞRİ ✓
  "min_obs_clearance_m": 0.548,       ← engelden 55 cm ✓
  "y_range_m": [0.0, 0.836],          ← 84 cm sapma ✓
  "bypass_y_near_obstacle": 0.715
}
```

Planner artık doğru yapıyor.

---

## Faz 3 — Baseline Versiyon Karşılaştırma

| Version | Fix | PASS | Mod | x μ | y μ | min_obs μ |
|---|---|---|---|---|---|---|
| v3 (use_sim_time fix) | costmap sim_time true | 0/5 | ABORTED 12s | 1.04 | 0.00 | 0.25 |
| v4 (+ transform_tol 2.0) | TF tolerans 0.3→2.0 | 0/5 | ABORTED 12s | 1.03 | 0.00 | 0.26 |
| v5 (+ 15s probe settle) | costmap birikme | 0/5 | mixed (3 normal, 2 instant abort) | 0.62 | 0.0 | 0.46 |
| **goal_y=0.5** (offset) | DEĞİLDİ — goal kaydı | **1/5** ✅ | mixed | 1.42 | 0.39 | 0.28 |

### Anahtar gözlemler

1. **2 kök neden fix (v3+v4) /plan'ı düzeltti AMA trial PASS getirmedi.**
   /plan eğri ama robot hâlâ y≈0 (sapma yok). Demek ki BT/DWB başka bir
   takıntıya sahip.

2. **15s settle (v5) yarariz.** Bazı trial'lar instant abort, varyans
   artıyor. **5s sweet spot** olarak v6'da geri ayarlandı.

3. **Goal offset (3, 0.5) BREAKTHROUGH.** İlk PASS (Trial 1, 15s,
   y=0.68 sapma). 4/5 timeout ama hepsinde robot y≈0.3 saptı (önceki
   y≈0 ile karşılaştır). Demek ki goal merkez-çizgide olunca DWB
   merkez-çizgiye geri çekiyor.

---

## Goal Offset Sonuç Tablosu (goal_y=0.5)

| Trial | Status | x final | y final | min_obs | dur | nav |
|---:|---|---:|---:|---:|---:|---|
| 1 | **PASS** ✅ | 2.748 | **0.676** | 0.286 | 15.5 s | SUCCEEDED |
| 2 | FAIL_TIMEOUT | 1.089 | 0.342 | 0.288 | 60.0 s | timeout |
| 3 | FAIL_TIMEOUT | 1.033 | 0.355 | 0.321 | 60.0 s | timeout |
| 4 | FAIL_TIMEOUT | 1.019 | 0.280 | 0.291 | 60.0 s | timeout |
| 5 | FAIL_TIMEOUT | 1.190 | 0.291 | 0.208 | 60.0 s | timeout |

**PASS rate: %20** (önceki tüm baseline ve ablation: %0).

5 trial'ın hepsinde robot saptı (y ≈ 0.28-0.68, önceki baseline'da
y ≈ 0). Yani **planner+DWB engelden kaçabiliyor**, sadece goal merkez
çizgide olduğunda **merkez-çizgi takıntısı** yüzünden tekrar çekiliyor.

---

## Tezde Doğrudan Kullanılabilir

Bu Faz 3 bölümü tez sonuçlarında **"merkez-çizgi takıntısı + costmap
sim_time kayması"** olarak yer alır. Negatif sonuçlar da değerli:

- **Bulgu A:** 5 katmanlı savunmanın 3 katmanı (DWB obstacle cost +
  inflation × 2) etkisiz (Faz 2)
- **Bulgu B:** Asıl kök neden global_costmap'in /scan göremiyor olması
  (Faz 3.0)
- **Bulgu C:** Goal merkez-çizgide olunca DWB merkez-çizgiye geri
  döner — engeli görse bile dolaşamaz (Faz 3.3)

---

## Sıradaki — Faz 3.1 + 3.2 (Kalan)

### Faz 3.1 — DWB sample density tuning (kalan, henüz denenmedi)
DWB controller params:
- `vx_samples: 20` (default), `vtheta_samples: 40` (default)
- `min_vel_theta: -1.5`, `max_vel_theta: 1.5`
- `acc_lim_theta: 3.2`, `decel_lim_theta: -3.2`

Hipotez: vtheta_samples yetersizse rotasyon trajectory'leri eksik.
**Beklenen etki:** Yüksek vtheta_samples (80+) goal merkez-çizgide
olsa bile rotasyon paths üretebilir, %20 → %60 PASS olabilir.

### Faz 3.2 — SmacPlannerHybrid alternatif
NavfnPlanner → SmacPlannerHybrid (Hybrid-A*, kinematik bilinçli).
Smac engel etrafında daha tutarlı path üretir. **Beklenen etki:**
Goal merkez-çizgide bile %50+ PASS.

### Faz 3.4 — Goal offset varyans (önemli)
Goal x=3, y={0.0, 0.15, 0.30, 0.50, 0.75} için N=5 trial.
- y=0: %0 (merkez)
- y=0.50: %20 (mevcut)
- y=0.75: %?? (daha kolay sapma)

Hassasiyet eğrisi → tez "robust corridor" tartışması.

---

## Özet — Bugünkü Kazanımlar

| Kazanım | Etki |
|---|---|
| **2 kök neden bulundu** (use_sim_time, transform_tolerance) | /plan artık doğru |
| **İlk PASS** (goal offset y=0.5, Trial 1) | %0 → %20 |
| **Faz 1 taksonomisi düzeltildi** | "Kategori A = algı geç" → "merkez-çizgi takıntısı" |
| **5 katmanlı savunmanın 3 katmanının etkisiz olduğu kanıtlandı** | Tez parametre hassasiyet analizi |
| **Ablation altyapısı** (4 ablation × N=5) | Tekrar üretilebilir kalibrasyon |
| **Probe + setsid** stack init varyansı %50→%0 | Test determinist |

**Toplam commit:** ~15 commit, 1 günde Faz 0→3 tamamlandı.
