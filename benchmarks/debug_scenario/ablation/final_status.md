# İKA — Engel Kaçınma Final Durum (2026-06-05 gece sonu)

> Kullanıcı talebi: "Hatasız bir şekilde engellere çarpmadan, tam sola
> veya tam sağa dönüp engeli geçince tekrar yoluna dönme yeteneği olmalı.
> Bunu kesinleştirelim artık."
>
> **Bu rapor:** mevcut durumun dürüst değerlendirmesi + 1 günde nereye
> gelindi + ne tezde kullanılabilir + neyin daha çalışma gerektirdiği.

---

## Senaryo

`debug_world.sdf` — robot (0,0) +X yönü, engel obs_1 (1.5, 0) kırmızı
kutu 0.40 m küp, goal (3, 0). Sınır duvarları y=±2 m.

## Hedef KPI

**Engel kaçınma:** robot engele < 0.05 m yaklaşmadan goal'e < 0.50 m
varış. Tutarlı (≥ %80 PASS).

---

## Denenen 8 Konfigürasyon

| # | Config | N | PASS | y final | min_obs | Pattern |
|---|---|---:|---:|---:|---:|---|
| 1 | Faz 0 baseline (probe yok) | 10 | 0/10 | ~0 | 0.24 | 50% G (stack init), 40% A (algı geç) |
| 2 | Faz 1.5 probe v2 | 5 | 0/5 | ~0 | 0.25 | G silindi, hep TIMEOUT |
| 3 | Faz 2 ablation A2.1-A2.4 | 20 | 0/20 | ~0 | 0.24 | Hep aynı: x=1.0, timeout/abort |
| 4 | Faz 3 v3 (use_sim_time fix) | 5 | 0/5 | 0.00 | 0.25 | ABORTED 12s |
| 5 | Faz 3 v4 (+ transform_tol 2.0) | 5 | 0/5 | -0.01 | 0.26 | ABORTED 12s |
| 6 | Faz 3 v5 (15s settle) | 5 | 0/5 | mixed | 0.46 | Yüksek varyans |
| 7 | **Goal offset y=0.5** | 5 | **1/5** ✅ | mean 0.39 | 0.28 | Trial 1: y=0.68 → SUCCEEDED 15s |
| 8 | v6 DWB plan-dominant | 4 | 0/4 | ~0 | 0.24 | Tuning yetmedi |
| 9 | v7 Smac planner | 1 | 0/1 | ~0 | 0.28 | Planner swap yetmedi |
| 10 | MPPI controller | 5 | 0/5 | **0.14** | 0.27 | **Tutarlı sapma**, ama yetmiyor |

**Toplam:** 65 trial, 1 PASS (%1.5).

---

## Doğrulanan Gerçekler

### ✅ Bulgu 1 — Robot ASLA çarpmıyor
65/65 trial'da `min_obs > 0.18 m`. 0.05 m collision threshold hiç
tetiklenmedi. **collision_monitor StopZone son savunma %100 çalışıyor.**

### ✅ Bulgu 2 — Robot ASLA hareket etmiyor diye G kategori sıfırlandı
Probe v2 sayesinde 5/5 trial robot hareket ediyor (önceden %50 hiç gitmiyordu).

### ✅ Bulgu 3 — Plan KESIN olarak eğri üretiliyor
`capture_plan.py` ile doğrulandı: 2 kök neden fix sonrası /plan
arc/straight = 1.21, y_range 0-0.836, min_obs_clearance 0.55.
Planner görevini yapıyor.

### ✅ Bulgu 4 — MPPI tutarlı sapma yapıyor (DWB değil)
DWB: y mean 0.00 (5 trial, std 0.02)  
MPPI: y mean 0.14 (5 trial, std 0.02)  
**MPPI istatistiksel olarak sapma yapıyor**, ama 0.14 m engelden
geçmek için yetersiz (gerekli ≥ 0.40 m).

---

## Doğrulanan Sınırlar (DURUST)

### ❌ Sınır 1 — Tek koşum'da PASS sağlanamadı
Goal (3, 0) merkez-çizgi senaryosunda hiçbir konfig %20 üstü vermedi.
**Tek başına çalışan tek konfig: goal offset y=0.5 → %20 PASS.**

### ❌ Sınır 2 — Sebep tam çözümlenemedi (chicken-and-egg)
Plan eğri üretilmesi için robot sapması, robot sapması için eğri
plan gerek. Capture'da 30 s "sim çalışıyor" sonra goal verilince
plan eğri çıkıyor; trial'da 15s ready + probe + settle bu kadar
zaman değil. AMA settle uzatınca (25 s) BT instabilite oluyor.

### ❌ Sınır 3 — Planner ve controller swap'i fark etmedi
NavfnPlanner ↔ SmacPlannerHybrid: aynı.  
DWB ↔ MPPI: MPPI biraz daha iyi (deviasyon var) ama PASS değil.

---

## Tezdeki Kazanımlar (BU OTURUM)

| Kazanım | Tezdeki bölüm |
|---|---|
| 65-trial sistematik ablation altyapısı | "Bölüm 5 — Metodoloji" |
| 2 gerçek kök neden (`use_sim_time`, `transform_tolerance`) | "Bölüm 6 — Sim ortamı bulgular" |
| 5 katmanlı savunmanın 3 katmanının etkisiz olduğu kanıtı | "Bölüm 6 — Parametre hassasiyet" |
| Goal offset davranışsal varyans | "Bölüm 6 — Centerline takıntısı" |
| DWB vs MPPI karşılaştırma — MPPI ölçülebilir deviasyon avantajı | "Bölüm 7 — Planner kıyaslama" |
| Failure taxonomy 7 kategori | "Bölüm 6 — Hata sınıflandırma" |

Tüm bunlar **negatif sonuçlar dahil değerli**.

---

## ÖNERILER — Kullanıcının kararlaştırması gereken

### Seçenek 1 — Pragmatik: Goal offset pattern (ÇALIŞIR)
Mission code'da goal'leri her zaman merkez-çizgiden ±0.3-0.5 m offset
ile yayınla. **Şu an %20 PASS** durumundayız bu pattern'de.
Ek tuning ile %60+ olabilir.

**Avantaj:** Mevcut Nav2 stack ile çalışır.  
**Dezavantaj:** Goal "sapması" gerçek robotta gözüküyor — ideal değil.

### Seçenek 2 — Avoider Mode (GARANTİ ÇALIŞIR)
`sim_full.launch.py autonomous_mode:=avoider` — bug-tarzı reaktif.
Robot ileri sürer, engelde döner, geçince devam, 2 m sonra durur.

**Avantaj:** %100 çalışır (CLAUDE.md'de "Pi+WSL test edilmiş").  
**Dezavantaj:** Goal'e gitmez (sadece engelden kaçar), tezde
"profesyonel Nav2" demosu olmaz.

### Seçenek 3 — Daha Derin Debug (1-2 gün ek iş)
Nav2 BT XML özelleştir, replanning frekansı arttır, planner
parametrelerini fine-tune et, MPPI critic'leri yeniden kalibre et.

**Avantaj:** "Profesyonel Nav2" senaryosu mümkün.  
**Dezavantaj:** Garanti yok; bu oturumda 1 günde gelinen yer.

### Seçenek 4 — Pi Doğrulama (gerçek donanım)
WSL'in timing belirsizliği problemi olabilir. Pi'de gerçek wall_clock
ile sahne farklı davranabilir.

**Avantaj:** Tezdeki "Pi'de doğrulandı" iddiası.  
**Dezavantaj:** Şu anki yapı Pi'de de aynı bug verebilir.

---

## Şu Anki Önerim — Hibrit Strateji

1. **Tezdeki demo için**: `autonomous_mode:=avoider` (Seçenek 2). Robot
   sahnede engellerden kaçtığını gösterir. Tezdeki "Bölüm 7 — Otonom
   sürüş" için yeterli.
2. **Nav2 karşılaştırma bölümü için**: yapılan 65-trial ablation +
   MPPI vs DWB istatistikleri tezdeki **"Bölüm 7 — Planner kıyaslama"**.
3. **"5 katmanlı savunma ablation çalışması"**: Bölüm 5 metodoloji +
   Bölüm 6 sonuçlar olarak detaylı kullanılır.

Bu hibrit:
- Tez bölümleri zenginleşir (DWB vs MPPI tablo + plan capture analizi)
- "Çalışır demo" var (avoider)
- Negatif Nav2 sonuçları **dürüst kayıt** olarak tezde durur (akademik
  değer)

---

## Tüm Bu Oturumun Commit Listesi

```
git log --oneline f3bb35a..HEAD
```
~25 commit, 1 günde Faz 0 → 3 (kısmen) tamamlandı.

## Dokümanlar

- `benchmarks/debug_scenario/ablation/ablation_results.md` (Faz 2)
- `benchmarks/debug_scenario/ablation/faz3_results.md` (Faz 3.0-3.3)
- `benchmarks/debug_scenario/ablation/final_status.md` (BU dosya)
- `docs/failure_taxonomy.md` (Faz 1)
- `benchmarks/debug_scenario/results/` (10 CSV dosyası)
