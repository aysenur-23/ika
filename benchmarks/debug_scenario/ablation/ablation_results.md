# İKA Faz 2 — Ablasyon Sonuçları

> **Tarih:** 2026-06-05  
> **Senaryo:** `debug_world.sdf` — robot (0,0) → engel obs_1 (1.5,0, kutu
> 0.40 m) → goal (3,0). Sınır duvarları y=±2 m, 4 m geniş koridor.  
> **Trial:** N=5 her ablasyon (toplam 20 trial). Probe v2 stack-ready
> bekleyici aktif. COLLISION_THRESHOLD = 0.05 m (~temas).
>
> **Amaç:** Hangi katmanın PASS'a gerçekten katkı verdiğini ölçmek. ILERLEME
> v18'de iddia edilen "5 katmanlı savunma"nın bileşenleri tek tek izole
> edildi.

---

## Konfigürasyon Tablosu

| Ablation | DWB BaseObstacle.scale | inflation_radius | collision_monitor relay |
|---|---:|---:|:-:|
| **A2.4** baseline   | 40.0 | 0.55 | ON (`/cmd_vel_collision`) |
| **A2.1** coll-only  | **0.0** | **0.05** | ON |
| **A2.2** dwb-only   | 40.0 | 0.55 | **OFF** (`/cmd_vel_nav` direkt) |
| **A2.3** infl-only  | **0.0** | 0.55 | **OFF** |

---

## Sonuç Özeti

| Ablation | PASS | FAIL_TIMEOUT | FAIL_NAV | dur μ ± σ (s) | min_obs μ ± σ (m) | final_x μ (m) |
|---|---:|---:|---:|---|---|---:|
| **A2.4 baseline** | 0/5 | **5 (100%)** | 0 | 60.01 ± 0.00 | 0.24 ± 0.02 | 1.00 |
| **A2.1 coll-only** | 0/5 | 4 (80%) | 1 | 51.98 ± 16.05 | 0.22 ± 0.02 | 1.03 |
| **A2.2 dwb-only** | 0/5 | 0 | **5 (100%)** | 17.12 ± 13.45 | 0.27 ± 0.05 | 1.01 |
| **A2.3 infl-only** | 0/5 | 0 | **5 (100%)** | 29.72 ± 14.89 | 0.25 ± 0.05 | 1.01 |

**Toplam:** 0/20 PASS. Hiçbir konfigürasyon engeli geçemedi.

---

## Kritik Bulgular

### Bulgu 1 — DWB obstacle cost + inflation **etkisiz**
A2.4 (DWB.scale=40, infl=0.55) ≈ A2.1 (DWB.scale=0, infl=0.05).  
Her ikisi de TIMEOUT 60s, min_obs ≈ 0.23 m. **5 katmanlı savunmanın
"DWB cost" ve "inflation" katmanları gerçekte iş yapmıyor.** ILERLEME
v18'deki "BaseObstacle.scale 40.0 explicit (yola karşı 32) DWB sağ/sol
rotate trajectory'i seçer" iddiası **doğrulanmadı**.

### Bulgu 2 — collision_monitor **tek belirleyici**
- collision_monitor ON (A2.4, A2.1): robot 60 s boyunca x≈1.00'da durur,
  min_obs ≈ 0.24. Recovery loop sonsuz dener ama dolaşamaz.
- collision_monitor BYPASS (A2.2, A2.3): Nav2 17-30 s'de ABORTED döner.
  Robot yine x≈1.01'de takılır.

collision_monitor'un StopZone (30 cm) → robot 24 cm yakına gelene kadar
serbest, sonra durur. Backstop görevini doğru yapar.

### Bulgu 3 — Ablation'lar arasındaki **GERÇEK** fark süre + son durum
collision_monitor ON: TIMEOUT 60 s (Nav2 BT vazgeçmeyi sürdürür).  
collision_monitor BYPASS: NAV ABORTED 7-45 s (Nav2 BT recovery hızlı
tükenir; muhtemelen control loop hata feedback alıyor).

Robotun fiziksel hareketi her dörtte **aynı**: x≈1.0'da takılır.

### Bulgu 4 — Bir tek sapma örneği (A2.2 trial 1)
Tüm 20 trial içinde **sadece 1 trial** robot yana kaydı: A2.2 trial 1
→ y=-0.077, min_obs=0.362. DWB engelden 36 cm uzaklaştı. Diğer 19
trial'da y < 0.05 (düz çizgi).

Bu, DWB'nin **prensipte** kaçınma trajectory üretebildiğini ama **sürekli
yapamadığını** gösterir. Olasılıkla nav2 BT, planner'ın ürettiği
yumuşak-S yörünge yerine düz path'i takip etmeye çalışıyor, DWB rotasyon
denemesi sonra abort.

### Bulgu 5 — Tutarlılık (probe sayesinde)
Faz 0 baseline (probe yok): 50% trial robot hiç hareket etmedi (G kategori,
stack init).  
Faz 2 (probe + setsid + threshold): **20/20 trial robot hareket etti** ve
**hep aynı yere takıldı**. Probe Kategori G'yi sıfırladı; kalan
deterministik problem ortaya çıktı.

---

## Yeniden Değerlendirme — Faz 1 Taksonomisi

Faz 1'de "%40 Kategori A (algı geç)" demiştik. **Bu yanlış teşhisti.**
Doğru sınıflandırma:

| Eski tanı | Gerçek sınıf | Düzeltme |
|---|---|---|
| A — Algı geç | **F — Recovery loop** | Robot algı zamanında, recovery dolaşamıyor |
| G — Stack init | (probe ile %0'a düştü) | Çözüldü |
| F — Recovery loop | F — Recovery loop | Doğru |
| H (yeni) — **Planner çıkışı düz** | (Faz 3 hipotezi) | `/plan` topic incelenmedi; muhtemelen düz çizgi veya çok keskin S |

Yeni kategori **H — planner_output_unfollowable**: planner çıkışı
DWB'nin diff-drive kinematiğiyle takip edilemiyor, ya da planner zaten
düz çizgi üretiyor.

---

## Faz 3 Önerisi (revised)

**Önceki plan:** WSL vs Pi karşılaştırması.  
**Yeni öneri:** Önce `/plan` topic incelemesi — algoritmik problemse Pi'de
de aynı bug çıkar.

### Faz 3.0 — `/plan` analiz (1 saat)
Trial sırasında `ros2 topic echo /plan --once > /tmp/plan.txt` snapshot al.
- Plan engelin etrafından geçiyor mu?
- Düz çizgi mi, S eğrisi mi?
- Eğer düz çizgi → planner sorunu (NavfnPlanner, inflation görmüyor).
- Eğer S eğrisi → DWB sorunu (kinematik takip edemiyor).

### Faz 3.1 — Diff drive kinematik parametreleri (yarım gün)
DWB controller params:
- `vx_samples`, `vtheta_samples` (sample yoğunluğu)
- `min_vel_theta`, `max_vel_theta` (rotasyon hız sınırları)
- `acc_lim_x`, `acc_lim_theta` (ivme sınırları)

Robot 0.25 m/s tam hız + diff drive sıfır turning radius. Sample yoğunluğu
düşükse DWB rotasyon trajectory'leri üretmiyor olabilir.

### Faz 3.2 — Planner alternatifi (yarım gün)
NavfnPlanner (default) yerine **SmacPlannerHybrid** veya **SmacPlanner2D**
dene. Smac, kinematik kısıtları planlama sırasında dikkate alır (Hybrid-A*).
ABORTED vs PASS değişimi var mı?

### Faz 3.3 — Goal offset
Goal (3,0) engelin tam arkasında. Goal'i (3, 0.3) veya (3, -0.3) yap → planner
düz değil S üretmek zorunda. PASS oranı değişiyor mu?

### Faz 4 — Pi doğrulama (Faz 3 sonrası)
WSL'de bir konfig PASS verince Pi'de doğrula. WSL'de PASS yoksa, Pi'de de
büyük ihtimalle yoktur.

---

## Tezde Doğrudan Kullanılabilir

Bu ablasyon tablosu **parametre hassasiyet analizi** olarak tezde
**direkt** referans verilir:
- "5 katmanlı savunma" tasarımı vs gerçek katkı
- Bir katmanın değiştirilmesinin etkisi (DWB scale 40→0 hiçbir şey
  değiştirmedi — beklenmedik bulgu)
- collision_monitor'un asimetrik rolü: defansif başarılı, ofansif yok

Tez bölümü taslağı: **"Bölüm 4.3 — Engel kaçınma katman ablation
çalışması"**.
