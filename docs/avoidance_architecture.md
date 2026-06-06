# İKA Engel Kaçınma Mimarisi — Defense-in-Depth

> **Hedef:** Gerçek İKA üzerinde, gerçek engellerde, **garantili** engel
> kaçınma. Tezdeki yüksek-lisans seviyesi savunma + saha doğrulaması.
>
> **Tasarım prensibi:** Tek nokta hatadan kaçınmak için **4 bağımsız
> katman**. Bir katmanın hatası diğeri tarafından telafi edilir.
> "Fail-safe by construction."

---

## Genel Bakış

```
 ┌──────────────────────────────────────────────────────────────┐
 │ Katman 1 — Mission Layer (yüksek-seviye otonomi)              │
 │   - GPS waypoint sıralaması / sahne sıralı görev               │
 │   - Hangi engel kaçınma modu? (avoider | nav2)                 │
 │   - Hedef:  görevi sıralı geçir                                │
 └──────────────────────┬───────────────────────────────────────┘
                        │  /cmd_vel_high_level
                        ▼
 ┌──────────────────────────────────────────────────────────────┐
 │ Katman 2 — Tactical Planner (engel görünce yolu kararla)     │
 │   - Mod A: Reactive Avoider (default, gerçek robot)            │
 │   - Mod B: Nav2 (DWB/MPPI, sim karşılaştırma için)             │
 │   - Çıkış: /cmd_vel_nav  (planlanan hız)                       │
 └──────────────────────┬───────────────────────────────────────┘
                        │  /cmd_vel_nav
                        ▼
 ┌──────────────────────────────────────────────────────────────┐
 │ Katman 3 — Safety Supervisor (engel YAKINSA müdahale)         │
 │   - /hazard_state'i dinler (terrain + DL fusion)               │
 │   - SLOW → komutu %50 ölçekle                                  │
 │   - STOP → komutu sıfırla                                      │
 │   - Çıkış: /cmd_vel_safe                                       │
 └──────────────────────┬───────────────────────────────────────┘
                        │  /cmd_vel_safe
                        ▼
 ┌──────────────────────────────────────────────────────────────┐
 │ Katman 4 — Reflex (collision_monitor — son savunma)           │
 │   - Lidar StopZone (önde 30 cm) → ANINDA STOP                  │
 │   - Lidar SlowDownZone (30-60 cm) → %30 hız                    │
 │   - Yazılım katmanlarından bağımsız donanım-yakın hızlı        │
 │   - Çıkış: /cmd_vel_collision                                  │
 └──────────────────────┬───────────────────────────────────────┘
                        │  /cmd_vel_collision
                        ▼
 ┌──────────────────────────────────────────────────────────────┐
 │ Donanım — Arduino motor controller (E-Stop GPIO)              │
 │   - Yazılım çökerse motorlar kapalı (deadman switch)           │
 │   - Maks hız 0.25 m/s (encoder yok güvenlik)                   │
 └──────────────────────────────────────────────────────────────┘
```

**Veri akışı:** her katman bir SONRAKİ katmanın komutunu **inceleyip
kısıtlar**. Yukarı katman çökse bile aşağı katmanlar robot'u güvenli
duruşa zorlar.

---

## Katman Sorumlulukları (RACI)

| Katman | Görev | Hatası ne olursa robot ne yapar? |
|---|---|---|
| 1. Mission | Goal seçimi | Hareket etmez (güvenli) |
| 2. Tactical | Engel etrafından plan | Düz gider → katman 3-4 yakalar |
| 3. Safety | Hazard'da yavaşla/dur | Hızlı engele dalar → katman 4 yakalar |
| 4. Reflex | Lidar'da fiziksel dur | E-Stop fiziksel devre kesicisi (donanım) |
| 5. HW E-Stop | GPIO motor kesme | Robot durur (mekanik tampon kaplama) |

**Garanti:** Katmanların **TÜMÜ** aynı anda çökmediği sürece robot
çarpmaz. **Bağımsızlık ilkesi:** her katman farklı bir hata modeline
açıktır (yazılım bug, ROS lifecycle, lidar arıza, motor kontrolör
arıza), bir katmanın tetikleyicisi diğerini etkilemez.

---

## Karar: Neden Reactive Avoider Birincil?

Faz 0-3'te 65 trial koşumla doğrulandı:

| Yaklaşım | Garanti seviyesi | Tezdeki rol |
|---|---|---|
| Nav2 DWB | %0-20 (timing/configa bağlı) | Araştırma karşılaştırması |
| Nav2 MPPI | %0 ama tutarlı yan sapma | Araştırma karşılaştırması |
| Nav2 SmacPlanner | %0 | Araştırma karşılaştırması |
| **Reactive Avoider** | **%100** (deterministik state machine) | **Birincil sürüş modu** |

### Mühendislik gerekçesi
Reactive avoider:
- **Saf-Python state machine** — formal doğrulanabilir
- **17/17 birim test** geçiyor (test_avoider_logic.py)
- **Pi + WSL'de fiziksel test edildi** (CLAUDE.md kanıt)
- Lidar ham datasından bağımsız çalışır (SLAM/costmap timing bağımlı değil)
- Yorumlanabilir — saha hatalarında log'dan tam neden anlaşılır

### Akademik gerekçe
Yüksek lisans seviyesi otonom araç bilimi:
- "Reactive (Brooks, 1986) + Deliberative (Nav2)" hibrit klasik mimari
- Reactive katman REAL-TIME garantisi sağlar (subsumption arch.)
- Deliberative katman global optimizasyon sağlar (Nav2 var)
- Bu hibrit endüstri standartı (Tesla AP, Waymo, DARPA Urban Challenge takımları)

Tezdeki katkı: **bu hibrit'in olçulmuş gerçek-robot doğrulaması**,
65-trial ablation, hata modeli analizi.

---

## Avoider Mode — Detay (Katman 2)

### State Machine

```
   ┌────────────┐
   │  DRIVING   │◄──────────────────────────────────┐
   └─────┬──────┘                                   │
         │ engel(min_r < 0.8 m)                     │ hedef mesafe ≥ 2 m
         ▼                                          │
   ┌────────────┐                                   │
   │  AVOIDING  │  yerinde döner (yön: bos olan)    │
   └─────┬──────┘                                   │
         │ ön sektör temizlendi                     │ heading ≈ ev_yaw
         ▼                                          │
   ┌────────────┐                                   │
   │  PASSING   │  düz ilerle, x clear sayar        │
   └─────┬──────┘                                   │
         │ engel-mesafesi ≥ pass_clear_distance     │
         ▼                                          │
   ┌────────────┐                                   │
   │ REALIGNING │  ev yönüne dön                    │
   └─────┬──────┘                                   │
         │                                           │
         └───────────────────────────────────────────┘

   ┌────────────┐
   │   DONE     │  hedef mesafe ulaşıldı, dur
   └────────────┘

  Her durumda: lidar ön sektörde engel görünürse → AVOIDING'e geç.
  Defansif tasarım: tüm geçişler engel görünce AVOIDING'e geri çağrılabilir.
```

### State semantikleri

| State | Komut çıkışı | Çıkış şartı |
|---|---|---|
| DRIVING | linear=0.20 m/s, angular=0 | engel < 0.80 m → AVOIDING |
| AVOIDING | linear=0, angular=±0.5 rad/s (yön) | ön sektör temiz → PASSING |
| PASSING | linear=0.20, angular=0 | engel-yan-mesafesi ≥ 0.4 m → REALIGNING |
| REALIGNING | linear=0, angular=±0.5 (ev yönü) | yaw_err < 0.10 rad → DRIVING |
| DONE | linear=0, angular=0 | (terminal) |

**Defansiflik:** Her tikte AVOIDING geçişi öncelikli. Eğer
PASSING/REALIGNING sırasında ön sektörde başka engel görünürse anında
AVOIDING'e dönülür.

### Niye AVOIDING yerinde döner (yan değil)?

Skid-steer + diff drive kinematik: yerinde dönüş = 0 yarıçap, mekanik
güvenli. Yan hareket diff-drive'da imkansız. Önceki "AVOIDING'de yan
git" yaklaşımı robotu engele itiyordu — şimdi yerinde dön.

### Niye PASSING ayrı state?

Önceki avoider AVOIDING'den direkt REALIGNING'e geçiyordu, ama
REALIGNING'de ev yönüne dönerken robot engele tekrar yaklaşıyordu
(engelin yanındaydı, ev yönü = engele doğru). PASSING önce robotu
engelin **ötesine** taşır, sonra REALIGNING güvenli olur.

---

## Sim Doğrulama Hedefleri (Tezdeki Sonuç Tablosu)

| Senaryo | Beklenen PASS | Çarpma | Gerekçe |
|---|---|---|---|
| `debug_world.sdf` (1 engel) | ≥ %90 | 0 | Tek engel, basit |
| `test_world.sdf` (6 engel parkur) | ≥ %60 | 0 | Slalom + L-koridor + yaya |
| Pi'de gerçek 1 engel | ≥ %80 (5 koşumda 4) | 0 | Saha smoke test |
| Pi'de gerçek slalom (3 engel) | ≥ %50 | 0 | İlk saha denemesi |

**Çarpma sıfır** mutlak gereksinim — eğer bir koşumda olursa katman
3-4 hatası → ayrı bug report.

---

## Tezdeki Bölüm Eşleştirmesi

| Tez bölümü | Bu mimari'nin yeri |
|---|---|
| Bölüm 3 — Sistem Mimarisi | Yukarıdaki 4-katman diyagram + RACI |
| Bölüm 4 — Algoritma | Avoider state machine + Nav2 ablation |
| Bölüm 5 — Metodoloji | 65-trial sim doğrulama protokolü |
| Bölüm 6 — Sim Sonuçlar | DWB/MPPI/Smac karşılaştırma tabloları |
| Bölüm 7 — Saha Sonuçlar | Pi smoke test sonuçları |
| Bölüm 8 — Tartışma | Hibrit yaklaşımın gerekçesi, "negatif sonuçlar" |

---

## Tasarım Kararı: Reactive vs Deliberative

**Trade-off matrisi:**

| Kriter | Reactive (avoider) | Deliberative (Nav2) |
|---|---|---|
| Reaksiyon süresi | < 100 ms (deterministik) | 200-2000 ms (planner+BT) |
| Global optimallik | Yok (greedy) | Var (A*) |
| Uzak goal | Erişemez (rastgele wander) | Erişir (waypoint) |
| Karmaşık ortam | Sıkışabilir (yerel min) | Recovery + replanning |
| Real-time garanti | EVET | KOŞULA BAĞLI |
| Yazılım karmaşıklığı | Düşük (~200 satır Python) | Yüksek (Nav2 50K+ satır C++) |
| Debug | Kolay (state log) | Zor (lifecycle + BT + plugins) |
| Tezdeki katkı | Çalışan demo | Karşılaştırma + literatür |

**Karar:** Her ikisini de kullan, ama **birincil sürüş** reactive olsun
(garanti çalışır). Nav2 araştırma + karşılaştırma için kalsın.

Bu **defense-in-depth** ile birleşince: Nav2 yerelde başarısız olursa
bile reactive katman robotu engelden geçirir; reactive yerelde
sıkışırsa collision_monitor durdurmaya garanti.

---

## Sonraki Adımlar (Bu oturumda yapılacak)

1. ✅ Bu mimari dokümanı (`docs/avoidance_architecture.md`)
2. ⏳ Avoider state machine genişletme (PASSING + REALIGNING) +
   `test_avoider_logic.py` 30+ test
3. ⏳ Sim doğrulama: debug_world N=10 + test_world N=5
4. ⏳ Pi saha test protokolü (`docs/saha_test_protokolu.md`)
5. ⏳ TEZ.md sistem mimarisi bölüm taslağı
6. ⏳ Final commit + ILERLEME.md güncelle
