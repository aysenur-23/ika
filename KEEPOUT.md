# İKA — Keepout Zone Kullanımı

Keepout zone, Nav2'nin **yasak alanlar** (mutfak, merdiven, çocuk odası, vb.)
olarak işaretlenen bölgelerden geçmesini engelleyen costmap katmanıdır.

Kod tarafı hazır ([`nav2_params.yaml`](ika_ws/src/ika_navigation/config/nav2_params.yaml)
içinde `keepout_filter` config'i var). Senin yapman gereken: bir **maske dosyası**
oluşturmak (yaml + pgm).

> Sim için zorunlu değil. İlk denemelerinde keepout'a ihtiyacın yok. Gerçek
> ortamda bilinen yasak alanlar olduğunda devreye al.

---

## 1. Maske Nasıl Oluşturulur?

### Yöntem A — SLAM haritasından türetme (önerilen)

1. Önce normal bir SLAM haritası oluştur (DENEMELER.md Deneme 5):
   ```bash
   ros2 launch ika_navigation slam.launch.py use_sim_time:=true
   # haritayı kaydet
   ros2 run nav2_map_server map_saver_cli -f ~/ika/ika_ws/src/ika_navigation/maps/test_map
   ```

2. `test_map.pgm` dosyasını GIMP veya benzeri bir resim editöründe aç.

3. Yasak alanları **SİYAH** (0) boya, izinli alanları **BEYAZ** (255), bilinmeyenleri **GRİ** (205).

4. Yeni isimle kaydet: `keepout_mask.pgm`.

5. `keepout_mask.yaml` oluştur (test_map.yaml'i kopyala, `image:` satırını güncelle):
   ```yaml
   image: keepout_mask.pgm
   resolution: 0.050000
   origin: [-10.000000, -10.000000, 0.000000]
   negate: 0
   occupied_thresh: 0.65
   free_thresh: 0.196
   mode: scale
   ```

### Yöntem B — Tam baştan oluştur

```bash
# Tum harita beyaz (izinli) - sonradan editle
convert -size 400x400 xc:white ~/ika/ika_ws/src/ika_navigation/maps/keepout_mask.pgm
```

Sonra `keepout_mask.yaml` yaz (yukarıdaki şablon).

---

## 2. Etkinleştirme

`navigation.launch.py` içinde yorum satırlarını aç:

```python
# Costmap filter info server
Node(
    package='nav2_map_server',
    executable='costmap_filter_info_server',
    name='costmap_filter_info_server',
    output='screen',
    parameters=[nav2_yaml, {'use_sim_time': use_sim_time}],
),
# Filter mask server
Node(
    package='nav2_map_server',
    executable='map_server',
    name='filter_mask_server',
    output='screen',
    parameters=[
        nav2_yaml,
        {'use_sim_time': use_sim_time,
         'yaml_filename': '/home/ubuntu/ika/ika_ws/src/ika_navigation/maps/keepout_mask.yaml'},
    ],
),
```

Ve `lifecycle_manager_navigation` node listesine bu ikisini ekle:
```python
nav2_lifecycle_nodes = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'collision_monitor',
    'costmap_filter_info_server',     # YENI
    'filter_mask_server',             # YENI
]
```

Build et:
```bash
cd ~/ika/ika_ws && colcon build --symlink-install
```

---

## 3. Doğrulama

```bash
ros2 topic echo /costmap_filter_info --once
ros2 topic echo /keepout_filter_mask --once
```

RViz'de `Map` display ekleyip `/keepout_filter_mask` topic'i seç — siyah/beyaz maske görünmeli.

Global costmap'i göster (`/global_costmap/costmap`); siyah bölgeler kırmızı (lethal) olmalı.

Nav2 hedef ver, planlanan yol siyah bölgelerden geçmemeli.

---

## 4. Sorun Giderme

- **`Lifecycle node costmap_filter_info_server failed to activate`**:
  Maske dosyası yolu yanlış. `yaml_filename:` mutlak yol olmalı.

- **`Map message did not contain valid map data`**:
  `.pgm` dosyası corrupt. Resmi yeniden export et (gri tonlamalı / 8-bit PGM).

- **Keepout görünüyor ama Nav2 geçiyor**:
  `keepout_filter` plugin'i global_costmap plugins listesine eklenmiş mi? `nav2_params.yaml`'da:
  ```yaml
  plugins: ["static_layer", "obstacle_layer", "terrain_layer", "inflation_layer"]
  filters: ["keepout_filter"]      # AYRI ALAN - plugins ile karistirma
  ```

- **`/costmap_filter_info` 0 Hz**:
  `costmap_filter_info_server` ayağa kalkmamış. Lifecycle manager içinde mi:
  ```bash
  ros2 lifecycle get /costmap_filter_info_server
  ```

---

## 5. Daha Fazla

Resmi Nav2 dokümanı: https://docs.nav2.org/tutorials/docs/navigation2_with_keepout_filter.html
