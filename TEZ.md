# TEZ İÇERİĞİ — İKA Projesi

> Bu doküman tez yazımı sırasında kullanabileceğin **içerik şablonu, teknik
> dayanaklar, paragraf taslakları ve kaynak önerileridir.** Birebir kopyalama yerine
> kendi cümlelerinle uyarla; ben kavramsal iskelet ve teknik dolguyu veriyorum.

---

## Önerilen Tez Başlıkları

İhtiyacına göre birini seç veya uyarla:

1. **"ROS 2 Tabanlı Skid-Steer Otonom Kara Aracı için Encoder'sız Lokalizasyon ve Çok Katmanlı Güvenlik Mimarisi Tasarımı ve Simülasyonu"**
2. **"Düşük Maliyetli Robotik Platformlarda Lidar Tabanlı Odometri ve Depth Kamera ile Terrain Algılamanın Entegrasyonu: İKA Otonom Araç Vaka Çalışması"**
3. **"Raspberry Pi 5 Üzerinde ROS 2 Jazzy ile Gerçek Zamanlı Otonom Navigasyon: Tasarım, Simülasyon ve Saha Doğrulaması"**

Akademik formalite için kısa varyant:
- **"ROS 2 Jazzy Tabanlı Otonom Kara Aracı Geliştirme ve Doğrulama"**

---

## ÖZET (Türkçe)

### Şablon

Bu çalışmada, **dört tekerlekli skid-steer kinematiğe** sahip otonom bir kara
aracının (İKA) tasarımı, yazılım mimarisi ve simülasyon tabanlı doğrulaması
sunulmaktadır. Sistem; Raspberry Pi 5 (16 GB) üzerinde çalışan **ROS 2 Jazzy**
yazılım çerçevesini kullanmakta, **Gazebo Harmonic** simülasyon ortamında
test edilmektedir. Donanım katmanı RPLIDAR C1 (2B lidar), Luxonis OAK-D Lite
(stereo + derinlik kamera), atalet ölçüm birimi (IMU) ve GPS modülünden
oluşmaktadır. Düşük seviye motor kontrolü USB seri bağlantı üzerinden Arduino
Uno mikrodenetleyici ile gerçekleştirilmektedir.

Çalışmanın özgün katkıları şunlardır: (i) **encoder kullanılmadan**
`rf2o_laser_odometry` ile lidar tabanlı odometri ve `robot_localization`
genişletilmiş Kalman filtresi (EKF) ile IMU entegrasyonu; (ii) derinlik
kamerasından **RANSAC tabanlı zemin düzlemi tahmini** kullanan, rampa ve
çukur sınıflandırması yapabilen bağımsız bir terrain perception düğümü;
(iii) sensör zaman aşımı izleme ve katmanlı güvenlik mimarisini yöneten
bir **Safety Supervisor** lifecycle düğümü; (iv) Nav2, SLAM Toolbox ve
özel düğümlerin entegrasyonu ile **uçtan uca otonom navigasyon** çerçevesi.

Tüm sistem, 12 farklı senaryo üzerinden Gazebo Harmonic'te doğrulanmıştır:
düz zemin üzerinde manuel sürüş, lidar tabanlı haritalama, statik engelden
kaçınma, %10 ve %30 eğimli rampa sınıflandırması, çukur kenarında otomatik
duruş, sensör kaybı durumunda E-Stop tetikleme ve GPS waypoint takibi. **22
birim test** ile RANSAC zemin uydurma, terrain sınıflandırma ve güvenlik
karar mantığının doğruluğu garanti altına alınmıştır.

**Anahtar Kelimeler:** ROS 2, Otonom Kara Aracı, SLAM, Nav2, Lidar Odometri,
RANSAC, Terrain Perception, Skid-Steer Kinematik, Gazebo Harmonic, Raspberry Pi 5

---

## ABSTRACT (English)

### Template

This thesis presents the design, software architecture, and simulation-based
verification of an autonomous **four-wheel skid-steer ground vehicle** named
İKA. The system runs **ROS 2 Jazzy** on a Raspberry Pi 5 (16 GB) and is
validated in the **Gazebo Harmonic** simulator. The hardware stack comprises
an RPLIDAR C1 2D lidar, a Luxonis OAK-D Lite stereo/depth camera, an inertial
measurement unit (IMU), and a GPS receiver. Low-level motor control is
implemented over USB serial through an Arduino Uno microcontroller.

The original contributions of this work are: (i) **wheel-encoder-free**
lidar-based odometry via `rf2o_laser_odometry`, fused with IMU through the
`robot_localization` extended Kalman filter (EKF); (ii) an independent
terrain perception lifecycle node that classifies ramps and detects drop-offs
using **RANSAC-based ground plane estimation** on depth camera point clouds;
(iii) a **Safety Supervisor** lifecycle node that orchestrates sensor
timeout monitoring and a layered safety architecture; (iv) an end-to-end
autonomous navigation pipeline integrating Nav2, SLAM Toolbox, and custom
nodes.

The complete system was verified in Gazebo Harmonic across 12 distinct
scenarios: manual driving on flat terrain, lidar-based SLAM mapping, static
obstacle avoidance, ramp classification at 10% and 30% grades, drop-off
edge detection with automatic stop, E-Stop triggering under sensor loss,
and GPS waypoint following. **22 unit tests** ensure correctness of the
RANSAC ground fitting, terrain classification, and safety decision logic.

**Keywords:** ROS 2, Autonomous Ground Vehicle, SLAM, Nav2, Lidar Odometry,
RANSAC, Terrain Perception, Skid-Steer Kinematics, Gazebo Harmonic, Raspberry Pi 5

---

## ŞEKİL VE TABLO ÖNERİLERİ

Tez içinde yer alması gereken görseller:

| # | Şekil | İçerik | Kaynak |
|---|---|---|---|
| Ş.1 | İKA aracı genel görünüm | Donanım fotoğrafı veya CAD rendering | Senin çekimin |
| Ş.2 | Donanım blok şeması | Pi5 ↔ Arduino ↔ Motorlar + sensörler | README §3 |
| Ş.3 | Yazılım katmanları | Nav2 → CM → SS → BC zinciri | README §3.1 |
| Ş.4 | ROS 2 topic akış grafiği | rqt_graph çıktısı | Pi'de üret |
| Ş.5 | TF ağacı | view_frames PDF | Pi'de üret |
| Ş.6 | URDF görselleştirme | RViz'de robot modeli + frame'ler | RViz screenshot |
| Ş.7 | Gazebo test dünyası | Rampa, engeller, çukur, dar geçit | Gazebo screenshot |
| Ş.8 | SLAM tabanlı oluşturulan harita | RViz'de /map + path | Test sonrası |
| Ş.9 | Costmap görselleştirmesi | Global + local + terrain layer | RViz screenshot |
| Ş.10 | RANSAC zemin uydurma | Algoritmanın görsel açıklaması | Çiziminle |
| Ş.11 | Terrain sınıflandırma durumları | SAFE/CAUTION/IMPASSABLE/DROPOFF örnekleri | Sim sırasında |
| Ş.12 | Safety zinciri durum diyagramı | E-Stop akışı | Çiziminle |
| Ş.13 | Skid-steer kinematik | sol/sağ tekerlek formülü | README §2 |
| Ş.14 | Mission durum makinesi | IDLE → RUNNING → PAUSED → DONE | Çiziminle |
| Ş.15 | Test sonuçları | Her senaryo için başarı tablosu | DENEMELER.md |

| # | Tablo | İçerik |
|---|---|---|
| T.1 | Donanım listesi ve maliyetleri | İKA bileşenleri + fiyat aralığı |
| T.2 | ROS 2 paket yapısı | 8 paket + açıklama |
| T.3 | Topic listesi | Yayımlayıcı/dinleyici eşleştirme |
| T.4 | Test senaryoları matrisi | Senaryo × beklenen × ölçülen |
| T.5 | Kalibrasyon parametreleri | Parametre, değer, yöntem |
| T.6 | Kıyaslama: encoder'lı vs encoder'sız | Hız, hata, kısıt |
| T.7 | Karşılaştırılan benzer çalışmalar | Mevcut literatür özeti |

---

## BÖLÜM 1 — GİRİŞ

### 1.1 Problem Tanımı

**Yazılması gereken:**

Otonom kara araçları (Unmanned Ground Vehicle — UGV), savunma sanayisinden tarımsal
robotikten lojistiğe ve yardım operasyonlarına kadar geniş bir uygulama alanına
sahiptir. Bununla birlikte, akademik ve hobi düzeyinde UGV geliştirmek isteyen
araştırmacılar, üç temel zorlukla karşılaşır: (1) ticari sistemlerin yüksek
maliyeti, (2) yazılım ve donanım entegrasyonunun karmaşıklığı, (3) güvenli
test ortamlarının eksikliği.

Bu çalışma, **düşük maliyetli (sub-$300 donanım)** bileşenlerle, açık kaynak
yazılım yığını üzerinde tam fonksiyonel bir otonom navigasyon platformu
geliştirmeyi hedeflemektedir. Sistemin tüm davranışları, **fiziksel risk almadan
önce simülasyonda doğrulanır**; bu, akademik ortamda yapılan deneylerin sistemli
olarak tekrarlanabilir olmasını sağlar.

### 1.2 Motivasyon

- ROS 2 Jazzy son nesil ROS dağıtımıdır ve Ubuntu 24.04 ile uzun süreli destek (LTS) ekosistemine sahiptir.
- Gazebo Harmonic, klasik Gazebo'nun halefi olarak donanım hızlandırmalı, modüler bir simülatör sunar.
- Raspberry Pi 5 (16 GB), 2024 itibariyle robotik için yeterli işlem kapasitesine sahiptir (4-core Cortex-A76, VideoCore VII GPU).
- Teker encoder'larının olmaması, mekanik basitlik sağlar ama lokalizasyon açısından **lidar odometri + IMU füzyonu** gerektirir — bu çalışma bu tasarım kararının fizibilitesini gösterir.

### 1.3 Tezin Hedefleri

1. ROS 2 Jazzy üzerinde, modüler ve YAML tabanlı parametrik bir otonom araç yazılım yığını geliştirmek.
2. Encoder kullanmadan, **`rf2o_laser_odometry` + IMU EKF füzyonu** ile lokalizasyon doğruluğunu değerlendirmek.
3. Derinlik kamerasından **RANSAC tabanlı zemin düzlemi tahmini** ile rampa sınıflandırması ve çukur algılama gerçekleştirmek.
4. Sensör kaybı, terrain riski ve Nav2 kararlarını birleştiren **çok katmanlı güvenlik mimarisi** kurmak.
5. Tüm sistemi Gazebo Harmonic simülasyonu içinde 12 senaryo ile doğrulamak.
6. Geliştirme sürecini ve sonuçları tezde belgelemek.

### 1.4 Tezin Katkıları

| # | Katkı |
|---|---|
| K1 | Encoder kullanmayan UGV için açık kaynak referans implementasyon |
| K2 | RANSAC tabanlı bağımsız terrain perception lifecycle düğümü |
| K3 | Çok katmanlı güvenlik mimarisinin formal tasarımı ve doğrulanması |
| K4 | 12 test senaryosu içeren tekrarlanabilir doğrulama paketi |
| K5 | Tek komutla kurulum scripti (`install_pi.sh`) ile reprodüksiyon kolaylığı |

### 1.5 Tezin Organizasyonu

Bölüm 2'de ilgili literatür özetlenir. Bölüm 3 donanım ve yazılım mimarisini
detaylandırır. Bölüm 4 yazılım modüllerinin geliştirme sürecini, Bölüm 5 ise
simülasyon kurulumu ve test senaryolarını anlatır. Bölüm 6 bulguları sunar,
Bölüm 7 sonuçları değerlendirir ve gelecek çalışmaları önerir.

---

## BÖLÜM 2 — LİTERATÜR TARAMASI

### 2.1 Otonom Kara Araçları (UGV)

**Yazılması gereken konular:**

- Tanım: insansız, kendi karar verebilen kara aracı.
- Tarihçe: DARPA Grand Challenge (2004-2007), Stanley, Carnegie Mellon Boss.
- Modern uygulamalar: Otonom tarım robotları (John Deere), depo robotları (Amazon Kiva, Locus Robotics), askeri keşif (Rheinmetall Mission Master).
- Düşük maliyetli UGV platformları: TurtleBot, Clearpath Husky/Jackal, Linorobot.

**Kaynak önerileri (BibTeX formatında):**

```bibtex
@article{thrun2006stanley,
  title={Stanley: The robot that won the DARPA Grand Challenge},
  author={Thrun, Sebastian and Montemerlo, Mike and Dahlkamp, Hendrik and others},
  journal={Journal of field Robotics},
  volume={23}, number={9}, pages={661--692}, year={2006}
}

@article{urmson2008autonomous,
  title={Autonomous driving in urban environments: Boss and the urban challenge},
  author={Urmson, Chris and others},
  journal={Journal of Field Robotics}, volume={25}, number={8}, year={2008}
}
```

### 2.2 ROS ve ROS 2

**Yazılması gereken:**

- ROS 1 (2007) → ROS 2 (2017) — DDS (Data Distribution Service) tabanlı, gerçek zamanlı, çok platformlu.
- ROS 2 Jazzy Jalisco (2024) — Ubuntu 24.04 LTS destekli son sürüm.
- Bileşenler: ros2cli, rclcpp/rclpy, lifecycle node'lar, parameter server, DDS middleware.
- Avantajlar: dağıtık mimari, network transparency, asenkron iletişim.

**Kaynaklar:**
```bibtex
@article{quigley2009ros,
  title={ROS: an open-source Robot Operating System},
  author={Quigley, Morgan and Conley, Ken and Gerkey, Brian and others},
  journal={ICRA workshop on open source software}, year={2009}
}

@inproceedings{macenski2022robot,
  title={Robot Operating System 2: Design, architecture, and uses in the wild},
  author={Macenski, Steven and Foote, Tully and Gerkey, Brian and Lalancette, Chris and Woodall, William},
  journal={Science Robotics}, volume={7}, number={66}, year={2022}
}
```

### 2.3 SLAM (Simultaneous Localization and Mapping)

**Yazılması gereken:**

- SLAM tanımı: robotun bilinmeyen ortamda haritasını oluştururken kendi konumunu da belirleme problemi.
- İki ana yaklaşım: filter-based (EKF SLAM, particle filter) vs graph-based (g2o, GTSAM).
- 2D SLAM: GMapping, Karto, **slam_toolbox** (graph-based, ROS 2 standardı).
- 3D SLAM: ORB-SLAM3, LIO-SAM, LeGO-LOAM.
- Loop closure'ın önemi (özellikle encoder yokken).

**Kaynaklar:**
```bibtex
@article{macenski2021slam,
  title={SLAM Toolbox: SLAM for the dynamic world},
  author={Macenski, Steven and Jambrecic, Ivona},
  journal={Journal of Open Source Software}, volume={6}, number={61}, year={2021}
}

@inproceedings{grisetti2007improved,
  title={Improved techniques for grid mapping with Rao-Blackwellized particle filters},
  author={Grisetti, Giorgio and Stachniss, Cyrill and Burgard, Wolfram},
  journal={IEEE Trans. on Robotics}, year={2007}
}
```

### 2.4 Navigasyon (Nav2)

**Yazılması gereken:**

- ROS 2'nin resmi navigasyon yığını: nav2_bringup.
- Bileşenler: BT Navigator, Planner Server (Dijkstra/A*), Controller Server (DWB, RPP), Behavior Server, Collision Monitor, Costmap 2D.
- Behavior Tree mimarisi — XML tabanlı, modüler karar verme.
- Costmap katmanları: Static, Obstacle, Voxel, Inflation, Keepout Filter.

**Kaynaklar:**
```bibtex
@inproceedings{macenski2020marathon,
  title={The Marathon 2: A navigation system},
  author={Macenski, Steven and Martin, Francisco and White, Ruffin and Clavero, Jonatan Gines},
  booktitle={IROS}, year={2020}
}
```

### 2.5 Lidar Tabanlı Odometri (Encoder'sız)

**Yazılması gereken:**

- Klasik tekerlek odometri kaymaya ve yüke bağlı hatalara açık.
- Lidar odometri yaklaşımları: ICP (Iterative Closest Point), NDT (Normal Distributions Transform), RF2O (Range Flow-based 2D Odometry).
- **RF2O** özetle: ardışık taramaların range flow analizini kullanır, 2D düzlemde diferansiyel hareket tahmini yapar.
- Sınırlamalar: dinamik ortamda hatalı, geometrik özelliği zayıf (örn. uzun koridor) ortamda kayar.

**Kaynaklar:**
```bibtex
@inproceedings{jaimez2016planar,
  title={Planar odometry from a radial laser scanner. A range flow-based approach},
  author={Jaimez, Mariano and Monroy, Javier G and Gonzalez-Jimenez, Javier},
  booktitle={ICRA}, year={2016}
}
```

### 2.6 IMU Füzyonu — robot_localization EKF

**Yazılması gereken:**

- Kalman filtresinin temelleri (Wiener-Kalman 1960).
- Genişletilmiş Kalman filtresi (EKF) — doğrusalsızlıkta lineerizasyon.
- `robot_localization` paketi — esnek konfigürasyonlu ROS 2 EKF/UKF.
- Kovariyans matrisi ayarlamanın kritikliği — özellikle encoder olmadığında lidar odom'a güveni artırma.

**Kaynaklar:**
```bibtex
@inproceedings{moore2016generalized,
  title={A generalized extended Kalman filter implementation for the Robot Operating System},
  author={Moore, Tom and Stouch, Daniel},
  booktitle={Intelligent autonomous systems 13}, year={2016}
}
```

### 2.7 Derinlik Kamerası ile Terrain Algılama

**Yazılması gereken:**

- RGB-D / stereo kamera çıktısı: depth image + point cloud.
- Yerleşik (embedded) zemin analiz yöntemleri: hızlı zemin segmentasyonu, ray-based ground detection.
- **RANSAC (RANdom SAmple Consensus)** — outlier varlığında model uydurma için temel algoritma.
- Zemin düzlemi tahmini ile rampa eğimi, basamak yüksekliği, çukur tespiti yapılabilir.

**Kaynaklar:**
```bibtex
@article{fischler1981random,
  title={Random sample consensus: a paradigm for model fitting with applications to image analysis and automated cartography},
  author={Fischler, Martin A and Bolles, Robert C},
  journal={Communications of the ACM}, volume={24}, number={6}, year={1981}
}

@inproceedings{rusu2008towards,
  title={Towards 3D point cloud based object maps for household environments},
  author={Rusu, Radu Bogdan and others},
  booktitle={Robotics and Autonomous Systems}, year={2008}
}
```

### 2.8 Skid-Steer Kinematik

**Yazılması gereken:**

- Differential drive ile benzer ama 4-teker karaketrli, sürtünme tabanlı dönüş.
- Cinematic vs dynamic models. Skid-steer'da dönüş açısı sürtünme dağılımına bağlı.
- Yazılımda differential drive olarak modellenir: sol/sağ teker grubu ortak hız.

**Kaynaklar:**
```bibtex
@article{mandow2007experimental,
  title={Experimental kinematics for wheeled skid-steer mobile robots},
  author={Mandow, Anthony and others},
  booktitle={IROS}, year={2007}
}
```

### 2.9 Gazebo Harmonic ve Simülasyon Tabanlı Doğrulama

**Yazılması gereken:**

- Gazebo Classic → Ignition Gazebo → Gazebo (sürümler Garden, Harmonic).
- Donanım hızlandırmalı render, modüler plugin mimarisi.
- ros_gz_bridge ile ROS 2 ↔ Gazebo köprüsü.
- Simülasyon tabanlı geliştirme (Sim2Real) yöntemleri ve sınırları.

**Kaynaklar:**
```bibtex
@inproceedings{koenig2004design,
  title={Design and use paradigms for Gazebo, an open-source multi-robot simulator},
  author={Koenig, Nathan and Howard, Andrew},
  booktitle={IROS}, year={2004}
}
```

---

## BÖLÜM 3 — SİSTEM MİMARİSİ

### 3.1 Genel Yaklaşım

**Yazılması gereken:**

İKA, **katmanlı, modüler ve parametrik** bir mimari prensibi üzerine
tasarlanmıştır. Mimari karar olarak:

1. **Her sensör tipi için ayrı sürücü düğümü** — sensörler bağımsız geliştirilebilir.
2. **Lifecycle node'lar** — terrain, safety, base_controller gibi kritik düğümler `unconfigured → inactive → active` lifecycle akışını izler. Konfigürasyon hatalarında temiz çıkış.
3. **YAML parametre dosyaları** — kodda hiçbir araç boyutu, hız sınırı, eşik hard-coded değil.
4. **ROS 2 lifecycle_manager** — Nav2 ve İKA özel düğümleri için ayrı yöneticiler. Otomatik başlatma.

### 3.2 Donanım Mimarisi

**İçerik:**

- (Şekil Ş.2'yi referansla)
- Tablo T.1 — donanım listesi
- Pi 5: ana işlem birimi, ROS 2 + Gazebo (geliştirme aşamasında) burada koşar.
- Arduino Uno: düşük seviye motor kontrolü, watchdog.
- USB Serial üzerinden JSON protokolü tercih edildi (micro-ROS yerine) — debug kolaylığı için.
- E-Stop fiziksel anahtarı motor sürücülerin güç hattını doğrudan keser; yazılıma bağımlı değildir.

### 3.3 Yazılım Mimarisi

**İçerik:**

- (Şekil Ş.3'ü referansla)
- 8 ROS 2 paketi (Tablo T.2):
  - `ika_description`, `ika_simulation`, `ika_navigation`, `ika_bringup` — ament_cmake (config + launch + URDF)
  - `ika_terrain`, `ika_safety`, `ika_base_controller`, `ika_mission` — ament_python (düğüm kodu)

### 3.4 İletişim Topic Mimarisi

**İçerik:**

- Tablo T.3 — kritik topic listesi (kim yayımlar, kim dinler).
- Topic akışı sıralı: sensör → preprocessing → Nav2 → güvenlik → motor.
- QoS (Quality of Service) tercihleri: lidar best_effort, /map transient_local.

### 3.5 Güvenlik Mimarisi

**İçerik:**

- 5 katmanlı güvenlik zinciri (Şekil Ş.12).
- Her katman bağımsız çalışabilmeli (failover).
- Watchdog mekanizması: sensör topic'i `T` saniyeden uzun gelmezse E-Stop.

### 3.6 TF (Transform) Ağacı

**İçerik:**

- (Şekil Ş.5)
- map → odom → base_link — standart ROS 2 navigasyon TF ağacı.
- Statik TF'ler URDF/Xacro üzerinden.
- Dinamik odom → base_link EKF tarafından.

---

## BÖLÜM 4 — YAZILIM GELİŞTİRME

### 4.1 ROS 2 Workspace Yapısı

**İçerik:**

- ament_python vs ament_cmake seçimi.
- src/ altında 8 paket organizasyonu.
- Üçüncü taraf bağımlılıklar (sllidar, rf2o) için `third_party/` alt dizini.

### 4.2 URDF/Xacro Modeli

**İçerik:**

- Xacro makroları ile parametrik tanım.
- 4 teker + base_link + 4 sensör frame.
- Gazebo plugin entegrasyonu (`ika_gazebo.xacro`).
- Diff-drive plugin yapılandırması, joint state publisher.

**Önemli kod parçacığı (referans için):**
```xml
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
  <left_joint>front_left_wheel_joint</left_joint>
  <left_joint>back_left_wheel_joint</left_joint>
  <right_joint>front_right_wheel_joint</right_joint>
  <right_joint>back_right_wheel_joint</right_joint>
  <wheel_separation>0.34</wheel_separation>
  <wheel_radius>0.05</wheel_radius>
  ...
</plugin>
```

### 4.3 Düşük Seviye Kontrol — Base Controller + Arduino

**İçerik:**

#### 4.3.1 Pi Tarafı — base_controller_node.py
- `/cmd_vel_safe` subscriber → skid-steer dönüşümü → JSON seri yayım.
- Watchdog: cmd_vel 0.5s'den uzun gelmezse motorlar durur.
- `/e_stop` boolean subscriber, geldiğinde anında sıfırlanır.

#### 4.3.2 Arduino Tarafı — ika_motor_controller.ino
- ArduinoJson kütüphanesi ile satır tabanlı JSON parse.
- L298N motor sürücü PWM kontrolü.
- Ölü bant (`MIN_PWM`) tazminatı: motorun hareket etmediği eşik altında PWM uygulanmaz.
- Bağımsız Arduino watchdog: 500ms USB sessizliğinde motorlar durur.

### 4.4 Odometri — Encoder Yok / Lidar Odometri

**İçerik:**

#### 4.4.1 rf2o_laser_odometry
- `/scan` LaserScan'i dinler, ardışık taramalardan range flow tabanlı 2D hız tahmini yapar.
- `/odom` topic'ine `nav_msgs/Odometry` yayımlar.
- 10 Hz çalışma frekansı (RPLIDAR C1'in scan rate'i).

#### 4.4.2 robot_localization EKF
- İki girdi: `/odom` (rf2o) + `/imu/data`.
- 15 boyutlu durum vektörü: pozisyon (3), oryantasyon (3), lineer hız (3), açısal hız (3), lineer ivme (3).
- Encoder yokken `odom0_config`: x, y, yaw + bunların türevleri.
- IMU'dan: roll, pitch, yaw + açısal hızlar + ivmeler.
- Kovariyans matrisi ayarı: lidar odom hatasını yansıtacak şekilde process_noise yüksek.

#### 4.4.3 navsat_transform_node
- GPS'ten gelen `/gps/fix` (NavSatFix) → UTM koordinatlarına dönüşüm → map frame.
- Statik manyetik sapma açısı (magnetic_declination_radians).

### 4.5 SLAM — slam_toolbox

**İçerik:**

- Graph-based SLAM, optimize edilmiş loop closure.
- İki mod:
  - **mapping**: yeni harita oluşturma. async_slam_toolbox_node.
  - **localization**: mevcut harita üstünde sadece konum belirleme.
- Encoder yokken loop closure parametreleri agresifleştirildi.
- `minimum_travel_distance: 0.3 m` — her 30 cm'de bir scan match.

### 4.6 Nav2 Yapılandırması

**İçerik:**

#### 4.6.1 Planner Server (Dijkstra)
- `nav2_navfn_planner::NavfnPlanner`, `use_astar: false`.
- Tolerans 0.5 m.

#### 4.6.2 Controller Server — DWB Local Planner
- 15 hız örneklemi, 20 açısal örneklem.
- Kritikler: PathAlign, PathDist, GoalAlign, GoalDist, RotateToGoal, Oscillation, BaseObstacle.
- max_vel_x: 0.25 m/s (encoder yok kısıtı).

#### 4.6.3 Collision Monitor
- Yakın engellerde `StopZone` polygon (yarıçap 0.25 m) — anında durdurur.
- Orta mesafede `FootprintApproach` polygon — hız yavaşlatır.
- 2 kaynaklı: lidar `/scan` + depth `/oak/points`.

#### 4.6.4 Costmap 2D
- Global costmap: static + obstacle + terrain + inflation.
- Local costmap: voxel + terrain + inflation (4×4 m rolling window).
- `terrain_layer` özel olarak `/terrain_obstacles` topic'inden static layer olarak okur — bu projenin özgün entegrasyonu.

### 4.7 Terrain Perception Düğümü (Özgün Katkı)

**İçerik:**

#### 4.7.1 Algoritmik Akış

```
Depth Image (16UC1 / 32FC1)
       │
       ▼
   intrinsics ile 3D point cloud (optical frame)
       │
       ▼
   Statik kamera pose ile base_link frame'e dönüştür
       │
       ▼
   Yakın bölgeden RANSAC ile zemin düzlemi
       │
       ▼
   Düzlem normalinden EĞİM hesabı
       │
       ▼
   Uzak bölge → yakın düzlemden DROPOFF kontrolü
       │
       ▼
   Düzlem üzeri pozitif sapma → MAX_STEP_HEIGHT
       │
       ▼
   Sınıflandırma: SAFE / CAUTION / IMPASSABLE / DROPOFF_DANGER / UNKNOWN
```

#### 4.7.2 RANSAC Detayı

```
60 iterasyon, 0.04 m tolerans
Her iterasyonda:
  - 3 rastgele nokta seç
  - Düzlem normalini cross-product ile hesapla
  - Tüm noktaların düzleme uzaklığını ölç
  - Tolerans içinde olanları inlier say
En çok inlier veren düzlem → kazanan
```

#### 4.7.3 Sınıflandırma Eşikleri (kalibrasyon gerekli)

| Sınıf | Koşul |
|---|---|
| DROPOFF_DANGER | İleri bölgede zemin > 0.15 m altta |
| IMPASSABLE | Eğim > 25° |
| CAUTION | Eğim 15° - 25° arası |
| SAFE | Eğim < 15° ve max_step < 4 cm |
| UNKNOWN | Inlier oranı < 0.6 (güven düşük) |

### 4.8 Safety Supervisor Düğümü (Özgün Katkı)

**İçerik:**

- Lifecycle node, `/cmd_vel_collision` → `/cmd_vel_safe` filtre.
- Sensör watchdog: lidar (1.0s), depth (1.5s), IMU (0.5s) zaman aşımları.
- Terrain class'ına göre eylem:
  - DROPOFF_DANGER, IMPASSABLE → DUR (0 cmd_vel)
  - CAUTION, UNKNOWN → YAVAŞLA (×0.3 hız)
  - SAFE → GEÇİR
- 20 Hz watchdog timer.

### 4.9 Görev Yöneticisi (Mission)

**İçerik:**

- YAML waypoint listesi → Nav2 NavigateToPose action client.
- Dış komut topic'i `/mission_cmd`: cancel, pause, resume, skip, restart.
- Durum makinesi: idle → running → paused → done.
- Retry mantığı: başarısız hedefler max_retries kez denenir.

---

## BÖLÜM 5 — SİMÜLASYON VE TEST

### 5.1 Gazebo Harmonic Yapılandırması

**İçerik:**

- World dosyası: `test_world.sdf`.
- Eklenmiş sistem plugin'leri: Physics, SceneBroadcaster, UserCommands, Contact, IMU, Sensors, NavSat.
- Spherical coordinates: İstanbul (41.015137°N, 28.979530°E) — GPS testleri için.

### 5.2 Test Dünyası Tasarımı

**İçerik:**

- (Şekil Ş.7)
- Düz zemin (40×40 m).
- 3 statik kutu engel (manuel sürüş + costmap testi).
- 2 rampa: %10 (5.7°, SAFE), %30 (~17°, CAUTION/IMPASSABLE eşiğinde).
- Mavi platform (yükseltilmiş) — kenar dropoff testi için.
- 2 duvar arasında 0.6 m dar geçit.

### 5.3 ROS-Gazebo Köprüsü (ros_gz_bridge)

**İçerik:**

- YAML config tabanlı topic eşleştirmesi.
- Yön: `GZ_TO_ROS` (sensörler), `ROS_TO_GZ` (cmd_vel).
- Önemli karar: Gazebo'nun odom çıkışı `/odom_truth` olarak köprülenir, gerçek `/odom` rf2o tarafından yayımlanır. Bu, rf2o doğruluğunun karşılaştırılabilmesini sağlar.

### 5.4 Birim Test Stratejisi

**İçerik:**

- pytest tabanlı, ROS bağımlılığı olmayan saf-Python testler.
- 22 test:
  - 11 test: RANSAC ground plane (sentetik nokta bulutlarında doğrulanma)
  - 6 test: Safety supervisor karar mantığı
  - 5 test: Skid-steer kinematik dönüşümü

### 5.5 Entegrasyon Test Senaryoları (12)

**İçerik:**

Tablo T.4'ü referansla. Her senaryo:
- Amaç
- Komut
- Beklenen sonuç
- Başarı kriteri
- Test süresi

(DENEMELER.md tüm detayları içerir.)

### 5.6 Doğrulama Otomasyonu

**İçerik:**

- `deploy_sim.sh`: build + sim launch + verify_sim.sh + log topla.
- `verify_sim.sh`: gz topic listesi vs ros topic listesi karşılaştırması, hz ölçümleri, TF ağacı kontrolü.
- `check_workspace.sh`: pre-commit lint + test koşturma.

---

## BÖLÜM 6 — BULGULAR VE TARTIŞMA

> Bu bölüm gerçek test verileri toplandıktan sonra doldurulacak. Aşağıdaki başlıklar
> ve beklentiler şablon olarak verilmiştir; gerçek sayılarla doldur.

### 6.1 Birim Test Sonuçları

**Beklenen:**
- 22/22 PASSED
- RANSAC zemin uydurma: %95+ doğruluk (synthetic test verisinde)
- Skid-steer kinematik: birebir doğru (analitik)

### 6.2 Simülasyon Senaryo Sonuçları

**Doldurulacak tablo (Tablo T.4 için veri):**

| # | Senaryo | Başarı (Y/H) | Ölçülen Metrik | Not |
|---|---|---|---|---|
| 0 | Build | Y | 0 hata | — |
| 1 | URDF görüntü | ? | TF ağacı düzgün mü? | — |
| 2 | Bare sim | ? | Topic Hz | — |
| 3 | Topic akışı | ? | Eksik topic sayısı | — |
| 4 | Manuel sürüş | ? | cmd_vel response time | — |
| 5 | SLAM | ? | Harita kapsama %, drift | — |
| 6 | Tam stack | ? | Tüm node aktif mi? | — |
| 7 | Navigasyon | ? | Hedefe ulaşma süresi | — |
| 8 | Terrain | ? | Sınıflandırma doğruluğu | — |
| 9 | Safety | ? | E-Stop tetikleme süresi | — |
| 10 | Mission | ? | Tamamlanan WP / toplam | — |
| 11 | Diagnostics | ? | Tüm node OK mi? | — |
| 12 | Keepout | ? | Plan engel kaçınıyor mu? | — |

### 6.3 Lidar Odometrinin Performansı

**Yazılması gereken (test sonrası):**

- Düz çizgide 10 m ileri-geri test: konum hatası X cm.
- Köşelerdeki drift miktarı.
- Loop closure'la ne kadar düzeldi.
- Encoder'lı bir referansla (veya Gazebo /odom_truth ile) karşılaştırma.

### 6.4 Terrain Perception Doğruluğu

**Yazılması gereken:**

- Gazebo rampa testi: 5.7° rampa → ölçülen eğim X (hedef 5.7°).
- Çukur kenarı testi: 15 cm dropoff → tetiklendi mi?
- False positive oranı: düz zemine yanlış DROPOFF demesi.

### 6.5 Sistem Latencisi

**Yazılması gereken:**

- /goal_pose → /cmd_vel_safe arası gecikme.
- Sensör kaybından E-Stop'a kadar geçen süre.
- Nav2 plan üretim süresi.

### 6.6 Tartışma

**Yazılması gereken:**

- Encoder yokluğunun gerçek etkisi (planlanan vs ölçülen).
- RANSAC tabanlı terrain'in güçlü ve zayıf yönleri.
- Pi 5'in işlem kapasitesi: tam stack çalışırken CPU kullanımı.
- Simülasyon-gerçek arasındaki ayrım noktaları (Sim2Real gap).

---

## BÖLÜM 7 — SONUÇ VE GELECEK ÇALIŞMALAR

### 7.1 Sonuçlar

**Yazılması gereken:**

Bu çalışma, encoder kullanmayan, düşük maliyetli bir otonom kara aracı için
açık kaynak ROS 2 Jazzy tabanlı uçtan uca bir yazılım yığınının nasıl
geliştirilebileceğini ve doğrulanabileceğini göstermiştir. Önerilen mimaride
lidar tabanlı odometri ile IMU füzyonu, [%X başarı oranı] ile Nav2 tabanlı
otonom navigasyonu mümkün kılmıştır. Özgün geliştirilen RANSAC tabanlı
terrain perception düğümü, %10-%30 eğim aralığındaki rampaları doğru
sınıflandırmış, 15 cm üstü çukurları %X oranında tetiklemiştir. Çok
katmanlı güvenlik mimarisi, sensör kaybı senaryolarında [Y ms] içinde
araç durdurma garantisi sağlamıştır.

### 7.2 Karşılaşılan Zorluklar

- ROS 2 Jazzy + Gazebo Harmonic henüz yeni, bazı paketler (rf2o, sllidar) apt'te yok, source build gerek.
- Pi 5'in VideoCore VII GPU'sunun OGRE2 render motoru ile uyumluluğu sınırlı, headless mod tercih edilmek zorunda kaldı.
- Encoder yokluğu lidar odom'a aşırı bağımlılık yarattı; özelliği zayıf ortamlarda (örn. düz koridor) drift kabul edildi.

### 7.3 Gelecek Çalışmalar

| # | Konu | Öncelik | Yöntem |
|---|---|---|---|
| 1 | Optik tekerlek encoder entegrasyonu | YÜKSEK | QRE1113 + Arduino interrupt, /wheel_odom topic |
| 2 | IMU + depth füzyonu (RANSAC öncesi) | YÜKSEK | EKF tabanlı pose-aware ground plane |
| 3 | Görme tabanlı yerel haritalama | ORTA | OAK-D Lite Visual SLAM |
| 4 | micro-ROS geçişi | ORTA | Arduino ROS 2 doğrudan node |
| 5 | Behavior Tree özelleştirme | ORTA | Görev iptal + kurtarma davranışları |
| 6 | RL tabanlı yerel planner | DÜŞÜK | DRL controller (Sim2Real) |
| 7 | Çoklu robot koordinasyonu | DÜŞÜK | DDS Discovery Server, sürü davranışları |

### 7.4 Etik ve Sürdürülebilirlik

- Açık kaynak yayımı (Apache 2.0) — akademik ve hobi topluluğu için referans implementasyon.
- Düşük maliyet (sub-$300) — düşük gelirli bağlamlarda da erişilebilir.
- Eğitim potansiyeli — robotik eğitiminde sıfırdan kurulabilen bir vaka çalışması.

---

## KAYNAKLAR

> Yukarıda her bölümde verdiğim BibTeX'leri birleştir, ek olarak şu kaynakları ekle:

```bibtex
@misc{ros2_jazzy_docs,
  title={ROS 2 Jazzy Jalisco Documentation},
  author={Open Robotics},
  year={2024},
  url={https://docs.ros.org/en/jazzy/}
}

@misc{nav2_docs,
  title={Nav2 Documentation},
  author={Open Navigation LLC},
  year={2024},
  url={https://docs.nav2.org/}
}

@misc{gazebo_harmonic,
  title={Gazebo Harmonic Documentation},
  author={Open Robotics},
  year={2024},
  url={https://gazebosim.org/docs/harmonic/}
}

@misc{rplidar_c1,
  title={SLAMTEC RPLIDAR C1 Datasheet},
  author={SLAMTEC Co., Ltd.},
  year={2023}
}

@misc{oak_d_lite,
  title={Luxonis OAK-D Lite Documentation},
  author={Luxonis Holding Corporation},
  year={2023},
  url={https://docs.luxonis.com/}
}

@book{thrun2005probabilistic,
  title={Probabilistic Robotics},
  author={Thrun, Sebastian and Burgard, Wolfgang and Fox, Dieter},
  publisher={MIT Press}, year={2005}
}

@book{siegwart2011introduction,
  title={Introduction to Autonomous Mobile Robots},
  author={Siegwart, Roland and Nourbakhsh, Illah R and Scaramuzza, Davide},
  publisher={MIT Press}, edition={2nd}, year={2011}
}
```

---

## EKLER

### Ek A — Tam Donanım Listesi ve Maliyetleri

| Bileşen | Adet | Birim Fiyat (TL) | Tedarikçi |
|---|---|---|---|
| Raspberry Pi 5 16GB | 1 | ~4500 | Direnç |
| MicroSD 64GB | 1 | ~300 | — |
| RPLIDAR C1 | 1 | ~2500 | Robotistan |
| OAK-D Lite | 1 | ~3000 | Robolink |
| Arduino Uno | 1 | ~300 | — |
| L298N motor sürücü | 2 | ~80 | — |
| 4× DC motor + redüktör | 1 set | ~600 | — |
| GPS UBLOX NEO-6M | 1 | ~400 | — |
| IMU MPU-9250 | 1 | ~150 | — |
| Şasi + tekerlekler | 1 | ~500 | — |
| Pil + DC-DC | 1 | ~500 | — |
| **TOPLAM** | | **~12830 TL** | (~$400 @USD 32) |

### Ek B — Tam Komut Listesi (Reprodüksiyon)

```bash
# Pi'de
git clone https://github.com/aysenur-23/ika.git ~/ika
cd ~/ika
chmod +x scripts/*.sh
./scripts/install_pi.sh
exit && ssh ubuntu@<PI_IP>  # grupları yenile
cd ~/ika
./scripts/deploy_sim.sh
```

### Ek C — Test Komutları

```bash
# Birim testler
cd ~/ika/ika_ws/src/ika_terrain && python3 -m pytest test/ -v
cd ~/ika/ika_ws/src/ika_safety && python3 -m pytest test/ -v
cd ~/ika/ika_ws/src/ika_base_controller && python3 -m pytest test/ -v

# Pre-push doğrulama
./scripts/check_workspace.sh
```

### Ek D — Konfigürasyon Dosyaları

(Tezde tam dosyaları paste etme; sadece kritik satırları ve referans ver.)

- `ika_navigation/config/nav2_params.yaml` — 336 satır
- `ika_navigation/config/ekf_params.yaml` — 76 satır
- `ika_navigation/config/slam_params.yaml` — 52 satır
- `ika_terrain/config/terrain_params.yaml` — 29 satır
- `ika_safety/config/safety_params.yaml` — 26 satır

### Ek E — Önemli Kod Parçacıkları

(Sadece RANSAC + safety filtre + skid-steer dönüşüm için.)

#### RANSAC Zemin Düzlemi
```python
def fit_plane_ransac(points, iterations=60, tolerance=0.04):
    n = points.shape[0]
    best = None
    for _ in range(iterations):
        idx = rng.choice(n, 3, replace=False)
        p0, p1, p2 = points[idx]
        normal = np.cross(p1-p0, p2-p0)
        normal /= np.linalg.norm(normal)
        d = -np.dot(normal, p0)
        distances = np.abs(points @ normal + d)
        inliers = distances < tolerance
        if best is None or inliers.sum() > best.inlier_count:
            best = Plane(normal, d, inliers.sum(), inliers)
    return best
```

#### Skid-Steer Dönüşüm
```python
def twist_to_wheels(linear_x, angular_z, wheel_base):
    v_left = linear_x - (angular_z * wheel_base / 2.0)
    v_right = linear_x + (angular_z * wheel_base / 2.0)
    return v_left, v_right
```

#### Safety Karar
```python
if terrain_class in stop_classes:        # DROPOFF, IMPASSABLE
    publish_zero_velocity()
elif terrain_class in slow_classes:      # CAUTION, UNKNOWN
    publish_velocity(input * 0.3)
else:
    publish_velocity(input)
```

### Ek F — Şekil Üretim Kılavuzu

- **Ş.4** rqt_graph: `ros2 run rqt_graph rqt_graph` → File → Save as PNG.
- **Ş.5** TF ağacı: `ros2 run tf2_tools view_frames` → frames.pdf.
- **Ş.6** URDF: RViz → screenshot.
- **Ş.7** Gazebo dünyası: gz sim → Ctrl+Shift+S → PNG.
- **Ş.8** SLAM haritası: rviz_map_saver_cli + RViz screenshot.

---

## YAZIM İPUÇLARI

### Akademik Üslup

- Birinci tekil değil, **edilgen veya çoğul**: "Sistem geliştirilmiştir", "Bu çalışmada incelenmiştir".
- Pasajlar **3-5 cümle** ideal, **paragraf başı tezi** belirgin.
- Teknik terimler ilk geçtiğinde Türkçe + parantez içinde İngilizce: "Lidar tabanlı odometri (LiDAR-based odometry)".

### Kaynak Verme

- Her teknik iddia için kaynak: "Lidar tabanlı odometri encoder hatalarına karşı bağışıklık sağlar (Jaimez vd., 2016)".
- Kendi katkını net belirt: "Bu çalışmada... farklı olarak..." veya "Mevcut literatürden farklı olarak...".

### Şekil ve Tablo Numaralama

- Bölüm.numara: "Şekil 3.2", "Tablo 5.1".
- Tüm şekiller metin içinde **en az bir kez** referanslanmalı: "Şekil 3.2'de görüldüğü üzere...".

### LaTeX Şablonu Önerisi

YTÜ, İTÜ, ODTÜ tez şablonları genelde aynı format yapısına sahiptir:
- `\documentclass{report}` veya kurumsal şablon
- Bölümler: `\chapter`, `\section`, `\subsection`
- Şekiller: `\begin{figure}[h]` + caption + label + ref
- Bibliography: BibTeX (`\bibliographystyle{ieeetr}` veya `\bibliographystyle{apalike}`)

### Tezi Kaç Sayfaya Yazmalısın?

Yüksek lisans tezi tipik 50-80 sayfa. Yukarıdaki bölümlere göre tahmini dağılım:

| Bölüm | Sayfa |
|---|---|
| Özet + içindekiler | 5-7 |
| 1 Giriş | 4-6 |
| 2 Literatür | 8-12 |
| 3 Sistem mimarisi | 6-10 |
| 4 Yazılım geliştirme | 10-15 |
| 5 Simülasyon ve test | 6-10 |
| 6 Bulgular | 6-10 |
| 7 Sonuç | 3-5 |
| Kaynaklar | 3-5 |
| Ekler | 5-10 |
| **TOPLAM** | **56-90** |

---

## SON NOT

Bu doküman **sadece içerik şablonu** — birebir kopyalamak intihal olur. Her
paragrafı kendi cümlelerinle yeniden yaz, kendi yorumlarını ekle, sayısal
verileri kendi testlerinden al.

Tez savunması için:

- 15-20 slayt sunum hazırla
- Demo videosu (Gazebo + gerçek araç) kaydet
- Sorulara hazırlık: "Neden encoder yok?", "Neden Pi 5?", "RANSAC neden tercih edildi?", "Sim2Real geçişinde ne tür sorunlar bekliyorsun?"

Başarılar.
