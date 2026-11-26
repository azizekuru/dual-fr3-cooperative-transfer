# fr3_cooperative_control

Çift FR3 kolu için yörünge planlama ve senaryo testleri (ROS 2 Jazzy, MoveIt 2, Gazebo).

Senaryo: arm_1 (sol) shelf_1'deki kutuyu alır, masaya bırakır; arm_2 (sağ) masadan alır, shelf_2'ye koyar.

## Durum

| Aşama | İçerik | Durum |
|---|---|---|
| 1 | Sadece uzanma testi: kutu yok, gripper yok | Çalışıyor (`dual_arm_reach_test`) |
| 2 | Kutu + kavrama + bırakma | Yapılacak |

Aşama 1'de her kol sırayla hedeflere uzanır, geri çekilir. Çarpışma olursa hangi iki linkin değdiğini loga yazar.

## Gereksinimler

Aynı workspace'te şu paketler derlenmiş olmalı:

- `franka_gazebo_bringup` (Gazebo dünyası, robotlar, raflar, masa)
- `dual_fr3_moveit_config` (MoveIt ayarları, controller adları `arm_1_controller`, `arm_2_controller`)
- `fr3_cooperative_control` (bu paket)

## Derleme

```bash
cd ~/ros2_ws
colcon build --packages-select fr3_cooperative_control
source install/setup.bash
```

## Çalıştırma (tek komut)

```bash
ros2 launch fr3_cooperative_control reach_test.launch.py
```

Bu komut sırasıyla Gazebo'yu, 8 saniye sonra `move_group`'u, 30 saniye sonra testi başlatır.
Testin sonunda terminalde `Senaryo tamamlandi.` yazar.

Kutu spawn edilmez. Gazebo launch'ı (`franka_gazebo_bringup`) olduğu gibi kullanılır, değiştirilmez.

### Seçenekler

```bash
# Sadece ilk 3 adımı çalıştır (1=ready, 2=shelf_1, 3=masa, 4=arm_2 masa, 5=shelf_2)
ros2 launch fr3_cooperative_control reach_test.launch.py stop_after:=3

# Hızlandır
ros2 launch fr3_cooperative_control reach_test.launch.py speed:=0.8 approach_speed:=0.4 settle_timeout:=1.0

# Gazebo penceresiz (daha hızlı). Görmek için ayrı terminalde: gz sim -g
ros2 launch fr3_cooperative_control reach_test.launch.py headless:=true

# Bilgisayar yavaşsa Gazebo'ya daha fazla zaman tanı
ros2 launch fr3_cooperative_control reach_test.launch.py move_group_delay:=15.0 coordinator_delay:=50.0
```

| Parametre | Varsayılan | Anlamı |
|---|---|---|
| `stop_after` | 99 | Bu adımdan sonra dur |
| `speed` | 0.7 | Serbest hareketlerin hızı (joint limits ölçeği) |
| `approach_speed` | 0.3 | Hedefe düz çizgide yaklaşma hızı |
| `settle_timeout` | 2.0 | Planlamadan önce kolun durmasını bekleme süresi (sn). 0 = bekleme |
| `via_ready` | false | shelf_1'den sonra masaya geçmeden ready'ye uğra |
| `headless` | false | Gazebo GUI açma |
| `rviz` | false | Gazebo launch'ındaki RViz'i aç |

### Elle (üç terminal)

```bash
ros2 launch franka_gazebo_bringup gazebo_fr3_duo_example.launch.py rviz:=false
ros2 launch dual_fr3_moveit_config move_group.launch.py
ros2 run fr3_cooperative_control dual_arm_reach_test --ros-args -p use_sim_time:=true -p stop_after:=3
```

## Bilinen noktalar

- **Koordinatlar MoveIt (URDF) çerçevesindedir.** Robot Gazebo'da `z=+0.05` ile spawn edilir, yani Gazebo dünya z'si = URDF z + 0.05.
- **Masada** arm_1 için 180° döndürülmüş el yönü (+2 cm) gerekti. Diğer yönde bilek/dirsek masaya çarpıyor.
- **Gazebo yavaşsa** (`gz topic -e -t /world/empty/stats -n 1` ile `real_time_factor`'a bak) RViz'i kapatmak ve `headless:=true` kullanmak yardımcı olur.
- **Kolun oturması:** Gazebo'da hareket "bitti" dedikten sonra kol birkaç saniye kayarak oturuyor. `settle_timeout` bu yüzden var. Bir hareket `aborted` olursa bu değeri artır.
- **Hata ayıklama:** Cartesian yaklaşma başarısız olursa log hem `%` değerini hem `temas: <link> <-> <link>` satırını yazar.

## Dosyalar

- `src/dual_arm_reach_test.cpp`: Aşama 1 testi (kutu yok, gripper yok)
- `src/dual_arm_coordinator.cpp`: ilk sürüm (değiştirilmedi)
- `launch/reach_test.launch.py`: Gazebo + move_group + test tek komutla

## Kod yapısı (masa aktarmalı tam görev)

Görev düğümleri yalnızca görev mantığını içerir; ortak işler `fr3_cooperative_core` kütüphanesindedir.

| Dosya | Görev |
|---|---|
| `src/tasks/left_transfer.cpp` | Sol kol görevi (`LeftTransfer`): kaynak raf hücresinden alma, masaya taşıma ve koyma |
| `src/tasks/right_transfer.cpp` | Sağ kol görevi (`RightTransfer`): sol kolu park etme, masadan alma, hedef raf hücresine koyma |
| `src/transfer_task.cpp` | Tek süreçli koordinatör: iki görevi sırayla çalıştırır (`single_process:=true`) |
| `src/single_arm_pick_test.cpp`, `src/right_arm_table_pick.cpp` | Kolları ayrı süreçlerde çalıştıran ince giriş noktaları (varsayılan) |
| `include/.../world_model.hpp` | Masa, raf, kat ve direk geometrisi robot modelinden okunur; görev hücreleri buradan hesaplanır |
| `include/.../arm_motion.hpp` | Düz/hassas Kartezyen yol, TOTG zamanlama, bekleme, planla-yürüt, planlama sahnesi araçları |
| `include/.../box_tracker.hpp` | Gazebo'dan kutu konumu (taze/sabit ölçüm, eğiklik, taban yüksekliği) |
| `include/.../contact_monitor.hpp` | Temas sensörleri: parmak teması, destek teması, beklenmeyen temaslar |
| `include/.../gripper.hpp` | Gripper komutu ve ölçülen parmak açıklığı |

Kodda sabit koordinat yoktur. Görev hücreleri launch argümanlarıyla seçilir (varsayılan: 2. kat → masa → 2. kat):

```bash
ros2 launch fr3_cooperative_control table_handoff_test.launch.py right_stage:=shelf_place \
  source_level:=2 target_level:=2
```

Ek launch argümanları (varsayılanlar bugünkü davranışı verir):

| Argüman | Varsayılan | Anlamı |
|---|---|---|
| `source_slot`, `target_slot` | `center` | Kutunun raftaki yeri (`left` / `center` / `right`, masadan rafa bakınca). Sol/sağ: kat merkezinden ±0.10 m |
| `single_process` | `false` | `true`: iki kol görevi tek `transfer_task` sürecinde çalışır |
| `verify` | `full` | `fast`: tutma, destek ve bırakma sonrası izleme süreleri ×0.2 (en az 0.3 s); kontroller aynı kalır |
| `speed_scale` | `1.0` | Test modu: tüm hareket hızlarını çarpar (en fazla 1.0'a kadar) |
| `rigid_grasp` | `false` | Test modu: kavramada kutu ele kaynaklanır, bırakmadan önce çözülür (Gazebo DetachableJoint) |

Hızlı yol testi örneği:

```bash
ros2 launch fr3_cooperative_control table_handoff_test.launch.py right_stage:=shelf_place \
  single_process:=true rigid_grasp:=true speed_scale:=3.0
```

Sol kol kavramayı hareketten önce prova ederek seçer (`PICK GRASP SELECTED`): düz yandan, 20° veya
35° aşağı eğik kavrama ve 12 veya 8 cm geri çekilme. Seçim, ulaşılabilirlik haritasının
(`reachability_map.launch.py`) sonuçlarına dayanır: her rafta L1–L3, sol/sağ yuvalar, ön kenarın 3 cm içi.

Not: `source_level`/`target_level` şimdilik yalnızca 2. kat için doğrulanmıştır. Diğer katlar için
kavrama ve yaklaşma adaylarının seçimi (geçiş planı 4–6. adımlar) henüz yapılmadı.

## Toplu test (`batch_transfer_test.py`)

Birçok görevi arka arkaya, hiçbir zaman üst üste binmeden çalıştırır; her koşunun sonucunu, süresini ve
konum hatalarını `~/fr3_batch/<tarih>/results.csv` dosyasına yazar ve sonunda bir özet verir.

```bash
ros2 run fr3_cooperative_control batch_transfer_test.py --dry-run --random 10   # sadece listeler
ros2 run fr3_cooperative_control batch_transfer_test.py --random 10             # 10 rastgele görev
ros2 run fr3_cooperative_control batch_transfer_test.py --all --levels 2 3      # 36 kombinasyon
ros2 run fr3_cooperative_control batch_transfer_test.py --random 5 --extra rigid_grasp:=true speed_scale:=2.0
```

Başka bir Gazebo/launch süreci çalışıyorsa başlamaz. Varsayılan headless'tır (`--gui` ile pencere açılır).
