# İlk aşama: gripper arayüzü ve hedef doğruluğu

Mevcut uzanma senaryosu korunur. Bu aşama kutu oluşturmaz veya kavrama/taşıma/bırakma yapmaz.

Gazebo kontrolcüleri sol/sağ için GripperCommand action sunar:
- /left_gripper_controller/gripper_cmd
- /right_gripper_controller/gripper_cmd

MoveIt aynı kontrolcü isimlerini ve action türünü kullanır. Komut konumu tek parmağın
metre cinsinden açıklığıdır: 0.04 m yaklaşık 0.08 m toplam çene aralığıdır.
İkinci parmak URDF mimic eklemi olarak, komut arayüzü olmadan Gazebo kontrolüne kayıtlıdır.
Position kontrolünde max_effort ayarı fiziksel kavrama kuvveti garantisi değildir.

## Derleme

Önceki realtime_tools sürüm çakışması nedeniyle yerel paketi derlemeyin. Temiz derlemede:

~~~bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-ignore realtime_tools
source install/setup.bash
~~~

## Sol kol hedef ölçümü

Eski simülasyonu Ctrl+C ile kapatıp yeniden başlatın:

~~~bash
ros2 launch fr3_cooperative_control reach_test.launch.py stop_after:=2 tcp_diagnostics:=true
~~~

Ölçüm modu her hedefte mevcut 0.5 saniyelik beklemeden sonra 5 saniye ek gözlem yapar.
TCP DIAG alanları milimetre cinsindendir:
- planned_endpoint_error: yürütülen planın son noktasının istenen hedefe farkı.
- target_error: güncel eklem durumundan FK ile hesaplanan TCP'nin hedefe farkı.
- tracking_error: güncel TCP'nin planlanan son noktaya farkı.
- joint_error: planın son noktasına göre en büyük eklem farkı, radyan.

Bu tanı raporu otomatik hassasiyet geçme koşulu koymaz; süreler duvar saatidir.
Varsayılan tcp_diagnostics:=false olduğunda ek gözlem yapılmaz. Hız, kontrolcü toleransları
ve mevcut TOTG ayarları bu değişiklikle değiştirilmemiştir.

## Boş sol gripper: MoveIt üzerinden aç–kapat–aç

Gazebo ve MoveIt açıkken, uzanma düğümü tamamlandıktan sonra ikinci terminalde:

~~~bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run fr3_cooperative_control gripper_alignment_check.py
~~~

Test MoveIt move_action üzerinden gripper_1 grubunu 0.04, 0.005, 0.04 m hedeflerine götürür.
Her hedefte iki parmağın güncel joint_states ölçümü 1 mm içinde ve en az 0.5 saniye
boyunca doğrulanır. Çıktı GRIPPER ALIGNMENT PASSED olmalıdır. Bu test boş gripper içindir;
kutu kavrama veya fiziksel tutma başarısı anlamına gelmez. Eski test_grippers.py de artık
gripper_cmd action kullanır; commands konusuna yayın yapan dış betikler uyarlanmalıdır.

## İlk hedef ölçümü

İlk başsız çalışmada raf hedefine ait plan sonu hatası 0.000 mm idi. Yürütme sonucundan
0.5 saniye sonra hedef hatası 15.747 mm, ek 5 saniye sonra 0.002 mm ölçüldü.
Bu, bu denemede kontrolün yerleşmesini beklemeden alınan ölçümün yanıltıcı olduğunu gösterir.
Kavrama adımında yalnızca SUCCEEDED sonucuna güvenmek yerine güncel durumla hedef yakınlığı
ve durulma koşulu aranmalıdır. Bu henüz uygulanmış bir kavrama düzeltmesi değildir.

İkinci izole ölçümde raf hatası 14.1 mm'den ek 5 saniye sonunda 0.002 mm'ye indi.
Aynı simülasyonda MoveIt üzerinden boş sol gripper testi geçti: iki parmak sırasıyla
0.0393/0.0393, 0.0056/0.0056, 0.0394/0.0394 m ölçüldü.
Kayıtlar: log/alignment_check/reach.log ve log/alignment_check/gripper.log (Git dışı).
Görev tamamlandıktan sonraki SIGINT kapanışında kurulu move_group segmentation fault
verdi; bu mevcut kapanış sorunu bu aşamada düzeltilmedi. Gazebo sarmalayıcısı SIGTERM
ile sonlandırıldı. Ölçüm ve gripper başarısı kapanıştan önce raporlandı.
