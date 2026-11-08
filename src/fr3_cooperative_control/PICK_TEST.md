# Tek kol fiziksel kavrama ve kaldırma testi

~~~bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch fr3_cooperative_control single_arm_pick_test.launch.py
~~~

Önce açık simülasyonu Ctrl+C ile kapatın. Her denemede simülasyon yeniden başlatılmalıdır.
Başsız doğrulama için headless:=true eklenebilir. Mevcut reach_test senaryosu korunur.

Bu ayrı test rafta 40 mm / 50 g kutu oluşturur. Gazebo kutu pozunu yayınlar;
MoveIt dünya koordinatına geçerken robotun 0.05 m spawn yüksekliği çıkarılır.
Sol kol ready, ön poz ve kavrama hedefinden geçer. Kavrama hedefi kutu merkezidir.

Gripper kapanmadan önce güncel eklem durumundan hesaplanan TCP hedefe 3 mm ve
0.025 rad içinde olmalıdır. Ardışık üç 200 ms ölçümde eklem değişimi 0.0002 rad
altında olmalıdır. Bekleme 20 duvar saati saniyesinde başarısız olur; sonraki harekete geçilmez.
Kontrolcü SUCCEEDED sonucu tek başına yeterli sayılmaz.

Gripper önce 0.04 m açılır, sonra 0.018 m tek-parmak hedefiyle kapanır. Kutuyla engellenen
kapanmada stalled sonucu ve ABORTED durumu beklenebilir. Bu yalnızca bir kavrama adayıdır:
iki fiziksel parmağın güncel açıklığı ayrıca 0.0185–0.022 m aralığında olmalıdır.
Position kontrolü kuvvet kontrollü kavrama garantisi sağlamaz.

Gazebo nesnesi dinamik kalır. MoveIt attachment yalnızca planlama içindir; Gazebo'da
sabit eklem veya attach eklentisi kullanılmaz. Parmağın kutuya planlama teması serbesttir;
diğer çarpışma kontrolleri korunur. Kutu kapanma sonrasında ölçülen pozuyla planlamaya eşitlenir.

Başarı koşulları:
- Çarpışma kontrollü Kartezyen yolun en az %99.9 tamamlanması ve yürütülmesi.
- Gerçek kutunun başlangıca göre 80 ± 5 mm yükselmesi; yanal değişimin 10 mm altında olması.
- Ardından 3 simülasyon saniyesi tutma; Gazebo pozunun güncel kalması.
- Kapanma sonrası referansa göre kutunun ele göre konum değişimi 5 mm,
  yönelim değişimi 5 derece altında olması. Bu ölçüm kaldırma sırasında oluşan net kaymayı da kapsar;
  sürekli hareket sırasındaki geçici en büyük kaymayı ölçmez.

Başarı çıktısı PICK TEST PASSED olmalıdır. Kutu tutulur, gripper kapalı kalır.
Varsayılan stage:=pick modunda masa aktarımı, elin döndürülmesi, bırakma ve ready dönüşü yapılmaz.
Hata durumunda otomatik açma veya kurtarma uygulanmaz; simülasyonu inceleyin.

## Doğrulama

İlk düzeltilmiş başsız deneme geçti: fiziksel yükselme 79.99 mm,
3 simülasyon saniyesi tutma sırasında göreli konum değişimi 0.006 mm,
yönelim değişimi 0.000 derece. Kayıt: log/pick_check/pick_pass_1.log.
Bu sonuç hassas yerleştirme veya masa aktarımını doğrulamaz.

İkinci denemede kapanma doğrulandı, ancak kaldırma sonrası kolun durulma kontrolü
zaman aşımına uğradı; test başarısız sayıldı. Kayıt: log/pick_check/pick_settle_failed.log.
Tek başarılı deneme tekrarlı güvenilirlik kanıtı olarak kabul edilmez.

Yeni launch, yalnızca bu testte sol joint5 için ±2.60 rad planlama sınırı kullanır;
fiziksel URDF sınırları ve reach_test değişmez. Server ve client aynı sınırı paylaşır.
Önceki tanılı başarılı yolda joint5 hedefi fiziksel üst sınıra (2.876303 rad)
denk geliyordu; sınır payı, bu uç noktaları planlamadan çıkarır. İkinci denemedeki
durulma hatasının tek nedeninin bu olduğu kanıtlanmış değildir.
Sınır payıyla ilk deneme geçti: 79.98 mm yükselme ve 2.014 mm göreli kayma;
kayıt log/pick_check/pick_margin_pass_1.log.

Son sürüm sınır payıyla ikinci temiz simülasyon denemesini de geçti: 79.99 mm yükselme,
3 simülasyon saniyesi tutma, 2.014 mm göreli konum değişimi ve 0.000 derece açı değişimi.
Kayıt: log/pick_check/pick_margin_pass_2.log. Son sürüm için iki ardışık başsız
deneme başarılıdır; bu uzun süreli başarı oranı veya masa aktarımı garantisi değildir.
C++/Python biçim kontrolleri ve git diff --check geçti. Görevden sonra kurulu MoveIt'in
SIGINT kapanış hatası devam eder; görev başarısı ve test düğümünün temiz kapanışı
bu kapanış çıktısından önce doğrulanır.

## Ayrı masa taşıma aşaması

~~~bash
ros2 launch fr3_cooperative_control single_arm_pick_test.launch.py stage:=transfer
~~~

Önce eski simülasyonu kapatın. Varsayılan stage:=pick akışı korunur. Transfer modu
aynı kavrama ve kaldırma kontrollerinden sonra, aynı el yönüyle 120 mm raftan
geri çekilir. Hedefte durulur ve kutunun ele göre konum/yönelim değişimi 3 simülasyon
saniyesi boyunca kontrol edilir. Ardından el yönü masaya uygun (-0.5, -0.5, -0.5, 0.5)
quaternionuna değiştirilerek çarpışma kontrollü plan yürütülür. Taşıma hız/ivme ölçeği 0.10'dur.
Bu yol boyunca el yönü sabit değildir; raf ve masa için gereken iki yön arasında geçiş yapılır.

Hedef kutu merkezi MoveIt world çerçevesinde (0, 0.05, 0.60) m'dir; merkez masa
yüzeyinden 20 cm, kutu altı yaklaşık 18 cm yukarıdadır. TCP hedefi, kapanma sonrasında
ölçülen kutu–el dönüşümüyle hesaplanır. Masanın üzerinde bir 3 simülasyon saniyesi daha
tutulur. Kapanma sonrası kavrama referansı her aşamada aynı kalır; referans yenilenerek
birikmiş kayma gizlenmez. Göreli kayma 5 mm ve açı değişimi 5 derece sınırlarını korur.
Kontroller hareket sonrasındaki beklemelerde yapılır; hareket sırasında geçici kaymanın
tamamını ölçme veya otomatik hareket durdurma garantisi yoktur.

Son kutu merkez hatası ayrıca 10 mm, yüzey normalinin masa normalinden eğimi 5 derece
altında olmalıdır. TRANSFER TEST PASSED çıktısını bekleyin. Gripper kapalı kalır;
bırakma veya ready dönüşü yapılmaz. Başarı hassas yerleştirme veya çarpışma sensörüyle
doğrulanmış temassızlık anlamına gelmez. Gazebo fiziksel pozu başarı kontrolünün parçasıdır.

İlk taşıma denemesi geçti: merkez hatası 1.362 mm, eğim 0.689 derece, kavrama
referansına göre kayma 0.320 mm / açı değişimi 0.004 derece.
Kayıt: log/transfer_check/transfer_first_pass.log.

Tam 3 saniye boyunca örnekleyen son tutma kontrolüyle tekrar da geçti: merkez hatası
1.853 mm, eğim 0.572 derece; aynı kavrama referansına göre en büyük konum değişimi
2.014 mm, açı değişimi 0.002 derece. Kayıt: log/transfer_check/transfer_final_pass.log.
Derleme, C++/Python biçim kontrolleri ve git diff --check geçti. İki taşıma denemesi
uzun süreli başarı oranı garantisi değildir; transfer aşamasında masa üstüne bırakma yapılmaz. Ayrı stage:=place için PLACE_TEST.md dosyasına bakın.
