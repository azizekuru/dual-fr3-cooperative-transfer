# Masa üzerinden ikinci kola aktarım

## Tam zincir

Önce eski simülasyonu Ctrl+C ile kapatın. Yeni simülasyonda:

~~~bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fr3_cooperative_control table_handoff_test.launch.py
~~~

Önce mevcut single_arm_pick_test düğümü stage:=place ile sol kolun raftan alıp
masaya yerleştirmesini yapar. Yalnızca bu süreç sıfır çıkış koduyla tamamlanırsa
yeni right_arm_table_pick düğümü başlar. İlk aşama başarısızsa sağ kol başlatılmaz.
Son başarı işareti TABLE HANDOFF TEST PASSED olmalıdır.

Masaya bırakmadan sonra sol kolun 80 mm dikey yükselişi ve o yükselişin
son eklem konumundan ready pozuna park rotası önceden planlanır. Her iki
rota kabul edildikten sonra yürütülür. Yükselişin ve parkın sonunda eklem
konumları, park tamamlanınca da kutunun masa desteği ve konumu doğrulanır.
Yalnız bu bölümü denemek için right_stage:=left_park kullanılır.
Başarı işareti LEFT PARK SCENARIO PASSED olur.

## Yalnızca sağ kol testi

~~~bash
ros2 launch fr3_cooperative_control table_handoff_test.launch.py table_only:=true
~~~

Bu mod kutuyu doğrudan masada oluşturur; sol kolun raftan alma aşaması çalışmaz.
Sağ kol testi yine sol kolu park eder. RIGHT TABLE PICK SCENARIO PASSED sonucunu bekleyin.
Her denemeden önce simülasyonu yeniden başlatın.

## Sağ kol akışı ve fiziksel kontroller

- Kutunun gerçek pozu masada beklenen (0, 0.05, 0.42) m konumuna 6 mm içinde
  olmalı; masa temas sensörüyle destek doğrulanmalıdır. Gazebo spawn yüksekliği
  nedeniyle fiziksel dünya Z koordinatından 0.05 m çıkarılır.
- Önceki sol parmak–kutu temas izinleri kaldırılır. Sol kol ve ardından sağ kol
  ready pozuna park edilir. Planın başlangıcında 1e-5 rad'a kadar sayısal sınır
  taşması eklem sınırına yuvarlanır. Daha büyük bir taşma testi durdurur.
  Kutunun bu sıradaki konum değişimi 1 mm altında olmalıdır.
- Sağ gripper 0.04 m açılır. Ön poz seçilirken ölçülen kutu merkezine kadar
  çarpışma kontrollü iniş ve sağ kolun tüm eklemlerinde en az 0.05 rad sınır payı aranır.
  Gerçek iniş ve kaldırma, ölçülen durumdan aynı eklem payıyla tekrar hesaplanır.
  Kol durulduğunda ön poz hatası 1 mm veya 0.01 rad sınırını aşıyorsa ve
  hata 3 mm ile 0.015 rad içinde kalıyorsa bir kısa Kartezyen düzeltme
  denenir. Parmak–kutu temas izni bu düzeltmeden sonra verilir. Aynı ön poz
  toleransları yeniden doğrulanmadan kavrama inişi başlamaz.
  Kapanma öncesinde güncel TCP hatası 0.5 mm / 0.005 rad içinde ve kol durulmuş olmalıdır.
- Sağ parmak–kutu planlama teması serbesttir; sol kolun kutuya teması ve robot–masa
  üstü teması fiziksel sensörlerde hata olarak tutulur. Planlama sahnesinde diğer
  çarpışma kontrolleri korunur; masa–kutu teması başlangıçta izinlidir.
- Kapanma hedefi tek parmak için 0.018 m'dir. Kutuyla engellenen stalled/ABORTED
  sonucu yalnızca iki fiziksel parmağın açıklığı 0.0185–0.022 m içindeyse kabul edilir.
  Bu tek başına kavrama başarısı sayılmaz.
- Kutu planlamada sağ TCP'ye attach edilir; Gazebo'da dinamik kalır. Sabitleme
  veya Gazebo attach eklentisi kullanılmaz. Sonra 80 mm dikey kaldırılır.
- Gerçek kutunun yükselmesi 80 ± 5 mm, toplam yanal değişimi 5 mm içinde olmalıdır.
  Kapanma sonrası aynı kutu–el referansına göre 3 simülasyon saniyesi tutmada
  konum değişimi 5 mm, yönelim değişimi 5 derece altında kalmalıdır.
- İki sağ parmak için güncel fiziksel temas ölçümü aranır. Kaldırma sonrasında
  kutunun masayla güncel temasının kalmadığı doğrulanır.

Başarılı test sağ gripper kapalı ve kutu havada tutularak biter. Sağ kolun rafa
taşıması veya bırakması bu aşamada yoktur. Hata durumunda otomatik kurtarma uygulanmaz.
Kontroller hareket sonları ve bekleme fazlarında yapılır; hareket sırasında kesintisiz
kuvvet denetimi veya anlık durdurma garantisi değildir. Position kontrolündeki max_effort
alanı gerçek kuvvet kontrollü kavrama garantisi sağlamaz.

Yeni launch her iki kolun joint5 planlama sınırını ±2.60 rad olarak server/client'a
aynı biçimde verir. Mevcut single_arm_pick_test.cpp ve onun launch dosyası değiştirilmez;
önceki doğrulanmış tek kol komutları kullanılmaya devam eder.

## İlk tekil sağ kol doğrulaması

İzole başsız test geçti: gerçek yükselme 79.988 mm, 3 simülasyon saniyesi tutmada
kutu–el konum değişimi 0.005 mm ve yönelim değişimi 0.000 derece raporlandı.
İki sağ parmak teması doğrulandı. Kayıt: log/right_pick_check/right_only_pass.log.
Bu tekil test, sol kolun yerleştirmesinden sonra tam zincirin başarısı anlamına gelmez.

## İlk tam zincir doğrulaması

Sol kolun yerleştirmesi 0.091 mm XY hatası ve 0.211 mm açma sonrası değişimle
geçti. Sol kolun parkı sırasında kutunun korunması kontrol edildi; sağ kol kutuyu
79.988 mm kaldırdı ve 3 simülasyon saniyesi tutmada 0.004 mm göreli kayma,
0.000 derece dönme ölçüldü. TABLE HANDOFF TEST PASSED ve sağ düğümün temiz
kapanışı görüldü. Kayıt: log/right_pick_check/handoff_pass_1.log.
Derleme, yeni C++/Python biçim kontrolleri ve git diff --check geçti.

İkinci temiz simülasyonla tam zincir de geçti: sol yerleştirme XY hatası
1.929 mm, açma sonrası değişim 0.213 mm; sağ fiziksel kaldırma 79.988 mm,
göreli kayma 0.005 mm ve yönelim değişimi 0.000 derece. İki parmağın güncel
fiziksel teması ve kaldırma sonrası masa temasının kalmaması kontrol edildi.
TABLE HANDOFF TEST PASSED ve temiz sağ düğüm kapanışı görüldü.
Kayıt: log/right_pick_check/handoff_pass_2.log. Tekil sağ kol testi ve iki tam
zincir başsız denemesi başarılıdır; bu uzun süreli başarı oranı garantisi değildir.
Doğrulanan single_arm_pick_test.cpp ve single_arm_pick_test.launch.py dosyalarının
baseline commit'ine göre farkı olmadığı git diff ile kontrol edildi. Değişiklikler localdedir.

## İkinci raf önüne taşıma: right_stage:=shelf_hold

~~~bash
ros2 launch fr3_cooperative_control table_handoff_test.launch.py right_stage:=shelf_hold
~~~

Önce sol kolun yerleştirmesi, sonra sağ masadan alma ve 80 mm kaldırma yapılır.
Yeni aşama kutuyu mevcut el yönüyle 100 mm daha yükseltir ve 3 simülasyon saniyesi
tutar. Ardından el yönünü rafın içine bakacak (-0.5, -0.5, -0.5, 0.5) quaternionuna
değiştiren çarpışma kontrollü planla ikinci rafın önüne taşır. Hız/ivme ölçeği 0.10'dur.
Hedef kutu merkezi MoveIt world içinde (0, 0.92, 0.5625) m'dir. Bu, ilerideki
(0, 1.04, 0.4825) m raf hedefinin 120 mm önünde ve 80 mm yukarısındadır.
Kutu 3 simülasyon saniyesi daha tutulur; gripper kapalı kalır, bırakma yapılmaz.

TCP hedefi kapanma sonrası ölçülen aynı kutu–el dönüşümüyle hesaplanır. Kavrama
referansı aşamalar arasında yenilenmez. Duraklama fazlarında kayma 5 mm ve göreli
açı değişimi 5 derece altında kalmalı; iki fiziksel parmak teması güncel olmalıdır.
Son merkez hatası 10 mm, küp yüz normaliyle raf düzlemi arasındaki eğim 5 derece
sınırındadır. Hareket içindeki geçici en büyük kayma sürekli örneklenmez.

Kaldırma sonrası bu mod kutu–masa planlama temas iznini kaldırır. Sağ parmaklar
dışında kutunun herhangi bir yüzeye fiziksel teması hata bayrağı oluşturur. Bu,
raf yerleştirme aşaması değildir; raf temasına izin verilmez. Robot–raf çarpışmaları
MoveIt geometrisiyle denetlenir; fiziksel raf sensörü ayrıca eklenmemiştir.

TABLE TO SHELF TEST PASSED son başarı işaretidir. Mevcut right_stage:=pick
varsayılanı masadan alıp tutarak biter; önceki komutun davranışı korunur.

Yalnızca sağ kolu masadaki kutuyla denemek için:

~~~bash
ros2 launch fr3_cooperative_control table_handoff_test.launch.py table_only:=true right_stage:=shelf_hold
~~~

İlk izole sağ taşıma geçti: merkez hatası 2.426 mm, aynı kavrama referansına göre
en büyük konum değişimi 2.046 mm, açı değişimi 0.087 derece, yüz eğimi 0.104 derece.
Kayıt: log/shelf_hold_check/right_only_pass.log. Bu tekil sonuç tam zinciri doğrulamaz.

Raf önünde tutma içeren ilk tam zincir de geçti: sol yerleştirme XY hatası
0.095 mm ve açma sonrası değişim 0.210 mm, sağ ilk kaldırma 79.985 mm oldu.
Raf önünde son merkez hatası 1.742 mm, kapanma sonrası aynı referansa göre
en büyük konum değişimi 2.038 mm, açı değişimi 0.038 derece, yüz eğimi
0.070 derece raporlandı. TABLE TO SHELF TEST PASSED ve temiz sağ düğüm
kapanışı görüldü. Kayıt: log/shelf_hold_check/full_chain_pass.log.
Bir izole sağ taşıma ve bir tam zincir başsız denemesi başarılıdır; raf üzerine
yerleştirme veya uzun süreli başarı oranı bu aşamada doğrulanmış değildir.
Derleme, C++/Python biçim kontrolleri ve git diff --check geçti.

## İkinci rafa temas doğrulamalı yerleştirme: right_stage:=shelf_place

~~~bash
ros2 launch fr3_cooperative_control table_handoff_test.launch.py right_stage:=shelf_place
~~~

Bu mod masadan kaldırma ve ek güvenlik yükselişinden sonra iki aşamada raf
önüne gider. Ek yükseliş için 100, 80, 60 ve 40 mm adayları sırayla denetlenir.
Çarpışmasız yolu ve en az 0.05 rad eklem payı olan ilk aday yürütülür.
Yükseliş sonunda ölçülen eklem payı tekrar kontrol edilir. Yalnızca sağ kolu masadan başlatmak için table_only:=true
eklenebilir. Tam zincir başarı işareti FULL TRANSFER TEST PASSED, tekil modunki
RIGHT SHELF PLACE SCENARIO PASSED olur.

Sağ kol, son destek konumuna (kutu merkezi 0, 1.04, 0.4825 m) çarpışma kontrollü
IK ve ters Kartezyen yol ön kontrolü yapar. Önce yatay el yönü, gerekirse ters
yatay ve üstten yön denenir. Aday yolda sağ kolun tüm eklemlerinin konum sınırlarına
en az 0.05 rad pay bırakılır. Raf önündeki eklem duruşundan geriye doğru
hesaplanan Kartezyen yol, kutu merkezinin raf merkezinden 0.48 m önce ve
0.59 m yüksekte olduğu bir ara poza bağlanır. Sağ kol önce MoveIt planıyla
bu ara poza gidip yönünü değiştirir. Tutuş ve kolun durulması doğrulanınca
daha önce denetlenmiş düz Kartezyen yol üzerinden raf önüne ilerler.
Hareket başlamadan iki parça da kontrol edilir. Kutu merkezi yön değişiminde
raf dikmelerinin en az 200 mm, ilerleme sırasında en az 50 mm önünde kalır.
Her eklemin sınır payı ve kutunun döndürülmüş alt köşesinin masa üstünden en
az 80 mm yukarıda kalması rotanın noktalarında kontrol edilir. Raf önüne
taşımanın hız ve ivme ölçeği 0.06'dır. Bu hareket sürerken kutu yüksekliği,
kutu–el dönüşümü ve parmak teması 100 ms arayla izlenir. Kutu masa üstüne
45 mm'den çok yaklaşırsa, beklenmeyen temas kaydedilirse, kayma iki ardışık
ölçümde sınırı aşarsa veya parmak teması kaybolursa hareket için durdurma
isteği gönderilir ve raf girişine geçilmez.
Aday ara poza ulaşmak için en çok üç rota denenir. Üçü de reddedilirse başka bir
eklem çözümü denenir. Yalnızca seçilen plan
yürütülür. Sensörden ölçülen kutu–el dönüşümüyle kalan hizalama hatası için
en fazla iki düzeltme yapılır. Küpün en yakın eksenlere paralel dönüşü seçilir.
Açma öncesi eğim sınırı 0.5 derecedir.

Raf konumu karşılaştırması için komuta shelf2_y:=1.15 veya shelf2_y:=1.25
eklenebilir. Varsayılan raf merkezi y=1.20 m'dir. Gazebo ve MoveIt aynı raf
konumunu kullanır. Raf önü, taşıma sınırı ve bırakma hedefi rafla birlikte kayar.
İzole sağ kol denemesi için ayrıca table_only:=true eklenir.

Kutu 120 mm içeri, raf merkezinin 0.16 m önüne (varsayılan y=1.04 m) alınır ve
z=0.4845 m merkez yüksekliğine indirilir. MoveIt zamanlaması bu iniş için
başarısız olursa ham yolun seçilen noktaları arasında sıfır uç hız ve ivmeli
beşinci dereceden geçişler kullanılır. Segment süreleri eklem hız ve ivme
sınırlarına göre hesaplanır. Ara örneklerde çarpışma ve Kartezyen yoldan
sapma kontrol edilir. Aynı son nokta toleransı uygulanır. Bu yedek iniş
daha uzun sürebilir. Burada kapalı sıkma hedefi 0.0198 m'ye yumuşatılır; bu geçişteki
konum değişimi ayrıca loglanır ve 3 mm ile sınırlıdır. Son oturtma en çok
0.5 mm adımlarla yapılır. Pozitif boşlukta son ince adım sürdürülür; temas
olmadan bekleyip açılmaz. Tahmini penetrasyon 0.3 mm'yi aşarsa test durur.

İkinci rafın sekiz fiziksel çarpışması isimlendirilip tek temas sensöründe
izlenir. Gazebo dönüşümünün eklediği sıra ekleri gerçek SDF üzerinden
doğrulanmıştır. Kutu ve raf sensörlerinden gelen güncel temasın özellikle
shelf_2_level_2_collision yüzeyine ait olması gerekir. Düşey normal, 1 mm'den
küçük temas derinliği, 1 mm içinde alt yüzey konumu ve 0.5 derece içinde
yüz eğimi 1 simülasyon saniyesi doğrulanmadan gripper açılmaz. Robotun
rafın herhangi bir çarpışmasına teması hata olarak tutulur.

MoveIt izinleri link düzeyindedir: son oturtmada kutu–shelf_2_link teması
serbest bırakılır; fiziksel kontrol ise yalnızca hedef ikinci katı kabul eder.
Diğer kat/dikme teması, masa teması veya beklenmeyen robot teması başarı
sayılmaz. Robot–raf geometrisi planlamada kontrol edilmeye devam eder.

Açma 0.021 ve ardından 0.04 m tek-parmak hedefleriyle iki kademelidir. Ara
açmadan sonra raf desteği tekrar doğrulanır; açma öncesi konum referansı
yenilenmez. MoveIt attachment kaldırılır ve ölçülen kutu dünya nesnesi olur.
Açık gripper önce 80 mm yükselir, ardından kol 120 mm dışarı çekilir.
Sonraki 3 simülasyon saniyesinde hedef raf
desteği, 3 mm XY hata, 1 mm merkez yükseklik hatası, 1 derece eğim ve
açma öncesinden itibaren 2 mm konum değişimi sınırları kontrol edilir.
Ready dönüşü yapılmaz; kuvvet kontrollü oturtma veya hareket sırasında
kesintisiz otomatik durdurma garantisi yoktur.
