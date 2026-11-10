# Kontrollü yerleştirme (stage:=place)

~~~bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch fr3_cooperative_control single_arm_pick_test.launch.py stage:=place
~~~

Önce eski simülasyonu kapatın; her deneme yeni simülasyonda yapılır. Varsayılan pick
ve ayrı transfer aşamaları korunur. place, önce aynı kavrama ve taşıma doğrulamasını yapar.

Bu aşamaya özel placement.sdf dünyası Contact sistemini yükler. Kutunun col çarpışması
üzerindeki temas sensörü /placement/box_contacts konusuna, masa üstü temas sensörü
/placement/table_contacts konusuna köprülenir. Başlangıçta bu
sensörden veri gelmezse test hareket etmeden durur. Masa üstü çarpışmasının adı
(table_top_collision), Gazebo'nun sabit eklem birleştirme öneklerinden bağımsız ayırt edilir.

Yerleştirme için kutu–el dönüşümü gerçek kutu pozundan yenilenir. Kutu bir küp olduğu
(üç boyut da 40 mm) için farklı bir yüzü üzerine düz oturabilir. Hedef dünya dönüşü
X ekseninde 90 derecedir; parmaklar üstten yaklaşır. Eğim, küp yüz normallerinden
masa normaline en yakın olanıyla ölçülür. Bu seçim farklı boyutlu nesnelere doğrudan uygulanmaz.

Ön yerleştirme merkez yüksekliği 0.50 m'dir. Kol hareket etmeden önce aday eklem
pozundan 0.422 m'ye kadar düz Kartezyen iniş çarpışma ve eklem sıçraması açısından
denetlenir. İniş veya aday poza ulaşan MoveIt rotası doğrulanamazsa başka bir
eklem çözümü denenir. Seçilen yaklaşım hız/ivme ölçeği 0.05 ile yürütülür.
TCP yakınlığı 0.3 mm ve 0.0015 rad koşullarıyla beklenir. Kutu eğimi 0.5 derecenin
altına inmelidir. Gerçek iniş ölçülen kutu ve eklem durumundan tekrar hesaplanır,
hız/ivme ölçeği 0.02 ile yürütülür ve merkez 0.422 m'ye getirilir. Masa yüzeyi
MoveIt world içinde 0.40 m'dir. Gazebo dünya yüksekliği 0.05 m daha fazladır.

Ölçülen durumdan hesaplanan 78 mm iniş yolu yürütmeden önce reddedilirse,
doğrulanan yol yüzdesi veya sıçrayan eklem ve radyan farkı kaydedilir. Kutu
tutulurken masa üstünde farklı bir eklem duruşuna geçiş ve o duruştan iniş
planlanır. En az bir eklemin konumu 0.1 rad farklı olmalıdır. Bu yeniden
deneme bir kez yapılır. İkinci iniş reddedilirse gripper kapalı kalır.
Yürütme hatası veya fiziksel temas halinde yeniden deneme yapılmaz.

Son inişin 2 mm açıklığında, kapalı gripper hedefi 0.018 m yerine 0.0198 m'ye
çekilir. Bu hâlâ kutunun 0.020 m yarı genişliğinden küçüktür; açık çene boşluğu
istenmez. Bu geçişte gerçek kutunun konum değişimi ve alt yüzey aralığı ayrıca
raporlanır; değişim 3 mm'yi aşarsa test durur. Ardından planlanan kutu pozu tekrar
eşitlenir. Son oturtma adımları en çok 0.5 mm'dir. Fiziksel kutunun en alt noktası ölçülen
dönüşten hesaplanır. Tahmini masa penetrasyonu 0.3 mm'den fazlaysa devam edilmez.
Yalnızca planlamadaki kutu–masa teması serbest bırakılır; robot–masa kontrolleri değişmez.
Bu, Gazebo nesnesini sabitlemez veya masaya attach etmez.

Gripper açılması için:
- Kutunun masa üstüyle güncel temas verisi gelmeli; temas normali düşey olmalı.
- Temas derinliği 1 mm'den küçük olmalı.
- Alt yüzey aralığı 1 mm, eğim 0.5 derece sınırında kalmalı.
- Bu koşullar 1 simülasyon saniyesi korunmalı.

Sonra gripper önce 0.021 m tek-parmak açıklığına getirilerek sıkma azaltılır.
Masa desteği yeniden 1 simülasyon saniyesi doğrulanır, sonra 0.04 m tam açıklık
komutu verilir. Açma öncesi konum referansı değiştirilmez; ara açmada oluşan
konum değişimi de son kayma ölçümüne dahil edilir. MoveIt attachment kaldırılır ve ölçülen kutu dünya nesnesi
olarak güncellenir. Kol, gripper tamamen açıkken 120 mm yatay geri çekilir; ready dönüşü yapılmaz.
Bırakma sonrası 3 simülasyon saniyesi boyunca masa teması, 3 mm XY hedef sınırı,
1 mm merkez yükseklik sınırı, 1 derece yüz eğimi ve açma öncesi referanstan 2 mm
konum değişimi sınırı kontrol edilir. PLACE TEST PASSED son başarı işaretidir.

Bu aşama kuvvet kontrollü bir yerleştirme denetleyicisi değildir. Robot–masa üstü
fiziksel teması masa sensöründe bir hata bayrağı olarak tutulur; böyle bir temas varsa
yerleştirme başarılı sayılmaz. Masa sensöründen kutu desteği verisi de gelmelidir.
Robot çarpışmaları ayrıca MoveIt geometrisiyle denetlenir.
Bir koşul sağlanmazsa otomatik açma/kurtarma yapılmaz. Açma sonrasında saptanan
hata kutuyu yeniden kavramayı tetiklemez. Hareket içindeki geçici kayma için sürekli
durdurma denetimi bulunmaz; hareket sonları ve bekleme fazları kontrol edilir.

İlk yatay el yönü denemesinde son inişin çarpışma kontrollü Kartezyen yolu tamamlanmadı;
gripper kapalı kaldı. Üstten yöne düz Kartezyen geçiş de tamamlanmadığından,
yüksekteki yön değişimi genel planlamaya ayrıldı. Yürütülmeyen çarpışmasız IK
probları yalnızca tanı içindir; çarpışma kontrolü kapalı hareket yürütülmez.

## İlk başarılı deneme

Üstten yaklaşma, yüzeyde açma ve yatay geri çekilme içeren ilk deneme geçti:
XY hatası 0.274 mm, açma öncesinden sonraki konum değişimi 0.631 mm, yüz eğimi
0.000 derece. Gripper açılmadan önce alt yüzey aralığı -0.005 mm ve 1 saniyelik
masa teması doğrulandı. Kayıt: log/place_check/place_first_pass.log. Ek masa sensörüyle yapılan dikey geri çekilme denemesinde bırakma sonrası kontrol
başarısız oldu; son sürüm ilk başarılı yatay geri çekilmeyi ve ek masa sensörünü kullanır.

Oturtmadaki hız ölçeği 0.01 ve altındaki adımlarda TOTG ara nokta eleme eşiği
1e-6 rad, yeniden örnekleme aralığı 5 ms ve yol toleransı 1e-5 rad kullanılır.
Bu, 0.5 mm'den küçük son adımların elenmesini önler. Ham ve zamanlanmış son
TCP noktaları raporlanır; ince adımın zamanlanmış uç hatası 0.05 mm'yi aşarsa
yürütülmez. Normal kavrama ve taşıma zamanlama ayarları korunur.

MoveIt Jazzy GetCartesianPath servisi, yanıtını göndermeden önce varsayılan TOTG
zamanlamasını uygular. Bu denemelerde 0.346 mm'lik son adım servis yanıtında
zaten kaybolmuştu; yalnızca istemci zamanlamasını değiştirmek yeterli olmadı.
Son sürüm ince adımları sunucunun tam PlanningScene kopyasında, 0.1 mm örnekleme
ve çarpışma/geçerli eklem sınırı callback'i ile yerel CartesianInterpolator kullanarak
hesaplar. Attached nesne, ACM, dünya geometrisi, octomap ve link padding kopyalanır.
Noktalar arasında 0.02 rad'dan büyük eklem sıçraması kabul edilmez. Normal uzun
hareketler mevcut sunucu yolunu kullanır. İnce adımlar tek kez özel TOTG ile zamanlanır.
Kaynak: https://github.com/moveit/moveit2/blob/jazzy/moveit_ros/move_group/src/default_capabilities/cartesian_path_service_capability.cpp

## Son sürümün ilk doğrulaması

Kapalı hedefi 0.0198 m'ye yumuşatma, yerel ince-adım hesabı, iki kademeli açma ve
masa sensörüyle son sürüm geçti. XY hatası 2.031 mm, açma sonrası konum değişimi
0.003 mm, yüz eğimi 0.000 derece raporlandı. Robot–masa üstü temas bayrağı oluşmadı.
Kapalı hedefi yumuşatma sırasında oluşan hareket SEAT GRIP RELAX satırında ayrı
raporlanır; bu, henüz masa temasından önce ve komut açıklığı küp genişliğinin
altındayken gerçekleşir. Açma referansı bundan sonraki doğrulanmış oturma pozudur.
Kayıt: log/place_check/place_final_pass_1.log.

Aynı son sürümün ikinci temiz simülasyon denemesi de geçti: XY hatası 2.002 mm,
açma sonrası konum değişimi 0.218 mm, yüz eğimi 0.000 derece. Masa desteği
açma öncesinde ve ara açma sonrasında doğrulandı; robot–masa üstü temas bayrağı
oluşmadı. Kutunun merkezi (0.00200, 0.05010, 0.42000) m ölçüldü.
Kayıt: log/place_check/place_final_pass_2.log. Son sürümün iki ardışık başsız
denemesi başarılıdır; bu uzun süreli başarı oranı veya gerçek robot doğrulaması değildir.
Derleme, C++/Python biçim kontrolleri ve git diff --check geçti. Kurulu MoveIt'in
SIGINT kapanış hatası görev tamamlandıktan sonra görülebilir; görev başarısı ve
test düğümünün temiz kapanışı bu kapanış çıktısından önce doğrulanır.

## Kullanıcının beş tekrar denemesi

Aynı sürümle kullanıcı tarafından paylaşılan beş ayrı denemenin tamamında
PLACE TEST PASSED ve temiz test düğümü kapanışı görüldü.

| Deneme | XY hedef hatası (mm) | Açma sonrası konum değişimi (mm) | Yüz eğimi (derece) |
|---|---:|---:|---:|
| 1 | 0.096 | 0.184 | 0.000 |
| 2 | 2.009 | 0.003 | 0.000 |
| 3 | 2.026 | 0.215 | 0.000 |
| 4 | 2.027 | 0.215 | 0.000 |
| 5 | 2.050 | 0.199 | 0.000 |

Beş denemede gözlenen başarı 5/5; en büyük XY hatası 2.050 mm, açma sonrası
konum değişimi 0.215 mm oldu. Her son kutu merkez yüksekliği 0.420 m raporlandı.
Bu örneklemde gözlenen sonuçlar genel başarı oranı garantisi olarak yorumlanmamalıdır.
Yaklaşık 2 mm tekrarlayan hedef ofseti, açma sonrası kaymadan ayrı değerlendirilmelidir.
