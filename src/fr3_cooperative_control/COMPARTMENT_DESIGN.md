# Sekiz bölmeli raf: tasarım ve doğrulama notları

Bu dosya farklı sürümlerin kronolojik çalışma ve test kayıtlarını içerir.
Sonraki bölümlerde güncellenen kapsam ve sonuçlar esas alınmalıdır; farklı
sürümlerin denemeleri tek başarı oranında birleştirilmemelidir.

Bu taslak mevcut Gazebo rafını veya çalışan aktarım komutunu değiştirmez.
`compartment_reach_check.launch.py`, robot açıklamasının ayrı bir kopyasında
iki rafı değiştirir; Gazebo, MoveGroup ve kontrolcü başlatmaz. Hareket komutu göndermez.
Boyut ve hedeflerin tek kaynağı `config/compartment_shelves.json` dosyasıdır.

Güncel görev: Raf 1'in sekiz bölmesi başlangıçta doludur; Raf 2'nin sekiz bölmesi
boştur. Kullanıcı Raf 1'deki kaynak ve Raf 2'deki boş hedef bölmeyi seçer.
Başlangıç doluluğu `initially_occupied_shelves: [1]` ile tanımlanır. Ters yönde
aktarım bu görevin mevcut kapsamına dahil değildir. Önizlemede şimdilik aynı
boyutta küpler kullanılır; farklı şekiller daha sonraki aşamadır.

## Geometri ve adresleme

- Her raf: 4 satır, 2 sütun. Satır 1 en üst, satır 4 en alt.
- Sütun 1 dünya -X tarafı, sütun 2 dünya +X tarafı; karşılıklı raflara bakarken
  görsel sol/sağ tersine dönebilir. Kullanıcı arayüzünde açık etiketlenecek.
- Dış genişlik 0.60 m, derinlik 0.38 m.
- Dış yanlar açıktır: dört köşede 0.025 × 0.025 m kesitli, zemine kadar uzanan dikmeler vardır.
- Orta bölümde ön ve arka kenarda 0.020 × 0.020 m kesitli iki ince dikme vardır;
  orta panel yoktur. Yatay tahta 0.025 m kalınlığındadır.
- `side_style: open_posts` önizleme ve erişim modelinde aynı açık yan yapısını seçer.
- `divider_style: open_posts` orta kısmı da açık tutar; iki sütunun adresleri korunur.
- Her bölmenin net genişliği 0.265 m, net yüksekliği 0.155 m.
- Tahtaların alt yükseklikleri: 0.28, 0.46, 0.64, 0.82 m; çatı altı 1.00 m.
- Taşıma yüzeyleri yukarıdan aşağıya: 0.845, 0.665, 0.485, 0.305 m.
- Raf merkezleri mevcut düzende kalır: y=-1.20 m ve y=1.25 m.
- Kutu merkezlerinin X değerleri -0.1425 ve +0.1425 m; Y değerleri -1.04 ve +1.09 m.
- Bütün değerler MoveIt `world` çerçevesindedir; Gazebo spawn yüksekliği eklenmemiştir.

40 mm küp için tarama merkez yükseklikleri 0.867, 0.687, 0.507 ve 0.327 m'dir.
Bu değerler yüzey üstü +20 mm yarı yükseklik +2 mm tarama açıklığı içerir.
2 mm açıklık yalnızca geometrik tarama içindir; fiziksel bırakma yüksekliği değildir.
Küpün ön kenarı raftan 10 mm içeride başlar. 80 mm çekilme sonunda küpün raf
tarafındaki kenarı raf önünden 30 mm dışarıdadır. Gripper/kol ayrıca çarpışma kontrolünden geçer.

## Kontrol kapsamı

Robot modeli ve KDL IK çözücüsü kullanılır. Diğer kol ready duruşunda sabittir.
Güncel tarama, fiziksel bölmeli raf testindeki gibi her iki kolun joint5
konum sınırlarını -2.60/+2.60 rad alır. Aşağıdaki eski 16/16 sonuçları bu
sınır eşleştirmesinden önceki modele aittir; güncel fiziksel başarı sayılmaz.
Küp gripper'a geometrik olarak bağlıdır; fiziksel sürtünme veya kayma hesaplanmaz.
Küp merkezi TCP'de ve dünya eksenlerine paralel kabul edilir. İki simetrik gripper
yönü (yerel Z ekseni etrafında 0/180 derece), toplam en fazla 32 IK denemesiyle taranır.
Eklem sınırlarından en az 0.05 rad pay, çarpışmalar ve 2 mm yol örnekleri arasında
en fazla 0.05 rad eklem farkı kontrol edilir.
Eski SRDF'deki raf-robot çarpışma istisnaları yalnızca bu özel tarama sahnesinde kaldırılır.

## Gözlemler

1. İlk taslak (yüzey aralıkları 200 mm, 120 mm çekilme): 14/16 yerel yol bulundu.
2. Alt sıra yükseltilmiş taslak (180 mm aralık, 120 mm çekilme): yine 14/16.
   Sol rafın alt iki bölmesinde IK bulundu, ancak çıkış yolu tamamlanamadı.
3. Aynı yükseltilmiş taslak, iki simetrik kavrama yönü ve 80 mm çekilme:
   16/16 yerel kaldırma-çıkış yolu bulundu.

Sonuçlar sınırlı ve rastgele tohumlu bir geometrik aramadır. Yol bulunamaması
ulaşılamazlık ispatı değildir; yol bulunması tam görevin çalıştığını göstermez.

## Henüz doğrulanmayanlar

- Gerçek kuvvet/sürtünme ile kavrama ve parmak dinamiği.
- Farklı şekillerle dolu bölmelerde yaklaşma ve geri çekilme.
- Raf dışındaki pozdan bölmeye genel hareket planı ve masaya geçiş.
- Yerleştirme, yüzey teması, fiziksel kararlılık ve bırakma sonrası gripper çıkışı.
- Farklı nesne şekilleri, boyutları, kütleleri ve sürtünme koşulları.
- Her iki yönde kullanıcı girdisiyle tam aktarım.

Başlangıç yüksekliğindeki 2 mm açıklık ve 40 mm kaldırma yalnızca bu küpün
tarama koşullarıdır; farklı nesnelerde yeniden hesaplanmalıdır.
Bu raf kopyası yalnızca planlama amaçlıdır. Gazebo'ya alınırken temas sensörleri,
collision adları ve fiziksel destek yapısı ayrıca oluşturulacak; eski rafın sensör
adları yeni model için kullanılmayacaktır.

## Çalıştırma

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fr3_cooperative_control compartment_reach_check.launch.py
```

Her satırda `shelf`, `row`, `col`, `ik`, `local_lift_exit` ve seçilen `roll_pi`
değerleri görünür. Başarısız tarama programı kod 2 ile bitirir; bu fiziksel
çarpışma veya simülasyon çökmesi değildir.

İlk taramadan sonraki açık gripper ve komşu nesne kontrolü aşağıda belgelenmiştir.

## Genişletilmiş tarama: açık giriş ve dolu komşular

İlk genişletilmiş tarama, daha zor bir komşuluk kontrolü olarak iki rafta toplam
16 küple yapılmıştır. Güncel başlangıç düzeninde `occupied_cells:=true`,
`initially_occupied_shelves` içindeki rafları doldurur: Raf 1'de sekiz küp,
Raf 2'de sıfır küp. Kaynak taramasında diğer yedi kaynak küp engel olarak tutulur.
Boş hedef bölmelerin yerel geometrisini sınamak için yalnız taranan hedefe geçici
bir test küpü eklenir; bu, Gazebo önizlemesinde hedef rafı doldurmaz ve fiziksel
yerleştirme doğrulaması anlamına gelmez.
Diğer kol ready konumunda kalır. Bu testte raflara ek olarak masa-robot SRDF
istisnaları kaldırılır ve gripper dahil bütün robotun çarpışmaları kontrol edilir.

Bir adayın geçmesi için aynı kavrama duruşunda şu zincirin tamamı gerekir:

1. Parmaklar 40 mm/40 mm açıkken kavrama pozunda çarpışmasız IK.
2. Açık gripper ile bölme dışından 80 mm düz giriş. Aynı IK dalını korumak için
   ters yönde hesaplanan yol geometrik olarak ters çevrilebilir kabul edilir.
   Hedef küp dünyada sabittir; girişte parmak-küp çarpışma istisnası yoktur.
3. Parmakların 40 mm'den 20 mm'ye 2 mm aralıklarla geometrik kapanması.
   Yalnızca hedef küple ilgili iki parmağın teması bu aşamada serbesttir.
4. Hedef küpün planlamada ele bağlanması, 40 mm kaldırma ve 80 mm dışarı çekilme.
   Hedefin dünya nesnesi silinir, başlangıçtaki diğer nesnelerin korunması ayrıca kontrol edilir.

Tam robot çarpışmalarıyla yapılan taramada 16/16 bölme geçti. Bu, fiziksel
kavrama denemesi veya tam görev başarı oranı değildir. Kapanma örneklemesi
kontak kuvveti, sürtünme, nesne sıkışması ve kaymayı hesaplamaz. Küpler hâlâ
yüzeyden 2 mm yukarıdaki tarama pozlarındadır. Giriş/çıkış yolu 2 mm uzaysal
örneklerle denetlenir; sürekli çarpışmasızlık garantisi verilmez.

Yalnız hedef nesneyle karşılaştırma için:

```bash
ros2 launch fr3_cooperative_control compartment_reach_check.launch.py occupied_cells:=false
```

Çıktıda `open_entry`, `closure` ve `local_lift_exit` değerleri ayrıdır.
Her biri en az bir denemede ilgili aşamaya ulaşılmış olduğunu gösterir; bir bölme
ancak tek bir adayda bütün aşamalar tamamlanınca başarılı sayılır.
Mevcut tam aktarım komutu yeni rafı hâlâ kullanmaz.

Sıradaki iş ayrı Gazebo önizlemesi ve tek bölmede gerçek kavrama/yerleştirme
denemesidir. Hedef rafta dolu bölmeye bırakma henüz desteklenmez; kaynak taramasında
bütün bölmelerin dolu olması, dolu hedefe yerleştirme doğrulaması değildir.

## Ayrı Gazebo önizlemesi

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fr3_cooperative_control compartment_preview.launch.py
```

Bu komut mevcut aktarım görevini başlatmaz. Kolların ready duruşu eklem
dönüşümlerine işlenip sabit bir Gazebo modeli oluşturulur; robot kontrolcüleri,
MoveGroup veya kavrama düğümleri çalıştırılmaz. Önizleme kolları hareket ettiremez.
Robot, masa ve raflar aynı world koordinatlarında gösterilir; eski senaryodaki
50 mm spawn ofseti bu bağımsız önizlemede kullanılmaz.

Raf geometrisi ve küp hedefleri çevrimdışı taramayla aynı fonksiyonlardan üretilir.
Gazebo'ya özgü eski kontrolcü/sensör eklentileri oluşturulan kopyadan kaldırılır.
Dış yan paneller yerine dört köşe dikmesi, orta panel yerine ön/arka iki ince dikme kullanılır.
Bu geometriyle kaynak dolu/hedef boş düzeninin yerel giriş-kapanma-çıkış taraması
tekrar 16/16 geçmiştir. Her raf beş yatay tahta ve toplam altı dikme içerir.

Raf 1'de sekiz küp 50 g kütle, 40 mm kenar ve 1.0 sürtünme katsayısıyla dinamiktir.
Raf 2 tamamen boştur. `cells.json` bütün 16 bölmenin adresini ve `occupied`
değerini içerir; boş bölmelerde `model` alanı `null` olur.
Renkler yukarıdan aşağıya mavi, turuncu, yeşil ve mordur. Sütunlar dünya -X/+X
sırasındadır. Gazebo varlık adları örneğin `shelf_1_r2_c1_cube` biçimindedir;
bu ad Raf 1, Satır 2, Sütun 1'i gösterir. Görsel üzerine metin etiketi çizilmez.

Üretilen dosyanın yolu terminalde `PREVIEW WORLD` satırında yazılır. Aynı geçici
dizindeki `cells.json` dosyası nesne adlarını ve adreslerini içerir. Ayrı bir
`GZ_PARTITION` kullanılır; önizleme var olan Gazebo dünya/konularına bağlanmaz.

Doğrulama: SDFormat kontrolü ve kısa arayüzsüz Gazebo çalıştırması geçti.
3000 fizik adımı (3 simülasyon saniyesi) sonrasında Raf 1'deki sekiz küpün tamamı kendi raf
yüzeyinde, beklenen konumdan 1 mm'nin altında sapmayla gözlendi. Bu, kısa bir
oturma kontrolüdür; uzun süreli kararlılık veya robotla kavrama testi değildir.
Grafik arayüzün görüntüsü henüz kullanıcıyla birlikte değerlendirilmedi.
Üretilen dünyada Raf 2'de nesne bulunmadığı ayrıca kontrol edilmiştir.

Arayüzsüz kısa çalışma:

```bash
ros2 launch fr3_cooperative_control compartment_preview.launch.py headless:=true iterations:=3000
```

Önizleme kapatıldıktan sonra eski `table_handoff_test.launch.py` komutu eski
raf düzenini kullanmaya devam eder. Yeni rafın ayrı fiziksel kavrama testi aşağıdadır.

## İlk fiziksel kaynak bölme testi

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fr3_cooperative_control compartment_pick_test.launch.py source_row:=3 source_col:=1
```

Bu deneysel komut Raf 1'de seçilen küpü kavrar ve 40 mm kaldırır. Raf 1'deki
diğer yedi küp dinamiktir; Raf 2 boş kalır. Henüz masaya taşıma, geri çekilme,
hedef bölmeye yerleştirme veya kullanıcı arayüzü yoktur. Başarı sonunda gripper
kapalı kalır; her denemede simülasyon yeniden başlatılmalıdır.

İlk arayüzsüz deneme: Raf 1 / Satır 3 / Sütun 1 geçti.
PHYSICAL LIFT 40.00 mm; 3 simülasyon saniyesi tutmada en büyük göreli kayma
0.008 mm, göreli açı 0.000 derece; COMPARTMENT PICK PASSED.
Bu tek deneme genel başarı oranı veya diğer yedi kaynağın doğrulaması değildir.

Raf geometrisi Gazebo ve MoveIt için aynı fonksiyondan üretilir. Yedi komşu nesne
MoveIt sahnesinde engel olarak bulunur. Fiziksel komşuların bağımsız hareketi
henüz sürekli izlenmediğinden dokunulmadıkları yönünde ayrı bir ölçüm iddiası yoktur.
Eski raf/table SRDF istisnaları yalnızca bu deneysel planlama açıklamasından kaldırılır.

Teknik ayrım: `gazebo_fr3_duo_example.launch.py` dosyasına varsayılan boş olan
`robot_description_file` seçeneği eklendi. Yeni senaryo, yeni rafı içeren geçici
URDF dosyasını bu seçenekle geçirir. Eski komutlar boş varsayılanla mevcut
xacro akışını kullanır. `single_arm_pick_test` için özel `source_xyz`, `pick_lift`
ve `pick_approach` yalnız pick aşamasında kullanılır; varsayılan hedef ve
80 mm kaldırma mevcut senaryolarda korunur.

Gazebo'nun mevcut 50 mm robot spawn yüksekliği bu fiziksel testte bütün nesne
konumlarına eklenir; görev ve planlama koordinatları MoveIt world içinde kalır.
Hedef küpün mevcut temas sensörü ve poz yayıncısı kullanılır. Eski rafın geçersiz
temas sensörü referansları bu geçici kopyadan silinir; yeni bölme başına yüzey
temas sensörleri yerleştirme aşamasında ayrıca eklenecektir.

### Kullanıcının üçüncü satır denemeleri

Her denemede simülasyon yeniden başlatılmıştır; bunlar yalnız kavrama/kaldırma
ve 3 simülasyon saniyesi tutma testleridir.

| Raf 1 bölmesi | Başarı | Kaldırma (mm) | Tutmada en büyük göreli kayma (mm) |
|---|---|---|---|
| Satır 3 / Sütun 1 | 3/3 | 40.00, 39.99, 40.00 | 2.018, 0.010, 0.010 |
| Satır 3 / Sütun 2 | 3/3 | 39.99, 39.99, 39.99 | 2.010, 2.010, 2.010 |

Raporlanan tutma açısı bu altı testte 0.000 derecedir. Önceki arayüzsüz Satır 3 /
Sütun 1 testi bu tabloya dahil değildir. Diğer altı kaynak bölme ve tam aktarım
bu fiziksel testlerle doğrulanmış sayılmaz. Başlangıçtaki `link 'link0' is not unique`
mesajı bu denemeleri durdurmamıştır; model adlandırması ayrıca incelenecektir.

### Satır 2 / Sütun 2: ilk başarısız fiziksel kaldırma

Kullanıcının üç denemesi: 2 başarılı, 1 başarısız.
Başarılı kaldırmalar 40.00 ve 39.99 mm; tutma kaymaları 1.351 ve 1.313 mm'dir.
Üçüncü denemede yaklaşma ve gripper kapanması tamamlanmıştır. Ardından 40 mm
kaldırma yolu yalnız %4.761905 oranında hesaplanmış ve yürütülmemiştir.
Çarpışma denetimi kapalı geometrik tanı da aynı oranda kalmıştır; bu tanı hareketi
çalıştırılmamıştır. Logdaki kavrama hedefinde ilk eklem -2.899846 rad'dır.
Bu durum uygun olmayan bir IK dalı/eklem payının araştırılmasını gerektirir;
fiziksel çarpışma olduğunun kanıtı değildir. Bu bölme henüz güvenilir sayılmaz.
Önerilen düzeltme, hareket başlamadan yaklaşma, kavrama ve kaldırma yolunu
birlikte doğrulayan ve eklem payını koruyan kavrama duruşu seçmektir.

### Bölmeli raf için kavrama ön kontrolü

Özel `source_xyz` kullanılan testlerde yaklaşma hedefi için IK adayları ve
gripper ekseni etrafında 0/180 derece simetrileri denenir. Adayın kabulü için
ön pozdan kavrama pozuna açık giriş, planlama küpü bağlıyken istenen kaldırma
ve ready'den ön poza ulaşan plan kontrol edilir. Eklem sınırlarından en az
0.05 rad pay gerekir. Yerel yol adımı 1 mm, izin verilen komşu eklem farkı
0.02 rad'dır. Yaklaşım planının zamanlanmış noktalarında da sınır payı aranır.

Gerçek giriş ve kaldırma yolları ölçülen durumdan yeniden hesaplanır ve
zamanlanmış noktalarında aynı eklem payını sağlamadan yürütülmez.
Kaldırma yolunun önceden bulunması, fiziksel kavramayı veya her denemede
başarıyı garanti etmez; mevcut fiziksel yükselme ve 3 saniye tutma kontrolleri korunur.
Yeni seçim yalnız bölmeli raf testinde devrededir; özel kaynak parametresi
olmayan eski senaryoların yaklaşım seçim akışı değişmez.

Ön kontrol eklenmiş sürümle Satır 2 / Sütun 2'de birbirinden ayrı üç arayüzsüz
fiziksel deneme yapıldı; üçü de COMPARTMENT PICK PASSED ile tamamlandı.
Üçünde de 0/180 seçeneklerinden ilk aday kabul edildi; giriş ve kaldırma
yolları ön kontrolden geçti. Bunlar önceki 2/3 seriden ayrı yeni sürüm testleridir.
Üç denemelik örneklem genel başarı garantisi değildir.

Ardından kullanıcı aynı Satır 2 / Sütun 2 bölmesinde dört deneme daha yaptı;
dördü de ön kontrolün ilk adayıyla geçti. Kaldırmalar 39.99, 40.00, 40.00,
40.00 mm; 3 saniyelik tutmada göreli kaymalar 2.010, 0.008, 2.010, 2.010 mm;
raporlanan göreli açıların tamamı 0.000 derecedir. Böylece ön kontrollü sürümde
bu bölme için 3 arayüzsüz + 4 kullanıcı denemesi = 7/7 gözlenen başarı vardır.
Diğer bölmelerin eski sürüm sonuçları bu sayıya eklenmez.

Ön kontrollü sürümde Raf 1 / Satır 1 / Sütun 1 için kullanıcının dört denemesi
4/4 başarılıdır. Kaldırmalar 39.99, 40.00, 40.00, 40.00 mm; 3 saniyelik tutmada
göreli kaymalar 0.008, 0.008, 0.012, 1.979 mm'dir. Göreli açı her testte
0.000 derece raporlanmıştır. Dördünde de ön kontrolün ilk adayı kabul edilmiştir.

Ön kontrollü sürümde Raf 1 / Satır 1 / Sütun 2 için kullanıcının üç denemesi
3/3 başarılıdır. Üçünde de kaldırma 40.00 mm, tutmada göreli açı 0.000 derecedir.
Göreli kaymalar sırasıyla 2.000, 1.753 ve 0.007 mm; ön kontrolün ilk adayı
her üçünde de kabul edilmiştir.

Ön kontrollü sürümde Raf 1 / Satır 4 / Sütun 1 için kullanıcının iki denemesinde
de ön kontrol kabul edilen aday bulamamıştır (0/2). Hazırlık duruşundan sonra,
kavrama ve kaldırma başlamadan `No compartment approach with complete entry and
lift and joint margin` hatasıyla durulmuştur. Bu, fiziksel çarpışma kanıtı veya
bölmenin ulaşılamazlık ispatı değildir. Mevcut hata satırı hangi alt kontrolün
adayları elediğini ayırmadığından ek tanı gereklidir. Çevrimdışı yerel yol taraması,
bu gerçek başlangıç durumu ve fiziksel senaryonun daraltılmış planlama sınırlarıyla
aynı test değildir; önceki 16/16 sonucu bu 0/2 sonucun yerine kullanılamaz.

### Alt bölme tanısı ve yükseltilmiş giriş adayı

Raf 1 / Satır 4 / Sütun 1 için arayüzsüz tanı denemesinde 32 adayın
12'si IK, 20'si giriş yolu aşamasında elendi; kaldırma ve genel yaklaşım
planı aşamalarına ulaşılmadı. Girişteki retler eklem sınırı payıyla ilişkiliydi.
Fiziksel testteki joint5 sınırlarıyla eşleştirilmiş geometrik taramada 14/16
yerel yol bulundu; kaynak rafın iki alt bölmesinde düz giriş tamamlanamadı.
Bu sınırlı arama bölmelerin ulaşılamazlık ispatı değildir.

Bölmeli raf kavrama ön kontrolü artık ilk iki eski yaklaşım adayına ek olarak
kavrama pozundan geriye doğru bir alternatif de dener. Bu alternatifte ön poz,
kavrama pozundan yaklaşma mesafesinin yarısı kadar önde ve kaldırma mesafesi
kadar yukarıdadır (mevcut ayarlarla 40 mm önde, 40 mm yukarıda). Gerçek giriş
bu ön pozdan kavramaya eğimlidir. Ters yol, ileri giriş, bağlı nesneyle kaldırma
ve mevcut duruştan ön poza genel plan kabul edilmeden hareket başlamaz.
0.05 rad eklem payı ve 0.02 rad yerel eklem farkı sınırı korunur.

Ön kontroldeki çarpışma sorguları gripper dahil tüm robotu kapsar. Sabit duran
hedef küpün kaynak rafla beklenen destek teması yalnız özel giriş sahnesinde
izinlidir; robotun rafla teması izinli değildir. Küp bağlandıktan sonraki
kaldırma ön kontrolünde küp-raf istisnası kaldırılır. Bu özel sahne değişikliği
canlı planlama sahnesine uygulanmaz. Gerçek giriş ve kaldırma ölçülen durumdan
yeniden hesaplanır; ön kontrol fiziksel başarı garantisi değildir.

Bu son sürümde Raf 1 / Satır 4 / Sütun 1 için üç ayrı arayüzsüz fiziksel
başlangıçta 3/3 kavrama-kaldırma-tutma başarısı gözlendi:

| Deneme | Kabul edilen aday | Kaldırma | 3 sim saniyesinde göreli kayma | Göreli açı |
|---|---:|---:|---:|---:|
| 1 | 16 | 40.01 mm | 0.010 mm | 0.001 derece |
| 2 | 3 | 40.01 mm | 0.010 mm | 0.001 derece |
| 3 | 7 | 40.01 mm | 0.011 mm | 0.001 derece |

Üçünde de yükseltilmiş giriş adayı kabul edilmiştir. Kayıtlar
`log/compartment_preflight_r4c1_support_1.log` ile `_3.log` arasındadır.
Aynı sürümle Satır 2 / Sütun 2 tekrar testi 1/1 geçti: ilk eski yaklaşım adayı,
40.00 mm kaldırma, 2.010 mm göreli kayma, 0.001 derece göreli açı.
Kayıt: `log/compartment_preflight_r2c2_regression_1.log`.
Başarısız geliştirme denemeleri bu yeni sürümün 3/3 sayısına dahil değildir;
önceki kullanıcı sonuçlarının yerine bu sayı kullanılmaz.

Raf ölçüleri ve konumları değiştirilmemiştir. Satır 4 / Sütun 2 fiziksel
kavraması, kaldırılmış nesneyi bölmeden çıkarma ve yeni raflarla tam aktarım
bu sonuçlarla doğrulanmış sayılmaz. Eski sabit kaynak senaryosunun yaklaşım
seçimi bu alternatifle değiştirilmez.

Kullanıcının sonraki dört denemesi de Raf 1 / Satır 4 / Sütun 1 içindir.
Yapıştırılan kayıtların başında bölme seçimi bulunmadığından adres, her logdaki
özel Gazebo dünya dosyasında diğer yedi kaynak küpün listesiyle doğrulandı:
`z_285jlv`, `4azkvpqn`, `gwcvyxs0`, `fie3dhc8` sahnelerinde yalnız r4_c1 ayrı
hedef olarak bırakılmıştır. Dört denemede de `COMPARTMENT PICK PASSED` görülür.

| Kullanıcı denemesi | Aday | Kaldırma | Göreli kayma | Göreli açı |
|---|---:|---:|---:|---:|
| 1 | 20 | 40.01 mm | 0.489 mm | 0.001 derece |
| 2 | 7 | 40.00 mm | 2.010 mm | 0.000 derece |
| 3 | 24 | 40.01 mm | 2.010 mm | 0.001 derece |
| 4 | 11 | 40.00 mm | 2.010 mm | 0.000 derece |

Böylece bu son sürümde r4_c1 için 3 arayüzsüz + 4 kullanıcı denemesiyle 7/7
gözlenen kavrama-kaldırma-tutma başarısı vardır. Uygun aday bulunmadan önce
genel yaklaşım planlarında el/parmak-raf teması bildiren `INVALID_MOTION_PLAN`
retleri de vardır; bu planlar yürütülmemiş, başka adaylar denenmiştir. Bunlar
fiziksel çarpışma kanıtı değildir; plan aramasındaki retler çözülmüş sayılmaz.
İlk başarılı adaylar 7–24 arasında değişir; bütün koşullarda başarı veya sabit
arama süresi garantisi yoktur. Başlangıçtaki yinelenen `link0` ve sensör uyarıları
ayrıca durmaktadır. Bu sonuçlar r4_c2 veya tam aktarım doğrulaması değildir.

### Raf 1 / Satır 4 / Sütun 2: ilk üç kullanıcı denemesi

Üç denemede 2 başarı, 1 başarısızlık gözlendi. İlk kayıtta kaynak adresi açıkça
r4_c2'dir; diğer iki kaydın özel Gazebo dünya dosyaları (`o9rtfy3o`, `s7gm_h42`)
diğer yedi küpü içerir ve seçilen ayrı hedefin r4_c2 olduğunu doğrular.

İlk denemede ön kontrol aday 4 ile geçmiştir. Genel yaklaşım ve bölmeye giriş
yürütmesi kontrolcü tarafından başarılı bildirilmiş, fakat girişten sonraki
ölçülen TCP hedefe oturmamıştır: 35.083 mm konum ve 0.1541 rad yönelim hatası.
joint5 hedefi 0.568970, ölçümü 0.597888 rad; joint6 hedefi 4.419766, ölçümü
4.571101 rad'dır. Diğer beş eklem raporlanan altı ondalıkta hedefleriyle aynıdır.
`waitAt` kontrolü gripper kapanmadan testi durdurmuştur. Bu fiziksel hedef
takibi/kararlılık sorunudur; ön kontrol reddi değildir. Temas, kontrolcü veya
dinamik etkiler arasından kesin neden bu kayıtla belirlenemez.

Sonraki iki deneme de aday 8 ile geçmiştir: 40.01 mm kaldırma, 3 sim saniyesi
tutma, 0.012 mm göreli kayma ve 0.001 derece göreli açı. Bu iki başarı ilk
başarısızlığı geçersiz kılmaz. Bölme henüz güvenilir kabul edilmez; bölmeden
çıkarma veya tam aktarım aşamasına geçmeden giriş sonundaki joint5/joint6
hedef-ölçüm farkı ve fiziksel temaslar araştırılmalıdır. Bu kayıtların analizi
sırasında hareket kodu veya toleranslar değiştirilmemiştir.

### r4_c2 için eklem takibi ve fiziksel temas tanısı

İlk başarısız kullanıcı kaydından sonra ayrı, arayüzsüz bir tanı sahnesinde
kontrolcünün referans/geri bildirim/hata değerleri, ham joint_states ve fiziksel
temaslar kaydedildi. Temas sensörleri yalnız bu sahnenin özel URDF kopyasına
eklenir; geometri, kazançlar ve başarı toleransları değişmez. Kaynak rafın,
sol link5/link6/link7 ve iki parmağın temasları izlenir; diğer bütün robot
parçaları için bağımsız fiziksel temas kapsaması iddia edilmez.

`pick_tracking_1` ve `_2` denemelerinde temas aboneliği veri alamamıştır;
bu iki test temas yokluğunun kanıtı değildir. Temasın topic değeri contact
elemanına da yazıldıktan sonra `_3`, `_4` ve `_seeded_1` kayıtlarında temas
verisi doğrulanmıştır. Üçünde de izlenen robot temasları yalnız gripper
kapandıktan sonraki beklenen parmak-hedef küp temaslarıdır. İzlenen parçalarda
raf teması görülmemiştir; bu gözlem önceki başarısız kullanıcı denemesinin
temas durumunu açıklamaz.

| Tanı kaydı | Kaldırma | Tutma kayması | Açı | Sonuç |
|---|---:|---:|---:|---|
| 1 | 40.01 mm | 0.011 mm | 0.001 derece | geçti; temas verisi yok |
| 2 | 40.00 mm | 0.009 mm | 0.001 derece | geçti; temas verisi yok |
| 3 | 40.00 mm | 2.010 mm | 0.001 derece | geçti |
| 4 | 40.00 mm | 0.009 mm | 0.001 derece | geçti |
| seeded_1 | 40.01 mm | 0.013 mm | 0.001 derece | geçti |
| normal_final | 40.01 mm | 0.010 mm | 0.001 derece | geçti; seeded=0 |

Son testte yalnız özel tanı parametreleri `compartment_grasp_seed` (yedi sonlu
eklem değeri) ve `compartment_grasp_roll_pi` (0/1) ile ilk IK adayı tohumlandı.
Başarısız kullanıcı kaydındaki kavrama eklem hedefleri başlangıç tahmini olarak
kullanıldı; aynı ölçülen küp hedefi için IK yeniden çözüldü. Ortaya çıkan
kavrama hedefleri eski kayda yaklaşık 0.00025 rad içinde yakındır; tam aynı
yörüngenin oynatıldığı iddia edilmez. Bütün eski giriş, kaldırma, çarpışma ve
eklem payı denetimleri korunur. Bu parametreler normal başlatmada verilmez;
varsayılan aday seçimi değişmez.

Altı tanı testinde hata tekrar görülmemiştir. Son test, tanı tohumu parametreleri
verilmeden varsayılan aday seçiminin de çalıştığını doğrulamıştır.
Bu, ilk kullanıcı hatasını çözmüş veya
geçersiz kılmış sayılmaz. Kesin neden belirlenemediği için kontrolcü kazançları,
raf konumu veya toleranslar değiştirilmemiştir. Çalışan varsayılan akışın yerine
bir deneysel düzeltme konmamıştır. Yerel tanı araçları `log/make_pick_diagnostic.py`
ve `log/run_pick_diagnostic.py` altında tutulur; GitHub'a gönderilmez.
İkinci araç `--visible` ile Gazebo arayüzünü açar ve sonucu aldıktan sonra kendi
başlattığı simülasyonu kapatır. Kayıtlar `log/pick_tracking_<etiket>.log` ve
`.jsonl` dosyalarıdır. Yeni kayıt almak önceki fiziksel temas eksikliğini geriye
dönük tamamlamaz. Eski simülasyonlar kapatılarak kullanılmalıdır.

### Gazebo arayüzü açık kullanıcı tanı denemeleri

Paylaşılan üç terminal kaydı r4_c2 tanı testinde başarılıdır. Yerel dosyalarla
sahne adları eşleştirilmiş ve ilgili JSONL kayıtları da incelenmiştir:

| Kayıt etiketi | Paylaşılan kayıt | Kaldırma | Tutma kayması | Açı |
|---|---|---:|---:|---:|
| 20261125_221846 | 1 | 40.01 mm | 0.012 mm | 0.001 derece |
| 20261125_222045 | 2 | 40.00 mm | 2.010 mm | 0.001 derece |
| 20261125_222154 | yalnız yerelde bulunan ek deneme | 40.00 mm | 2.010 mm | 0.000 derece |
| 20261125_222320 | 3 | 40.01 mm | 0.013 mm | 0.001 derece |

Dört yerel kayıtta da temas verisi vardır (sırasıyla 33122, 28093, 28697,
29089 dolu temas mesajı). İzlenen robot temas çiftleri yalnız sol/sağ parmak
ile hedef küptür ve ilk temas gripper kapanması başladıktan sonra görülür.
İzlenen parçalar için gripper/kol-raf teması kaydedilmemiştir. Bu gözlem tüm
robotun her parçasında fiziksel temas olmadığı iddiası değildir.

Gripper kapanmasından hemen önceki son kontrolcü örneğinde joint5 hedef-ölçüm
farkının mutlak değeri en fazla 0.000044 rad, joint6 için en fazla 0.000023
rad'dır. Önceki başarısız denemedeki yaklaşık 0.029 ve 0.151 rad sapmalar
bu kayıtlarda tekrarlanmamıştır. İlk başarısızlık açıklanmış veya giderilmiş
sayılmaz. Tanı sensörleri dışında hareket ayarı/tolerans değişikliği yapılmamıştır.
Özet: `log/user_pick_diagnostics_summary.json`. Bu sonuçlar yalnız kavrama,
40 mm kaldırma ve 3 sim saniyesi tutmayı kapsar; bölmeden çıkışı kapsamaz.

## Ayrı fiziksel bölmeden çıkarma aşaması

`compartment_pick_test.launch.py` artık `test_stage:=pick|withdraw` seçimini
kabul eder. Varsayılan `pick` davranışı değişmez: kavrama, 40 mm kaldırma,
3 sim saniyesi tutma ve kapalı gripper ile durma. `withdraw` buna bölmeden
80 mm +Y yönünde çıkarma ve tekrar 3 sim saniyesi tutmayı ekler; masaya gitmez,
diğer kolu çalıştırmaz ve nesneyi bırakmaz. Geometri değişikliği yapılmamıştır.

```bash
ros2 launch fr3_cooperative_control compartment_pick_test.launch.py \
  source_row:=4 source_col:=2 test_stage:=withdraw
```

Kavramadan önce tek aday için giriş, bağlı küple kaldırma, bağlı küple çıkarma
ve ready'den ön poza genel planın tamamı aranır. Sadece kaldırması mümkün olan
aday bu aşamada yeterli sayılmaz. İlk iki normal yaklaşım adayından sonra,
yeni çıkarma aşaması kavrama pozundan veya dışarıdaki çıkış pozundan geriye
doğru da IK dalı üretir. Varsayılan pick aşamasının aday stratejisi korunur.
Arama en fazla 32 aday ve döngü sınırında 25 saniye ile sınırlıdır; planlayıcı
çağrısının bitişi bu süreyi aşabilir. Yol bulunamaması ulaşılmazlık kanıtı değildir.

Çıkış adayları önce düz, sonra gerekirse raf içinde ek 20/40/60 mm yükselme
ve yatay çıkış olarak sınanır. Her yükseklikte nesneyi yatırmadan dünya Z
ekseni etrafında 0/+15/-15 derece yön değişimi denenebilir. Kaldırmanın ilk
40 mm kısmı aynıdır. Bu ek hareketler yalnız bütün zincir çarpışma ve eklem
denetimlerinden geçerse seçilir. Önceki yalnız düz/eğimli çıkış taslaklarında
alt sağ bölme için aday bulunamadı; bunlar son sürüm başarısı sayılmaz.

Gerçek kavrama ve kaldırma sonrası çıkış, ölçülen eklem durumundan ve gerçek
planlama ekiyle yeniden hesaplanır; ön kontroldeki yol doğrudan oynatılmaz.
Yerel örnek adımı 1 mm, komşu eklem farkı sınırı 0.02 rad, sınır payı 0.05
rad'dır. Zamanlanan çıkışın her noktasında da bütün robot için çarpışma ve
eklem payı kontrolü vardır. Zamanlanmış FK hedef hatası 0.05 mm'den küçük
olmadan yürütülmez; bu fiziksel ölçüm hassasiyeti iddiası değildir.

Taşıma sırasında mevcut örneklemeli kayma/açı ve beklenmeyen küp teması
kontrolleri kullanılır. Hedefte mevcut 3 mm / 0.025 rad oturma denetimi korunur.
Son tutmada göreli kayma 5 mm, açı yaklaşık 5 derece altında olmalıdır.
Ölçülen küp merkezi beklenen merkeze 10 mm içinde olmalı; küpün yönelimine göre
hesaplanan arka kenarı kaynak rafın önünden 10 mm'den fazla dışarı çıkmalıdır.
Nominal 80 mm çekilme, düz küpte yaklaşık 30 mm; 15 derece döndürülmüş küpte
yaklaşık 25.5 mm arka kenar açıklığı verir. İşlem sonunda gripper kapalı kalır.

Eklem ve temas kaydıyla aynı aşama için yerel tanı aracı kullanılabilir:

```bash
python3 log/make_pick_diagnostic.py
python3 log/run_pick_diagnostic.py --visible --withdraw
```

Bu tanı aracı halen yalnız r4_c2 seçer; doğrudan launch komutu bütün kaynak
bölmelerini seçebilir. Normal pick testi için `--withdraw` verilmez.

### Çıkarma doğrulaması ve yaklaşım planı tekrarı

Küçük yatay yön değişimleri eklenmiş sürümde sekiz bölmelik fiziksel taramanın
ilk serisi 7/8 başarılıdır. Alt solda (r4_c1) giriş, kaldırma ve çıkarma için
uygun üç aday bulunmuş, ancak ready'den ön poza genel planların üçü de
`ValidateSolution` tarafından parmak-raf çarpışması nedeniyle reddedilmiştir.
Bu denemede nesne kavranmamış ve hiçbir geçersiz yaklaşım yürütülmemiştir.

Son iyileştirme yalnız `withdraw` modunda, yerel zincirin tamamı uygun olan
bir adaya ulaşmak için en fazla üç farklı genel plan denemeye izin verir.
Çarpışma ve sınır payı denetimleri kaldırılmaz. Normal pick modundaki aday
başına tek genel plan denemesi korunur. İlk alt sol tekrarında aday 6 için
ilk genel plan reddedilmiş, ikinci plan kabul edilmiş ve fiziksel çıkarma
tamamlanmıştır: dy=80.00 mm, arka açıklık=25.50 mm, tutma kayması=0.759 mm,
göreli açı=0.002 derece. Yatay yön değişimi +15 derecedir; ek yükselme yoktur.
Kayıt: `log/compartment_withdraw_retry_r4c1_1.log`.

Başarılı son bölme kayıtları (her satır bir deneme; farklı plan arama
sürümlerindeki sonuçlar tek serilik 8/8 başarı oranı olarak sunulmaz):

| Kaynak | Fiziksel çekilme | Arka kenar açıklığı | Son tutma kayması | Yatay yön değişimi |
|---|---:|---:|---:|---:|
| r1_c1 | 80.00 mm | 29.99 mm | 2.010 mm | 0 derece |
| r1_c2 | 80.00 mm | 29.99 mm | 2.010 mm | 0 derece |
| r2_c1 | 80.00 mm | 30.00 mm | 1.283 mm | 0 derece |
| r2_c2 | 80.00 mm | 30.00 mm | 0.010 mm | 0 derece |
| r3_c1 | 80.52 mm | 26.02 mm | 2.010 mm | -15 derece |
| r3_c2 | 80.00 mm | 30.00 mm | 2.012 mm | 0 derece |
| r4_c1 (yaklaşım tekrarı eklenmiş, ikinci deneme) | 79.48 mm | 24.99 mm | 2.010 mm | +15 derece |
| r4_c2 | 80.00 mm | 25.50 mm | 0.014 mm | -15 derece |

Bu başarılı kayıtlarda ek yükselme seçilmemiştir; ek yükselme seçeneğinin
fiziksel yürütmesi bu seriyle doğrulanmış sayılmaz. Tamamında kaldırma ve
çıkarma sonrası 3 sim saniyelik tutma kontrolü vardır. r4_c2'nin tanı kaydında
47681 dolu temas mesajı incelenmiş; izlenen robot temasları yalnız beklenen
parmak-hedef küp temaslarıdır. Özet: `log/withdraw_final_r4c2_contacts.json`.
Diğer bölmelerin bu serisinde ek fiziksel kol/raf tanı sensörü kullanılmamıştır;
planlama çarpışma ve hedef küpün beklenmeyen taşıma teması kontrolleri aktiftir.

Geçerli son kayıtların özeti `log/withdraw_cells_summary.json` altındadır.
İlk başarısız kullanıcı kavramasındaki 35 mm hedef sapmasının nedeni bu
çalışmayla açıklanmış veya giderilmiş sayılmaz; hedefe oturma denetimi korunur.
Bu aşama masaya taşıma veya iki raf arasında tam aktarım doğrulaması değildir.

Yaklaşım planı tekrarı eklendikten sonra alt soldaki iki ayrı tekrar da geçti
(2/2); ikinci kayıt `log/compartment_withdraw_retry_r4c1_2.log` dosyasıdır.
Ardından `test_stage` verilmeden r2_c2 varsayılan pick tekrar testi geçti:
40.00 mm kaldırma, 3 sim saniyesi tutma, 2.010 mm kayma, 0.000 derece göreli
açı. Bu kayıtta `COMPARTMENT WITHDRAW START` yoktur ve `COMPARTMENT PICK PASSED`
görülür. Kayıt: `log/compartment_preflight_r2c2_withdraw_default_regression.log`.

### Kullanıcının r4_c2 çıkarma tekrarları

Son yaklaşım planı tekrarı sürümünde iki kullanıcı denemesi de
`COMPARTMENT WITHDRAW PASSED` ile bitmiştir (2/2). Kaynak adresi kesilmiş
terminal başlıklarından alınmamış; loglardaki `0q8x8cxo` ve `96j2ney2`
özel dünya dosyalarının diğer yedi kaynak küpü içerip r4_c2'yi ayrı hedef
olarak bırakmasıyla doğrulanmıştır.

| Deneme | Ön kontrol adayı | Kaldırma | Fiziksel çekilme | Arka açıklık | Son tutma kayması | Göreli açı |
|---|---:|---:|---:|---:|---:|---:|
| 1 | 22 | 40.00 mm | 79.99 mm | 25.50 mm | 0.017 mm | 0.002 derece |
| 2 | 9 | 40.01 mm | 80.52 mm | 26.02 mm | 2.010 mm | 0.002 derece |

İkisinde de ek yükselme 0 mm, yatay yön değişimi -15 derecedir. Hareket
sırasında örneklenen en büyük göreli kayma/açı birincide 0.779 mm/0.145 derece,
ikincide 2.159 mm/0.200 derecedir. Son hedefe oturma ve 3 sim saniyesi tutma
kontrolleri geçmiştir. Test sonunda kullanıcı Ctrl+C ile kapatmıştır; Gazebo
ve collision_detected_stub çıkış hataları bu kapanıştan sonradır. Başlangıçtaki
yinelenen link0 uyarısı ayrıca sürmektedir; çözülmüş sayılmaz.

Bu iki terminal kaydına bağımsız ek fiziksel kol/raf temas JSONL kaydı eşlik
etmediğinden bu tekrarlar için yalnız mevcut planlama/taşıma denetimlerinin
geçtiği söylenebilir. Önceki sensörlü testin temas sonucu bu iki denemeye
aktarılmaz. Hareket kodu bu sonuçlar incelenirken değiştirilmemiştir.

### Kullanıcının sonraki 16 çıkarma kaydı

Paylaşılan 16 ayrı kaydın tamamı `COMPARTMENT WITHDRAW PASSED` ile bitmiştir
(bu seri için 16/16). Bütün kayıtlarda kaldırma ve çıkarma sonrası tutma
denetimleri geçmiştir; `SETTLE FAILED`, `SINGLE ARM TEST FAILED` veya
`UNEXPECTED CARRY CONTACT` görülmemiştir. Her kayıt ayrı sonuç zamanına ve
özel Gazebo sahnesine aittir; yinelenen aynı loglar olarak sayılmamıştır.
Başlıklarda kaynak adresi bulunmadığı için bölmeler loglardaki özel dünya
dosyalarının diğer yedi kaynak küp listesi üzerinden doğrulanmıştır.

Bu mesajdaki bölme dağılımı şöyledir:

| Satır | Sütun 1 | Sütun 2 |
|---|---:|---:|
| 1 | 2/2 | 2/2 |
| 2 | 2/2 | 4/4 |
| 3 | 2/2 | 2/2 |
| 4 | 2/2 | bu 16 kayıtta yok |

Dolayısıyla bu mesaj yedi farklı bölmeyi kapsar. r4_c2 için hemen önce
paylaşılan iki başarılı kullanıcı kaydı ayrıca yukarıda belgelenmiştir.
Son yaklaşım tekrarı sürümünde bu iki kayıtla birlikte son 18 kullanıcı
çıkarma denemesinde 18 başarı gözlenmiş ve sekiz bölmenin tamamı kapsanmıştır.
Bu sayı önceki geliştirme başarısızlıklarını, yalnız pick denemelerini veya
arayüzsüz geliştirici testlerini içermez; genel başarı garantisi değildir.

Bu 16 kayıtta fiziksel kaldırma 39.99–40.01 mm, çekilme 79.99–80.52 mm ve
arka kenar açıklığı 26.02–30.00 mm'dir. Son tutmada göreli kayma
0.009–2.011 mm, göreli açı 0.000–0.003 derecedir.

İlk r4_c1 denemesinde ek 60 mm dikey yükselme ve 0 derece yatay dönüş
seçilerek başarıyla yürütülmüştür: çekilme 80.00 mm, arka açıklık 30.00 mm,
son tutma kayması 2.010 mm, göreli açı 0.000 derece. Böylece ilk 40 mm
kaldırmanın üstüne eklenen 60 mm yükselme seçeneği için de bir kullanıcı
fiziksel testi vardır; 20/40 mm ek yükselme seçenekleri bununla doğrulanmış
sayılmaz. İkinci r4_c1 denemesi ek yükselmesiz +15 derece yatay dönüşle
geçmiştir. Diğer 14 kayıtta ek yükselme ve yatay dönüş 0'dır.

Kapanışlardaki Gazebo/stub çıkış hataları bütün kayıtlarda başarılı testten
sonraki kullanıcı Ctrl+C işleminden sonradır. Bunlar test başarısızlığı olarak
sayılmamıştır. Bu seride bağımsız robot/raf temas sensörü kaydı bulunmadığından
fiziksel bütün-robot temas yokluğu iddia edilmez; mevcut planlama ve örneklemeli
taşıma denetimlerinin geçtiği raporlanır. Hareket kodu değiştirilmemiştir.

Dosya-bölme eşleştirmesi ve ölçümler: `log/user_withdraw_16_raw.json` ve
`log/user_withdraw_16_summary.json`. Sonuçlar kavrama–kaldırma–çıkarma içindir;
masaya taşıma ve tam raflar arası aktarım bu kayıtlarla doğrulanmış sayılmaz.

## Yeni bölmelerden mevcut masa akışına bağlantı

`compartment_pick_test.launch.py` için iki yeni isteğe bağlı aşama vardır:

- `test_stage:=transfer`: seçilen bölmeden kavrama–kaldırma–çıkarma,
  ardından mevcut akışla masanın üzerinde elde tutma.
- `test_stage:=place`: aynı zincirin ardından mevcut masa yaklaşması,
  temasla oturtma, iki adımlı açma, gripper geri çekilmesi ve bırakma sonrası
  kararlılık kontrolü.

Varsayılan `test_stage:=pick` ve önceki `test_stage:=withdraw` durma noktaları
korunur. Bölmeden çıkış tamamlanmışsa eski senaryonun 120 mm ek geri çekilmesi
yeniden yapılmaz. Eski sabit raf senaryosunda aynı 120 mm geri çekilme kalır.
Mevcut masaya taşıma ve bırakma fonksiyonları yeniden kullanılır; temas,
oturma, kayma, konum ve eğim toleransları artırılmamıştır. Yeni senaryoya masa
temas köprüsü de eklenmiştir. Masa hedefi yine (0, 0.05, 0.42) m kutu merkezidir.

```bash
ros2 launch fr3_cooperative_control compartment_pick_test.launch.py \
  source_row:=3 source_col:=1 test_stage:=place
```

Başarı için en son `COMPARTMENT PLACE PASSED` görülmelidir. Ara adımlardaki
`PICK TEST PASSED` veya `COMPARTMENT WITHDRAW TEST PASSED` bütünleşik testin
tamamlandığı anlamına gelmez. `transfer` son işareti `COMPARTMENT TRANSFER PASSED`
olup bu aşama gripper'ı açmaz.

Eski taşıma açıklığı denetiminde önceki rafın sabit tahta yükseklikleri ve
dikme konumları vardır. Yeni bölmeli raf senaryosu bunları kullanmaz:
`compartment_shelves.json` geometrisinden üretilen iki rafın bütün 22 kutu
parçası `compartment_shelf_boxes` parametresiyle taşıma denetimine verilir.
Yönelimi hesaba katılmış hedef küpün dünya AABB'siyle her raf parçasının
AABB'si arasındaki alt sınır açıklığı en az 15 mm olmalıdır. Taşıma sırasında
masa izdüşümü üzerinde kutu altı en az 0.43 m, zamanlanmış kol noktalarında
eklem payı en az 0.05 rad olmalıdır. Bunlar MoveIt çarpışma denetimine ek
kabul ölçütleridir. Eski sahnenin açıklık hesabı bu parametre olmadan değişmez.

Yeni sahnedeki hassas masa yaklaşması/inişi için IK ve yerel yol denetimi
gripper dahil bütün robotu ve 0.05 rad kol eklem payını da kontrol eder.
Eski sabit raf senaryosunun hassas yol geçerlilik kuralı korunur. Bırakma
sonrası mevcut eşikler aynıdır: 2 mm'den küçük kayma, 3 mm'den küçük XY
hatası, 1 mm'den küçük merkez yüksekliği hatası, yaklaşık 1 dereceden küçük
eğim ve doğrulanmış masa desteği. Bu çalışma sağ kolu veya hedef rafın yeni
bölmelerine yerleştirmeyi henüz başlatmaz.

### İlk bütünleşik gözlemler

İlk r3_c1 denemesi tamamen geçti: XY=0.107 mm, bırakma kayması=0.189 mm,
son eğim=0.000 derece; kayıt `log/compartment_table_r3c1_place_1.log`.
Takip eden r4_c2 denemesi masa akışına ulaşmadan, bölmeye girişten sonra
gripper kapanmadan hedefe oturma kontrolünde durdu: konum hatası 33.712 mm,
açı 0.1480 rad. joint5 hedef/ölçüm 0.575951/0.593518; joint6
4.448972/4.595928 rad'dır. Diğer beş eklem altı ondalıkta hedefiyle aynıdır.
Bu, önceki tekil kullanıcı giriş hatasına benzer; masa bırakma hatası olarak
sınıflandırılmaz, fakat bütünleşik test başarısızlığı olarak sayılır.
Kayıt: `log/compartment_table_final_r4c2_place.log`.

Fiziksel ve planlama açıklamalarının sol kol eklem origin/axis/limit ve
çarpışma geometrileri karşılaştırıldı; fark bulunmadı. Bu karşılaştırma
kontrol davranışının veya fiziksel temasların eşitliğinin kanıtı değildir.
Kayıt: `log/compartment_model_comparison.txt`. Sapmanın nedeni henüz kesinleşmedi;
bu bağlantı çalışması sırasında kazançlar veya oturma toleransı değiştirilmedi.

### Bütünleşik sekiz bölmelik ilk tarama

İlk bağlantı sürümünde 7/8 deneme masaya bırakma ve elin geri çekilmesiyle
tamamlandı. r4_c2, yukarıda açıklanan kavrama öncesi hedef sapmasıyla durdu.
Bu tablo sonraki erken temas/geri çekilme ön kontrolü düzeltmelerinden öncedir:

| Kaynak | Sonuç | Masa XY hatası | Bırakma kayması | Son eğim |
|---|---|---:|---:|---:|
| r1_c1 | geçti | 1.979 mm | 0.013 mm | 0.000 derece |
| r1_c2 | geçti | 2.026 mm | 0.218 mm | 0.000 derece |
| r2_c1 | geçti | 2.033 mm | 0.213 mm | 0.000 derece |
| r2_c2 | geçti | 2.037 mm | 0.213 mm | 0.000 derece |
| r3_c1 | geçti | 0.107 mm | 0.189 mm | 0.000 derece |
| r3_c2 | geçti | 2.044 mm | 0.218 mm | 0.000 derece |
| r4_c1 | geçti | 2.032 mm | 0.214 mm | 0.000 derece |
| r4_c2 | girişte durdu | — | — | — |

Kayıtlar ve dosya eşleştirmesi `log/table_cells_results.json` ile
`log/table_cells_summary.json` altındadır. Başarılı yedi bırakma, tek başarısız
girişin göz ardı edilmesiyle 8/8 olarak raporlanmaz.

### Erken parmak teması ve temas izninin faza bağlanması

r4_c2 başarısız kaydındaki kavrama eklem hedefleriyle ilk IK adayı tohumlandı.
Bu yakın duruş denemesinde sapma tekrar oluştu: 33.319 mm, 0.1463 rad.
Kontrolcü referans ve çıkışı istenen eklem hedefleriydi; geri bildirimde joint5
yaklaşık 0.0204, joint6 0.1449 rad ayrılmıştı. Temas kaydında gripper henüz
kapanmamışken yalnız sol parmak ile hedef küpün teması vardı. İlk temas
yaklaşık 16.39 saniyede başladı; test kapanma komutuna hiç ulaşamadı.
Diğer robot/nesne temasları bu izlenen parçalarda kaydedilmedi.
Kayıt: `log/pick_tracking_table_failed_seed.log` ve `.jsonl`.

Kodda hedef-parmak çarpışma izni yaklaşma başlamadan açılıyordu. Yeni bölmeli
raf akışında bu izin açık yaklaşma sırasında kapalı tutulur ve ancak hedefe
oturulduktan sonra, gripper kapanmasından hemen önce açılır. Kavrama sonrası
bağlı nesne ön kontrolünde parmak teması yine izinlidir. Eski sabit raf
senaryosunun izin sırası bu değişiklikle değiştirilmez.

Yeni bölmeli raf senaryosunda hem ön yaklaşma hem kavrama noktasına varışta
fiziksel kaynak küpün ilk sakin pozuna göre 2 mm'den az yer değiştirdiği ve
1 dereceden az döndüğü kontrol edilir. Bu yeni bir kaynak kararlılığı
denetimidir; mevcut hedefe oturma toleransları artırılmamıştır. Denetim
başarısızsa gripper kapanmaz.

İzin düzeltmesi sonrası aynı adaydan başlayan denemede iki kaynak kontrolü de
0.000 mm/0.000 derece bildirdi; giriş, taşıma, masa desteği ve açma geçti.
Ancak son yatay geri çekilmenin planlanan joint7 hedefi -3.050833 rad'a,
yani model sınırına ulaştı. Mevcut 0.05 rad pay denetimi bu hareketi yürütmedi.
Bu deneme bütünleşik başarı sayılmaz. Kayıt: `log/pick_tracking_table_contact_gate_seed.log`.

### Masa yaklaşımında bırakma sonrası geri çekilme ön kontrolü

Yalnız yeni bölmeli raf akışında masa yaklaşım adayı seçilirken şu zincir de
aranır: mevcut hassas iniş, son 2 mm oturma, hedef küpün masa üzerinde dünyada
sabitlenmesi, parmakların açık durumunun modellenmesi ve 120 mm geri çekilme.
Hedef küp masa üzerinde kalır; gripper ile taşınıyormuş gibi modellenmez.
Hedef-küp/masa desteği bu özel ön kontrol sahnesinde izinlidir; robot-masa
çarpışmaları ve 0.05 rad kol payı denetlenmeye devam eder. Uygun geri çekilmesi
olmayan aday, gripper açılmadan önce elenir ve başka masa yaklaşımı aranır.
Eski sabit raf senaryosunun yaklaşım seçimi bu ek denetimi kullanmaz.

İlk geliştirme denemesinde ön kontrol sahnesinin kopyalanması `bad_weak_ptr`
hatası verdi; sahne mesajıyla bağımsız kopya oluşturularak düzeltildi. Bu
başarısız geliştirme denemesi başarı sayılmaz. Sonraki aynı başlangıç adaylı
test bütünüyle geçti: XY=0.110 mm, bırakma kayması=0.008 mm, eğim=0.000 derece.
Kayıt: `log/pick_tracking_table_gate_retreat_seed_2.log` ve `.jsonl`.

Bu sonuç, önceki başarısızlıktaki yörüngenin bire bir tekrarlandığı veya bütün
olası sapma nedenlerinin elendiği anlamına gelmez. Aynı yakın başlangıç adayı
ile gözlenen erken temas sorunu, izin düzeltmesi sonrası girişte tekrarlanmamış;
geri çekilme ön kontrolüyle birlikte bütün zincir tamamlanmıştır. Normal aday
seçimi ve eski komutların tekrar kontrolleri ayrıca raporlanmalıdır.

### Son düzeltmelerden sonraki kontroller

Son derlenen sürümde aşağıdaki kontroller tamamlandı:

| Test | Sonuç | Masa XY hatası | Bırakma kayması | Eğim |
|---|---|---:|---:|---:|
| r4_c2, sorunlu giriş adayından tohumlanmış, temas kayıtlı bütünleşik place | geçti | 0.110 mm | 0.008 mm | 0.000 derece |
| r4_c2, normal aday seçimiyle bütünleşik place | geçti | 0.159 mm | 0.041 mm | 0.000 derece |
| Eski sabit raf, eski `single_arm_pick_test.launch.py stage:=place` | geçti | 0.633 mm | 0.210 mm | 0.000 derece |
| Yeni r2_c2, `test_stage` verilmeden varsayılan pick | geçti | — | — | — |
| Yeni r4_c1, withdraw | geçti | — | — | — |

Dosyalar: `log/pick_tracking_table_gate_retreat_seed_2.log`,
`log/compartment_table_gated_normal_r4c2.log`, `log/legacy_place_after_compartment.log`,
`log/compartment_preflight_r2c2_table_gated_pick.log`,
`log/compartment_withdraw_table_gated_withdraw.log`.
Son dört kontrolün çıkış kodları `log/table_regression_results.json` içindedir.
Varsayılan pick, kaldırma-tutma sonrasında; withdraw ise raf dışındaki tutma
sonrasında durmaya devam etmiştir. Eski sabit raf testi bütün bırakma ve geri
çekilme akışını tamamlamıştır.

Temas zamanları karşılaştırması `log/open_contact_comparison.json` içindedir.
Düzeltme öncesi yakın giriş adayında ilk sol parmak/küp teması 16.39 saniyede,
gripper kapanma komutuna ulaşılmadan başlamıştır. İzin düzeltmesi sonrasındaki
iki sensörlü tekrarda ilk temas kapanma komutundan yaklaşık 0.60 ve 0.62 saniye
sonradır; açık yaklaşmada erken parmak/küp teması kaydedilmemiştir. İki kaynak
kararlılığı kontrolü de her tekrarda 0.000 mm/0.000 derece bildirmiştir.
Son başarılı tekrarda kapanma öncesi joint5 ve joint6 hedef-geri bildirim
farkları yaklaşık 0.00000154 ve 0.000000284 rad'dır; ilk başarısız tekrardaki
yaklaşık 0.0204/0.1449 rad sapma bu denemelerde görülmemiştir.

Son kaynak temas izni ve masa geri çekilme ön kontrolü düzeltmelerinden sonra
sekiz bölmenin tamamı bütünleşik place modunda yeniden taranmamıştır. İlk
sekizli seri (7/8) ile sonraki r4_c2 tekrarları farklı sürümlerdir ve tek
serilik 8/8 olarak birleştirilmez. Son sürümün bütünleşik doğrulaması r4_c2
ile, geri uyumluluk doğrulaması ise yukarıdaki üç ayrı kontrolle sınırlıdır.
Başka kaynak bölmelerinden bütünleşik tekrarlar sonraki doğrulama adımıdır.
GitHub'a gönderim yapılmamıştır.

### Son sürümün dört kullanıcı bütünleşik r4_c2 denemesi

Dört kayıt da r4_c2 içindir. Son kayıtta komut ve kaynak adresi açıktır;
ilk üçü `a9hv0hts`, `kz086fuv`, `lxn7unir` özel dünyalarının diğer yedi
küp listesi üzerinden doğrulanmıştır. Son dünya adı `n8jehmvz`'dir.
Bu seride 3 başarılı, 1 başarısız bütünleşik test vardır (3/4).

| Deneme | Sonuç | Ön kontrol adayı | Ek yükselme / yatay dönüş | Masa XY hatası | Bırakma kayması | Son eğim |
|---|---|---:|---|---:|---:|---:|
| 1 | geçti | 9 | 0 mm / -15 derece | 0.123 mm | 0.218 mm | 0.000 derece |
| 2 | geçti | 8 | 40 mm / 0 derece | 2.005 mm | 0.015 mm | 0.000 derece |
| 3 | geçti | 5 | 0 mm / -15 derece | 2.031 mm | 0.213 mm | 0.000 derece |
| 4 | ön kontrolde durdu | bulunamadı | — | — | — | — |

Başarılı üçünde iki kaynak kararlılığı kontrolü de 0.000 mm/0.000 derece
bildirmiş; masa geri çekilme ön kontrolü geçmiş; bırakma öncesi eğimler
0.005, 0.005 ve 0.002 derece olmuştur. İkinci test, ilk 40 mm kaldırmaya ek
40 mm yükselme seçeneği için de fiziksel bütünleşik başarı kaydıdır. Bu,
20 mm ek yükselme seçeneğini doğrulamaz.

Dördüncü kayıtta 32 adayın 5'i IK, 14'ü giriş, 13'ü çıkışla ilgili ön kontrol
veya ters yönden aday üretimi sırasında elenmiştir. Kaldırma, genel yaklaşım
planı veya zamanlanmış marj ret sayıları 0'dır. Kabul edilen aday yoktur;
nesneye yaklaşma ve gripper kapanması başlamadan test durmuştur. Bu sonuç,
bilinen erişilebilir bölmede sınırlı/rastgele aday aramasının her başlangıçta
başarı garantisi vermediğini gösterir. Hedefe oturma sapması veya masa bırakma
başarısızlığı olarak sınıflandırılmaz; bütünleşik başarısızlık sayılır.

Uygun sonraki iyileştirme, aynı bölmede önceki başarılı kavrama/çıkış
duruşlarını ilk IK tohumları olarak denemek ve bütün zinciri güncel sahnede
yeniden doğrulamaktır. Eski yörüngeyi doğrudan yürütmek veya güvenlik
eşiklerini gevşetmek bu önerinin parçası değildir. Rastgele adaylar yedek
arama olarak tutulabilir. Bu kayıtlar incelenirken hareket kodu değiştirilmedi.

## Başarılı kavrama duruşlarını önce deneyen arama

`config/compartment_grasp_seeds.json`, r4_c2 için son başarılı üç kullanıcı
place kaydının kavrama anındaki yedi eklem hedefini ve roll_pi değerini içerir.
Değerler ilk `PICK TARGET SETTLED` satırından önceki giriş yörüngesinin
`TIMED JOINT` hedeflerinden alınmıştır; masa veya geri çekilme sonu eklem
değerleri kullanılmamıştır. Doğrulama kaydı kimlikleri dosyada saklanır.
Kavrama pozundaki eklem payı daha geniş olan iki roll=0 duruşu önce, joint6
payı yaklaşık 0.0503 rad olan roll=1 duruşu yedekte sıralanmıştır.

Bu dosya bir yörünge deposu değildir. Her kayıt yalnız IK başlangıç tahminidir.
Güncel fiziksel küp pozu için IK yeniden çözülür; açık giriş, parmak temas
izni sırası, bağlı nesneyle kaldırma/çıkarma, bütün robot çarpışması, 0.05 rad
eklem payı ve genel yaklaşım planı tekrar denetlenir. Gerçek hareket yolları
ölçülen durumdan yeniden üretilir. Masa desteği, bırakma, geri çekilme ve
bırakma sonrası kararlılık kontrolleri korunur.

Kayıtlar yalnız doğru raf/satır/sütun, nominal kaynak XYZ, küp kenarı,
kaldırma ve yaklaşma/çıkarma mesafeleri eşleştiğinde kullanılır. Kaynak pozu
eşleşmesi için 1 mikrometre, boyut/mesafe sayıları için 1e-9 m veri eşleştirme
toleransı kullanılır; bunlar robotun fiziksel başarı toleransları değildir.
Başka bölmede veya eşleşmeyen bağlamda eski arama çalışır. Dosya yoksa normal
arama kullanılabilir; bozuk biçimli eşleşen bir kayıt yapılandırma hatasıdır.
Joint sırası dosyada açıkça joint1..joint7 olarak belirtilmiştir.

Varsayılan pick-only aşaması kayıtlı duruş kullanmaz. Şimdilik yalnız r4_c2
withdraw/transfer/place aşamalarında üç kayıt vardır. Diğer yedi bölmede
tercihli kayıt yoktur. İsteğe bağlı tanı tohumu verilmişse önce o, sonra
kayıtlı tohumlar denenir. Bunlar elenirse önceki normal aramanın 32 adaylık
sırası korunur. Aynı 25 saniyelik döngü son tarihi bütün aramaya uygulanır;
bu dosyayla normal kullanımda en fazla 3 kayıtlı + 32 normal aday denenebilir.
Normal adayların tamamının son tarihten önce denenmesi garanti değildir.
Başlamış bir planlayıcı çağrısının tamamlanması süreyi aşabilir.

`COMPARTMENT SEARCH START` tercihli aday sayısını; başarılı ön kontrol
`seed_origin=saved|normal|diagnostic`, `seed_index` ve `search_ms` değerlerini
yazar. `search_ms` ön kontrol/IK/ilk yaklaşım planı aramasının duvar süresidir;
tam görev veya fiziksel hareket süresi değildir.

Derlemeden sonra yapılandırma kontrolü, r4_c2'nin üç uygun aşamasında 3 kayıt
yüklendiğini, pick-only/diğer yedi bölme/değişmiş kaynak pozu veya küp boyutunda
kayıtların kullanılmadığını doğrulamıştır. Kontrol dosyası
`log/check_grasp_seed_config.py` altındadır. Sürtünme, kütle veya farklı şekil
genellemesi bu yapılandırma kontrolüyle doğrulanmış sayılmaz.
## Kayıtlı duruşlarla yerel doğrulama (26 Kasım 2026)

r4_c2 için üç bağımsız place ve iki withdraw denemesinin beşi de geçti.
Her denemede ilk kayıtlı IK tahmini kabul edildi; diğer iki yedek kayıt bu
seride fiziksel olarak kullanılmadı. Place denemelerinde XY hatası
2.027–2.029 mm, bırakma kayması 0.215–0.218 mm, son eğim 0.000 dereceydi.
İlk üç place aramasının süresi 1.804, 1.627 ve 2.235 saniyeydi. Bu değerler
tüm görev süresi değildir. Sonuçlar yalnız bu sahne ve bölme içindir;
beş başarılı deneme genel güvenilirlik garantisi oluşturmaz.

Kayıtsız r4_c1 withdraw kontrolünün ilk denemesinde ön plan kabul edildi,
ancak açık yaklaşımda kaynak küp 31.303 mm hareket edip 53.853 derece döndü.
Kaynak hareket kontrolü parmakları kapatmadan testi başarısız sonlandırdı.
Aynı hücrenin bağımsız tekrarında yaklaşımda hareket ölçülmedi ve withdraw
geçti (dy=79.58 mm, arka açıklık=25.09 mm). İki denemede de preferred_seeds=0
ve seed_origin=normal idi. Dolayısıyla normal aramanın devrede olduğu
kontrol edildi; bu hücrede fiziksel yaklaşım güvenilirliği çözülmüş sayılmaz.
Bu hata kayıtlı r4_c2 denemelerinin başarısından ayrı tutulmalıdır.

Varsayılan r2_c2 pick-only kontrolü de kayıtsız normal aramayla geçti.
Kayıtlar log/compartment_table_saved_*.log ve
log/compartment_preflight_r2c2_saved_bank_pick_regression.log altındadır.
Bu çalışma sırasında commit veya GitHub'a gönderim yapılmadı.
## Raf çıkışından masaya geçişin ön kontrolü

Transfer/place aşamasında kavrama adayının kaldırma ve raf çıkışı yoluna ek
olarak masanın üzerindeki hedefe ulaşması da nesne alınmadan önce aranır.
Bu kontrol yalnız source_xyz ile açılan yeni bölmeli sahneyi etkiler;
pick-only, withdraw ve eski sabit raf akışı korunur.

Önce doğrudan bir masa hedefi aranır. Yol bulunamazsa aynı yükseklik ve
bilek yöneliminde +Y yönünde ek 80 veya 160 mm Cartesian uzaklaşma adayları
denenir. Ara yol tamamen erişilebilir ve çarpışmasız değilse kullanılmaz.
Masa hedefi için farklı IK çözümleri denenir. Yollar özel planlama sahnesinde
bağlı küp ve bütün robotla yeniden kontrol edilir; eklem araları en fazla
0.01 rad adımlarla örneklenir. 0.05 rad eklem payı ve mevcut raf/masa açıklığı
kontrolleri korunur. Planlayıcının yol üretmiş olması tek başına yeterli değildir.

Ön kontrolün yörüngeleri hareket için saklanmaz. Yalnız kabul edilmiş masa
IK duruşu, fiziksel kavramadan sonra tekrar hesaplanacak hedef için başlangıç
tahmini olur. Gerçek taşıma yolu ölçülen eklemler ve bağlı nesneyle yeniden
hesaplanır, denetlenir ve mevcut temas/kayma izleyicisiyle yürütülür.
Ön kontrol nominal kavramanın erişilebilirliğini gösterir; gerçek kavrama
kayması veya rastgele planlayıcı araması nedeniyle sonraki başarısızlığı
kesin olarak önleyemez. Yol yoksa hareket başlatılmaz.

Loglarda phase=pregrasp nesne alınmadan önceki denemeyi, phase=measured gerçek
kavramadan sonraki denemeyi belirtir. bridge ek uzaklaşma mesafesidir; sıfır
ise doğrudan global geçiş seçilmiştir. Eski test sonuçları bu değişikliğin
sonuçlarıyla birleştirilmemelidir.

Alt sol (r4_c1) için yalnız transfer/place aşamalarında bir kayıtlı kavrama
başlangıcı eklendi (table_route_place_roll1). Kaynak kayıt, bu değişikliğin
ilk başarılı alt sol tam masaya bırakma denemesidir:
log/compartment_table_route_r4c1_place_2.log. Son PICK TARGET SETTLED öncesindeki
yedi TIMED JOINT hedefi kullanılmıştır. R4_c1 withdraw/pick-only bu kaydı
kullanmaz. İsteğe bağlı stages alanı kayıtların aşama kapsamını sınırlar;
alan yoksa önceki withdraw/transfer/place kapsamı geçerlidir.

Geliştirme sırasında saklanan başarısız denemeler:
- route_r4c1_place_1: ilk sürümde ön kontrol geçti, gerçek kavramadan sonraki
  üç adaylı yeniden arama başarısız oldu. Sonraki sürüm, ön kontrolde bulunan
  masa IK duruşunu yalnız başlangıç tahmini olarak kullanır ve ölçülen durumda
  daha fazla aday dener. Eski yörüngeyi doğrudan çalıştırmaz.
- route_r4c1_place_2: bu güncellemeyle masaya bırakma geçti; XY=0.105 mm,
  release_slip=0.029 mm, tilt=0.000 derece.
- route_r4c1_place_3: kayıtlı alt sol kavrama eklenmeden yapılan bağımsız
  tekrarda 32 normal adayın hiçbiri kavrama/çıkış kontrolünü geçemedi
  (ik=9, entry=15, withdraw=8, table_route=0). Küp alınmadı. Bu kayıt masaya
  geçiş aramasının reddiyle karıştırılmamalıdır.

Son yapılandırmayla bağımsız masaya bırakma kontrolleri:

| Kayıt | Kaynak | Sonuç | XY (mm) | Bırakma kayması (mm) | Son eğim (derece) |
|---|---|---|---:|---:|---:|
| route_seed_r4c1_place_1 | alt sol | geçti | 0.117 | 0.007 | 0.000 |
| route_r4c2_place_1 | alt sağ | geçti | 2.057 | 0.204 | 0.000 |
| route_r1c1_place_1 | üst sol | geçti | 0.264 | 0.045 | 0.000 |

Üçünde de hem pregrasp hem measured geçiş kontrolü geçti; bridge=0 mm
seçildi. Ek 80/160 mm ara uzaklaşma seçeneğinin fiziksel yürütülmesi bu seriyle
doğrulanmış sayılmaz. Alt sağda ikinci kayıtlı kavrama kabul edildi; alt solda
ilk kayıt kullanıldı. Diğer beş hücrede masaya bırakma bu son sürümle yeniden denenmedi.
Bu üç olumlu sonuç, sekiz bölme veya tüm rastgele planlar için garanti değildir.
Seri log/table_route_regression_results.json ile kayıt altına alınmıştır;
geliştirme sırasındaki başarısız denemeler yukarıda ayrı tutulur.

Varsayılan r2_c2 pick-only kontrolü de geçti (table_route_pick_regression).
Bu aşamada preferred_seeds=0 kaldı; kaldırma/tutma akışı korundu.
Derleme, Python sözdizimi, kayıt bağlam/aşama kontrolü ve git diff --check geçti.

## Bırakma sonrası geri çekilme: aynı yol üretimi ve zamanlama

Kullanıcının 16 bölme denemesinin 15'i tam geçti; r2_c2'nin ikinci denemesinde
nesne destek doğrulanarak masaya bırakıldı, ancak açık gripperın 120 mm yatay
çekilmesi zamanlanınca 0.05 rad eklem payı sağlanamadı ve hareket çalıştırılmadı.
Kaynak kayıt: 8d22b916-c407-4bf8-91db-246c861eefd5. Bu deneme tam görev başarısı
değildir; bırakma sonrası kontrol tamamlanmadı.

Önceki ön kontrol checkedFinePath (0.1 mm) kullanırken gerçek hareket linear
üzerinden 2 mm adımlı MoveIt Cartesian yolu kullanıyordu. Yeni bölmeli sahnede
iki aşama da tableRetreatTrajectory fonksiyonuyla aynı hassas interpolasyonu,
aynı 0.10 hız/ivme ölçeğini ve aynı hassas TOTG ayarlarını kullanır. Zamanlanan
bütün durumlar yeniden bütün robot çarpışmasına ve eklem payına karşı kontrol
edilir. Son TCP hedefi için 0.05 mm konum ve 0.025 rad yönelim kontrolü korunur.
Gerçek hareket, gripper açılıp nesne ayrıldıktan sonraki ölçülen eklemler ve
güncel sahneden yeniden hesaplanır. Önceden hesaplanan yol çalıştırılmaz.

Ön kontrolde en az 0.10 rad eklem payı aranır; gerçek hareketin korunan alt
sınırı 0.05 rad'dır. İlave ön kontrol payı, kavrama ve oturtma sırasındaki küçük
farklar için rezervdir; fiziksel başarı garantisi değildir. Yeni sahnede
ölçülen kavrama kaymasına göre hedef düzeltildikten sonra, henüz aşağı inmeden,
iniş + oturtma + açık gripperla geri çekilme yeniden yoklanır. Yoklama
başarısızsa mevcut alternatif masa yaklaşımı denenir; uygunsuz duruşta nesne
bırakılarak devam edilmez. Kavrama, destek doğrulama, açma ve eski sabit raf
akışının geri çekilmesi değiştirilmemiştir.

Hata kaydının son oturtma hedefindeki yedi eklem değeriyle çevrimdışı
tekrar hesaplama (log/retreat_replay): 2 mm yönteminde tam yol üretilmiş,
zamanlanan en düşük joint1 payı yaklaşık 1.69e-8 rad olmuştur. 0.1 mm ve 0.05
rad geçerlilik şartıyla yatay yolun yalnız yaklaşık %64.70'i elde edilmiş,
tam yol olarak kabul edilmemiştir. Bu karşılaştırma eklem sınırı davranışını
gösterir; canlı sahne temaslarının yeniden üretimi değildir. Denenen 20/40/80
mm yükselme seçenekleri de bu duruşta tam yatay yol vermediği için ürüne
alternatif yükselme hareketi eklenmemiştir.

İlk ortak yol üretimi sürümünde retreat_r2c2_1 ve retreat_r2c2_2 tam masaya
bırakma geçti. Bunlar ilave 0.10 rad ön kontrol rezervi ve ölçülen iniş öncesi
yoklama eklenmeden alınmış sonuçlardır; son sürüm sonuçlarından ayrı tutulur.

İlk ortak yöntem sürümünün dört kontrolü de tamamlandı: retreat_r2c2_1,
retreat_r2c2_2, retreat_r2c1_1, retreat_r4c1_1. Son rezerv ve ölçülen iniş öncesi
kontrolü eklenmeden alınmış bu dört sonuç son sürüm serisiyle birleştirilmez.

Son sürümde retreat_final_r2c2_1 ve retreat_final_r2c2_2, hem ölçülen iniş
öncesi kontrol hem de gerçek geri çekilme dahil geçti. Gerçek geri çekilmede
en düşük eklem payları sırasıyla 0.991511 ve 0.668512 rad oldu. Son konum
hataları 2.055 ve 1.604 mm, bırakma kaymaları 0.203 ve 0.004 mm, son eğimler
0.000 dereceydi. İlkinde roll=0, ikincisinde roll=1 kaynak kavraması kullanıldı.

Aynı son sürüm serisindeki retreat_final_r4c1_1 başarısız oldu: kaynak
kavrama/masaya geçiş ön araması aday bulamadı (ik=8, entry=5, withdraw=13,
table_route=4); kaynak küp alınmadı ve geri çekilme aşamasına ulaşılmadı.
Bu sonuç ne başarılı test sayılır ne de geri çekilmenin fiziksel hatasıdır.
Makine kayıtları log/table_retreat_final_results.json içindedir. Alt solun
bağımsız tekrarı ayrı kayıtla tutulur; önceki başarısızlık silinmez.

Alt solun bağımsız tekrarı retreat_final_r4c1_retry, aynı son sürümle tam
masaya bırakma ve geri çekilmeyi geçti. Kavrama adayı 22 (normal, roll=0)
kabul edildi; gerçek geri çekilmenin en düşük eklem payı 0.766710 rad,
son XY hatası 2.045 mm, bırakma kayması 0.205 mm ve son eğim 0.000 dereceydi.
Son sürümdeki sonuçlar: sağ r2_c2 iki başarı; alt sol r4_c1 bir ön arama
başarısızlığı ve bir bağımsız tekrar başarısı. Bu sonuçlar 4/4 başarı olarak
sunulmamalıdır. Sekiz hücrenin tamamı bu son sürümle yeniden doğrulanmadı.
Derleme ve hedeflenmiş çevrimdışı eklem payı kontrolü geçti. GitHub'a gönderim
veya commit yapılmadı.

Kullanıcının son sürümdeki iki r2_c2 denemesi de tam geçti:
9ef431bd-3fef-47ff-976e-690c9ea9ec2b ve
3e94d785-573e-4224-82c8-645386fcedad. Son XY hataları 0.129 ve 0.230 mm,
bırakma kaymaları ikisinde de 0.208 mm, son eğimler 0.000 dereceydi.
Gerçek geri çekilmenin en düşük eklem payları 1.129262 ve 1.129429 rad;
izlenen en yüksek geçici kavrama sapmaları 7.421 ve 10.212 mm idi.
Ölçülen iniş öncesi geri çekilme yoklaması iki denemede de geçti.
Bu sonuçlar r2_c2 doğrulamasına aittir; diğer bölmelerin sonucu sayılmaz.

## Son sürümde kalan altı hücrenin bağımsız masaya bırakma serisi

Kod ve ayarlar seri boyunca değiştirilmedi. Kaynak dosyaların SHA-256 değerleri
log/compartment_remaining_cells_results.json içinde kaydedildi; seri sonunda
yeniden eşleştiği doğrulandı. Her denemede yeni ve bağımsız simülasyon açıldı.

| Kaynak | Sonuç | XY (mm) | Kayma (mm) | Son eğim (derece) |
|---|---|---:|---:|---:|
| r1c1 | geçti | 0.17 | 0.212 | 0 |
| r1c2 | geçti | 0.277 | 0.213 | 0 |
| r2c1 | geçti | 0.131 | 0.209 | 0 |
| r3c1 | geçti | 1.984 | 0.188 | 0 |
| r3c2 | geçti | 1.372 | 0.216 | 0 |
| r4c2 | başarısız | — | — | — |

Bu seri 5/6 tam başarıdır. Başarılı beş denemede ölçülen iniş öncesi geri
çekilme kontrolü ve gerçek geri çekilme tamamlandı. r4_c2 başarısızlığında
kaynak kaldırma ve raftan çıkarma geçti. TABLE TRANSFER MONITOR en yüksek
7.996 mm geçici sapma ve 6.314 derece geçici açı kaydetti; sonraki ABOVE TABLE
tutma kontrolü kayma/dönme sınırı nedeniyle testi durdurdu. Nesne masaya
bırakılmadı ve geri çekilme aşamasına ulaşılmadı. Hata anında hangi tutma
sınırının aşıldığı ve kesin anlık değerler mevcut hata satırında yazmadığı
için 6.314 derece değeri durma anının açısı diye sunulmamalıdır.

r4_c2 sonucu başarılı sayılmaz; önceki sürümlerdeki başarılı alt sağ
denemeleri bu son sürümdeki başarısızlığın yerine konulmaz. İlerlemek için
masa geçişindeki kavrama kararlılığı ayrıca incelenmelidir. Bu seri aynı
sahnede ardışık birden fazla nesnenin taşınmasını veya ikinci rafa yerleştirmeyi
doğrulamaz. GitHub gönderimi ve commit yapılmadı.
## Alt sağdan masaya taşımada kavrama kararlılığı incelemesi

Önce yalnız ölçüm eklendi: HOLD LIMIT EXCEEDED hata satırı hem sapmayı hem
açıyı ve korunmuş sınırlarını yazar. GRIP SAMPLE; ölçümde kullanılan nesne
pozu paketinin simülasyon/zaman yaşını ve gripper açıklıklarını içerir.
Bu yaşlar TCP eklem örnekleriyle zaman eşleme yapıldığı anlamına gelmez.
Ayrıntılı periyodik örnekler compartment_grip_diagnostics ile açılır;
sınır aşımındaki anlık kayıt varsayılan olarak da yazılır.
LEFT TCP RATES, zamanlanmış yörünge noktaları arasındaki FK farklarından
örneklenmiş dönüş/öteleme hızını ve toplam yönelim yolunu raporlar;
sürekli zamanlı bir üst sınır ispatı değildir.

carry_diag_r4c2_1: hareket ayarları aynıyken tam place geçti. Masa geçişinin
örneklenmiş tepe dönüş hızı 0.8308 rad/s, yönelim yol uzunluğu 4.5895 rad
(yaklaşık 263 derece) idi. ABOVE TABLE tutma kontrolü geçti. Önceki 6.314
derecelik hata bu denemede yeniden oluşmadı; bunun nedeni yalnız bu başarılı
tekrardan kesinleştirilemez. Taşıma anındaki konum örneklerinin gecikmesi de
ölçülen geçici sapmaya katkıda bulunabilir.

Yönelimi tamamen koruma deneyi carry_preserve_r4c2_1, nesne alınmadan önce
uygun masa yolu bulamadı. Bu seçenek normal akışa alınmadı ve C++ davranış
seçeneği kaldırıldı; deney kopyası yalnız log altında tutulur. Normal masa
hedefi önceki quaternion ile aynı kaldı. Ortak tableTransferTarget fonksiyonu
ön kontrol ve gerçek hareket hedefini aynı formülle üretir.

Sınırlı taşıma deneyi compartment_table_angular_limit=0.45 rad/s kullanır.
Planın geometrik noktaları değiştirilmez. Örneklenmiş açısal hız bu değeri
geçiyorsa tüm zamanlar tek bir katsayıyla uzatılır; eklem hızları aynı katsayıya,
ivmeleri katsayının karesine bölünür. Hareket hızlandırılmaz. Çarpışma, eklem
payı, destek, kavrama kayması ve açı eşikleri korunur. Bu deneyin yörüngeleri
önceki denemelerle aynı rastgele planlar olmadığı için başarılı sonuçlar tek
başına hız sınırlamasının nedensel etkisini veya hatanın tamamen giderildiğini
kanıtlamaz.

Ayrı deney komutuyla carry_limited_transfer_1 ve carry_limited_transfer_2
geçti: yürütülen örneklenmiş tepe dönüş hızları 0.4500 rad/s, ABOVE TABLE
maksimum açıları 0.009 ve 0.004 dereceydi. Bu iki test masanın üzerinde tutmada
biter; bırakma/geri çekilme başarısı olarak sayılmaz.

carry_limited_place_1 tam geçti: ABOVE TABLE maksimum açı 0.006 derece,
son XY hatası 1.432 mm, bırakma kayması 0.203 mm ve son eğim 0.000 derece.

Başarılı sınırlama yalnız r4_c2 transfer/place için normal launch parametresine
alındı; kullanıcıdan ek komut argümanı gerekmez. Diğer yedi kaynak bölmede,
pick-only/withdraw aşamalarında ve eski sabit raf komutlarında bu sınır etkin
değildir. Ayrıntılı periyodik GRIP SAMPLE normalde kapalıdır; hata anındaki
sayısal kayıt ve faz sonu özetleri korunur. Güncel metadata, sapmayı hesaplamak
için kullanılan nesne pozu paketiyle aynı kilit altında alınır.

carry_final_normal_r4c2, normal komutla kavrama, kaldırma, çıkarma, masaya
getirme, oturtma, bırakma ve geri çekilme dahil geçti. Örneklenmiş tepe dönüş
hızı 0.4500 rad/s, ABOVE TABLE maksimum açı 0.003 dereceydi. Son XY hatası
2.010 mm, bırakma kayması 0.208 mm, son eğim 0.000 derece. Kontrol edilen
kayma/açı/eklem payı eşikleri gevşetilmedi. Bazı yollarda bu ek hız sınırı
yalnız masa geçişinin süresini uzatabilir; farklı rastgele yollar arasında
bu kayıtlardan kesin bir toplam görev hız karşılaştırması yapılamaz.

Sınırlama etkin denemeler: iki transfer-only başarı + iki tam place başarı.
Bu sayılar aynı görev türü gibi toplanmamalıdır; yalnız normal komutla alınan
son kayıt, son launch varsayılanının uçtan uca doğrulamasıdır. Önceki aralıklı
alt sağ hatası sayısal ölçüm eklendikten sonra yeniden oluşmadığından kök neden
kesinleşmiş veya hata tamamen ortadan kalkmış sayılmaz. İkinci rafa aktarım
ve diğer hücrelerde yeni bir genel güvenilirlik serisi bu çalışmada yoktur.
Sonuçlar log/carry_stability_summary.json, kaynak hashleri
log/carry_final_source_sha256.json altında tutulur. Son derleme, Python sözdizimi
ve git diff --check geçti. Çalışma yereldir; commit/GitHub gönderimi yapılmadı.
## Sağ kol: masadan ikinci raf r3_c1 ayrı yerleştirme testi

compartment_right_place.launch.py yeni raf geometrisinde masadan sağ kolla
kavrama ve tek bölmeye yerleştirmeyi sınar. Küp masada başlar; birinci rafta
sekiz küp bulunur ve ikinci raf boştur. Sabit ilk hedef üstten üçüncü satır,
world-X yönünde sol sütundur: MoveIt world (-0.1425, 1.09, 0.505) m.
Taban üst yüzeyi z=0.485 m'dir. 40 mm küp dışındaki şekiller bu test kapsamında
değildir. Bu test birinci raftan başlayarak iki kolun tam aktarımı değildir.

Yeni test, mevcut right_arm_table_pick akışına isteğe bağlı destination_xyz,
destination_board_token ve destination_support_link parametrelerini verir.
Bu parametreler olmadan eski hedef, temas kimliği ve yukarı kalkarak geri
çekilme akışı korunur. Sol kolun compartment_pick_test komutu değiştirilmedi.
Yeni profilde bütün robotla çarpışma ve en az 0.05 rad eklem payı denetlenir;
Cartesian hareketler hassas interpolasyonla üretilir ve zamanlama sonrası
yeniden denetlenir. Yerleştirme için özel taban yüksekliği/XY hedefi kullanılır.

Seçilen tabanın planlama geometrisi destination_support_link adlı sabit linke
ayrılır; rafın diğer on parçası eski shelf_2_link üzerinde kalır. Küp/taban
izinleri yalnız seçilen destek linkine uygulanır. Destek linkiyle kendi raf
çerçevesi arasındaki sabit bileşen kesişimi dışında yeni robot/raf temas izni
eklenmez. Fiziksel destek hem küp hem raf temas sensöründen, doğru taban
kimliği, dik normal, sığ temas ve bir saniye kararlılık ile doğrulanır.

Yeni bölmede üst tabla bulunduğu için eski 80 mm yukarı kalkış kullanılmaz;
açık gripperla 120 mm yatay çıkış, yerleştirme duruşu kabul edilmeden önce
hesaplanır ve gerçek ölçülen durumdan yeniden üretilir. Eski raftaki 80 mm
kalkış, yeni hedef parametreleri verilmediğinde korunur.

Geliştirme kayıtları (başarısızlıklar silinmez):
- compartment_right_pick_1: masadan kavrama ve 79.999 mm kaldırma geçti;
  tutma sapması 0.010 mm, açı 0.000 derece. Bu yalnız pick-only sonucudur.
- compartment_right_place_1: yeni ön kontrolün IK sonrasında güncellenmemiş
  link dönüşümü okunması nedeniyle assertion ile durdu. IK sonrasında state.update
  eklenerek düzeltildi; rafa bırakma gerçekleşmedi.
- compartment_right_place_2: hedef tabana ulaşıldı, ancak Gazebo dönüştürmesinde
  yeniden adlandırılan collision referansları raf sensöründe eşleşmedi. İki
  sensörlü destek doğrulanamadığı için gripper açılmadı. Launch artık gz sdf -p
  çıktısından gerçek collision adlarını alır ve sayılarının eşleşmesini denetler.
- compartment_right_place_3: tam yerleştirme ve yatay geri çekilme geçti;
  XY=2.061 mm, release_slip=0.218 mm, tilt=0.000 derece.
- compartment_right_place_4: bağımsız tekrar geçti; XY=2.063 mm,
  release_slip=0.218 mm, tilt=0.000 derece.

Komut:

```bash
ros2 launch fr3_cooperative_control compartment_right_place.launch.py
```

Yalnız kavrama için test_stage:=pick, görünmez test için headless:=true
kullanılabilir. Başarı işareti COMPARTMENT RIGHT PLACE PASSED'dir. Henüz diğer
yedi hedef bölme veya iki kolun yeni raflarla tam aktarımı doğrulanmış değildir.

Eski sağ kol yerleştirme kontrolü de geçti (right_legacy_after_compartment):
mevcut table_handoff_test.launch.py table_only:=true right_stage:=shelf_place
komutu eski raf geometrisinde XY=2.062 mm, release_slip=0.218 mm,
tilt=0.000 derece ile tamamlandı. Bu kontrol eski sağ kol aşamasına aittir;
eski iki kollu tam aktarımın bu turda yeniden yapıldığı anlamına gelmez.
Sonuç kayıtları log/right_destination_regressions.json içindedir. Yeni hedefin
son iki tam denemesi (place_3, place_4) ve eski sağ kol kontrolü ayrı tutulur.
İlk assertion ve sensör eşleşmesi başarısızlıkları başarı sayısına katılmaz.
Derleme başarılıdır. Çalışma yereldir; commit veya GitHub gönderimi yapılmadı.
## Raf desteği kesildiğinde tanı ve sınırlı yeniden oturtma

Kullanıcının iki yeni sağ kol kaydı ayrı sonuçlardır:
cf4c0a8e-5ec7-42ec-b93d-d353b60bd682, temas başlangıçta görüldüğü halde
kesintisiz destek/paralellik doğrulanamadığı için açmadan durmuştur.
9a73bff6-e1b9-4b2d-8931-bce591321bf8 tam geçmiştir: XY=0.010 mm,
release_slip=0.002 mm, tilt=0.000 derece. Bu seri 1/2 başarıdır.

SHELF SUPPORT CHECK ve başarısızlık özeti artık küp tarafı temas tazeliğini,
raf tarafı temas tazeliğini, 250 ms simülasyon temas yaşını, taban boşluğunu,
eğimi, iyi pencere süresini ve pencere sıfırlanmalarını ayrı kaydeder.
Başarı koşulları değişmedi: her iki kanalda duvar yaşı <1 s, küp temasının
simülasyon yaşı <0.25 s, mutlak taban boşluğu <1 mm, eğim <0.00873 rad ve
kesintisiz bir simülasyon saniyesi destek. Bekleme sınırı yine 15 duvar saniyesidir.

Ayrı yönelim denemesi support_variant1_1, başarısız kullanıcı kaydındaki
variant=1 ile sorunu yeniden üretti. Küp ilk temastan sonra zemin üzerinde
0.2166 mm yukarıda kaldı; eğim 0.0281 dereceydi. 150 örnekte boşluk/eğim
başarı eşikleri hiç aşılmadı fakat destek temasları kesildi; en uzun iyi
pencere 0.366 sim s idi. Bu örnekte açmayı engelleyen koşul gerçek destek
sürekliliğidir; yalnız eğim veya sensör adlandırması değildir.

Yeni bölmeli sahnede, yalnız açmadan önce ve gripper kapalıyken küçük yeniden
oturtmaya izin verilir. Temas yokken, eğim mevcut sınırın altındayken ve
ölçülen pozitif boşluk 0.02–0.30 mm aralığındayken en fazla 0.15 mm aşağı
adım üretilir. Tek yerleştirmede toplam en fazla üç düzeltme vardır. Her adım
ölçülen kutu/elin ilişkisiyle yeniden modellenir; hassas yol, tam robot
çarpışması, 0.05 rad eklem payı ve son nokta doğrulamaları korunur. Zemin
penetrasyonu 0.30 mm'yi aşarsa açma reddedilir. Düzeltme sonrası destek penceresi
sıfırdan başlar. Yalnız kayıp veri veya fazla eğim, düzeltme hareketi başlatmaz.
Gripper kısmen açıldıktan sonra ve eski raf profilinde yeniden oturtma kapalıdır.

support_reseat_variant1_1 aynı yönelimde tam geçti (XY=1.922 mm,
release_slip=0.134 mm, tilt=0.000 derece), fakat küçük boşluk oluşmadığı için
düzeltme adımı kullanılmadı; bu sonuç düzeltme mekanizmasını uygulamalı sınamış
sayılmaz.

Mekanizmayı sınamak için yalnız log/right_support_probe altındaki ayrı C++
kopyasına, açmadan önce kapalı elin 0.220 mm yukarı hareketi eklendi. Bu yapay
hareket üretim C++ kodunda ve normal komutta yoktur. Kopyanın üretim kaynak
hashi yanında tutulur. support_gap_1 denemesi, yapay boşluk oluşturulmadan önce
Right carry hold clock timed out ile durmuştur; destek düzeltme başarısı veya
başarısızlığı olarak yorumlanmaz. Bağımsız tekrar ayrı kayıtla tutulur.

support_gap_2 ayrı test kopyasında kontrollü boşluğu oluşturdu ve düzeltmeyi
uygulamalı sınadı. İlk düzeltme öncesi boşluk 0.2077 mm idi; 0.1500 mm aşağı
adım sonrası kalan boşluk 0.0609 mm oldu. İkinci 0.1109 mm adım iki sensörde
desteği geri getirdi. Sonrasında destek penceresi sıfırdan bir sim saniyesi
geçti, ancak ondan sonra gripper açıldı. Tam bırakma/geri çekilme sonucu:
XY=2.039 mm, release_slip=0.216 mm, tilt=0.000 derece. En büyük gözlenen
kontrollü negatif boşluk 0.0058 mm idi; 0.30 mm penetrasyon sınırı gevşetilmedi.

log/verify_right_support_gap.py; test hareketinin yalnız kopyada olduğunu,
üretim kaynak metninin kopya snapshot hashine uyduğunu, oturtma adımlarının
ölçü ve sayısını, destek doğrulamasının açmadan önce geldiğini ve açıldıktan
sonra hiçbir yeniden oturtma yapılmadığını doğruladı. Bu kontrollü koşul tek
bir hedefte sınanmıştır; tüm rastgele planlar için garanti değildir.

support_normal_1, normal compartment_right_place komutuyla geçti:
XY=2.068 mm, release_slip=0.218 mm, tilt=0.000 derece. Bu denemede yeniden
oturtma gerekmemiştir; düzeltmenin uygulandığı test diye sayılmaz. Ölçüm ve
oturtma kodu aynı üretim sürümündedir. Destek kaybı yeniden oluştuğunda normal
komutun önceden açması yerine kapalı gripperla sınırlı düzeltme yapması amaçlanır.

Eski raf komutu da after_support_reseat kontrolünde geçti: XY=0.069 mm,
release_slip=0.063 mm, tilt=0.000 derece; bu akışta yeniden oturtma çalışmadı.
Son regresyon sonuçları log/right_support_regression_results.json içindedir.
Kontrollü yapay boşluk testi, normal hedef bölme testi ve eski raf testi farklı
amaçlara aittir; aynı koşulda üç tekrar veya genel başarı oranı gibi sunulmaz.
Kullanıcı komutu aynı kalmıştır. Çalışma yereldir; commit ve GitHub gönderimi
yapılmamıştır. Diğer yedi hedef bölme ve yeni raflarla tam iki kollu aktarım
bu değişiklikle ayrıca doğrulanmış sayılmaz.

## Tek komutla iki kollu bolmeli raf aktarimi (2026-11-26)

Yeni baslatma dosyasi: compartment_transfer.launch.py.
Ornek: ros2 launch fr3_cooperative_control compartment_transfer.launch.py
Varsayilan kaynak birinci raf satir 3 sutun 1; hedef ikinci raf satir 3 sutun 1.
Kaynak source_row ve source_col ile secilir. Hedef bu asamada sabittir.

Tek Gazebo/MoveIt sahnesinde ayni target_box sol kol tarafindan alinir,
masaya birakilir, ardindan sag kol tarafindan hedefe tasinir. Secilen kaynak
bolmenin genel kupu uretilmez: 7 komsu kup ve 1 target_box bulunur.
Sag kol yalnizca sol gorev sifir cikis koduyla tamamlandiktan sonra baslar;
mevcut sol kol park kontrolu sag kol asamasinda korunur. Tam basari bildirimi
sag kol asamasi da basariyla bittiginde uretilir. Bu entegrasyonda C++ hareket
kodlari degistirilmemistir; ayri sol/sag kol komutlari korunmustur.

Bastan sona iki fizik simulasyonu denemesi gecti:
- Kaynak r3c1 -> hedef r3c1: XY 1.921 mm, birakma kaymasi 0.130 mm, egim 0.000 derece.
- Kaynak r4c2 -> hedef r3c1: XY 1.786 mm, birakma kaymasi 0.028 mm, egim 0.000 derece.

Loglarda sol basarinin sag surec baslangicindan once geldigi, sag fiziksel
raflama basarisinin tam basari bildiriminden once geldigi ve secilen kaynakta
ikinci bir kup bulunmadigi ayrica dogrulandi. Ozet: log/compartment_full_summary.json.
R4c2 icin mevcut 0.45 rad/s tasima acisal hiz siniri etkin kalmistir.

Iki ayri mantik testi, gecici sahne kopyalarinda sahte gorev cikislariyla
sol hata halinde sagin baslamadigini ve sag hata halinde tam basari
bildirilmedigini dogruladi. Bunlar fiziksel aktarim tekrari olarak sayilmaz.
Ayrica eski ayri sag kol masa kavrama komutu gecti: kaldirma 79.997 mm,
kayma 0.010 mm, aci 0.000 derece. Kontrol raporu:
log/compartment_transfer_checks.json.

Iki tam deneme tum kaynak/hedef kombinasyonlarinin dogrulandigi anlamina gelmez.
Diger hedef bolmeler bu komutta henuz secilebilir degildir. Calisma yereldir;
commit veya GitHub gonderimi yapilmamistir.

## Masaya gecis on kontrol yolunun korunmasi (2026-11-26)

Kullanicinin iki yeni denemesinden biri tam gecti (hedef XY 2.077 mm,
kayma 0.218 mm, egim 0 derece); digeri sol raftan cikistan sonra masaya
guvenli yol bulamadi. Ikinci logda phase=pregrasp basarili oldugu halde
phase=measured aramasi basarisizdi. On kontrol zaten masaya devam yolunu
sinamaktaydi; eksik olan bu yolun korunmasiydi. Yalnizca hedef eklem tohumu
saklaniyor, yol geometrisi atilip gercek kavramadan sonra tekrar araniyordu.

single_arm_pick_test.cpp artik kabul edilen kavrama adayinin masaya gecis
planlarini da saklar. Gercek kavramadan ve raftan cikistan sonra bu geometri
ilk secenek olarak guncel durumla yeniden zamanlanir ve dogrulanir:
- Her bacagin ilk durusu ile guncel baslangic farki eklem basina en cok 0.03 rad.
- Guncel baslangic da yola eklenir; baglanti dahil ara eklem durumlari 0.01 rad
  ornekleme ile tam sahnede kontrol edilir. 0.05 rad eklem payi korunur.
- Olculen cisim/TCP donusumu ile raf acikligi ve masa yuksekligi kontrol edilir.
- Son TCP hedef hatasi 3 mm ve aci farki 0.025 rad altinda olmalidir.
- Varsa mevcut tasima acisal hiz siniri yeni zamanlama sonrasinda tekrar uygulanir.

Bir kontrol gecmezse kayitli yol kullanilmaz; mevcut olculen durumdan plan
arama akisi devam eder. Iki yol da uygun degilse hareket reddedilir. Sag kol,
kavrama ve birakma ayarlari bu duzeltmede degistirilmemistir. Bu mekanizma
basarisiz denemenin aynisini kesin olarak tekrar urettigimiz anlamina gelmez;
saklanan yolun yeniden aramada kaybedilmesini onlemeyi amaclar.

Bu duzeltmenin fizik simulasyonu sonuclari:
- route_reuse_default_1 (r3c1): kayitli yol yeniden dogrulandi, TCP hedef farki
  0.031 mm. Sol masa birakma XY 0.131 mm, kayma 0.209 mm, egim 0 derece.
  Tam aktarim gecti: sag hedef XY 2.057 mm, kayma 0.218 mm, egim 0 derece.
- route_reuse_low_1 (r4c2): kayitli yol yeniden dogrulandi, TCP hedef farki
  0.021 mm; 0.45 rad/s acisal sinir tekrar uygulandi. Sol masa birakma XY
  1.826 mm, kayma 0.184 mm, egim 0 derece. Sag kol masa kavramasindan sonra
  hedef rafa carpisma kontrollu iki asamali yaklasma yolu bulamadi; tam aktarim
  BASARISIZDIR. Sol gecis duzeltmesi bu sag planlama sorununu cozmus sayilmaz.

Loglar: log/compartment_full_route_reuse_default_1.log ve
log/compartment_full_route_reuse_low_1.log. Bu iki denemede yeniden kullanim
kabul edildi; yeniden kullanim reddi ve yedek arama dali bu testlerde
calistirilmis sayilmaz. Derleme ve git diff --check gecti. Degisiklik yereldir;
GitHub gonderimi yapilmadi. Genel guvenilirlik icin sag raf yaklasmasindaki
plan arama basarisizligi halen aciktir.

## Sag raf donus aramasina cisim merkezi bolgesi (2026-11-26)

Onceki r4c2 tam aktariminda sol masa birakma gecti, ancak sag raf aramasi
iki tur sonunda basarisizdi. Loglarda cok sayida donus adayi, cisim merkezinin
rack_y - 0.40 sinirini asmasi nedeniyle son kontrolde eleniyordu. Yeni raf
profilinde dogrudan eklem donusu uygun degilse OMPL yedek aramasina da bir
konum bolgesi verilir. Hedef nokta, olculen eldeki cisim merkezidir:
y <= rack_y - 0.40 ve z >= 0.48 + 0.02*sqrt(3). Son kosul donen 40 mm kupun
en buyuk yarim yuksekligini kullanir. Son carpisma, 0.05 rad eklem payi,
cisim alt yuksekligi ve y siniri kontrolleri korunur. Bolge yalnizca yedek
arama boyunca etkindir, ardindan temizlenir. Eski raf profili ve mevcut
gecerli dogrudan donus secimi aynidir.

Ilk uygulama hatasi ve duzeltmesi: Konum kisiti gruba dahil olmayan TCP
baglantisina verildiginde MoveIt -11 ile kapandi. Bu test BASARISIZDIR
(log/compartment_full_right_region_low_1.log); eski denemenin kalan surecleri
kapatildi. Son surum kisiti arm_2 icindeki right_fr3v2_link8 baglantisina
verir, cisim merkezi ofsetini TCP'den bu baglantiya donusturur ve baglantinin
grupta oldugunu once dogrular. TCP uzerinden kisit veren uygulama korunmadi.

Kontrollu mekanizma testi: log/right_turn_probe altindaki ayri C++ kopyasinda
sadece dogrudan donus secimi kapatildi; uretim koduna bu degisiklik eklenmedi.
Boylece kisitli yedek arama zorunlu olarak calisti ve fiziksel raf yerlesimi
gecti: XY 2.006 mm, release_slip 0.030 mm, tilt 0.000 derece. Iki sensorle
raf destegi acmadan once dogrulandi. Log:
log/compartment_right_turn_region_forced_1.log. Bu tek kollu kontrollu deneme,
tam aktarim veya ayni kosulda normal tekrar olarak sayilmaz.
log/verify_right_turn_region.py kaynak karmasini, test kopyasinin tek farkini,
yedek aramanin kullanildigini ve fiziksel basari bildirimini denetler.

Asil surum tam aktarim testi: right_region_fixed_low_1, kaynak r4c2 ->
hedef raf 2 r3c1, GECTI. Sol masa birakma XY 0.281 mm, kayma 0.199 mm,
egim 0 derece. Sag son yerlesim XY 0.012 mm, kayma 0.003 mm, egim 0 derece.
Log: log/compartment_full_right_region_fixed_low_1.log. Bu denemede mevcut
dogrudan donus kabul edildi; yeni yedek dalin calistigi deneme olarak sayilmaz.
Yeni dal yalnizca yukaridaki kontrollu testte zorunlu olarak sinanmistir.
Derleme, test kopyasi kaynak dogrulamasi ve git diff --check gecti. Ilk hatali
kisit denemesinin kapanmasi kayitlarda korunmustur; son iki fiziksel denemede
MoveIt kapanmasi gorulmedi. Bu sonuclar uzun tekrar serisinin veya tum bolme
kombinasyonlarinin basari garantisi degildir. Yerel degisikliklerdir, commit
veya GitHub gonderimi yapilmamistir. Kullanici tam aktarim komutu degismedi.

## Iki parcali masa gecisinde guncel baslangic (2026-11-26)

Kullanicinin son 10 denemesi: 9 tam basari, 1 basarisizlik (7. deneme).
Basarili 9 denemede son XY ortalama 1.129 mm, en buyuk 2.056 mm; en buyuk
birakma kaymasi 0.219 mm, son egimlerin hepsi 0 derece. Bu, yalnizca bu
serinin gozlenen sonucudur. Yeni sag yedek arama 3., 8. ve 9. denemelerde
calisarak tam aktarimi tamamladi.

7. denemede sol TABLE TRANSFER ilk bacagi gecti. Ikinci bacagin baslangici
left_fr3v2_joint2 icin MoveIt'in 0.01 rad toleransini asiyordu; ikinci hareket
baslatilmadan ABORTED oldu. Bu hata fiziksel carpisma olarak yorumlanmaz.

Duzeltme yalnizca source_xyz akisi icindeki cok bacagli masa rotasinin devam
bacaklarina uygulanir. Ilk bacaktan sonra waitAt(nullptr) ile durulma beklenir,
guncel sahne ve olculen sol kol durusu alinir. Beklenen baslangictan fark
0.03 rad'i asarsa hareket reddedilir; bu esik MoveIt toleransini degistirmez.
Kayitli ilk nokta olculen durusla degistirilir, kalan yol yeniden zamanlanir.
Zamanlama sonrasi ilk nokta olculen durustan eklem basina 0.001 rad'dan az
farkli olmalidir. Son TCP konumu 0.05 mm, acisi 0.001 rad icinde korunur.
Yeni baglanti dahil tum yol guncel sahnede 0.01 rad ara durumlarla carpisma,
0.05 rad eklem payi, cisim/raf acikligi ve masa yuksekligi acisindan tekrar
sinanir. Mevcut acisal hiz siniri yeniden uygulanir. Sag kol kodu, tek bacagli
gecis ve kontrolcu toleransi bu duzeltmede degismemistir.

Ilk kontrollu denemede eski ilk noktaya geri donus ekleyen zamanlama,
TOTG negatif yol hizi hatasiyla reddedildi (continuation_forced_1); ikinci
hareket baslamadi. Bu uygulama korunmadi. Son surum eski ilk noktayi ziyaret
etmek yerine guncel durusla degistirir ve mevcut normal tasima zamanlama
ayarlarini kullanir; sonuc yine sikica dogrulanir.

Kontrollu test kopyasi log/continuation_probe altindadir. Yalnizca bu kopyada
iki bacagli rota zorunlu ve ikinci bacagin eski ilk joint2 noktasi +0.015 rad
kaydirilmistir. Uretim koduna bu test mudahalesi eklenmemistir.

Son kontrollu test continuation_forced_2 tam aktarimi GECTI. Iki bacagli
rota zorunluydu; ikinci baslangic farki 0.014945 rad olarak olculdu ve
COMPARTMENT CONTINUATION SYNC PASSED bildirimi uretildi. Sol masaya birakma
XY 1.397 mm, kayma 0.003 mm, egim 0 derece; sag hedef XY 1.926 mm,
kayma 0.134 mm, egim 0 derece. Onceki 0.01 rad baslangic reddi gorulmedi.
Log: log/compartment_full_continuation_forced_2.log. Bu, 7. denemenin birebir
tekrari degil, ayni baslangic uyusmazligi turunun kontrollu sinamasidir.

Uretim surumuyle normal kontrol continuation_normal_1 de tam GECTI
(kaynak r4c2 -> raf 2 r3c1). Sol masa XY 2.027 mm, kayma 0.218 mm,
egim 0 derece; sag hedef XY 2.076 mm, kayma 0.218 mm, egim 0 derece.
Bu denemede tek bacagli rota secildi; ikinci-parca kontrolunu sinamis sayilmaz.
Log: log/compartment_full_continuation_normal_1.log.

log/verify_continuation_sync.py kaynak karmasini, test kopyasinin yalnizca
belirtilen mudahaleleri icerdigini, kontrollu farkin 0.01 rad ustunde oldugunu,
her iki tam basariyi ve baslangic reddinin olmadigini dogrular. Ozet:
log/continuation_sync_checks.json. Ilk kontrollu zamanlama basarisizligi da
kayitlarda tutulmustur. Bu iki sonuc genel basari garantisi veya yeni bir
10 tekrar serisi degildir. Derleme ve son bicim kontrolu tamamlanmistir.
Degisiklikler yereldir; branch, commit veya GitHub gonderimi yapilmamistir.
Kullanici tam aktarim komutu aynidir.

## Sag rafta temas ile kucuk pozitif boslugun birlikte gorulmesi (2026-11-26)

Kullanicinin son uc denemesi 1 tam basari ve 2 farkli basarisizlik iceriyor.
Ikinci denemede sag SHELF SEAT, temas=1 oldugu halde gap=0.253..0.265 mm
arasinda kaldi ve 10 adim sonunda destek kurulamadigi gerekcesiyle durdu.
Ucuncu deneme sol kavrama on kontrolunde durdu; bu degisiklik onu cozmez.

Kodda sag oturtma dongusu gap<=0 olmadan surekli destek kontrolune
gecemiyordu. Arkasindaki waitShelfSupport zaten iki fizik sensorunun taze
ve ayni simulasyon zamanina ait temasini, |gap|<1 mm, egim<0.00873 rad
ve 1 kesintisiz simulasyon saniyesini birlikte ister. Son testte bu kontrol
hic cagrilmamisti. Pozitif geometrik boslugun fiziksel kaynagi yalnizca bu
loglardan kesinlestirilemez; temas/poz geometri farki ile sureksiz kose
temasini ayirmak icin kararlilik ve iki sensor birlikte kullanilmalidir.

Yeni raf profilinde temas=1 ve 0<gap<=0.3 mm ise kapali gripperla mevcut
waitShelfSupport(true) kontrolune gecilir. Giriste egim ve penetrasyon
kontrolleri korunur. Kontrol basarisizsa gripper acilmaz. Destek dogrulaninca
acmadan once tekrar kontrol, yumusak acma sonrasinda destek kontrolu ve tam
birakmadan sonra 3 simulasyon saniyelik konum/kayma/egim/destek kontrolu aynen
korunur. Desteksiz pozitif bosluk yeni dala girmez. Eski raf profili, sol kol
ve sensor dogrulama esikleri bu duzeltmede degistirilmemistir.

Kontrollu test log/shelf_contact_probe altindaki ayri kopyayi kullanir. Gercek
fiziksel temas ilk goruldugunde yalnizca bottom() olcumune sabit bir hesap
farki eklenerek tahmini bosluk 0.260 mm yapilir. Sensor mesajlari, fiziksel
obje ve raf konumu degismez. Bu bir olcum/karar dali testidir; fiziksel olarak
havada asili nesne veya kullanici denemesinin birebir tekrari degildir.
Test mudahalesi uretim kodunda yoktur.

Kontrollu contact_window_forced_1 sag kol testi GECTI. Fiziksel geometrik
bosluk -0.0063 mm iken yalnizca hesaplanan bosluk 0.2600 mm olarak ayarlandi.
SHELF CONTACT WINDOW START calisti; acmadan once iki ayri 1 simulasyon
saniyelik destek dogrulamasi ve yumusak acmadan sonra bir dogrulama gecti.
Son hedef XY 0.015 mm, release_slip 0.006 mm, tilt 0 derece. Log:
log/compartment_right_contact_window_forced_1.log.

Test mudahalesi olmayan uretim surumunde contact_window_normal_1 sag kol
masa kavrama -> raf yerlestirme -> geri cekilme testi de GECTI. Son XY
2.017 mm, release_slip 0.216 mm, tilt 0 derece. Bu denemede bosluk negatife
indi, yeni pozitif-bosluk dali gerekmedi. Log:
log/compartment_right_contact_window_normal_1.log.

log/verify_shelf_contact_window.py kaynak karmasini, test kopyasindaki
olcum farkinin tek mudahale oldugunu, acmadan onceki destek sirasini ve iki
fiziksel sag kol sonucunu denetler. Ozet: log/shelf_contact_window_checks.json.
Bu ikisi ayri sag kol testleridir; iki kollu tam aktarim tekrarlari olarak
sayilmaz. Son degisiklikle yeni tam aktarim serisi yapilmamistir. Sol kolun
kavrama on kontrolunde plan bulamama sorunu halen aciktir. Degisiklikler
yereldir; commit/branch/GitHub islemi yapilmamistir.

## Sol kavrama oncesi yaklasmaya ara durus (2026-11-26)

Kullanicinin 180c4417-69af-4391-8c71-ab5b694a3b1e logunda tek kayitli
kavrama adayi kaldirma, raftan cikis ve masaya devam on kontrolunu gecti.
Ancak bu adayin acik gripperla on kavrama durusuna ulasma plani uc kez
ValidateSolution tarafindan elendi: el veya parmak rafla temas ediyordu.
Aday elendikten sonra rastgele kavrama aramasi da sonuc bulamadi. Bu nedenle
sorun yalnizca kavrama pozunun IK'si degil, o poza yaklasma yoludur.

single_arm_pick_test.cpp mevcut dogrudan yaklasma aramasini korur. Bu arama
basarisizsa ve bolmeli raftan cikis etkinse, ayni kavrama dali icin ara durus
aranir. On kavrama durusundan disariya dogru ters Cartesian yol hesaplanir;
kol once bu dis durusa planlanir ve ters yol geri izlenerek on kavramaya
ulasilir. 20 mm kisa adaylar once, daha uzun/yukseltilmis adaylar sonra
sinanir. Butce mevcut 25 saniyelik kavrama arama suresi icindedir; kesin
suresi tekil planlayici cagrisinin donus suresine de baglidir.

Birlesik yol yeniden zamanlandiktan sonra tam sahnede en cok 0.01 rad eklem
aralikli ara durumlarla carpisma ve 0.05 rad eklem payi kontrol edilir.
Son TCP on kavrama konumundan 0.05 mm ve 0.0015 rad icinde olmalidir.
Kavrama oncesi fiziksel cismin yerinden oynamama kontrolu, kavrama/kaldirma,
olculen baglanti ile masa gecisi ve sag kol kontrolleri korunur. Ortak eklem
yolu dogrulamasi ayri fonksiyona alinmistir; tasima icin mevcut raf acikligi
ve masa yuksekligi kontrolleri bunun ardindan aynen uygulanir.

Kontrollu test kopyasi log/staged_approach_probe altindadir; bu kopyada sadece
normal yaklasma deneme sayisi 0 yapilarak yeni dal zorunlu calistirilir.
Carpisma veya eklem kontrolleri kapatilmaz. Ilk iki kontrollu deneme,
40 mm ve daha uzun ara duruslarda uygun yol bulamadan durdu
(staged_approach_forced_1 ve staged_approach_forced_2). Kayitlari korunmustur.
Son surum 20 mm adaylari da icerir. Test mudahalesi uretim kodunda yoktur.

staged_approach_forced_3 kontrollu tam aktarimi GECTI. Yeni ara yaklasma
20 mm dis mesafeyle kabul edildi; rastgele aday 27 secildi (kavrama on kontrolu
21.65 saniye). Acik yaklasma ve kavrama oncesinde kaynak kup hareketi 0 mm
olarak dogrulandi. Sol masaya birakma XY 0.088 mm, kayma 0.005 mm,
egim 0 derece. Sag hedef XY 1.920 mm, kayma 0.135 mm, egim 0 derece.
Log: log/compartment_full_staged_approach_forced_3.log. Bu, kullanicinin
basarisizliginin birebir tekrari degil, yedek yaklasmanin zorunlu sinamasidir.

Ilk normal kontrol staged_approach_normal_1 BASARISIZDI. Bu kez yaklasma
asamasina gelen aday olmadi: diag ik=10, entry=4, withdraw=11, table_route=5,
route=0. Yeni yedek yaklasma bu daha erken elemeleri cozmus sayilmaz.
Kayit log/compartment_full_staged_approach_normal_1.log icindedir.

Ek olarak, kontrollu tam aktarimda gercek kavrama sonunda olculen eklem
hedefleri compartment_grasp_seeds.json icinde raf 1 r4c1 icin yeni bir IK
baslangic tahmini olarak eklendi: staged_approach_full_transfer_roll1.
Eski tahmin ikinci sirada korunur; transfer/place disindaki asamalar ve
geometri eslestirme kurallari aynidir. Kayitli hareket oynatilmaz: IK, giris,
kaldirma, cikis, masaya devam ve yaklasma her seferinde yeniden dogrulanir.
Olculen yedi eklem hedefi forced_3 logundaki kapanistan hemen onceki TIMED
JOINT kayitlarindan alinmistir. Bu ek tahmin her seferinde kabul garantisi
degildir; ikinci normal kontrolde kayitli ikinci (eski) aday secilmistir.

Guncel banka ile staged_approach_normal_2 uretim tam aktarimi GECTI
(kaynak raf 1 r4c1 -> raf 2 r3c1). Iki tercihli tahmin yuklendi; eski tahmin
ikinci aday olarak 3.117 saniyede kabul edildi. Dogrudan yaklasma yeterli oldu,
yeni ara yaklasma gerekmedi. Sol masa XY 0.108 mm, kayma 0.008 mm,
egim 0 derece; sag son hedef XY 1.864 mm, kayma 0.028 mm, egim 0 derece.
Log: log/compartment_full_staged_approach_normal_2.log.

log/verify_staged_approach.py, test kopyasinin uretim C++ kodundan yalnizca
normal yaklasmayi kapatan satirla ayrildigini, yeni kayitli tahminin yedi eklem
degerinin basarili fiziksel kavrama logundan geldigini, zorunlu yedek yaklasma
ve normal tam basariyi dogrular. Ozet: log/staged_approach_checks.json.
Kontrollu basari onceki tek tahminli banka ile, normal basari son iki tahminli
banka ile alinmistir. Ilk iki kontrollu ve ilk normal basarisizlik yukarida
ayrica kayitlidir; silinmemistir ve basari oranindan gizlenmez. Bu iki son
basari yeni bir uzun tekrar serisi ya da tum bolmeler icin garanti degildir.
Derleme ve git diff --check gecti. Sag kol dosyasi bu calismada degismedi.
Degisiklikler yereldir; branch, commit veya GitHub gonderimi yapilmadi.

## Sol kola ozel OMPL cozum ve zamanlama ayari (2026-11-26)

Son kullanici serisi 4/5 tam basariydi. Basarisiz b5/d441 serisinin son
logunda kayitli aday masaya devam on kontrolunu gecti, fakat uc yaklasma
plani ValidateSolution tarafindan el/parmak/link6-raf temasi yuzunden
reddedildi. Ara yaklasma da uygun yol bulamadi. Yeni calisma tekrar sayisini
artirmak yerine bu plan uretimi ile son dogrulama farkini inceler.

Ilk tanisal olcumler gecersizdir: resolution_comparison_1/2 ve
resolution_timing_comparison_1/2, aktif arm_1 konfigurasyonu yerine bos
planner_config ile arm_1[] kaydini sorguladi veya kayit bulunamadi. Ozellikle
resolution_comparison_2 icindeki coarse_dense=3 / fine_dense=5, cozum
ayarlarinin etkisi olarak KULLANILMAZ. Bu sayilar sohbet sirasinda once
paylasildi ve daha sonra acikca geri cekildi. Kayitlar silinmedi.

Gecerli tanisal olcum resolution_timing_comparison_3'tur. Ayni dondurulmus
baslangic ve ayni eklem hedefinde iki farkli pipeline, sirayla 12'ser kez
cagrildi. GetPlannerParams istegi pipeline_id ile ve planner_config=arm_1,
bos group ile yapilarak gercek varsayilan grup kaydi sorgulandi. Her cagri
oncesinde longest_valid_segment_fraction geri okundu ve beklenen degerle
eslestirildi. Bu sorgu anahtarinin anlami MoveIt query_planners_service
kaynak kodundan dogrulandi.

Mevcut ayar (OMPL fraction 0.01, TOTG varsayilan path_tolerance 0.1):
12 istegin 6'si pipeline dogrulamasini, 4'u ek 0.05 rad payli yogun yerel
kontrolu gecti. Ozel ayar (fraction 0.001 ve path_tolerance 0.001):
12 istegin 12'si iki kontrolu de gecti. Bu bir sabit hedef planlama deneyi;
fiziksel tam aktarim basari orani degildir. Iki ayar birlikte degisti, tek
tek etkileri bu olcumden ayrilamaz. Gripper yaklasma hareketi uygulanmadi;
baslangic gripper acma/ready sonrasi tanisal program bilerek durdu. Sondaki
TEST ONLY planning comparison complete hata bildirimi bu bilincli durustur.
Log: log/compartment_full_resolution_timing_comparison_3.log.

Uygulama yalnizca compartment_transfer el degistirme sahnesinde ek
ompl_compartment_left pipeline'ini olusturur. Orijinal OMPL konfigurasyonu
kopyalanir; arm_1.longest_valid_segment_fraction=0.001 ve
totg.path_tolerance=0.001 atanir. Sol goreve compartment_planning_pipeline
parametresi verilir; C++ yalnizca bu parametre ve source_xyz varsa pipeline'i
secer. Sag kol varsayilan ompl'i kullanir. Mevcut eski komutlar ve bagimsiz
sag kol sahnesine bu pipeline otomatik eklenmez. Carpisma son kontrolu,
0.05 rad eklem payi, fiziksel kavrama ve destek kontrolleri korunur.

Kaynaklar: MoveIt OMPL cozumleme aciklamasi
https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/ompl_interface/ompl_interface_tutorial.rst
ve TOTG yol degisimi aciklamasi
https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/time_parameterization/time_parameterization_tutorial.rst
Ayarin uygulanmasi, tanisal testte kullanilan API kaydi ve zamanlama
adapteri resmi MoveIt kaynaklari ile karsilastirildi.

Fiziksel dogrulama (2026-11-26/27):
- precise_left_r4c1_1 tam aktarimi GECTI: sol masa XY 0.088 mm,
  kayma 0.005 mm, egim 0 derece; sag hedef XY 1.930 mm,
  kayma 0.138 mm, egim 0 derece.
- precise_left_r4c1_2 dis kesintide yaklasma sirasinda yarim kaldi. Surec
  yeniden acilista mevcut degildi. Bu kayit tamamlanmis basari/basarisizlik
  olarak SAYILMAZ; log korunmustur.
- precise_left_r4c1_3 ayni surumle yeni normal tam aktarim olarak GECTI:
  ilk kayitli aday 1.418 saniyede on kontrolu gecti. Sol masa XY 0.107 mm,
  kayma 0.009 mm, egim 0 derece; sag hedef XY 2.054 mm,
  kayma 0.210 mm, egim 0 derece.

Her iki tamamlanmis denemede get_parameters servisiyle etkin degerler
ayrica okundu: ompl_compartment_left.arm_1.longest_valid_segment_fraction
0.001, ompl_compartment_left.totg.path_tolerance 0.001 ve varsayilan
ompl.totg.path_tolerance 0.1. C++ logunda sol gorevin ozel pipeline'i
sectigi dogrulandi. Ilk Python tanisal parametre okuyucusunun context hatasi,
harici duzeltilmis okuyucuyla giderildi; ilk deneyin ayarlari o sirada
sunucudan yeniden dogrulandi. Ikinci tamamlanmis deneyde duzeltilmis okuyucu
kullanildi. Bunlar robot hareket hatasi olarak sayilmaz.

log/verify_precise_left.py gecersiz ilk olcumleri kullanmaz; gecerli
12+12 plan karsilastirmasinin aktif kayitlarini ve sayilarini, uretim C++
farkinin yalnizca secilebilir pipeline baglantisi oldugunu ve iki tam
aktarimi dogrular. Ozet: log/precise_left_checks.json. Sonuclar yalnizca
raf 1 r4c1 -> raf 2 r3c1 tam aktarimi ile sinirlidir; 12 plan fiziksel
tekrar sayisi degildir. 2/2 tamamlanmis fiziksel deneme uzun sureli kararlilik
garantisi olarak sunulmaz. Python derleme ve git diff --check tamamlandi.
Degisiklikler yereldir; branch/commit/GitHub islemi yapilmamistir.

## Yerel kontrol noktasi ve ikinci kaynak bolme (2026-11-27)

Baslatma loglari sonradan eslestirildiginde alti basarili kullanici denemesinin
besinin r4c1, birinin r3c2 kaynagindan oldugu dogrulandi. Bu alti denemede
hedef XY ortalama 1.344 mm, en buyuk 2.061 mm; en buyuk kayma 0.218 mm,
son egimlerin hepsi 0 dereceydi. Yeni kaynak bolmeye gecmeden mevcut kaynak
surumunun yerel kontrol noktasi olusturuldu:
log/checkpoints/20261127_111329_compartment_r4c1/

Kontrol noktasinda 17 degisen/yeni kaynak dosyasi, tracked_changes.patch,
git_status.txt ve SHA256 listeli manifest.json vardir. changed_sources.tar.gz
arsivinin icerigi hashlerle dogrulandi. Python onbellekleri dahil edilmedi.
Base commit 76ff6a6f7c22a061867f69f1cd382ed944120ed3 ve mevcut
compartment-transfer-development branch'i korundu. Commit veya uzak Git
islemi yapilmadi. Bu bir yerel kaynak arsividir.

Ardindan kod/ayar degistirmeden kaynak raf 1 r4c2 -> hedef raf 2 r3c1 icin
uc ayri headless tam aktarim yapildi:
- precise_left_r4c2_1: GECTI; son XY 2.067 mm, kayma 0.218 mm, egim 0 derece.
- precise_left_r4c2_2: GECTI; son XY 0.019 mm, kayma 0.003 mm, egim 0 derece.
- precise_left_r4c2_3: GECTI; son XY 2.028 mm, kayma 0.217 mm, egim 0 derece.

Ucunde de ilk kayitli kavrama adayi kabul edildi. R4c2 icin mevcut 0.45 rad/s
acisal tasima siniri uygulandi. Sol ozel pipeline 0.001/0.001 ve sag varsayilan
OMPL zamanlama toleransi 0.1 sunucudan ayrica dogrulandi. Loglar:
log/compartment_full_precise_left_r4c2_1.log,
log/compartment_full_precise_left_r4c2_2.log,
log/compartment_full_precise_left_r4c2_3.log.

log/verify_compartment_r4c2.py her logun kaynak/hedef secimini, tam basariyi,
planlama parametrelerini, bildirilen acisal siniri ve kaynak kontrol noktasi
ile calisma dosyalarinin hash eslesmesini denetler. Dokumantasyon yeni test
notlariyla guncellenebilir; calisan kod ve ayarlar arsivdekiyle aynidir.
Ozet rapor: log/compartment_r4c2_validation.json.

Bu asamada son surumle alt siradaki iki kaynak bolme icin basarili seriler
vardir. Diger alti kaynak bolme ve hedef raftaki diger yedi bolme bu seriyle
dogrulanmis sayilmaz. Siradaki kaynak r3c1, hedef yine raf 2 r3c1 olarak
sabit tutulabilir. GitHub gonderimi yapilmamistir.

## 4-1 plan arama ve 2-1 saat bekleme istisnalari (2026-11-27)

Son 16 kullanici logu baslatma loglariyla zaman damgasi uzerinden eslestirildi:
her kaynak bolme ikiser kez denenmis, hedef hep raf 2 r3c1. Sonuc 14/16 tam
aktarimdir. R2c1 ve r4c1 birer basari/basarisizlik; diger alti kaynak 2/2.
Ayrintilar log/user_source_matrix_20261127.json icindedir. R2c1 basarisizligi
PREPLACE beklemesinde 1 simulasyon saniyesi icin 30 gercek saniyelik sinirin
dolmasidir. Kullanici bilincli duraklatma yapmadigini bildirdi. O log tek
basina saatin neden yeterince ilerlemedigini kesinlestirmez.

R4c1 icin iki fiziksel tam basaridan ek IK tahmini cikarildi: 305556 ve
310705 numarali sol dugum loglarinin kavrama hedefleri. Baslatma loglarinda
kaynak r4c1 ve tam basari dogrulandi. Ilk iki tahmin ve diger bolmelerin
ayarlari korundu; bu bolmenin bankasi 2'den 4'e cikti. Her tahmin yine guncel
IK/giris/kaldirma/cikis/masa/yaklasma kontrollerinden gecmek zorundadir.
Kaniti log/r4c1_additional_seed_evidence.json, eski banka kopyasi
log/r4c1_seed_bank_before.json icindedir. Baslangic tahminleri robot yolu
olarak dogrudan oynatilmaz.

Hareket yapilmayan bank_comparison_1 on kontrol karsilastirmasinda eski iki
tahmin 6/6, dort tahmin de 6/6 gecti; dort tahminli seride bile ilk iki
tahmin yeterli oldu. Bu olcumden guvenilirligin arttigi sonucu CIKARILMAZ;
nadir kullanici basarisizligi bu olcumde tekrarlanmadi. Test ready/acik
parmak hazirligindan sonra kavrama/aktarim yapmadan bilerek sonlandirildi.
TEST ONLY planning bank comparison complete bildirimi beklenen test cikisidir.

Ek tahminlerin fiziksel testi new_seed_full_1'de eski iki tahmin yalnizca
ayri test kopyasinda cikarildi. Ek tahminlerin ikincisiyle tam aktarim GECTI:
sol masa XY 0.109 mm, kayma 0.011 mm, egim 0 derece; sag hedef XY 2.066 mm,
kayma 0.218 mm, egim 0 derece. Log:
log/compartment_full_new_seed_full_1.log. Bu test planlama-tekrari sarmalayicisi
eklenmeden onceki kodla, ayni saat politikasiyla alinmistir.

Son uretim surumunde planCompartmentPickup yalnizca aday aramasi tukenmesi
(CompartmentSearchExhausted) icin bir kez daha arama yapabilir. Ikinci
aramadan once kaynak cismin eski konum/yone gore 2 mm/1 derece kontrolu ve
kolun ilk eklemlerine gore 0.002 rad kontrolu gecmelidir. Guncel sahne ve
on kontrol yeniden olusturulur; onceki probe rotasi/tohumu temizlenir.
Her arama mevcut 25 saniye/aday sayisi butcesini korur (tek planlayici
cagrisinin donus suresi nedeniyle toplam sure sinir kontrolunu asabilir).
En fazla iki arama vardir. Sensor, konum degisimi, carpisma veya yurutme
hatalari bu kapsama alinmaz; kavrama/aktarim basladiktan sonra yeniden
baslatma YOKTUR. Tercihli tahmin reddi artik tahmin numarasi ve asamasiyla
loglanir. Bu korumali yeniden planlama nadir hatayi birebir urettigimiz veya
her aramada yol bulunacagi anlamina gelmez.

Bolmeli kaynak akisi icindeki holdTransferred beklemesi normal, yavas
ilerleyen, duran ve geri giden saati ayirir. Gerekli simulasyon suresi aynidir.
30 saniye saat ilerlemesi yoksa durur; saat geri giderse durur; toplam
120 gercek saniyede ust sinir vardir. Saat ilerliyorsa eski 30 saniyelik
mutlak duvar suresi yuzunden erken reddedilmez. Cisim pozu tazeligi,
mevcut fiziksel temas ve 5 mm / 0.0873 rad kayma kontrolleri her dongude korunur.
Eski source_xyz'siz akisin 30 saniyelik davranisi degismedi. Hata loglari
sim/duvar/bos gecen sure, /clock yayinci sayisi ve cisim poz yasi icerir.
Bu, ilk R2c1 hatasinin fiziksel kok nedeninin kesinlestigi anlamina gelmez.

simulation_hold_clock_test normal tamamlanma, 50 saniyede ilerleyen saat,
30 saniye donmus saat, geri atlama, 120 saniyelik ust butce ve gecersiz zaman
verilerini sinar; CTest gecti. Sag kol, raf geometrisi, hizlar, carpisma
matrisi ve eklem paylari bu degisiklikte degistirilmemistir.

Son surumden ayri test kopyalariyla yapilan ek dogrulamalar:

- retry_guard_1: Ilk arama bilerek tuketildi, tekrar kontrolundeki kaynak
  referansina 3 mm fark eklendi (fiziksel cisim oynatilmadi). Ikinci arama
  ve kavrama baslatilmadan reddedildi; sag kol calismadi. Beklenen negatif
  testtir, bir aktarim basarisi veya gercek kaynak kaymasi deneyi degildir.
- retry_full_1: Ilk arama bilerek tuketildi. Kaynak ve eklemler degismedigi
  dogrulaninca ikinci aramadan sonra r4c1 -> raf 2 r3c1 tam aktarim GECTI.
  Son XY 0.015 mm, birakma kaymasi 0.003 mm, egim 0 derece.
- slow_hold_full_1: Yalnizca PREPLACE icindeki gecen simulasyon suresi
  hesabina 0.02 katsayisi uygulandi; Gazebo fizigi, global /clock ve sensor
  verileri degistirilmedi. Gerekli 1 saniye bu hesapta 55.523 gercek saniyede
  tamamlandi. Eski 30 saniyelik mutlak sinir asildigi halde ilerleme vardi;
  temas ve kayma kontrolleri korundu. R2c1 -> raf 2 r3c1 tam aktarim GECTI.
  Son XY 1.924 mm, birakma kaymasi 0.134 mm, egim 0 derece. Bu, gercek Gazebo
  yavaslamasinin veya kullanicidaki kok nedenin birebir yeniden uretimi degildir.

Bu iki pozitif senaryo ve yukaridaki new_seed_full_1 toplam uc kontrollu
tam aktarimdir; degistirilmemis kosullarda uc tekrar olarak sayilmaz.
Uretim dosyalarinda test enjeksiyonu yoktur. Son uretim kaynak/hashleri,
test loglari, korumali reddetme ve diger calisma dosyalarinin kontrol
noktasiyla eslesmesi log/verify_seed_clock.py ile denetlendi. Derleme,
simulation_hold_clock_test ve git diff --check gecti. Ozet rapor:
log/seed_clock_validation.json. Commit, push veya yeni branch yapilmadi.
Sonraki saha kontrolu, ayni hedef sabit tutularak r4c1 ve r2c1 kaynaklarini
normal komutla yeniden denemektir; aralikli hatalarin tamamen giderildigi
veya basari oraninin arttigi henuz ileri surulmez.
