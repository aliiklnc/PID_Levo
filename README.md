# PID_Levo

STM32F407 tabanlı dört köşe elektromanyetik levitasyon kontrol sistemi
prototipi.

Bu proje şu anda levitasyon kontrolünü çalıştırmıyor. Doğrulanmış kapsam;
TCA9548A üzerinden dört VL6180X mesafe sensörünün okunması, ölçümlerin
filtrelenmesi ve heave, roll, pitch düzlem kestiriminin üretilmesidir.

Projenin ayrıntılı devir-teslim ve doğruluk kaynağı ozet.md dosyasıdır.
Yeni bir çalışmaya başlamadan önce o dosya baştan sona okunmalıdır.

## Donanım

- MCU kartı: STM32F407G-DISC1 / STM32F407VGT6
- I2C1: PB6 SCL, PB9 SDA, 400 kHz
- I2C çoklayıcı: TCA9548A, adres 0x70
- Çoklayıcı reseti: PE4, aktif düşük
- Mesafe sensörleri: 4 adet TOF050C / VL6180X, adres 0x29

Kanal eşlemesi sabittir:

| TCA kanalı | Fiziksel köşe |
|---:|---|
| 0 | FL, ön-sol |
| 1 | FR, ön-sağ |
| 2 | RL, arka-sol |
| 3 | RR, arka-sağ |

Tezgâh bağlantısında TCA9548A 3.3 V, TOF050C modülleri 5 V ile
beslenir ve bütün GND hatları ortaktır. Farklı sensör breakout kartları
kullanılırsa I2C pull-up ve lojik seviyeleri ölçülmeden aynı besleme
düzeni varsayılmamalıdır.

## Doğrulanmış durum

2026-08-29 tarihinde gerçek kart ve dört sensörle:

    g_gap_found        = 4
    g_gap_valid_n      = 4
    g_gap_present      = [1, 1, 1, 1]
    g_gap_fault        = [0, 0, 0, 0]
    g_gap_hz           = yaklaşık [100, 100, 100, 100]
    g_i2c_err_count    = 0
    g_est_valid        = 1
    g_est_n_used       = 4

FL, FR, RL ve RR sensörleri tek tek yaklaştırma testiyle ilgili dizi
indekslerine karşı fiziksel olarak doğrulanmıştır.

## Derleme

Windows üzerinde STM32CubeIDE 2.2.0 ARM GCC araç zinciri ve Git Bash
kullanılır:

    bash tools/build.sh

Beklenen çıktılar:

    build/PID_Levo.elf
    build/PID_Levo.bin

Son doğrulanan derleme sıfır warning ve sıfır error üretmiştir. Build
betiği proje kökünü kendi konumundan bulur; monorepo içinden de
çalıştırılabilir:

    bash PID_Levo/tools/build.sh

## SWD ile yükleme

STM32CubeProgrammer CLI ile:

    STM32_Programmer_CLI.exe -c port=SWD mode=UR -d build/PID_Levo.elf -v -rst

CubeIDE debug oturumu ST-LINK bağlantısını tutuyorsa önce oturum
kapatılmalıdır.

## Canlı değişken okuma

Varsayılan sensör ve estimator değişkenlerini okumak için:

    python tools/read_vars.py --wait 8

Örnek özel okuma:

    python tools/read_vars.py g_gap_raw g_gap_hz g_heave_mm g_roll_mrad g_pitch_mrad

## Güvenlik

- g_levitate_req değeri 1 yapılmamalıdır.
- LEVITATING durumunda gerçek PID veya kontrol yasası henüz yoktur.
- Bobin, BTS7960 ve güç katı testleri açık fiziksel onay olmadan yapılmamalıdır.
- State machine güvenlikleri ve sensör zaman aşımı kontrolleri bypass edilmemelidir.
- Sensör hataları rastgele timeout artırılarak gizlenmemelidir.
- Bobin karakterizasyon yolu yazılım düzeyinde sertleştirilmiştir ancak son
  güvenlik değişikliklerinden sonra fiziksel bobin testi yapılmamıştır.

## Sıradaki fiziksel doğrulamalar

1. Sert ve düz plaka ile heave ve residual testi
2. Roll işareti doğrulaması
3. Pitch işareti doğrulaması
4. Bir sensör kapatılarak üç sensör failover testi
5. Gerçek sensör X ve Y mesafelerinin ölçülüp geometri sabitlerine işlenmesi

Bir aşama fiziksel olarak doğrulanmadan sonraki aşamaya geçilmemelidir.

## Temel dizinler

| Yol | İçerik |
|---|---|
| Core/Inc | Uygulama başlıkları ve yapılandırma |
| Core/Src | Sensör, estimator, güç ve state machine kaynakları |
| Drivers | STM32 HAL ve CMSIS |
| Middlewares | USB Host bileşenleri |
| tools | Komut satırı build ve SWD değişken okuma araçları |
| ozet.md | Ana devir-teslim ve doğruluk kaynağı |
| PID_Levo.ioc | STM32CubeMX proje yapılandırması |
