# PID_Levo — Proje Devir Teslim Belgesi

**Son güncelleme:** 2026‑08‑31

> Bu belge, projeyi hiç bilmeyen birinin (veya yeni bir sohbet oturumunun) sıfırdan
> devralabilmesi için yazıldı. Sırayla okunduğunda: ne yapıldığı, neden öyle yapıldığı,
> nerede kalındığı ve hangi tuzaklara düşülmemesi gerektiği çıkar.
>
> **Yeni oturumda ilk yapılacak:** §6'daki derleme/yükleme/okuma akışını çalıştırıp
> kartın canlı durumunu görmek. Sonra §12 "nerede kaldık"a bakmak.

---

## 1. Amaç ve şartname

2026 TÜBİTAK/TEKNOFEST **Hyperloop Geliştirme Yarışması**, Bölüm 3 —
*Teknoloji Gösterim Kategorisi / Levitasyon Sistemi*.

Çelik ray üzerinde **EMS** (elektromıknatısla çekme) tipi aktif levitasyon kontrol yazılımı.
Şartname `levoSartname.md` dosyasında. Yazılımı şekillendiren maddeler:

| Madde | İstenen | Yazılım karşılığı |
|---|---|---|
| 3.m | Ağırlık merkezi dışına yük binince de dengede kalma | 4 köşe ölçümü → roll/pitch kontrolü (`estimator.c`) |
| 3.o | Her yük kademesinde ≥1 dk kesintisiz çalışma | Termal bütçe (§9.3), IWDG, durum makinesi |
| 3.p | Bir modül devre dışı kalırsa stabil kal **ya da** levitasyonu tamamen kes | 3 sensörle düzlem çözme + mandallı `SAFE_SHUTDOWN` |
| 3.q / 3.r | Kalkış ve stabil durumda akım/gerilim ölçümü | `current_sense.c`, zirve kayıtları |
| 3.s | Güç kesintisinde raya zarar vermeden güvenli iniş | `SM_LANDING` — ani kesme değil, kontrollü duty rampası |
| 3.t | FMEA / fail‑safe | 11 bitlik hata maskesi, IWDG, reset sebebi kaydı |
| 3.u | Hava aralığı ≥2 mm | `GAP_MIN_MM` ihlalinde anında hata |
| EK1‑3.4 | Yer istasyonu bağlantısı koparsa güvenli duruma geç | `SM_ReportGroundStation()` + `GS_TIMEOUT_MS` |

Yani hedef "bir PID döngüsü" değil: **durum makinesi + kestirim + kontrol tahsisi + hata yönetimi.**

---

## 2. Donanım envanteri

### Elde olan

| Bileşen | Detay | Durum |
|---|---|---|
| MCU kartı | STM32F407G‑DISC1 (STM32F407VGT6, 168 MHz) | ✅ çalışıyor |
| Mesafe sensörü | 4 × TOF050C (VL6180X, I²C 0x29) | ✅ 4 adet bağlı ve birlikte doğrulandı |
| I²C çoklayıcı | TCA9548A (0x70) | ✅ çalışıyor |
| Güç sürücüsü | 4 × BTS7960 / IBT‑2 H‑köprü | ⛔ henüz bağlanmadı |
| IMU | **BNO055** (Boardoza breakout) | ⛔ bağlanmadı, sürücü yazılmadı |
| Batarya | 8S **LiFePO4** (20.0–29.2 V, nominal 25.6 V) | ⛔ bağlanmadı |
| Laboratuvar güç kaynağı | SUNLINE U203, 0–30 V / 0–10 A, akım sınırlamalı | ✅ elde |
| Osiloskop | FNIRSI 1014D, iki kanal | ✅ elde, PE9 PWM testinde kullanıldı |
| Mıknatıs/bobin | Tasarım: 4 adet, 600 sarım, 1 mm tel, 30 mm çekirdek | ⛔ test tezgâhında bobin yok, ölçülmedi |
| Şasi | 40 kg, hedef hava aralığı 5 mm (kararı §9.3) | — |

### Alınmayacak

Kullanıcı **sipariş verecek zamanı olmadığını** belirtti (2026‑08‑29). Bu yüzden:

- **SPI ham‑gyro IMU:** gerekmiyor. Elde BNO055 var ve ham (`AMG`, non‑fusion) modda
  kullanılırsa ivmeölçer 1000 Hz, jiroskop 523 Hz'e kadar çıkıyor. Füzyon modunun
  100 Hz + iç gecikme sorunu bypass edilmiş oluyor.
- **Harici 3.3 V regülatör:** TOF050C modülünde kendi LDO'su varsa (`662K`/`XC6206` gibi
  3 bacaklı parça) sensörler 5 V rayından beslenir ve gerekmez. Tezgâhta dört modülün
  beslemesi yaklaşık **4.5 V**, TCA9548A beslemesi **2.942 V** ölçüldü; dört sensör birlikte
  100 Hz civarında ve sıfır I²C hatasıyla çalıştı. Modül üzerindeki LDO parça kodu yine de
  görsel olarak belgelenmedi.
- **Şönt + INA240:** sadece B planıydı. Bedava karşılığı PWM frekansını düşürmek (§10.3).

---

## 3. Mimari ve dosya haritası

```
Core/Inc/                 Core/Src/
  app_config.h              i2c_bus.c         ← Aşama 1: I²C sarmalayıcı
  i2c_bus.h                 tca9548a.c        ← Aşama 1: çoklayıcı
  tca9548a.h                vl6180x.c         ← Aşama 1: ToF sürücüsü
  vl6180x.h                 gap_sensor.c      ← Aşama 2: 4 kanal yönetimi
  gap_sensor.h              estimator.c       ← Aşama 2: heave/roll/pitch
  estimator.h               bts7960.c         ← Aşama 3: güç sürücüsü
  bts7960.h                 current_sense.c   ← Aşama 3: akım/gerilim
  current_sense.h           state_machine.c   ← Aşama 3: emniyet + IWDG
  state_machine.h           coil_test.c       ← teşhis: bobin R/L
  coil_test.h               main.c            ← yalnız USER CODE blokları
tools/
  build.sh                  komut satırı derleme
  read_vars.py              SWD ile canlı değişken okuma
ozet.md                     bu belge
pinout.md                   kesin pin ve kart bağlantı tablosu
levoSartname.md             yarışma şartnamesi
```

`app_config.h` **tek doğruluk kaynağıdır**: pin, adres, eşik, zaman sabitleri hep orada.
Sürücü dosyaları doğrudan pin/adres sabiti içermez.

Tüm dosyalar `Core/` altında olduğu için CubeMX bunlara dokunmaz ve ek IDE ayarı gerekmez.
**Kodlar ASCII yazılmıştır** (Türkçe karakter yok) — CubeIDE kod üretimi ve derleyici
kodlama sorunlarını önlemek için.

### Katman sorumlulukları

```
main.c
 ├─ 1 ms: SM_Task()          emniyet + IWDG beslemesi + kontrol
 └─ 1 ms: GapSensor_Task()   round-robin tek kanal yoklama
          Estimator_Update() düzlem oturtma

GapSensor  →  TCA9548A  →  VL6180X ×4        (mesafe)
Estimator  →  GapSensor                       (heave/roll/pitch)
StateMachine → BTS7960 (çıkış) + CurrentSense (geri besleme) + IWDG
```

---

## 4. Pin haritası (kesinleşti, `.ioc`'ye işlendi)

```
kanal   RPWM (TIM1)   LPWM      EN (R_EN+L_EN köprü)   IS (ADC1)
FL      PE9   CH1     GND       PE7                    PA1  IN1
FR      PE11  CH2     GND       PE2                    PA2  IN2
RL      PE13  CH3     GND       PE5                    PA3  IN3
RR      PE14  CH4     GND       PE6                    PB0  IN8
                                                VBUS → PB1  IN9

I2C1     PB6 (SCL) / PB9 (SDA)  — 400 kHz
MUX_RST  PE4
LED      PD12 yeşil / PD13 turuncu / PD14 kırmızı / PD15 mavi
Buton    PA0 (B1)
```

**PE0 KULLANILMAZ** — DISC1'de ivmeölçerin INT1 çıkışına bağlı, kendi çıkışımızı koyarsak
iki sürücü çakışır. `EN_FL` bu yüzden PE7'de.

`R_EN` ve `L_EN` modül üzerinde köprülenip tek MCU pinine bağlanır: 8 pin yerine 4 pin gider
ve iki yarım köprü birlikte etkinleşir/kapanır. Sistem tek yönlü sürüleceği için `LPWM`
STM32'ye bağlanmaz, kart üzerinde doğrudan GND'ye bağlanır. PE8/PE10/PE12/PE15 boştur.

### BTS7960 kablolaması (kanal başına, örnek FL)

```
PE9  ───────────────────  RPWM
GND  ───────────────────  LPWM
PE7  ──┬────────────────  R_EN
       └────────────────  L_EN
PA1  ───────────────────  R_IS ──┬── 2.2 kΩ ── GND   (bu direnç ŞART)
GND  ───────────────────  GND    │
5V   ───────────────────  VCC    ┘

Ortak bus gerilimi (tek adet):
PB1  ──┬── 100 kΩ ────  B+       ← bölücü 11:1
       └── 10 kΩ  ────  GND

Güç tarafı: B+/B- → 8S LiFePO4 (SİGORTALI), M+/M- → bobin
```

### Bobine akım vermeden önce ŞART olanlar

1. Ortak `R_EN+L_EN` ve `RPWM` hatlarına GND'ye **10 kΩ pull‑down**.
   STM32 reset ve programlama sırasında tüm GPIO'lar yüksek empedanslı girişe döner;
   pull‑down olmadan sürücünün durumu belirsizdir. `LPWM` doğrudan GND'dir ve pull-down gerekmez.
2. B+ hattına **10–15 A sigorta**. Yazılım aborte edemezse tek koruma budur.
3. Bobinin DC direncini multimetreyle ölçmek — IS pininin mutlak kalibrasyonu buradan çıkar.

### Sensör kablolaması

```
TOF050C #1 → TCA9548A SD0/SC0   (kanal 0 = FL)
TOF050C #2 → TCA9548A SD1/SC1   (kanal 1 = FR)
TOF050C #3 → SD2/SC2            (kanal 2 = RL)
TOF050C #4 → SD3/SC3            (kanal 3 = RR)
```

Adres ayarı yok, dört sensör de 0x29'da kalır; çoklayıcı onları ayırır.
TCA9548A `A0/A1/A2` üçü birden GND → adres 0x70.

**BNO055 çoklayıcının ÖNÜNE, kök I²C hattına** bağlanmalı (arkasına konursa jiroskopu her
okumada kanal değiştirmek gerekirdi). `ADR/COM3` pini LOW → adres 0x28. HIGH yapılırsa
0x29 olur ve VL6180X ile aynı adrese düşer — gereksiz risk. 32 kHz kristal gerekmiyor
(o sadece füzyon modu için). Şasiye sıkı monte edilmeli.

---

## 5. CubeMX / `.ioc` durumu

`PID_Levo.ioc` içinde yapılandırılmış olanlar:

- **I2C1**: 400 kHz Fast Mode, `DutyCycle = I2C_DUTYCYCLE_2`
- **TIM1**: 4 kanal PWM, merkez hizalı mod 1, PSC=0, **ARR=4199 → 20.0 kHz**, TRGO=Update
- **TIM2**: PSC=0, **ARR=2099 → 40.0 kHz**, TRGO=Update (ADC tetiği)
- **ADC1**: 5 kanal tarama (IN1, IN2, IN3, IN8, IN9), 28 cycle, PCLK2/4,
  **Continuous=Disabled**, harici tetik **T2_TRGO** yükselen kenar, DMA sürekli istek
- **DMA2_Stream0**: ADC1 → dairesel, half‑word, MemInc
- **GPIO/PWM**: 4 RPWM (TIM1) + 4 ortak EN; PE8/PE10/PE12/PE15 serbest, LPWM'ler donanımda GND
- Kalıntı: I2S3, SPI1, USB_HOST hâlâ etkin (temizlenmedi, zarar vermiyor)

**Generate Code her an güvenle çalıştırılabilir.** Elle düzenlenen üretilmiş dosya kalmadı.
(Eskiden `stm32f4xx_hal_conf.h`'de ADC/TIM modülleri elle açılıyordu; artık `.ioc`'de oldukları
için CubeMX kendisi açıyor.)

⚠️ **`.ioc`'yi elle yamalamanın sınırı var** — bkz. §14.2.

---

## 6. Derleme, yükleme, okuma iş akışı

Bu proje **CubeIDE açılmadan** komut satırından yönetilebiliyor. Yeni oturumda ilk denenecek şey bu.

### Derleme

```bash
cd /c/STM32Projects/PID_Levo
bash tools/build.sh build
```

CubeIDE'nin kendi ARM GCC'sini kullanır. Çıktı: `build/PID_Levo.elf`.
Uyarı taraması: `bash tools/build.sh build 2>&1 | grep -iE 'warning|error'`
(şu an **sıfır uyarı** vermeli — verirse bir şey bozulmuş demektir).

### Yükleme

```bash
CLI="/c/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe"
"$CLI" -c port=SWD mode=UR -d build/PID_Levo.elf -v -rst
```

`Download verified successfully` görülmeli.

### Canlı değişken okuma (Live Expressions'a gerek yok)

```bash
python tools/read_vars.py --wait 8          # varsayılan set
python tools/read_vars.py g_heave_mm g_roll_mrad
```

ELF'in sembol tablosundan adresleri bulup RAM'i SWD ile okur, float/dizi çözümlemesi yapar.

⚠️ İlk yazılan (`= 0xAA` gibi) değişkenler `.data` bölümünde, sıfırla başlayanlar `.bss`'te.
İkisi RAM'de uzak adreslerde olabilir; tek bir pencere okuyup ikisini birden çözmeye
çalışırsanız yanlış (sıfır) değer alırsınız. `read_vars.py` bunu doğru yapıyor.

### Sorun giderme araçları

```bash
# Prob bağlı mı?
"$CLI" -l | grep -A4 'STLink Interface'

# Çekirdek nerede takıldı?
"$CLI" -c port=SWD mode=HOTPLUG -halt -coreReg
A2L=".../arm-none-eabi-addr2line.exe"; "$A2L" -f -e build/PID_Levo.elf 0x0800XXXX

# Kesme içindeyse (LR = 0xFFFFFFxx) ana kodun yeri: yığın çerçevesinde SP+0x18
"$CLI" -c port=SWD mode=HOTPLUG -r32 <SP+0x18> 4
```

PC'yi birkaç kez örnekleyip dağılımına bakmak, kesme fırtınası teşhisinde çok işe yaradı (§14.4).

⚠️ **CubeIDE'de açık bir debug oturumu varsa** `ST-LINK_gdbserver` probu tutar ve komut satırı
`DEV_CONNECT_ERR` alır. Kontrol:
`Get-Process | Where-Object { $_.ProcessName -match 'gdbserver' }`

---

## 7. Live Expressions değişken sözlüğü

Kullanıcı CubeIDE'den bakmak isterse: `Run → Debug (F11)`, sonra **F8 (Resume)** —
Live Expressions yalnızca hedef koşarken güncellenir. Pencere: `Window → Show View → Live Expressions`.

`uint8_t` değişkenler karakter olarak gösterilir; başına `(int)` yazın veya
sağ tık → `Number Format → Decimal`.

| Değişken | Anlamı | Beklenen |
|---|---|---|
| `g_gap_found` | Açılışta bulunan sensör sayısı | takılı sensör sayısı |
| `g_gap_valid_n` | Şu an geçerli kanal sayısı | = found |
| `g_gap_present[4]` | Kanal başına "var mı" | 1 |
| `g_gap_fault[4]` | Kanal başına zaman aşımı | 0 |
| `g_gap_raw[4]` | Ham okuma [mm] | hedefe göre |
| `g_gap_ch[4]` | Kalibre hava aralığı [mm] | — |
| `g_gap_hz[4]` | Kanal başına gerçek hız | ~100 |
| `g_est_valid` | Düzlem çözüldü mü | 1 |
| `g_est_n_used` | Çözüme giren sensör | 2–4 |
| `g_heave_mm` | Merkez hava aralığı | — |
| `g_roll_mrad` | Sol taraf yukarı = pozitif | — |
| `g_pitch_mrad` | Burun yukarı = pozitif | — |
| `g_est_resid_mm` | En büyük düzlem sapması | küçük |
| `g_sm_state` | Durum (0..7, §11.5) | — |
| `g_sm_faults` | Hata bit maskesi | 0 |
| `g_vbus_v` | Bus gerilimi [V] | 20–29 |
| `g_coil_i_a[4]` | Bobin akımları [A] | — |
| `g_duty[4]` | PWM duty (0..0.95) | — |
| `g_peak_i_a` / `g_peak_p_w` | Kalkış zirveleri (3.q) | — |
| `g_i2c_err_count` | I²C hata sayacı | **0'da kalmalı** |
| `g_levitate_req` | **1 yazılınca kalkış denenir** | 0 |
| `g_coil_arm` | **0xA5 yazılınca bobin testi** | 0 |

Eski tek sensör aynaları hâlâ yayımlanıyor: `g_range_mm`, `g_range_status`, `g_gap_mm`,
`g_ok_count`, `g_sample_hz`, `g_stat_mean/std/pp/n` — hepsi kanal 0'ı (FL) gösterir.

**LED durumu** (Live Expressions açmadan):
yeşil LD4 yanıp sönüyor = geçerli ölçüm akıyor · turuncu LD3 = bulunan sensörlerden biri arızalı ·
kırmızı LD5 = hiç geçerli ölçüm yok · mavi LD6 = bobin testi çalışıyor.

---

## 8. ToF sensörü — ölçülen gerçekler

### 8.1 Hız / gürültü taraması (30 mm'de metal zemin, her satır 1 s)

| # | AVG | Yakınsama | Hz | hata | p‑p | ort [mm] | **σ [mm]** |
|---|---|---|---|---|---|---|---|
| 0 | 0x18 | 10 ms | **101** | 0 | 5 | 35.50 | **1.149** |
| 1 | 0x18 | 49 ms | 101 | 0 | 7 | 35.46 | 1.368 |
| 2 | 0x30 | 10 ms | 100 | 0 | 6 | 37.03 | 1.135 |
| 3 | 0x30 | 49 ms | 100 | 0 | 6 | 36.93 | 1.194 |
| 4 | 0x60 | 10 ms | 52 | 0 | 5 | 36.87 | 1.345 |
| 5 | 0x60 | 49 ms | 52 | 0 | 5 | 36.60 | 1.165 |
| 6 | 0xA0 | 10 ms | 0 | **52** | – | – | – |
| 7 | 0xA0 | 49 ms | 51 | 0 | 4 | 37.08 | **1.007** |

**Seçilen ayar: AVG 0x18, yakınsama 0x0A (10 ms), ölçümler arası periyot 10 ms → 101 Hz, σ ≈ 1.15 mm.**

Sensörün *kendi içindeki* ortalamayı 6.7 kat arttırmak σ'yı yalnız 1.15 → 1.01 düşürüyor ama
hızı yarıya indiriyor. Sensör içi ortalama işe yaramıyor.
Mesafe arttıkça ikisi de kötüleşiyor: ~120 mm'de 52 Hz ve σ = 1.9 mm.

⚠️ **AVG 0xA0 + yakınsama 10 ms kombinasyonu tamamen çöküyor** (52 ölçümün 52'si hata).
Sürücüde henüz engellenmedi — açık iş.

### 8.2 Yazılım filtresi karşılaştırması (284 örnek, 3 s, 100 Hz)

| Filtre | **σ [mm]** | p‑p | gecikme | kazanç |
|---|---|---|---|---|
| ham | 1.209 | 7.00 | 0 ms | 1.0× |
| 4‑örnek ortalama | 0.641 | 3.25 | ~15 ms | 1.9× |
| 8‑örnek ortalama | 0.439 | 2.00 | ~35 ms | 2.8× |
| 16‑örnek ortalama | 0.302 | 1.31 | ~75 ms | **4.0×** |
| 5‑örnek medyan | 0.690 | 4.00 | ~20 ms | 1.8× |

16 örnekte tam **4.0× = 1/√N**. Gürültü beyaz ve örnekler bağımsız; yazılımda ortalamak
işe yarıyor **ama gecikme bedeliyle**.

> Not: Bir ara "bu gürültü ortalamayla azalmaz" demiştim — **yanlıştı**, ölçüm bunu çürüttü.

### 8.3 Kalibrasyon açığı

Sensör 30 mm'de **35.1 mm** okuyor → ~5 mm ofset. Parça‑parça kalibrasyon yapılmalı
(`GapSensor_Calibrate(ch, gerçek_mm)` bunun için var; `SYSRANGE__PART_TO_PART_RANGE_OFFSET`
register 0x024 alternatifi).

Şu an `GAP_SENSOR_OFFSET_MM = 25.0` varsayılan. **Kalibrasyon yapılmadan `g_gap_mm`'e
güvenmeyin, `g_gap_raw`'a bakın.**

### 8.4 Dört sensörlü tezgâh doğrulaması (2026-08-29)

Dört TOF050C, TCA9548A arkasında aynı anda çalıştırıldı. Kararlı, düz hedef ölçümünde:

```
g_gap_found     = 4
g_gap_valid_n   = 4
g_gap_present   = [1, 1, 1, 1]
g_gap_fault     = [0, 0, 0, 0]
g_gap_hz        = [99, 100, 100, 102]   (tekrar ölçümlerde 98-103 aralığı)
g_i2c_err_count = 0
g_est_valid     = 1
g_est_n_used    = 4
```

2026-08-29 güvenlik düzeltmelerinden sonra firmware yeniden derlendi, SWD ile
yüklendi ve aynı masa düzeneğinde tekrar doğrulandı:

    g_levitate_req   = 0
    g_coil_run_count = 0
    g_gap_found      = 4
    g_gap_valid_n    = 4
    g_gap_fault      = [0, 0, 0, 0]
    g_gap_hz         = [100, 100, 100, 99]
    g_i2c_err_count  = 0
    g_est_valid      = 1
    g_est_n_used     = 4
    g_sm_faults      = 0x0110

0x0110, güç busı bağlı olmadığı için beklenen SELFTEST | UNDERVOLTAGE
sonucudur. Yeni ADC_TIMEOUT biti set olmadı; ADC/DMA veri tazeliği kartta
kanıtlandı. Bobin ve BTS7960 çıkışları sürülmedi.

Fiziksel kanal eşlemesi tek tek cisim yaklaştırılarak kanıtlandı:

| Fiziksel köşe | Referans `g_gap_raw` | Cisim yaklaştırılınca | Değişen indeks |
|---|---:|---:|---|
| FL | `[58,60,52,61]` | `[36,62,55,60]` | yalnız `[0]` |
| FR | `[58,60,52,61]` | `[60,46,52,60]` | yalnız `[1]` |
| RL | `[58,60,52,61]` | `[60,59,24,60]` | yalnız `[2]` |
| RR | `[58,60,52,61]` | `[60,61,56,27]` | yalnız `[3]` |

Sonuç kesin: **TCA 0=FL, 1=FR, 2=RL, 3=RR.** İlk kurulumda kanal 1'de SD1 ile
SC1 ters bağlanmıştı; düzeltildikten sonra dördüncü sensör bulundu. Parmağı 20 mm'den
fazla yaklaştırmak ölü bölgeye sokup `16 mm` civarında donmuş gibi görünen değer
üretebildi; kanal testi 20 mm'nin dışında yapılmalı.

Esnek kartonla alınan ilk heave/roll/pitch düzlem ölçümü **geçersiz sayıldı** ve sonuç
olarak kullanılmayacak. Test 2, dört sensörü örten sert ve düz bir plakayla yeniden yapılacak.

### 8.5 Oturum açılışı ve PE9 PWM doğrulaması (2026-08-30)

Firmware temiz çalışma ağacından yeniden derlendi; `build/PID_Levo.elf` üretildi,
SWD ile yüklendi ve `Download verified successfully` alındı. Sert plaka sabitken
alınan başlangıç canlı durumu:

```
g_sm_state       = 6 (FAULT)
g_sm_faults      = 0x0110
g_vbus_v         = 8.006 V       (VBUS bölücüsü bağlı değil; geçerli bus ölçümü değil)
g_gap_found      = 4
g_gap_valid_n    = 4
g_gap_present    = [1, 1, 1, 1]
g_gap_fault      = [0, 0, 0, 0]
g_gap_raw        = [46, 48, 43, 46]
g_gap_hz         = [100, 99, 100, 103]
g_est_valid      = 1
g_est_n_used     = 4
g_heave_mm       = 19.750
g_roll_mrad      = -8.333
g_pitch_mrad     = 1.250
g_est_resid_mm   = 0.250
g_i2c_err_count  = 0
```

Estimatorin hareket/işaret testi kullanıcı isteğiyle burada durdurulup BTS7960
hazırlığına geçildi. Bobin ve BTS7960 güç katı bu ölçümde enerjilendirilmedi.

TIM1_CH1 / PE9 sinyali, BTS ve güç kaynağı bağlı değilken debug altında güvenli
olarak `%50` duty'ye getirildi. FNIRSI 1014D ile kare dalga gözlendi:

| Ölçüm | Sonuç |
|---|---:|
| Frekans | **20 kHz** |
| Duty | **%50** |
| Multimetre DC ortalaması | **1.478 V** |
| Mantık seviyesi | yaklaşık 0–2.9 V |

Sonuç: `.ioc` pin eşlemesi, TIM1 kanal 1 ve fiziksel PE9 çıkışı elektriksel olarak
doğrulandı. Tek seferlik STM32CubeProgrammer yazma işlemi sonrasında normal firmware
güvenlik döngüsü duty'yi tekrar sıfırladığı için kararlı osiloskop ölçümü, çekirdek
`BTS_SetDuty()` içindeki CCR1 yazmasından hemen sonra durdurularak ve TIM1 serbest
çalıştırılarak alındı. Bu davranış bir PWM arızası değil, normal güvenli-kapalı
davranışıdır. Ölçüm sonunda STM32 USB'den ayrıldı; kart enerjisiz ve PWM kapalıdır.

---

## 9. Sistem analizi

### 9.1 ToF neden hızlı çevrimi kapatamaz

EMS'de mıknatıs açık çevrimde kararsız, kararsız kutup `p = √(2g/x₀)`:

| Hava aralığı | Kararsız kutup | Gereken sensör gecikmesi (p·τ < 0.3) |
|---|---|---|
| 3 mm | 80.9 rad/s | τ < 3.7 ms |
| 4 mm | 70.0 rad/s | τ < 4.3 ms |
| 5 mm | 62.6 rad/s | τ < 4.8 ms |
| 10 mm | 44.3 rad/s | τ < 6.8 ms |

Ölçülen ToF gecikmesi 100 Hz'de **~15 ms**:

```
ham          τ=15 ms → p·τ = 0.86   sınırda (teorik duvar p·τ = 1)
4-örnek ort  τ=30 ms → p·τ = 1.72   olanaksız
8-örnek ort  τ=50 ms → p·τ = 2.86   olanaksız
```

**Kıskaç:** ham veri gecikmede tam sınırda ama σ = 1.2 mm gürültü 5 mm'lik aralıkta sürücüyü
doyuma sokar; gürültüyü düşürmek için filtrelenirse gecikme sınırı aşılır. Ortada çalışan nokta yok.

**Sonuç: TOF050C tek başına hızlı levitasyon çevrimini kapatamaz.** Bu tahmin değil, ölçülmüş veri.
ToF mutlak referans / sürüklenme düzeltme / izleme sensörü olarak kalıyor.

Not: küçük hava aralığı daha kararsızdır (p büyür), yani 3 mm kontrol açısından 5 mm'den zordur.
Isınma açısından ise tersi (§9.3). Bu takas henüz karara bağlanmadı.

### 9.2 Mıknatıs karakteristiği (HESAPLANAN, ölçülmedi)

600 sarım, 1 mm tel, 30 mm çekirdek, 40 kg / 4 mıknatıs = 98 N/mıknatıs:

```
R ≈ 1.9 Ω          L(5 mm) ≈ 32 mH          L/R = 16.8 ms
B = 0.42 T (doyumun çok altında)            I = 5.54 A
```

**Kritik ilişki:** `L ∝ 1/g` → 5 mm'de **1 mm aralık değişimi = %20 endüktans değişimi.**
Akı gözleyicisini mümkün kılan şey bu.

⚠️ Bu değerlerin **hiçbiri ölçülmedi.** R/L testi (§12) tam olarak bunun içindir.

### 9.3 Hava aralığı seçimi — ısınma darboğazı

Akım hava aralığıyla doğru orantılı (`I ∝ g`):

| aralık | akım | 4 bobin toplam | akım yoğunluğu | τ bütçesi |
|---|---|---|---|---|
| 2 mm | 2.22 A | 37 W | 2.8 A/mm² ✅ | 3.0 ms |
| **3 mm** | **3.32 A** | **84 W** | **4.2 A/mm² ✅** | **3.7 ms** |
| 4 mm | 4.43 A | 149 W | 5.6 A/mm² | 4.3 ms |
| 5 mm | 5.54 A | 233 W | 7.1 A/mm² ❌ | 4.8 ms |

5 mm'de akım yoğunluğu 7.1 A/mm², sürekli çalışma için önerilen 3–5 A/mm²'nin üstünde.
Şartname 3.o (≥1 dk), 3.n (uzun süreli çalışma) ve 3.i (ray sıcaklığı) bunu sorun yapar.
Üstelik 3.m harici yük ekletiyor: 80 kg'da 7.8 A / 464 W.
**3 mm öneriliyor** ama kontrol zorluğu artıyor — karar R/L testinden sonra.

---

## 10. Hızlı sensör kararı: akı gözleyicisi

### 10.1 Değerlendirilen seçenekler

| Yöntem | Bant genişliği | Maliyet | Karar |
|---|---|---|---|
| Hall sensör | ~10 kHz | ~100 TL | elde yok |
| Eddy‑current sensör | ~10 kHz | 4×100–300 $ | pahalı |
| **Akı gözleyicisi (bobin akımı)** | ~20 kHz | 0 TL | ✅ **seçildi** |
| ATEK LMS lazer | **20 Hz** | endüstriyel | ❌ ToF'tan kötü |

**ATEK LMS neden elendi:** 20 Hz örnekleme (ToF'un beşte biri), 1 mm çözünürlük (aynı),
doğruluk ±%2 tam skala = **±40 mm** (2 m menzilli olduğu için). Yanlış alet.

### 10.2 Nasıl çalışır

Bobinin endüktansı hava aralığına bağlı. Her PWM periyodunda akım testere dişi çizer,
genliği `Δi = V·D·(1−D)/(L·f)`. Genlikten L, L'den hava aralığı çıkar.
Ekstra sensör yok, gecikme mikrosaniye mertebesinde.

### 10.3 Gözlenebilirlik (24 V, 5 mm, L = 32 mH varsayımıyla)

| PWM | dalgalanma | IS pininde mm başına | 2.5 ms senkron ortalama sonrası |
|---|---|---|---|
| 20 kHz | 9.4 mA | ~0.6 LSB | ~1.4 LSB/mm |
| 5 kHz | 37 mA | ~2.4 LSB | ~8 LSB/mm → **~0.12 mm** |
| 2.5 kHz | 75 mA | ~4.7 LSB | ~11 LSB/mm → **~0.09 mm** |

**BTS7960'ın IS pini zayıf:** 1/8500 oranı 43 A'lik otomotiv yükleri için tasarlanmış.
Çare para değil, **PWM frekansını düşürmek** — bedeli 5 kHz'de duyulabilir bir cızırtı.

TIM1 şu an 20 kHz'de; bu bir karar değil, başlangıç değeri. Frekans R/L testinden sonra seçilecek.

Karşılaştırma: **ToF 1.2 mm + 15 ms** ↔ **akı gözleyicisi ~0.1 mm + 2.5 ms.**

### 10.4 BNO055'in rolü

BNO055'i `AMG` (non‑fusion) modunda kullanınca iç füzyon devre dışı kalır:

| | Ne verir | Hangi soruna |
|---|---|---|
| Ham jiroskop (523 Hz'e kadar) | roll/pitch **hızı** | Dönme ekseninin türev terimi |
| Ham ivmeölçer Z (1000 Hz'e kadar) | dikey **ivme** | **Heave** — asıl kararsız mod |

İkincisi önemli: ToF'un mutlak ama yavaş konumuyla tamamlayıcı filtreye sokulunca konum
kestiriminin etkin gecikmesi ToF'un tek başına verdiğinden belirgin düşer. Maglev'de standart teknik.

Akı gözleyicisinin yerini almaz ama **yükünü paylaşır** — ikisi bağımsız olduğu için biri
beklenenden kötü çıkarsa diğeri açığı kapatabilir.

> Daha önce BNO055'i elemiştim; o değerlendirme **füzyon çıkışı** içindi. Ham modda
> kullanmak farklı bir öneri ve bedava olduğu için artık işimize yarıyor.

---

## 11. Kod katmanları — davranış detayı

### 11.1 `i2c_bus.c`
16‑bit register adresli erişim, 10 ms timeout, 2 kez yeniden deneme, hata sayacı,
I²C adres tarayıcı, takılan hattı 9 SCL darbesiyle kurtarma (`I2CBus_Recover`).

### 11.2 `tca9548a.c`
`TCA_SelectChannel(ch)` son kanalı önbelleğe alır — aynı kanal tekrar seçilirse I²C trafiği
üretmez. `TCA_Init()` PE4'ü yapılandırır ve RESET darbesi atar.

### 11.3 `vl6180x.c`
MODEL_ID doğrulama (0xB4), AN4545 zorunlu 30 girişlik tuning dizisi, sürekli mod,
yoklamalı okuma, 15 hata kodunun ayrıştırılması.
Varsayılan çalışma ayarı, ölçümle seçilen AVG 0x18 + 10 ms yakınsama + 10 ms
ölçümler arası periyot ile kaynakta eşitlendi.

**Sakinleştirme adımı:** init önce sürekli modu durdurur, kesmeleri temizler ve
`Device_Ready` bekler (200 ms). Bu olmadan, önceki çevrimden sürekli modda kalmış bir sensöre
yapılan yapılandırma yazmaları sessizce yok sayılıyordu (§14.5).

⚠️ `SYSRANGE__START` bit0 bir **TOGGLE**'dır: durdurmak için `0x01` yazılır, `0x03` değil.
Bu bir kez gerçek hataya yol açtı (stop→start dizisi sensörü kapalı bırakıyordu).

### 11.4 `gap_sensor.c` + `estimator.c`
Her sensör kendi sürekli modunda **serbest koşar**; MCU sırayla (round‑robin) çoklayıcı
kanalını değiştirip hazır olan örneği toplar. `GapSensor_Task()` her çağrıda **tek** kanal
yoklar, 1 ms periyotla çağrılır → 4 kanalda kanal başına 4 ms ziyaret aralığı.

**Eksik sensör sorun değil:** açılışta bulunamayan kanallar "yok" işaretlenir ve hiç I²C
trafiği üretmez. Aynı kod 1, 2, 3 veya 4 sensörle çalışır.

Filtre olarak **medyan‑3** seçildi: ToF'un asıl kusuru beyaz gürültü değil, arada gelen tek
örneklik sıçramalar; medyan bunları tek örnek gecikmeyle temizler. Hareketli ortalama daha az
gürültü bırakırdı ama getirdiği gecikme kararsız tesisi kontrol edilemez yapar (§9.1).

**Estimator** en küçük karelerle düzlem oturtur: `gap(x,y) = heave + pitch·x + roll·y`.
- **4 sensör:** tam çözüm + artık (`residual_mm`) tutarlılık göstergesi
- **3 sensör:** üç nokta düzlemi tam belirler → tam çözüm. *Şartname 3.p açısından kilit nokta:
  bir sensör düşünce levitasyonu kesmek gerekmiyor.*
- **2 sensör aynı eksende:** o eksenin eğimi çözülür (ön çift → roll, yan çift → pitch), diğeri 0
- **2 sensör çapraz / 1 sensör:** `valid = 0`, yalnız ortalama heave verilir

İki aynı eksenli sensör sonucu teşhis için geçerli kalır; fakat kontrol/emniyet
zincirine heave aktarılması için EST_MIN_CONTROL_SENSORS = 3 şarttır.

Geometri `GEOM_HALF_LENGTH_MM = 200`, `GEOM_HALF_WIDTH_MM = 150` **varsayım**.
Yanlış değer heave'i etkilemez, sadece açıları ölçekler. **Ölçülüp düzeltilecek.**

### 11.5 `state_machine.c`

```
INIT → SELFTEST → IDLE ──(g_levitate_req=1)──> SOFT_START ──> LEVITATING
                   ↑                               │              │
                   │                               ↓              ↓
                   └────────────────────────── LANDING <──────────┘
                                                   ↓
                                          FAULT → SAFE_SHUTDOWN (mandallı)
```

Durum numaraları: `0=INIT 1=SELFTEST 2=IDLE 3=SOFT_START 4=LEVITATING 5=LANDING 6=FAULT 7=SAFE_SHUTDOWN`

Hata bitleri (`g_sm_faults`):

| Bit | Değer | Anlam |
|---|---|---|
| 0 | 0x001 | SENSOR_TIMEOUT |
| 1 | 0x002 | GAP_MIN (2 mm ihlali, 3.u) |
| 2 | 0x004 | GAP_MAX |
| 3 | 0x008 | OVERCURRENT |
| 4 | 0x010 | UNDERVOLTAGE |
| 5 | 0x020 | OVERVOLTAGE |
| 6 | 0x040 | GS_TIMEOUT (EK1‑3.4) |
| 7 | 0x080 | SOFTSTART_TMO |
| 8 | 0x100 | SELFTEST |
| 9 | 0x200 | WATCHDOG_RESET (önceki çevrim IWDG ile resetlendi) |
| 10 | 0x400 | DRIVER_INIT |
| 11 | 0x800 | ADC_TIMEOUT (DMA verisi yenilenmiyor) |

**Sert hatalar** (OVERCURRENT / OVERVOLTAGE / DRIVER_INIT / ADC_TIMEOUT) → anında kesme, köprüye güvenilmez.
**Yumuşak hatalar** (sensör, aralık, düşük gerilim, yer istasyonu) → kontrollü iniş (`LANDING`).
`SAFE_SHUTDOWN` mandallıdır, yalnız donanım resetiyle çıkılır.

Trip kararları **filtresiz** akımdan verilir; filtre gecikmesi emniyeti yavaşlatmamalı.

**IWDG** 250 ms, doğrudan register'dan kuruluyor (HAL modülü `.ioc`'de kapalı).
Hata ayıklayıcı bağlıysa `__HAL_DBGMCU_FREEZE_IWDG()` ile donduruluyor — yoksa her breakpoint
250 ms sonra kartı resetlerdi. Prob yokken tam yetkiyle çalışır.

⚠️ **LEVITATING'de henüz kontrol yasası yok.** Aşama 4'e kadar duty, rampanın bittiği değerde
sabit kalır. **Bobin bağlıyken `g_levitate_req = 1` yapmayın** — sabit duty askı sağlamaz.

Bus gerilimi bağlı değilken sistem `SELFTEST`'i geçemez ve `FAULT / 0x0110`'da durur.
Bu **doğru davranış**: gerilimini ölçemediği bir donanımda levitasyona izin verilmiyor.

### 11.6 `bts7960.c` / `current_sense.c`
`BTS_AllOff()` her hata yolunda çağrılır, kesme içinden güvenlidir, hata döndürmez.
Sıra önemli: önce CCR sıfırlanır, sonra EN düşer.
`BTS_DUTY_MAX = 0.95` — %100'de yüksek yan sürücünün bootstrap kondansatörü dolamaz.
Tek yönlü kart kararıyla dört `LPWM` girişi doğrudan GND'ye bağlandı; sürücü ve bobin test
kodu artık LPWM GPIO makrolarına bağlı değil. CubeMX'te PE8/PE10/PE12/PE15 serbest bırakıldı.

`current_sense` 5 kanalı 40 kHz'de dairesel DMA ile okur, tamponu doğrudan okur.
`CS_ZeroCalibrate()` çıkışlar kapalıyken sıfır noktasını ölçer (~200 ms).
`CS_Init()` yeniden çağrılabilir (bobin testinden sonra gerekiyor).
DMA half/full bayrakları kesmesiz olarak izlenir; veri 10 ms boyunca yenilenmezse
CS_IsFresh() sıfır olur ve durum makinesi sert ADC_TIMEOUT hatasıyla çıkışı keser.

### 11.7 `coil_test.c`
Bobin R/L karakterizasyonu: 2 ms taban çizgisi → 12 ms tam gerilim → kesme → sönüm eğrisi,
hepsi 100 kHz'de RAM'e (`g_coil_cap`, 2048 çift: çift indeks IS, tek indeks VBUS).
Aşırı akımda (9 A) anında durur.

HAL_ADC_Start_DMA sonrasında Stream0 kesmeleri, TIM2 başlamadan önce tekrar
kapatılır. Böylece boş DMA ISR'sinin bayrağı temizlemeden kesme fırtınasına girip
bobin darbesini açık bırakma riski giderildi. Bu yol yalnız statik analiz ve
derlemeyle doğrulandı; fiziksel bobin testi yapılmadı.

**Güvenlik kapıları:** mavi butonu basılı tutarak reset **veya** `g_coil_arm = 0xA5`.
Başka hiçbir koşulda çıkış sürülmez. Toplam enerji ~0.7 J.

⚠️ Bu modül TIM1/TIM2/ADC1/DMA2'yi **kendi ölçüm ayarlarına göre yeniden programlar**,
bu yüzden yalnızca test istendiğinde çağrılır ve sonrasında `MX_TIM1/TIM2/ADC1_Init` ile
CubeMX yapılandırmasına dönülür (§14.3).

---

## 12. Nerede kaldık

### ✅ Tamamlanan

- **Aşama 0–1:** `.ioc`, I²C katmanı, çoklayıcı, ToF sürücüsü — doğrulandı: **100 Hz, 0 I²C hatası**
- **Aşama 2:** `gap_sensor` + `estimator` yazıldı, derlendi ve 4 sensörle doğrulandı:
  `found=4`, `valid=4`, kanal başına 98–103 Hz, `est_valid=1`, `n_used=4`, I²C hatası 0
- **Dört köşe kanal testi:** tek tek fiziksel cisimle doğrulandı — `0=FL, 1=FR, 2=RL, 3=RR`
- **Aşama 3:** `bts7960` + `current_sense` + `state_machine` + IWDG — derleme uyarısız
- Güvenlik sertleştirmesi: ADC/DMA tazelik denetimi, bobin testi DMA IRQ kapısı,
  minimum 3 sensör kontrol kapısı ve kanal bazlı LANDING rampası eklendi.
  Normal ve katı uyarılı derleme temiz; sensör/ADC yolu gerçek kartta doğrulandı,
  fiziksel bobin testi yapılmadı.
- Sensör karakterizasyonu (§8), kararlılık analizi (§9), hızlı sensör kararı (§10)
- Komut satırı derleme/yükleme/okuma zinciri (`tools/`)
- **PE9 / TIM1_CH1 fiziksel PWM doğrulaması:** FNIRSI 1014D ile 20 kHz, %50 duty;
  multimetre DC ortalaması 1.478 V (§8.5)
- **BTS7960 kart pin planı donduruldu (2026-08-31):** dört RPWM TIM1'de, dört ortak EN GPIO,
  dört R_IS ADC1'de; LPWM'ler doğrudan GND. `pinout.md`, `.ioc` ve firmware eşitlendi,
  Generate Code sonrası uyarısız derleme doğrulandı.

### ⏳ Sıradaki iş

**1. BTS7960 yüksüz bağlantı ve çıkış testi — mevcut fiziksel durak.** Bobin olmadığı
için yalnız lojik girişler, enable, yüksüz M+/M− anahtarlaması ve güvenli kapanma
doğrulanabilir; R/L, yük akımı, IS bant genişliği ve ısınma doğrulanamaz.

2026-08-30 sonunda güç verilmeden kullanıcı tarafından doğrulanan bağlantılar:

- BTS7960 kontrol `GND` → breadboard ortak GND
- STM32 `GND` → aynı ortak GND
- `PE9` → `RPWM`, bu noktadan GND'ye 10 kΩ pull-down
- `PE8` → `LPWM` geçici tezgâh bağlantısı 2026-08-31'de terk edildi. Son kartta ve sonraki
  tezgâh kurulumunda `LPWM` doğrudan GND'ye bağlanacak; PE8 kullanılmayacak.

Henüz tamamlandığı doğrulanmayan bağlantılar: `PE7 → R_EN+L_EN` ve 10 kΩ pull-down,
STM32 `5V → VCC`, `R_IS → PA1` ve GND'ye 2.2 kΩ, U203 `+ → B+`, U203
`− → B− + ortak GND`. `M+`/`M−` bobin olmadığı için boş kalacak. 100 kΩ üst
direnç olmadığı için VBUS bölücüsü ve `PB1` bağlanmayacak; ekrandaki `g_vbus_v`
geçerli kabul edilmeyecek. Sonraki oturumda güç vermeden önce multimetreyle kısa
devre/süreklilik kontrolü yapılmalı. U203 OUTPUT bu kontroller bitene kadar kapalı kalmalı.

**2. Sert düz plaka / estimator testi.** Esnek kartonla alınan ölçüm iptal edildi.
Dört sensörü örten sert bir plakayla sırasıyla:
- paralel yaklaşma/uzaklaşmada esas olarak heave'in değiştiğini,
- sol taraf geometrik olarak yukarıdayken `g_roll_mrad` değerinin **pozitif** olduğunu,
- ön taraf değiştirildiğinde pitch işaretinin koordinat tanımıyla uyumlu olduğunu,
- bir sensör devre dışıyken `g_est_valid=1`, `g_est_n_used=3` kaldığını doğrulayın.

Roll işareti PID'den önce mutlaka fiziksel olarak kanıtlanmalı; ters işaret pozitif geri
beslemeye dönüşüp sistemi devirebilir. Breadboard geometrisi gerçek `GEOM_*` değerleri
olmadığı için bu aşamada açıların yalnız işaretine güvenin, büyüklüğüne değil.

**3. Bobin R/L testi** (kritik yolda, şu an bobin ve 100 kΩ direnç eksik). Tek kanal
BTS7960 + bus bölücüsü bağlanınca:
firmware yüklenir, mavi butona basılı tutarak reset atılır, `g_coil_cap` (4096 halfword)
SWD ile çekilir, python'da eğri oturtulur. Çıkacaklar:
- **gerçek R ve L** (hesaplanan 1.9 Ω / 32 mH doğru mu)
- **BTS7960 IS pininin gerçek bant genişliği** ← akı gözleyicisi kararının kaderi buna bağlı
- **batarya çökmesi** yük altında

Bu testin sonucu §13'teki iki kararı (hava aralığı, PWM frekansı) birden çözecek.

---

## 13. Açık kararlar ve işler

| Konu | Durum |
|---|---|
| Hava aralığı 3 mm mi 5 mm mi | R/L testinden sonra (§9.3 — 3 mm öneriliyor) |
| PWM frekansı 20 kHz mi 5 kHz mi | R/L testinden sonra (§10.3) |
| BTS7960 pin planı / CubeMX | Tamamlandı: RPWM=TIM1, LPWM=GND, ortak EN GPIO, R_IS=ADC1; Generate Code ve derleme temiz |
| BTS7960 yüksüz bağlantı/çıkış testi | Pin planı hazır; yeni LPWM=GND düzeniyle güç verilmeden yeniden kablolanacak (§12) |
| R_IS ADC giriş koruması | PCB şematiğinden önce kesinleştirilecek; 2.2 kΩ ölçüm direnci yanında 3.3 V koruması gerekli |
| Bobin R/L karakterizasyonu | Bobin tezgâhta yok; test başlatılamaz |
| VBUS 100 kΩ / 10 kΩ bölücüsü | 100 kΩ direnç yok; PB1 bağlanmadı |
| ToF ofset kalibrasyonu (~5 mm) | Yapılmadı |
| Şasi sensör aralıkları (`GEOM_*`) | Ölçülmedi, 400×300 mm varsayım |
| Sert plaka ile heave/roll/pitch ve 3-sensör failover | Yapılmadı; sıradaki sensör testi (§12) |
| TOF050C'de LDO var mı | 4.5 V beslemede çalışması doğrulandı; parça kodu belgelenmedi (§2) |
| AVG 0xA0 + 10 ms kombinasyonunu sürücüde engelleme | Yapılmadı |
| BNO055 sürücüsü (AMG modu) | Yazılmadı |
| Aşama 4: PID + tahsis matrisi | Başlanmadı |
| Akı gözleyicisi | Başlanmadı (R/L bekliyor) |
| Telemetri / yer istasyonu (EK1‑3.4) | Arayüz hazır (`SM_ReportGroundStation`), link yok |
| I2S3 / USB_HOST kalıntılarını `.ioc`'den temizleme | Yapılmadı, zarar vermiyor |

---

## 14. Tuzaklar — aynı duvara iki kez toslamamak için

### 14.1 VL6180X register adresleri **16 bit**
8 bit adreslemeyle sensörden hiç cevap alınamaz. En sık yapılan hata.

### 14.2 `.ioc`'yi elle yamalamanın sınırı
Pin atamaları, DMA ve GPIO blokları elle yazıldığında CubeMX kabul etti;
**ADC1 ve TIM1 parametre bloklarını sessizce sildi** — pinler kaldı ama `MX_ADC1_Init`/
`MX_TIM1_Init` hiç üretilmedi. Yeni çevre birimi eklerken **parametreleri GUI'den girin.**

Ayrıca **F407'de ADC düzenli kanal tetikleyicileri arasında T1_TRGO yok** (sadece
T1_CC1/CC2/CC3). Bu yüzden ADC tetiği TIM2'ye bağlandı.

Bir de: CubeIDE'de `.ioc` düzenlemeleri **Ctrl+S yapılana kadar diske yazılmaz**.
Dosya damgasına bakmak (`ls -l --time-style=+%H:%M:%S PID_Levo.ioc`) en hızlı kontrol.

### 14.3 `CoilTest_Init()` çevre birimlerini ele geçirir
TIM2'yi 100 kHz'e çeker. Her açılışta çağrıldığında normal ADC tetiğini bozuyordu.
Artık yalnızca test istendiğinde çağrılıyor ve sonrasında CubeMX yapılandırmasına dönülüyor.

### 14.4 ADC'nin DMA kesmeleri **kapalı olmalı**
Tampon dairesel ve doğrudan okunuyor; TC+HT açık bırakılırsa tetik başına iki kesme oluşur.
100 kHz'lik yanlış tetikle birlikte **saniyede 200 bin kesme** oldu ve ana döngü açılışı
hiç bitiremedi. Teşhis yöntemi: PC'yi 8 kez örnekle, hepsi aynı ISR'de çıkıyorsa fırtına var.
Normal current_sense yolu gibi coil_test yolu da HAL_ADC_Start_DMA çağrısından hemen sonra
TC/HT/TE/DME kesmelerini kapatmalı ve DMA2_Stream0 NVIC hattını susturmalıdır.

### 14.5 VL6180X, MCU resetinden **etkilenmez**
Önceki çevrimden sürekli modda kalabilir ve o haldeyken yapılandırma yazmalarını sessizce
yok sayar. Init artık sakinleştirme yapıyor (§11.3). Ama sensör tamamen takılırsa
**yazılımla kurtarılamaz — fiziksel güç çevrimi gerekir** (USB'yi çıkarıp takmak).
2026‑08‑29'da bir kez yaşandı ve teşhisi uzun sürdü.

### 14.6 IWDG başlatma sırası kritik
LSI osilatörü ancak `KR = 0xCCCC` yazılınca çalışır. `PR/RLR` yazıp `SR`'nin temizlenmesini
**önce** beklemek sonsuz döngüdür (LSI durduğu için SR hiç temizlenmez).
Doğru sıra: `0xCCCC → 0x5555 → PR/RLR → SR bekle (sınırlı) → 0xAAAA`.

### 14.7 Çoklayıcıyı taramadan önce `TCA_Init()` çağırın
Reset darbesi almadan taranan çoklayıcı hiç görünmüyor.

### 14.8 PE0 kullanılamaz
DISC1'de ivmeölçerin INT1 çıkışına bağlı.

### 14.9 I2C1 (PB6/PB9) CS43L22 ses codec'i ile paylaşımlı
Codec `PD4 = 0` ile reset'te tutuluyor (`main.c` USER CODE 2'nin ilk satırı), hattı rahatsız etmiyor.
**Bu satırı silmeyin.**

### 14.10 TOF050C ölü bölgesi 0–20 mm
Sensör ray yüzeyinden ~25–30 mm geriye kaçırılmalı; `GAP_SENSOR_OFFSET_MM` bunu telafi ediyor.

### 14.11 4 sensörün güç bütçesi
4 × VL6180X ≈ 160 mA, DISC1'in 3.3 V regülatörü ~150 mA. Modülde LDO varsa 5 V'tan besleyin.

### 14.12 Tezgâh testinde görülen iki yanıltıcı durum

- Kanal 1 hiç bulunmuyorsa önce **SD1/SDA ile SC1/SCL'nin ters bağlanmadığını** kontrol edin.
  2026-08-29 testinde FR'nin kayıp olmasının gerçek nedeni buydu; timeout değiştirilmedi.
- `read_vars.py`, `build/PID_Levo.elf` sembollerini kullanır. CubeIDE farklı `Debug` ELF'ini
  yüklemişse veya çekirdeği durdurmuşsa yanlış adreslerden sıfır okuyabilir. CubeIDE debug
  oturumunu kapatıp belgelenmiş ELF'i yeniden yükleyin; Live Expressions kullanırken hedefin
  çalışması için F8/Resume gerekir.

---

## 15. Çalışma tarzı — kullanıcı tercihleri

- **Adım adım gidilir.** Kullanıcı "adım adım gidelim" dedi; büyük sıçramalar yerine her aşama
  doğrulanarak ilerleniyor.
- **CubeMX iş bölümü:** `.ioc`'yi asistan düzenler, **Generate Code'u kullanıcı yapar.**
  Ama §14.2'deki sınır yüzünden yeni çevre birimi eklerken parametreler GUI'den girilmeli.
- **Kart bağlıysa asistan doğrudan yükleyip okuyabilir** (`tools/`). Kullanıcı "bi baksana"
  dediğinde beklenen budur.
- **Ölçmeden karar verilmez.** Sensör hızı, gürültüsü, filtre kazancı — hepsi tahmin değil,
  kartta ölçüldü. Bu yaklaşım birkaç kez yanlış varsayımı yakaladı.
- Kullanıcı **sipariş veremiyor**; çözümler eldeki donanımla kurgulanmalı.
- Yanlış çıkan kendi ifadelerim açıkça düzeltildi; bu belgede de öyle işaretli.

---

## 16. TGR raporuna doğrudan girecek malzeme

| Şartname maddesi | Besleyen çalışma |
|---|---|
| **4.d "Veri Kalitesi"** | Hız/gürültü taraması, filtre takası, tekrarlanabilir ölçüm yöntemi (§8) |
| **4.d "Sistem Stabilitesi"** | p·τ kararlılık bütçesi, sensör seçiminin sayısal gerekçesi (§9.1) |
| **3.t FMEA** | Hata bit maskesi, mandallı kapanma, IWDG, I²C hattı kurtarma (§11.5) |
| **3.q / 3.r** | Akım/gerilim ölçüm zinciri, zirve kayıtları (§11.6) |
| **3.p** | 3 sensörle düzlem çözme — bozulmuş çalışma yeteneğinin kanıtı (§11.4) |
| **3.n / 3.i** | Termal analiz: akım yoğunluğu, hava aralığı takası (§9.3) |
