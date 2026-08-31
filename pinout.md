# PID_Levo Pinout

Bu belge STM32F407VGT6 / STM32F407G-DISC1 icin kararlastirilan pinleri ve
BTS7960 baglanti seklini tek yerde toplar.

## 1. BTS7960 pin tablosu

Sistem dort elektromiknatisi tek yonlu surecektir. Her BTS7960 icin yalnizca
`RPWM` donanim PWM cikisi olarak kullanilir. `LPWM` surekli LOW tutulur.

| Kanal | Konum | RPWM | Timer kanali | LPWM | R_EN + L_EN | R_IS | ADC kanali | L_IS |
|---|---|---|---|---|---|---|---|---|
| FL | On sol | PE9 | TIM1_CH1 / AF1 | GND | PE7 | PA1 | ADC1_IN1 | NC / test noktasi |
| FR | On sag | PE11 | TIM1_CH2 / AF1 | GND | PE2 | PA2 | ADC1_IN2 | NC / test noktasi |
| RL | Arka sol | PE13 | TIM1_CH3 / AF1 | GND | PE5 | PA3 | ADC1_IN3 | NC / test noktasi |
| RR | Arka sag | PE14 | TIM1_CH4 / AF1 | GND | PE6 | PB0 | ADC1_IN8 | NC / test noktasi |

### Kanal basina baglanti

```text
STM32 RPWM ------------------------ RPWM
GND ------------------------------- LPWM

STM32 EN --------------------------+-- R_EN
                                   +-- L_EN
                                   |
                                  10 kOhm
                                   |
                                  GND

STM32 ADC ------------------------- R_IS
                                     |
                                   2.2 kOhm
                                     |
                                    GND

L_IS ------------------------------ NC / test noktasi
STM32 5V -------------------------- VCC
Ortak GND ------------------------- GND
Bobin ----------------------------- M+ / M-
Guclu besleme --------------------- B+ / B-
```

### Surme mantigi

| EN | RPWM | LPWM | Durum |
|---:|---:|---:|---|
| 0 | X | 0 | Kopru kapali / guvenli durum |
| 1 | PWM | 0 | Tek yonlu bobin akimi |

- Her surucude `R_EN` ve `L_EN` birbirine baglanir ve tek STM32 GPIO'su ile
  kontrol edilir.
- Ortak EN hattindaki 10 kOhm pull-down, STM32 reset durumundayken surucuyu
  kapali tutar.
- `R_IS`, RPWM tarafindaki aktif high-side akimini olcmek icin kullanilir.
- `L_IS` normal kontrolde kullanilmaz; PCB uzerinde test noktasi birakilmasi
  onerilir.
- `R_IS` ADC girisine 3.3 V asiri gerilim korumasi eklenmelidir. IS pini ariza
  durumunda normal olcum seviyesinden daha yuksek gerilim uretebilir.

## 2. ADC1 tarama sirasi

ADC1, TIM2 tetigi ve dairesel DMA ile bes kanali sirayla okur.

| DMA sirasi | Pin | ADC kanali | Olculen |
|---:|---|---|---|
| 1 | PA1 | ADC1_IN1 | FL bobin akimi |
| 2 | PA2 | ADC1_IN2 | FR bobin akimi |
| 3 | PA3 | ADC1_IN3 | RL bobin akimi |
| 4 | PB0 | ADC1_IN8 | RR bobin akimi |
| 5 | PB1 | ADC1_IN9 | Ortak bus gerilimi |

Bus gerilimi bolucusu:

```text
B+ ---- 100 kOhm ----+---- PB1 / ADC1_IN9
                     |
                   10 kOhm
                     |
                    GND
```

## 3. Sensor ve yardimci pinler

| Islev | STM32 pini | Aciklama |
|---|---|---|
| I2C1 SCL | PB6 | TCA9548A ve kok hatta BNO055 |
| I2C1 SDA | PB9 | TCA9548A ve kok hatta BNO055 |
| TCA9548A RESET | PE4 | MUX reset cikisi |
| Kullanici butonu | PA0 | B1 |
| Yesil LED | PD12 | Olcum / durum gostergesi |
| Turuncu LED | PD13 | Sensor arizasi gostergesi |
| Kirmizi LED | PD14 | Hata gostergesi |
| Mavi LED | PD15 | Bobin testi gostergesi |

TCA9548A kanal eslemesi:

| TCA kanali | Fiziksel konum |
|---:|---|
| 0 | FL - on sol |
| 1 | FR - on sag |
| 2 | RL - arka sol |
| 3 | RR - arka sag |

## 4. Guvenlik ve kart tasarimi notlari

- `PE0` kullanilmaz; Discovery kartindaki ivmeolcer INT1 hatti ile cakisir.
- Dort RPWM cikisi TIM1 uzerindedir ve ayni PWM frekansinda senkron calisir.
- BTS7960, STM32 ve guc kaynagi topraklari ortak olmalidir.
- Her RPWM hattinda GND'ye 10 kOhm pull-down kullanilmalidir.
- B+ hattinda uygun sigorta bulunmalidir.
- Guclu akim yollari ile ADC/IS izleri PCB uzerinde birbirinden uzak tutulmali;
  ADC topragi surucu guc topragina kontrollu bir noktadan baglanmalidir.
- PE8, PE10, PE12 ve PE15 LPWM icin kullanilmaz; bu pinler bostur.
- LPWM girisleri PCB uzerinde dogrudan GND'ye baglanir.
