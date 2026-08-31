/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   Uygulama geneli yapilandirma sabitleri.
  *          Donanim degisirse ONCE burasi guncellenir; surucu dosyalari
  *          dogrudan pin/adres sabiti icermez.
  *
  * NOT: Dosyalar ASCII karakterlerle yazilmistir (Turkce karakter yok) --
  *      STM32CubeIDE kod uretimi ve derleyici kodlama sorunlarini onlemek icin.
  ******************************************************************************
  */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* CubeMX'in main.c icinde urettigi I2C tutamaci */
extern I2C_HandleTypeDef hi2c1;

/* ============================ I2C veri yolu ============================ */
#define APP_I2C                 (&hi2c1)
#define APP_I2C_TIMEOUT_MS      10U
#define APP_I2C_RETRY_COUNT     2U      /* hata halinde toplam deneme = 1+2 */

/* I2C1 pinleri -- takilan hatti kurtarmak (bus recovery) icin gerekli */
#define APP_I2C_SCL_PORT        GPIOB
#define APP_I2C_SCL_PIN         GPIO_PIN_6
#define APP_I2C_SDA_PORT        GPIOB
#define APP_I2C_SDA_PIN         GPIO_PIN_9

/* ============================ TCA9548A ================================= */
#define TCA9548A_ADDR_7B        0x70U       /* A0..A2 = GND */
#define TCA_RST_PORT            GPIOE
#define TCA_RST_PIN             GPIO_PIN_4  /* MUX_RST, aktif-dusuk */

/* Sensor -> coklayici kanal haritasi.
   Asama 1'de yalnizca CH_0 kullaniliyor; Asama 2'de 4'e cikacak. */
#define GAP_CH_FL               0U          /* on-sol   (front-left)  */
#define GAP_CH_FR               1U          /* on-sag   (front-right) */
#define GAP_CH_RL               2U          /* arka-sol (rear-left)   */
#define GAP_CH_RR               3U          /* arka-sag (rear-right)  */
#define GAP_CH_COUNT            4U

/* Sensorlerin sasi merkezine gore yerlesimi.
   Sensorler (+-GEOM_HALF_LENGTH_MM, +-GEOM_HALF_WIDTH_MM) koselerinde
   varsayiliyor: yani on-arka sensor aciklgi 2*HALF_LENGTH, sol-sag aciklik
   2*HALF_WIDTH olur.
   OLCULMESI GEREKIYOR -- su an 400 x 300 mm'lik bir sasi varsayimi.
   Yanlis deger heave'i etkilemez ama roll/pitch acilarini olceklendirir. */
#define GEOM_HALF_LENGTH_MM     200.0f
#define GEOM_HALF_WIDTH_MM      150.0f

/* Round-robin yoklama adimi. Her cagride TEK kanal yoklanir; 4 kanalda
   kanal basina yoklama araligi 4 x bu deger olur. Sensorler 100 Hz
   (10 ms) uretiyor, 4 ms'lik ziyaret araligi rahatlikla yetisir. */
#define GAP_POLL_STEP_MS        1U

/* Estimator iki sensorle sinirli bir teshis sonucu uretebilir; ancak
   kontrol/safety zinciri icin bir duzlemi belirleyen en az 3 nokta gerekir. */
#define EST_MIN_CONTROL_SENSORS 3U

/* ============================ Hava araligi ============================= */
/* TOF050C (VL6180X) modulunun olu bolgesi 0-20 mm oldugu icin sensor,
   ray yuzeyinden geriye kacirilarak monte edilir:
       hava_araligi = olculen_mesafe - GAP_SENSOR_OFFSET_MM
   Bu deger, sensor montaji yapildiktan sonra kalibrasyonla kesinlestirilir. */
#define GAP_SENSOR_OFFSET_MM    25.0f
#define GAP_MIN_MM              2.0f        /* sartname 3.u alt limiti */
#define GAP_TARGET_MM           6.0f        /* hedef askilama yuksekligi */

/* ============================ Zamanlama =============================== */
#define SENSOR_POLL_PERIOD_MS   5U          /* ana dongude yoklama periyodu */

/* ======================= Guc katmani (BTS7960) ========================
   Pinler .ioc'ye islendi; CubeMX MX_GPIO_Init / MX_TIM1_Init / MX_ADC1_Init
   uretir. Elle duzenlenen stm32f4xx_hal_conf.h'ye artik gerek yok.

   Kanal sirasi her yerde ayni: 0=FL, 1=FR, 2=RL, 3=RR (GAP_CH_* ile ortak).

     kanal   RPWM (TIM1)   LPWM           EN (R_EN+L_EN kopru)   IS (ADC1)
     FL      PE9  CH1      GND            PE7                    PA1  IN1
     FR      PE11 CH2      GND            PE2                    PA2  IN2
     RL      PE13 CH3      GND            PE5                    PA3  IN3
     RR      PE14 CH4      GND            PE6                    PB0  IN8
                                                        VBUS -> PB1  IN9

   NOT: PE0 KULLANILMAZ -- DISC1'de ivmeolcerin INT1 cikisina baglidir.
   NOT: EN pinlerinde donanimda 10k pull-down SART; reset aninda GPIO'lar
        yuksek empedanstir ve BTS7960 kendiliginden iletime gecebilir.   */
#define BTS_CH_COUNT            4U

/* Pin makrolari CubeMX tarafindan main.h'de uretilir; .ioc tek dogruluk
   kaynagidir. Pin degisirse burasi kendiliginden takip eder. */
#define BTS_EN_PORTS            { EN_FL_GPIO_Port,   EN_FR_GPIO_Port,                                     EN_RL_GPIO_Port,   EN_RR_GPIO_Port   }
#define BTS_EN_PINS             { EN_FL_Pin,         EN_FR_Pin,                                           EN_RL_Pin,         EN_RR_Pin         }
#define BTS_PWM_CHANNELS        { TIM_CHANNEL_1, TIM_CHANNEL_2, \
                                  TIM_CHANNEL_3, TIM_CHANNEL_4 }

/* PWM: merkez hizali mod 1 -> f = 168 MHz / (2*(ARR+1)) = 20 kHz.
   BTS7960'in ust siniri ~25 kHz; 20 kHz hem duyulabilir bandin disinda
   hem de anahtarlama kaybini makul tutuyor. */
#define BTS_PWM_ARR             4199U
#define BTS_PWM_FREQ_HZ         20000U

/* Tek kanalli testler (coil_test) icin FL takma adlari */
#define BTS_RPWM_PORT           GPIOE
#define BTS_RPWM_PIN            GPIO_PIN_9    /* TIM1_CH1, AF1 */
#define BTS_EN_PORT             EN_FL_GPIO_Port
#define BTS_EN_PIN              EN_FL_Pin

/* Duty ust siniri. %100'de BTS7960'in yuksek yan surucusu sarj pompasini
   besleyemez (bootstrap kondansatoru dolamaz); kisa bir OFF penceresi sart. */
#define BTS_DUTY_MAX            0.95f

/* ADC1 tarama sirasi = DMA tamponundaki indeks */
#define ADC_IDX_IS_FL           0U
#define ADC_IDX_IS_FR           1U
#define ADC_IDX_IS_RL           2U
#define ADC_IDX_IS_RR           3U
#define ADC_IDX_VBUS            4U
#define ADC_CH_COUNT            5U

/* ADC tetigi: TIM2 TRGO. TIM2 saati 84 MHz (APB1 x2).
   40 kHz -> PWM'in tam iki kati; 5 kanallik tarama ~9.5 us surdugu icin
   25 us'lik pencereye rahat sigar. */
#define ADC_TRIG_HZ             40000U
#define ADC_TRIG_ARR            ((84000000U / ADC_TRIG_HZ) - 1U)
#define ADC_DATA_TIMEOUT_MS     10U

/* Akim algilama: BTS7960 IS pini bir akim kaynagidir (I_yuk / 8500).
   GND'ye baglanan direnc onu gerilime cevirir.
   2200 ohm -> 0.2588 V/A, tam skala 3.3 V = 12.75 A                    */
#define BTS_IS_RESISTOR_OHM     2200.0f
#define BTS_IS_RATIO            8500.0f
#define BTS_IS_VOLT_PER_AMP     (BTS_IS_RESISTOR_OHM / BTS_IS_RATIO)

/* Bus gerilimi bolucusu: ust 100k, alt 10k -> 11:1, 29.2 V -> 2.65 V   */
#define VBUS_DIVIDER_RATIO      11.0f

#define ADC_VREF                3.3f
#define ADC_FULL_SCALE          4095.0f

/* Guvenlik: yakalama sirasinda bu akim asilirsa cikis derhal kesilir */
#define COIL_TEST_ABORT_AMP     9.0f
#define COIL_TEST_MAX_ON_MS     12U

/* ==================== Kontrol dongusu ve emniyet ======================
   Esikler ilk kurulum degerleridir; R/L testi ve ilk askilama denemesinden
   sonra olculen degerlere gore guncellenecek.                          */
#define CTRL_PERIOD_MS          1U          /* durum makinesi / govde dongusu */

/* Akim limitleri (sartname 3.q -- olculen akim ayni zamanda emniyet girdisi) */
#define COIL_CURRENT_MAX_A      8.0f        /* bu asilirsa FAULT           */
#define COIL_CURRENT_TRIP_MS    5U          /* asim bu kadar surerse trip  */

/* Bus gerilimi limitleri: 8S LiFePO4 -> 20.0 V (bos) .. 29.2 V (tam dolu) */
#define VBUS_MIN_V              19.0f       /* alt kesim: guvenli inis     */
#define VBUS_MAX_V              30.5f
#define VBUS_TRIP_MS            20U

/* Sensor ve haberlesme zaman asimlari */
#define SENSOR_TIMEOUT_MS       50U         /* sartname 3.p: modul arizasi */
#define GS_TIMEOUT_MS           1000U       /* EK1-3.4: yer istasyonu      */

/* Yumusak kalkis: duty bu hizla artar (birim: duty/saniye) */
#define SOFT_START_RATE         2.0f
#define SOFT_START_TIMEOUT_MS   3000U

/* Guvenli inis (sartname 3.s): ani kesme yerine kontrollu rampa. Kalkistan
   hizli, ama serbest dususten yavas -- sasi raya carpmadan oturur. */
#define LANDING_RATE            3.0f
#define LANDING_TIMEOUT_MS      1000U

/* Askiya gecis kabul esigi: olculen aralik hedefin bu kadar yakininda
   SETTLE_MS boyunca kalirsa SOFT_START -> LEVITATING */
#define SOFT_START_BAND_MM      1.5f
#define SOFT_START_SETTLE_MS    200U

/* Bagimsiz bekci kopegi. LSI ~32 kHz, bolucu 32 -> 1 tik = 1 ms.
   RLR = 250 -> ~250 ms. Kontrol dongusu bunu beslemezse donanim reseti
   olur ve tum EN pinleri (pull-down sayesinde) LOW'a duser.            */
#define IWDG_RELOAD_TICKS       250U

#ifdef __cplusplus
}
#endif
#endif /* APP_CONFIG_H */
