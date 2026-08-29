/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_config.h"
#include "i2c_bus.h"
#include "tca9548a.h"
#include "vl6180x.h"
#include "coil_test.h"
#include "bts7960.h"
#include "current_sense.h"
#include "state_machine.h"
#include "gap_sensor.h"
#include "estimator.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* ---- Asama 3: guc katmani ve emniyet (Live Expressions icin) ---- */
volatile uint8_t  g_levitate_req = 0;   /* 1 yazilinca kalkis denenir      */
volatile uint8_t  g_sm_state     = 0;   /* sm_state_t                      */
volatile uint32_t g_sm_faults    = 0;   /* SM_F_* bit maskesi              */
volatile float    g_coil_i_a[BTS_CH_COUNT];
volatile float    g_duty[BTS_CH_COUNT];
volatile float    g_vbus_v       = 0.0f;
volatile float    g_power_w      = 0.0f;
volatile float    g_peak_i_a     = 0.0f;  /* kalkis zirvesi (sartname 3.q) */
volatile float    g_peak_p_w     = 0.0f;
volatile uint8_t  g_drv_init_err = 0;

/* ---- Asama 2: 4 kanalli hava araligi + kestirim (Live Expressions) ---- */
volatile uint8_t  g_gap_found   = 0;   /* acilista bulunan sensor sayisi   */
volatile uint8_t  g_gap_valid_n = 0;   /* su an gecerli olan kanal sayisi  */
volatile uint8_t  g_gap_present[GAP_CH_COUNT];
volatile uint8_t  g_gap_fault[GAP_CH_COUNT];
volatile uint8_t  g_gap_raw[GAP_CH_COUNT];   /* ham okuma [mm]             */
volatile float    g_gap_ch[GAP_CH_COUNT];    /* kalibre hava araligi [mm]  */
volatile uint16_t g_gap_hz[GAP_CH_COUNT];
volatile uint8_t  g_gap_status[GAP_CH_COUNT];

volatile uint8_t  g_est_valid    = 0;
volatile uint8_t  g_est_n_used   = 0;
volatile float    g_heave_mm     = 0.0f;
volatile float    g_roll_mrad    = 0.0f;   /* milirad, okunakli olsun diye */
volatile float    g_pitch_mrad   = 0.0f;
volatile float    g_est_resid_mm = 0.0f;

/* ===== Asama 1 teshis degiskenleri =====================================
   Bunlar CubeIDE'de Debug -> Live Expressions penceresine eklenerek
   canli izlenir. volatile olmalari, derleyicinin optimize edip yok
   etmesini onler.                                                      */
volatile uint8_t  g_range_mm       = 0;     /* sensorun ham olcumu [mm]      */
volatile float    g_gap_mm         = 0.0f;  /* hava araligi = ham - offset   */
volatile uint8_t  g_range_status   = 0;     /* 0 = OK, digerleri hata kodu   */
volatile uint32_t g_ok_count       = 0;     /* hatasiz ornek sayisi          */
volatile uint32_t g_i2c_err_count  = 0;     /* I2C seviyesi hata sayisi      */
volatile uint32_t g_sample_hz      = 0;     /* olculen gercek ornekleme hizi */
volatile uint8_t  g_sensor_present = 0;     /* 1 = sensor bulundu            */

/* I2C tarama sonuclari -- ilk teshis adimi:
     g_scan_root[] icinde 0x70 (coklayici) gorulmelidir.
     g_scan_ch0[]  icinde 0x29 (VL6180X)  gorulmelidir.
   0x29 yoksa sorun sensor kablolamasinda ya da beslemesindedir.        */
volatile uint8_t  g_scan_root[8];
volatile uint8_t  g_scan_root_n = 0;
volatile uint8_t  g_scan_ch0[8];
volatile uint8_t  g_scan_ch0_n  = 0;

/* Sensor durumu (sample_count, io_err_count, status_err_count alanlari
   da Live Expressions'ta izlenebilir). */
vl6180x_t g_tof0;

/* ===== Teshis: sensorun kendi registerlarindan geri okunan degerler =====
   Yazdigimiz ayarlar sensore gercekten islendi mi, ve sensor ne
   raporluyor? Surekli mod ornek uretmezse cevap burada.                 */
volatile uint8_t g_dbg_mode_gpio1 = 0xAA;  /* 0x011, beklenen 0x10 */
volatile uint8_t g_dbg_int_cfg    = 0xAA;  /* 0x014, beklenen 0x04 */
volatile uint8_t g_dbg_imp        = 0xAA;  /* 0x01B, olcumler arasi periyot */
volatile uint8_t g_dbg_maxconv    = 0xAA;  /* 0x01C, maks yakinsama suresi  */
volatile uint8_t g_dbg_avg        = 0xAA;  /* 0x10A, ortalama periyodu      */
volatile uint8_t g_dbg_start      = 0xAA;  /* 0x018, beklenen 0x03          */
volatile uint8_t g_dbg_irq_raw    = 0xAA;  /* 0x04F, ham kesme durumu       */
volatile uint8_t g_dbg_rs_raw     = 0xAA;  /* 0x04D, ham menzil durumu      */

/* Tek atimlik (blokleyici) olcum: sensor hic olcum yapabiliyor mu?
   Bu, surekli moddan bagimsiz bir kanittir. */
volatile uint8_t g_single_mm     = 0;
volatile int8_t  g_single_ret    = -1;     /* 0 = HAL_OK */
volatile uint8_t g_single_status = 0xAA;

/* ===== Ornekleme hizi taramasi =====
   Acilista ortalama periyodu (AVG) ve olcumler arasi periyot (IMP)
   kombinasyonlari denenip her biri icin 500 ms'de kac ornek geldigi
   olculur. En hizli ve hatasiz kombinasyon secilerek calisilir.       */
/* Gurultu-hiz takasi taramasi: ortalama periyodu (AVG) ve maksimum
   yakinsama suresi (CONV) kombinasyonlari icin 1 saniyede kac ornek
   geldigi ve o orneklerin dagilimi olculur. Olcumler arasi periyot
   10 ms'de sabit -- sensor zaten olcum suresiyle sinirli.            */
#define SWEEP_N 8
volatile uint8_t  g_sweep_avg[SWEEP_N];
volatile uint8_t  g_sweep_conv[SWEEP_N];
volatile uint16_t g_sweep_hz[SWEEP_N];        /* 1 s icindeki ornek = Hz */
volatile uint16_t g_sweep_err[SWEEP_N];       /* olcum hatasi sayisi     */
volatile uint8_t  g_sweep_min[SWEEP_N];
volatile uint8_t  g_sweep_max[SWEEP_N];
volatile uint8_t  g_sweep_pp[SWEEP_N];        /* tepe-tepe [mm]          */
volatile float    g_sweep_mean[SWEEP_N];
volatile float    g_sweep_std[SWEEP_N];       /* sigma [mm]              */
volatile uint8_t  g_sweep_done = 0;
volatile uint8_t  g_best_idx   = 0xFF;
volatile uint16_t g_best_hz    = 0;

/* ===== Yazilim filtresi karsilastirmasi =====
   Ayni ornek akisi uzerinde es zamanli calisan filtreler:
     0 = ham, 1 = 4-ornek ortalama, 2 = 8-ornek, 3 = 16-ornek, 4 = 5-medyan
   Her biri icin 3 saniyelik pencerede sigma ve tepe-tepe olculur. Gecikme
   bedeli: N-ornek ortalama ~ (N-1)/2 ornek = 100 Hz'de (N-1)*5 ms.        */
#define FILT_N 5
volatile float    g_filt_mean[FILT_N];
volatile float    g_filt_std[FILT_N];
volatile float    g_filt_pp[FILT_N];
volatile uint16_t g_filt_n = 0;

/* ===== Gurultu istatistigi =====
   Her 2 saniyelik pencerede hatasiz orneklerin min/maks/ortalama/standart
   sapmasi. g_stat_pp (tepe-tepe bandi), ToF'un levitasyon icin yeterli olup
   olmadigini soyleyecek kritik sayidir. */
volatile uint8_t  g_stat_min  = 0;
volatile uint8_t  g_stat_max  = 0;
volatile uint8_t  g_stat_pp   = 0;    /* tepe-tepe [mm] */
volatile float    g_stat_mean = 0.0f;
volatile float    g_stat_std  = 0.0f;
volatile uint16_t g_stat_n    = 0;    /* penceredeki ornek sayisi */

/* ===== Bobin karakterizasyon testi tetikleri =====
   Test MIKNATISI ENERJILENDIRIR, bu yuzden kazara calismaz:
     - acilista mavi butonu (B1) basili tutarak reset  -> calisir
     - ya da hata ayiklayicidan g_coil_arm = 0xA5      -> bir kez calisir
   Baska hicbir kosulda cikis surulmez.                              */
volatile uint8_t g_coil_arm      = 0;
volatile uint8_t g_coil_run_count = 0;

/* ToF hiz/gurultu taramasi artik her acilista calismiyor (10 s suruyordu);
   olculen en iyi ayar dogrudan uygulaniyor. Tekrar taramak icin
   hata ayiklayicidan g_tof_do_sweep = 1 yapip reset atin.            */
volatile uint8_t g_tof_do_sweep = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ===== ToF karakterizasyon yardimcilari =====
   Bu fonksiyonlar sensorun hiz/gurultu/filtre davranisini olcmek icin
   yazildi ve sonuclari ozet.md'ye islendi. Normal calismada gerekmiyor,
   ama olcumu tekrarlamak gerekirse asagidaki satiri 1 yapmak yeterli.
   Derleme disi birakiliyor ki 'kullanilmiyor' uyarisi uretmesinler. */
#define TOF_CHARACTERIZATION  0
#if TOF_CHARACTERIZATION
/* Ortalama periyodu ve olcumler arasi periyot kombinasyonlarini deneyip
   en hizli hatasiz olani secer. Cagrilmadan once mux kanali acik olmalidir. */
static void TOF_ApplyConfig(uint8_t avg, uint8_t conv)
{
  (void)VL6180X_StopContinuous(&g_tof0);
  HAL_Delay(60);
  (void)VL6180X_WriteReg(0x010A, avg);      /* READOUT__AVERAGING_SAMPLE_PERIOD */
  (void)VL6180X_WriteReg(0x001C, conv);     /* SYSRANGE__MAX_CONVERGENCE_TIME   */
  (void)VL6180X_WriteReg(0x0015, 0x07);     /* kesmeleri temizle                */
  (void)VL6180X_StartContinuous(&g_tof0, 10);
}

/* ---- Yazilim filtresi karsilastirmasi ---- */
static uint8_t  s_rb[16];
static uint8_t  s_rb_i = 0, s_rb_cnt = 0;
static float    s_acc_sum[FILT_N], s_acc_sq[FILT_N];
static float    s_acc_min[FILT_N], s_acc_max[FILT_N];
static uint16_t s_acc_n = 0;

static float Filt_MovAvg(uint8_t n)
{
  uint32_t s = 0;
  for (uint8_t k = 0; k < n; k++)
  {
    s += s_rb[(uint8_t)((s_rb_i + 16U - 1U - k) & 0x0FU)];
  }
  return (float)s / (float)n;
}

static float Filt_Median5(void)
{
  uint8_t v[5];
  for (uint8_t k = 0; k < 5U; k++)
  {
    v[k] = s_rb[(uint8_t)((s_rb_i + 16U - 1U - k) & 0x0FU)];
  }
  for (uint8_t i = 0; i < 4U; i++)
  {
    for (uint8_t j = (uint8_t)(i + 1U); j < 5U; j++)
    {
      if (v[j] < v[i]) { uint8_t t = v[i]; v[i] = v[j]; v[j] = t; }
    }
  }
  return (float)v[2];
}

static void Filt_Push(uint8_t val)
{
  s_rb[s_rb_i] = val;
  s_rb_i = (uint8_t)((s_rb_i + 1U) & 0x0FU);
  if (s_rb_cnt < 16U) { s_rb_cnt++; return; }   /* tampon dolana kadar sayma */

  float f[FILT_N];
  f[0] = (float)val;
  f[1] = Filt_MovAvg(4);
  f[2] = Filt_MovAvg(8);
  f[3] = Filt_MovAvg(16);
  f[4] = Filt_Median5();

  for (uint8_t i = 0; i < FILT_N; i++)
  {
    if (s_acc_n == 0U) { s_acc_min[i] = f[i]; s_acc_max[i] = f[i]; }
    else
    {
      if (f[i] < s_acc_min[i]) { s_acc_min[i] = f[i]; }
      if (f[i] > s_acc_max[i]) { s_acc_max[i] = f[i]; }
    }
    s_acc_sum[i] += f[i];
    s_acc_sq[i]  += f[i] * f[i];
  }
  s_acc_n++;
}

static void Filt_Publish(void)
{
  if (s_acc_n < 2U) { return; }

  for (uint8_t i = 0; i < FILT_N; i++)
  {
    float mean = s_acc_sum[i] / (float)s_acc_n;
    float var  = (s_acc_sq[i] / (float)s_acc_n) - (mean * mean);
    if (var < 0.0f) { var = 0.0f; }

    g_filt_mean[i] = mean;
    g_filt_std[i]  = sqrtf(var);
    g_filt_pp[i]   = s_acc_max[i] - s_acc_min[i];

    s_acc_sum[i] = 0.0f;
    s_acc_sq[i]  = 0.0f;
  }
  g_filt_n = s_acc_n;
  s_acc_n  = 0;
}

/* Her kombinasyon icin 1 saniye olcum toplar; hiz ve dagilimi kaydeder. */
static void TOF_NoiseSweep(void)
{
  static const uint8_t avgs[4]  = { 0x18U, 0x30U, 0x60U, 0xA0U };
  static const uint8_t convs[2] = { 0x0AU, 0x31U };   /* 10 ms / 49 ms */
  uint8_t idx = 0;

  for (uint8_t a = 0; a < 4U; a++)
  {
    for (uint8_t c = 0; c < 2U; c++)
    {
      uint32_t t0, sum = 0, sumsq = 0;
      uint16_t cnt = 0, serr = 0;
      uint8_t  mn = 255, mx = 0;

      TOF_ApplyConfig(avgs[a], convs[c]);
      HAL_Delay(200);                        /* yerlesme */

      t0 = HAL_GetTick();
      while ((HAL_GetTick() - t0) < 1000U)
      {
        if (VL6180X_Poll(&g_tof0) == 1)
        {
          if (g_tof0.last_status != VL6180X_RS_NO_ERROR) { serr++; continue; }

          uint8_t v = g_tof0.last_range_mm;
          if (v < mn) { mn = v; }
          if (v > mx) { mx = v; }
          sum   += v;
          sumsq += (uint32_t)v * v;
          cnt++;
        }
      }

      g_sweep_avg[idx]  = avgs[a];
      g_sweep_conv[idx] = convs[c];
      g_sweep_hz[idx]   = cnt;
      g_sweep_err[idx]  = serr;

      if (cnt > 1U)
      {
        float mean = (float)sum / (float)cnt;
        float var  = ((float)sumsq / (float)cnt) - (mean * mean);
        if (var < 0.0f) { var = 0.0f; }

        g_sweep_min[idx]  = mn;
        g_sweep_max[idx]  = mx;
        g_sweep_pp[idx]   = (uint8_t)(mx - mn);
        g_sweep_mean[idx] = mean;
        g_sweep_std[idx]  = sqrtf(var);
      }
      idx++;
    }
  }
  g_sweep_done = idx;

  /* En yuksek hizi veren hatasiz kombinasyonu sec; esitlikte dusuk sigma.
     Yazilim filtresi karsilastirmasi en yuksek hizda yapilmali. */
  uint16_t best_hz  = 0;
  float    best_std = 1.0e9f;
  for (uint8_t i = 0; i < idx; i++)
  {
    if (g_sweep_err[i] != 0U) { continue; }
    if ((g_sweep_hz[i] > best_hz) ||
        ((g_sweep_hz[i] == best_hz) && (g_sweep_std[i] < best_std)))
    {
      best_hz    = g_sweep_hz[i];
      best_std   = g_sweep_std[i];
      g_best_idx = i;
    }
  }
  if (g_best_idx == 0xFFU) { g_best_idx = 1; }   /* hicbiri uymazsa varsayilan */

  g_best_hz = g_sweep_hz[g_best_idx];
  TOF_ApplyConfig(g_sweep_avg[g_best_idx], g_sweep_conv[g_best_idx]);
}

#endif /* TOF_CHARACTERIZATION */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* ---- Asama 2: coklayici + 4 kanalli hava araligi olcumu ---- */

  /* Kart uzerindeki CS43L22 ses codec'i ayni I2C1 hattinda (adres 0x94).
     Reset'te tutulur ki hatta karismasin. */
  HAL_GPIO_WritePin(Audio_RST_GPIO_Port, Audio_RST_Pin, GPIO_PIN_RESET);

  I2CBus_Init();

  /* Coklayiciyi once reset darbesiyle bilinen duruma cek, sonra tara:
     tum kanallar kapaliyken kok hatta yalnizca 0x70 gorunmelidir. */
  TCA_Init();
  g_scan_root_n = I2CBus_Scan((uint8_t *)g_scan_root, sizeof(g_scan_root));

  g_gap_found = GapSensor_Init();
  EstimatorInit();

  /* Geriye donuk uyumluluk: kanal 0'in durumu eski degisken adlariyla da
     yayimlaniyor, boylece mevcut Live Expressions kurulumu bozulmuyor. */
  g_sensor_present = (uint8_t)(GapSensor_Get(GAP_CH_FL)->present);

  /* Kanal 0'da hangi cihazlar var (teshis) */
  if (TCA_SelectChannel(GAP_CH_FL) == HAL_OK)
  {
    g_scan_ch0_n = I2CBus_Scan((uint8_t *)g_scan_ch0, sizeof(g_scan_ch0));
  }

  /* ---- Bobin karakterizasyon testi ----
     CoilTest_Init() TIM1/TIM2/ADC1/DMA2'yi kendi olcum ayarlarina gore
     YENIDEN programlar (TIM2'yi 100 kHz'e ceker). Bu yuzden yalnizca test
     gercekten istendiginde cagrilir; her acilista cagrilmasi normal
     calismadaki ADC tetigini bozuyordu. Cikislar bu noktada zaten pasif:
     MX_GPIO_Init tum EN/LPWM pinlerini LOW olarak kurdu ve donanimda
     pull-down var. */
  if ((HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET) ||
      (g_coil_arm == 0xA5U))
  {
    g_coil_arm = 0U;
    if (CoilTest_Init() == HAL_OK)
    {
      CoilTest_SafeOff();
      HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET); /* mavi: test */
      CoilTest_Run();
      g_coil_run_count++;
      HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
    }
    /* Cevre birimlerini CubeMX yapilandirmasina geri dondur */
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_ADC1_Init();
  }


  /* ---- Asama 3: guc katmani + emniyet durum makinesi ----
     Bu noktaya kadar tum acilis teshisleri bitmis olmali: SM_Init()
     bagimsiz bekci kopegini baslatir ve bundan sonra ana dongu 250 ms'den
     uzun sure bloklanamaz. */
  if (BTS_Init() != HAL_OK) { g_drv_init_err |= 1U; }
  if (CS_Init()  != HAL_OK) { g_drv_init_err |= 2U; }

  /* Cikislar kapaliyken IS pinlerinin sifir noktasini olc (~200 ms) */
  CS_ZeroCalibrate();

  SM_Init();
  if (g_drv_init_err != 0U) { SM_LatchShutdown(SM_F_DRIVER_INIT); }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    static uint32_t last_poll    = 0;
    static uint32_t last_hz_tick = 0;
    static uint32_t win_samples  = 0;

    /* gurultu istatistigi penceresi */
    static uint8_t  w_min = 255, w_max = 0;
    static uint32_t w_sum = 0, w_sumsq = 0;
    static uint16_t w_n = 0;
    static uint32_t last_stat_tick = 0;
    static uint32_t last_ok0       = 0;

    uint32_t now = HAL_GetTick();

    /* --- Asama 3: sabit periyotlu emniyet/kontrol gorevi ---
       ToF yoklamasindan ONCE cagriliyor ki I2C hattinda bir aksama olsa
       bile bekci kopegi beslenmeye ve cikislar denetlenmeye devam etsin. */
    static uint32_t last_ctrl = 0;
    if ((now - last_ctrl) >= CTRL_PERIOD_MS)
    {
      uint8_t k;
      last_ctrl = now;

      SM_RequestLevitate(g_levitate_req);
      SM_Task();

      g_sm_state  = (uint8_t)SM_GetState();
      g_sm_faults = SM_GetFaults();
      g_vbus_v    = CS_GetBusVoltage();
      g_power_w   = CS_GetPower();
      g_peak_i_a  = SM_GetPeakCurrent();
      g_peak_p_w  = SM_GetPeakPower();
      for (k = 0U; k < BTS_CH_COUNT; k++)
      {
        g_coil_i_a[k] = CS_GetCurrent(k);
        g_duty[k]     = BTS_GetDuty(k);
      }
    }

    /* --- Asama 2: hava araligi yoklamasi (round-robin) --- */
    if ((now - last_poll) >= GAP_POLL_STEP_MS)
    {
      uint8_t k;
      const est_state_t *e;

      last_poll = now;
      GapSensor_Task();
      Estimator_Update();

      for (k = 0U; k < GAP_CH_COUNT; k++)
      {
        const gap_ch_t *g = GapSensor_Get(k);
        g_gap_present[k] = g->present;
        g_gap_fault[k]   = g->fault;
        g_gap_raw[k]     = g->raw_mm;
        g_gap_ch[k]      = g->gap_med_mm;
        g_gap_hz[k]      = g->hz;
        g_gap_status[k]  = g->status;
      }
      g_gap_valid_n = GapSensor_ValidCount();

      e = Estimator_Get();
      g_est_valid    = e->valid;
      g_est_n_used   = e->n_used;
      g_heave_mm     = e->heave_mm;
      g_roll_mrad    = e->roll_rad  * 1000.0f;
      g_pitch_mrad   = e->pitch_rad * 1000.0f;
      g_est_resid_mm = e->residual_mm;

      /* Durum makinesi duzlem cozulebildigi surece merkez hava araligiyla
         beslenir. Cozulemiyorsa hic beslenmez ve SENSOR_TIMEOUT devreye
         girer -- sessizce eski degerle devam etmek tehlikeli olurdu. */
      if (e->valid != 0U)
      {
        SM_ReportGap(e->heave_mm, 1U);
      }

      /* Kanal 0 aynalari (eski Live Expressions kurulumu icin) */
      {
        const gap_ch_t *g0 = GapSensor_Get(GAP_CH_FL);
        g_range_mm     = g0->raw_mm;
        g_range_status = g0->status;
        g_gap_mm       = g0->gap_mm;
        g_ok_count     = g0->samples;
        if (g0->samples != last_ok0)
        {
          last_ok0 = g0->samples;
          win_samples++;
          if (g0->status == VL6180X_RS_NO_ERROR)
          {
            /* 2 saniyelik gurultu istatistigi (yalnizca kanal 0) */
            if (g0->raw_mm < w_min) { w_min = g0->raw_mm; }
            if (g0->raw_mm > w_max) { w_max = g0->raw_mm; }
            w_sum   += g0->raw_mm;
            w_sumsq += (uint32_t)g0->raw_mm * g0->raw_mm;
            w_n++;
          }
        }
      }

      /* LED durumu:
           yesil  = en az bir gecerli kanal, canlilik icin yanip soner
           turuncu= bulunan sensorlerden biri arizali (kismi calisma)
           kirmizi= hic gecerli olcum yok */
      if (g_gap_valid_n == 0U)
      {
        HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
      }
      else
      {
        HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
        if ((g_ok_count % 25U) == 0U)
        {
          HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
        }
      }
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin,
                        (g_gap_valid_n < g_gap_found) ? GPIO_PIN_SET
                                                      : GPIO_PIN_RESET);
    }

    /* Saniyede bir gercek ornekleme hizini olc */
    if ((now - last_hz_tick) >= 1000U)
    {
      last_hz_tick = now;
      g_sample_hz  = win_samples;
      win_samples  = 0;
    }

    /* Iki saniyede bir gurultu istatistigini kapat ve yayinla */
    if ((now - last_stat_tick) >= 2000U)
    {
      last_stat_tick = now;

      if (w_n > 1U)
      {
        float mean = (float)w_sum / (float)w_n;
        float var  = ((float)w_sumsq / (float)w_n) - (mean * mean);
        if (var < 0.0f) { var = 0.0f; }

        g_stat_min  = w_min;
        g_stat_max  = w_max;
        g_stat_pp   = (uint8_t)(w_max - w_min);
        g_stat_mean = mean;
        g_stat_std  = sqrtf(var);
        g_stat_n    = w_n;
      }

      w_min = 255; w_max = 0; w_sum = 0; w_sumsq = 0; w_n = 0;
    }

    g_i2c_err_count = I2CBus_GetErrorCount();

    /* Hata ayiklayicidan tetiklenen bobin testi (g_coil_arm = 0xA5).
       Test cevre birimlerini kendi ayarlarina cekip geri birakir, bu yuzden
       once durum makinesinin cikis surmedigi dogrulanir. */
    if (g_coil_arm == 0xA5U)
    {
      sm_state_t st = SM_GetState();
      g_coil_arm = 0;
      if ((st == SM_IDLE) || (st == SM_FAULT) || (st == SM_SAFE_SHUTDOWN))
      {
        BTS_AllOff();
        if (CoilTest_Init() == HAL_OK)
        {
          CoilTest_SafeOff();
          HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
          CoilTest_Run();
          g_coil_run_count++;
          HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
        }
        MX_TIM1_Init();
        MX_TIM2_Init();
        MX_ADC1_Init();
        (void)BTS_Init();
        (void)CS_Init();
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 4199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2099;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, EN_FR_Pin|CS_I2C_SPI_Pin|EN_RL_Pin|EN_RR_Pin
                          |EN_FL_Pin|LPWM_FL_Pin|LPWM_FR_Pin|LPWM_RL_Pin
                          |LPWM_RR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MUX_RST_GPIO_Port, MUX_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN_FR_Pin EN_RL_Pin EN_RR_Pin EN_FL_Pin
                           LPWM_FL_Pin LPWM_FR_Pin LPWM_RL_Pin LPWM_RR_Pin */
  GPIO_InitStruct.Pin = EN_FR_Pin|EN_RL_Pin|EN_RR_Pin|EN_FL_Pin
                          |LPWM_FL_Pin|LPWM_FR_Pin|LPWM_RL_Pin|LPWM_RR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin MUX_RST_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|MUX_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
