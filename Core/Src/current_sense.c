/**
  ******************************************************************************
  * @file    current_sense.c
  * @brief   4 kanal bobin akimi + bus gerilimi olcumu.
  ******************************************************************************
  */
#include "current_sense.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_adc1;

/* DMA hedefi. ADC 40 kHz'de yeniden yazar; okurken tek seferlik kopya alinir
   (16-bit erisim ARM'da atomiktir, ayrica yirtilma olsa bile bir sonraki
   tarama duzeltir). */
static volatile uint16_t s_raw[ADC_CH_COUNT];

static float s_zero_a[BTS_CH_COUNT];   /* kanal basina sifir ofseti, amper */
static float s_curr_a[BTS_CH_COUNT];   /* filtrelenmis akim                */
static float s_vbus_v;
static uint32_t s_dma_tick;
static uint8_t  s_dma_seen;

/* Tek kutuplu IIR: y += alpha*(x-y). 1 kHz cagri ve alpha=0.2 -> ~0.8 ms
   zaman sabiti. Trip karari filtresiz degerden verilir, bu yuzden filtre
   emniyeti geciktirmez. */
#define IIR_ALPHA   0.2f

static inline float adc_to_volt(uint16_t raw)
{
  return ((float)raw * ADC_VREF) / ADC_FULL_SCALE;
}

static inline float raw_to_amp(uint16_t raw)
{
  return adc_to_volt(raw) / BTS_IS_VOLT_PER_AMP;
}

/* ------------------------------------------------------------------ */

HAL_StatusTypeDef CS_Init(void)
{
  uint8_t i;

  for (i = 0U; i < ADC_CH_COUNT; i++) { s_raw[i] = 0U; }
  for (i = 0U; i < BTS_CH_COUNT; i++) { s_zero_a[i] = 0.0f; s_curr_a[i] = 0.0f; }
  s_vbus_v = 0.0f;
  s_dma_tick = 0UL;
  s_dma_seen = 0U;

  /* Yeniden cagrilabilir olsun (bobin testinden sonra gerekiyor). */
  (void)HAL_TIM_Base_Stop(&htim2);
  (void)HAL_ADC_Stop_DMA(&hadc1);

  /* TIM2 periyodunu burada da uygula: baska bir modul (bobin testi) TIM2'yi
     kendi olcum hizina cekmis olabilir ve ADC tetigi yanlis frekansta kalir. */
  __HAL_TIM_SET_AUTORELOAD(&htim2, ADC_TRIG_ARR);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_raw, ADC_CH_COUNT) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* DMA tamamlanma / yarilanma kesmelerini KAPAT.
     Tampon dairesel ve biz onu dogrudan okuyoruz; HAL'in geri cagirma
     zincirine ihtiyacimiz yok. Acik birakilirsa her tetikte iki kesme
     olusur (40 kHz'de 80 bin/s) ve CPU bosuna yanar. */
  __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
  HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
  __HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_FEIF0_4 | DMA_FLAG_DMEIF0_4 |
                                    DMA_FLAG_TEIF0_4 | DMA_FLAG_HTIF0_4  |
                                    DMA_FLAG_TCIF0_4);

  /* ADC harici tetikte; TIM2 TRGO baslamadan tek bir donusum bile olmaz. */
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    (void)HAL_ADC_Stop_DMA(&hadc1);
    return HAL_ERROR;
  }
  return HAL_OK;
}

void CS_ZeroCalibrate(void)
{
  uint32_t acc[BTS_CH_COUNT] = {0};
  uint16_t n;
  uint8_t  ch;

  /* 200 okuma ~ 200 ms. DMA arka planda dondugu icin sadece bekleyip
     okumak yeterli. */
  for (n = 0U; n < 200U; n++)
  {
    for (ch = 0U; ch < BTS_CH_COUNT; ch++) { acc[ch] += s_raw[ch]; }
    HAL_Delay(1);
  }
  for (ch = 0U; ch < BTS_CH_COUNT; ch++)
  {
    s_zero_a[ch] = raw_to_amp((uint16_t)(acc[ch] / 200U));
  }
}

void CS_Update(void)
{
  uint8_t ch;
  float   a;

  /* Kesme kullanmadan DMA ilerlemesini izle. Stream0 dairesel tamponu
     her taramada TC bayragini yeniden set eder; kontrol dongusu bunu
     gorup temizleyerek son gercek veri zamanini kanitlar. */
  if ((__HAL_DMA_GET_FLAG(&hdma_adc1, DMA_FLAG_HTIF0_4) != RESET) ||
      (__HAL_DMA_GET_FLAG(&hdma_adc1, DMA_FLAG_TCIF0_4) != RESET))
  {
    __HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4);
    s_dma_tick = HAL_GetTick();
    s_dma_seen = 1U;
  }

  for (ch = 0U; ch < BTS_CH_COUNT; ch++)
  {
    a = raw_to_amp(s_raw[ch]) - s_zero_a[ch];
    if (a < 0.0f) { a = 0.0f; }
    s_curr_a[ch] += IIR_ALPHA * (a - s_curr_a[ch]);
  }
  s_vbus_v += IIR_ALPHA * ((adc_to_volt(s_raw[ADC_IDX_VBUS])
                            * VBUS_DIVIDER_RATIO) - s_vbus_v);
}

uint8_t CS_IsFresh(void)
{
  if (s_dma_seen == 0U) { return 0U; }
  return ((HAL_GetTick() - s_dma_tick) <= ADC_DATA_TIMEOUT_MS) ? 1U : 0U;
}

float CS_GetCurrent(uint8_t ch)
{
  return (ch < BTS_CH_COUNT) ? s_curr_a[ch] : 0.0f;
}

float CS_GetCurrentRaw(uint8_t ch)
{
  float a;
  if (ch >= BTS_CH_COUNT) { return 0.0f; }
  a = raw_to_amp(s_raw[ch]) - s_zero_a[ch];
  return (a > 0.0f) ? a : 0.0f;
}

float CS_GetBusVoltage(void)
{
  return s_vbus_v;
}

float CS_GetPower(void)
{
  uint8_t ch;
  float   sum = 0.0f;
  for (ch = 0U; ch < BTS_CH_COUNT; ch++) { sum += s_curr_a[ch]; }
  return s_vbus_v * sum;
}

uint16_t CS_GetAdcRaw(uint8_t idx)
{
  return (idx < ADC_CH_COUNT) ? s_raw[idx] : 0U;
}
