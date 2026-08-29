/**
  ******************************************************************************
  * @file    coil_test.c
  * @brief   Bobin R/L karakterizasyonu -- bkz. coil_test.h
  ******************************************************************************
  */
#include "coil_test.h"

volatile uint16_t          g_coil_cap[COIL_CAP_LEN];
volatile coil_test_result_t g_coil_res;

static TIM_HandleTypeDef htim1_pwm;   /* BTS7960 RPWM */
static TIM_HandleTypeDef htim2_trg;   /* ADC tetigi   */
static ADC_HandleTypeDef hadc1_cap;
static DMA_HandleTypeDef hdma_adc1;

/* TIM1 saati 168 MHz, TIM2 saati 84 MHz (APB bolucusu x2) */
#define TIM1_CLK_HZ   168000000UL
#define TIM2_CLK_HZ    84000000UL
#define PWM_FREQ_HZ         5000UL   /* karakterizasyon 5 kHz'de yapilir:
                                        BTS7960 IS pininin yerlesme suresi
                                        20 kHz'de on-fazina sigmayabilir  */
#define TRIG_FREQ_HZ      100000UL   /* kanal basina ornekleme            */

#define PWM_ARR       ((TIM1_CLK_HZ / PWM_FREQ_HZ) - 1U)

/* Akim esigi -> ham ADC degeri */
#define ABORT_ADC_RAW \
  ((uint16_t)((COIL_TEST_ABORT_AMP * BTS_IS_VOLT_PER_AMP / ADC_VREF) * ADC_FULL_SCALE))

/* ------------------------------------------------------------------ */

static void gpio_init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* Enable ve LPWM: cikis, baslangicta LOW (surucu pasif) */
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;

  g.Pin = BTS_EN_PIN;   HAL_GPIO_Init(BTS_EN_PORT,   &g);
  g.Pin = BTS_LPWM_PIN; HAL_GPIO_Init(BTS_LPWM_PORT, &g);

  HAL_GPIO_WritePin(BTS_EN_PORT,   BTS_EN_PIN,   GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BTS_LPWM_PORT, BTS_LPWM_PIN, GPIO_PIN_RESET);

  /* RPWM -> TIM1_CH1 */
  g.Mode      = GPIO_MODE_AF_PP;
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF1_TIM1;
  g.Pin       = BTS_RPWM_PIN;
  HAL_GPIO_Init(BTS_RPWM_PORT, &g);

  /* Analog girisler: PA1 = IS_FL (IN1), PB1 = VBUS (IN9) */
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  g.Pin  = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOA, &g);
  g.Pin  = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOB, &g);
}

static HAL_StatusTypeDef tim_init(void)
{
  TIM_OC_InitTypeDef       oc = {0};
  TIM_MasterConfigTypeDef  mc = {0};

  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();

  /* --- TIM1: BTS7960 RPWM --- */
  htim1_pwm.Instance               = TIM1;
  htim1_pwm.Init.Prescaler         = 0;
  htim1_pwm.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1_pwm.Init.Period            = PWM_ARR;
  htim1_pwm.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1_pwm.Init.RepetitionCounter = 0;
  htim1_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1_pwm) != HAL_OK) { return HAL_ERROR; }

  oc.OCMode     = TIM_OCMODE_PWM1;
  oc.Pulse      = 0;                      /* baslangicta %0 */
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET; /* MOE kapaninca cikis LOW */
  if (HAL_TIM_PWM_ConfigChannel(&htim1_pwm, &oc, TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* --- TIM2: ADC tetigi, TRGO = update --- */
  htim2_trg.Instance           = TIM2;
  htim2_trg.Init.Prescaler     = 0;
  htim2_trg.Init.CounterMode   = TIM_COUNTERMODE_UP;
  htim2_trg.Init.Period        = (TIM2_CLK_HZ / TRIG_FREQ_HZ) - 1U;
  htim2_trg.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2_trg) != HAL_OK) { return HAL_ERROR; }

  mc.MasterOutputTrigger = TIM_TRGO_UPDATE;
  mc.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2_trg, &mc) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

static HAL_StatusTypeDef adc_init(void)
{
  ADC_ChannelConfTypeDef ch = {0};

  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  hdma_adc1.Instance                 = DMA2_Stream0;
  hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
  hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode                = DMA_NORMAL;
  hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) { return HAL_ERROR; }

  hadc1_cap.Instance                   = ADC1;
  hadc1_cap.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;  /* 21 MHz */
  hadc1_cap.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1_cap.Init.ScanConvMode          = ENABLE;
  hadc1_cap.Init.ContinuousConvMode    = DISABLE;
  hadc1_cap.Init.DiscontinuousConvMode = DISABLE;
  hadc1_cap.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1_cap.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1_cap.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1_cap.Init.NbrOfConversion       = 2;
  hadc1_cap.Init.DMAContinuousRequests = ENABLE;
  hadc1_cap.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1_cap) != HAL_OK) { return HAL_ERROR; }

  __HAL_LINKDMA(&hadc1_cap, DMA_Handle, hdma_adc1);

  /* Sira 1: IS_FL (PA1 = IN1),  Sira 2: VBUS (PB1 = IN9) */
  ch.Channel      = ADC_CHANNEL_1;
  ch.Rank         = 1;
  ch.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1_cap, &ch) != HAL_OK) { return HAL_ERROR; }

  ch.Channel = ADC_CHANNEL_9;
  ch.Rank    = 2;
  if (HAL_ADC_ConfigChannel(&hadc1_cap, &ch) != HAL_OK) { return HAL_ERROR; }

  return HAL_OK;
}

/* ------------------------------------------------------------------ */

void CoilTest_SafeOff(void)
{
  __HAL_TIM_SET_COMPARE(&htim1_pwm, TIM_CHANNEL_1, 0);
  HAL_GPIO_WritePin(BTS_EN_PORT,   BTS_EN_PIN,   GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BTS_LPWM_PORT, BTS_LPWM_PIN, GPIO_PIN_RESET);
}

HAL_StatusTypeDef CoilTest_Init(void)
{
  g_coil_res.state = COIL_TEST_IDLE;

  gpio_init();
  if (tim_init() != HAL_OK) { g_coil_res.state = COIL_TEST_ERR_INIT; return HAL_ERROR; }
  if (adc_init() != HAL_OK) { g_coil_res.state = COIL_TEST_ERR_INIT; return HAL_ERROR; }

  if (HAL_TIM_PWM_Start(&htim1_pwm, TIM_CHANNEL_1) != HAL_OK)
  {
    g_coil_res.state = COIL_TEST_ERR_INIT;
    return HAL_ERROR;
  }
  CoilTest_SafeOff();
  return HAL_OK;
}

/* DMA'nin o ana kadar yazdigi cift sayisi */
static inline uint16_t captured_pairs(void)
{
  uint32_t remaining = __HAL_DMA_GET_COUNTER(&hdma_adc1);
  uint32_t written   = (remaining > COIL_CAP_LEN) ? 0U : (COIL_CAP_LEN - remaining);
  return (uint16_t)(written / 2U);
}

/* Hedef ornek sayisina kadar bekler; asiri akimda 0 doner. */
static uint8_t wait_until(uint16_t target_pairs, uint32_t deadline_tick)
{
  while (captured_pairs() < target_pairs)
  {
    uint16_t n = captured_pairs();
    if (n > 0U)
    {
      if (g_coil_cap[(n - 1U) * 2U] > ABORT_ADC_RAW) { return 0U; }
    }
    if (HAL_GetTick() > deadline_tick) { return 0U; }
  }
  return 1U;
}

void CoilTest_Run(void)
{
  if (g_coil_res.state == COIL_TEST_ERR_INIT) { return; }

  const uint16_t pairs_baseline = 200U;                       /*  2 ms */
  const uint16_t pairs_on       = (uint16_t)(COIL_TEST_MAX_ON_MS * 100U); /* 12 ms */
  uint32_t deadline;

  CoilTest_SafeOff();
  HAL_Delay(50);

  for (uint32_t i = 0; i < COIL_CAP_LEN; i++) { g_coil_cap[i] = 0; }

  g_coil_res.state   = COIL_TEST_RUNNING;
  g_coil_res.idx_on  = 0;
  g_coil_res.idx_off = 0;

  if (HAL_ADC_Start_DMA(&hadc1_cap, (uint32_t *)g_coil_cap, COIL_CAP_LEN) != HAL_OK)
  {
    g_coil_res.state = COIL_TEST_ERR_INIT;
    return;
  }

  /* MX_DMA_Init Stream0 IRQ'sini etkinlestirir; fakat bu test DMA tamponunu
     yoklayarak okur ve ISR kullanmaz. HAL_ADC_Start_DMA kesmeleri yeniden
     actigi icin TIM2'den ONCE kapatmak zorunludur; aksi halde bos ISR,
     bayragi temizlemeden sonsuz kesme dongusune girebilir. */
  __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
  HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);

  if (HAL_TIM_Base_Start(&htim2_trg) != HAL_OK)
  {
    HAL_ADC_Stop_DMA(&hadc1_cap);
    g_coil_res.state = COIL_TEST_ERR_INIT;
    return;
  }

  deadline = HAL_GetTick() + 200U;

  /* 1) Taban cizgisi: cikis kapali */
  if (wait_until(pairs_baseline, deadline) == 0U) { goto abort; }

  g_coil_res.adc_baseline = g_coil_cap[(pairs_baseline - 2U) * 2U];
  g_coil_res.vbus_v = ((float)g_coil_cap[(pairs_baseline - 2U) * 2U + 1U]
                       / ADC_FULL_SCALE) * ADC_VREF * VBUS_DIVIDER_RATIO;

  /* 2) Tam gerilim darbesi */
  g_coil_res.idx_on = captured_pairs();
  HAL_GPIO_WritePin(BTS_EN_PORT, BTS_EN_PIN, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim1_pwm, TIM_CHANNEL_1, PWM_ARR + 1U);  /* %100 */

  if (wait_until((uint16_t)(g_coil_res.idx_on + pairs_on), deadline) == 0U)
  {
    goto abort;
  }

  /* 3) Kes -- akim dusuk taraftan serbest doner, sonum egrisi kaydedilir */
  g_coil_res.idx_off = captured_pairs();
  __HAL_TIM_SET_COMPARE(&htim1_pwm, TIM_CHANNEL_1, 0);

  g_coil_res.adc_peak = g_coil_cap[(g_coil_res.idx_off - 2U) * 2U];

  /* 4) Tampon dolana kadar sonumu kaydet, sonra tamamen kapat */
  (void)wait_until(COIL_CAP_PAIRS, deadline);
  CoilTest_SafeOff();

  HAL_TIM_Base_Stop(&htim2_trg);
  HAL_ADC_Stop_DMA(&hadc1_cap);

  g_coil_res.samples = captured_pairs();

  /* Ozet degerler */
  {
    float v_is = (((float)g_coil_res.adc_peak - (float)g_coil_res.adc_baseline)
                  / ADC_FULL_SCALE) * ADC_VREF;
    g_coil_res.i_peak_a = v_is / BTS_IS_VOLT_PER_AMP;

    uint16_t vmin = 0xFFFFU;
    for (uint16_t i = g_coil_res.idx_on; i < g_coil_res.idx_off; i++)
    {
      if (g_coil_cap[i * 2U + 1U] < vmin) { vmin = g_coil_cap[i * 2U + 1U]; }
    }
    g_coil_res.vbus_min_v = ((float)vmin / ADC_FULL_SCALE) * ADC_VREF * VBUS_DIVIDER_RATIO;
  }

  g_coil_res.state = COIL_TEST_DONE;
  return;

abort:
  CoilTest_SafeOff();
  HAL_TIM_Base_Stop(&htim2_trg);
  HAL_ADC_Stop_DMA(&hadc1_cap);
  g_coil_res.samples = captured_pairs();
  g_coil_res.state   = COIL_TEST_ABORTED_OVERCURRENT;
}
