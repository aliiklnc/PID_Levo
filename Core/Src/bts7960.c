/**
  ******************************************************************************
  * @file    bts7960.c
  * @brief   4 kanalli BTS7960 H-koprusu surucu katmani.
  ******************************************************************************
  */
#include "bts7960.h"

/* CubeMX'in urettigi TIM1 tutamaci (main.c) */
extern TIM_HandleTypeDef htim1;

static GPIO_TypeDef *const s_en_port[BTS_CH_COUNT]   = BTS_EN_PORTS;
static const uint16_t      s_en_pin[BTS_CH_COUNT]    = BTS_EN_PINS;
static const uint32_t      s_pwm_ch[BTS_CH_COUNT]    = BTS_PWM_CHANNELS;

static float   s_duty[BTS_CH_COUNT];
static uint8_t s_enabled;

/* Merkez hizali PWM'de bir periyot 2*ARR tiktir ve cikis 2*CCR tik boyunca
   aktiftir; dolayisiyla duty = CCR / ARR. */
static inline uint32_t duty_to_ccr(float duty)
{
  if (duty <= 0.0f)          { return 0U; }
  if (duty > BTS_DUTY_MAX)   { duty = BTS_DUTY_MAX; }
  return (uint32_t)((duty * (float)BTS_PWM_ARR) + 0.5f);
}

static inline void write_ccr(uint8_t ch, uint32_t ccr)
{
  switch (s_pwm_ch[ch])
  {
    case TIM_CHANNEL_1: htim1.Instance->CCR1 = ccr; break;
    case TIM_CHANNEL_2: htim1.Instance->CCR2 = ccr; break;
    case TIM_CHANNEL_3: htim1.Instance->CCR3 = ccr; break;
    default:            htim1.Instance->CCR4 = ccr; break;
  }
}

/* ------------------------------------------------------------------ */

void BTS_AllOff(void)
{
  uint8_t ch;

  /* Once duty, sonra enable: siralamayi tersine cevirmek, EN dustukten
     sonra CCR'nin bir periyot daha yuksek kalmasina yol acar. */
  for (ch = 0U; ch < BTS_CH_COUNT; ch++)
  {
    write_ccr(ch, 0U);
    s_duty[ch] = 0.0f;
  }
  for (ch = 0U; ch < BTS_CH_COUNT; ch++)
  {
    HAL_GPIO_WritePin(s_en_port[ch], s_en_pin[ch], GPIO_PIN_RESET);
  }
  s_enabled = 0U;
}

HAL_StatusTypeDef BTS_Init(void)
{
  uint8_t ch;

  /* PWM'i once %0 duty ile baslat; cikis pinleri zaten AF modunda ve
     CCR=0 oldugu icin surekli LOW kalirlar. */
  for (ch = 0U; ch < BTS_CH_COUNT; ch++)
  {
    write_ccr(ch, 0U);
    if (HAL_TIM_PWM_Start(&htim1, s_pwm_ch[ch]) != HAL_OK)
    {
      BTS_AllOff();
      return HAL_ERROR;
    }
  }
  BTS_AllOff();
  return HAL_OK;
}

void BTS_Enable(void)
{
  uint8_t ch;

  for (ch = 0U; ch < BTS_CH_COUNT; ch++)
  {
    write_ccr(ch, 0U);
    s_duty[ch] = 0.0f;
  }
  for (ch = 0U; ch < BTS_CH_COUNT; ch++)
  {
    HAL_GPIO_WritePin(s_en_port[ch], s_en_pin[ch], GPIO_PIN_SET);
  }
  s_enabled = 1U;
}

void BTS_SetDuty(uint8_t ch, float duty)
{
  if (ch >= BTS_CH_COUNT) { return; }

  /* NaN korumasi: NaN her karsilastirmada false doner, sessizce gecerdi. */
  if (!(duty >= 0.0f)) { duty = 0.0f; }
  if (duty > BTS_DUTY_MAX) { duty = BTS_DUTY_MAX; }

  s_duty[ch] = duty;
  write_ccr(ch, duty_to_ccr(duty));
}

void BTS_SetDutyAll(float duty)
{
  uint8_t ch;
  for (ch = 0U; ch < BTS_CH_COUNT; ch++) { BTS_SetDuty(ch, duty); }
}

float BTS_GetDuty(uint8_t ch)
{
  return (ch < BTS_CH_COUNT) ? s_duty[ch] : 0.0f;
}

uint8_t BTS_IsEnabled(void)
{
  return s_enabled;
}
