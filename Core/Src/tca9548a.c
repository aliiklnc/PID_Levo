/**
  ******************************************************************************
  * @file    tca9548a.c
  * @brief   TCA9548A surucusu.
  ******************************************************************************
  */
#include "tca9548a.h"
#include "i2c_bus.h"

/* Coklayiciya en son yazdigimiz kanal maskesi. 0xFF = bilinmiyor. */
static uint8_t s_mask = 0xFFU;

void TCA_Init(void)
{
  GPIO_InitTypeDef g = {0};

  /* MUX_RST pinini burada da kuruyoruz: CubeMX'te yeniden kod uretilmemis
     olsa bile surucu calisir. */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  g.Pin   = TCA_RST_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TCA_RST_PORT, &g);

  HAL_GPIO_WritePin(TCA_RST_PORT, TCA_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(TCA_RST_PORT, TCA_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(2);

  s_mask = 0xFFU;
  (void)TCA_DisableAll();
}

HAL_StatusTypeDef TCA_SelectChannel(uint8_t ch)
{
  if (ch > 7U) { return HAL_ERROR; }

  uint8_t mask = (uint8_t)(1U << ch);
  if (s_mask == mask) { return HAL_OK; }      /* zaten acik */

  HAL_StatusTypeDef st = I2CBus_WriteRaw(TCA9548A_ADDR_7B, &mask, 1);
  s_mask = (st == HAL_OK) ? mask : 0xFFU;     /* hata halinde onbellegi bozar */
  return st;
}

HAL_StatusTypeDef TCA_DisableAll(void)
{
  uint8_t mask = 0x00U;
  HAL_StatusTypeDef st = I2CBus_WriteRaw(TCA9548A_ADDR_7B, &mask, 1);
  s_mask = (st == HAL_OK) ? 0x00U : 0xFFU;
  return st;
}

HAL_StatusTypeDef TCA_ReadControl(uint8_t *val)
{
  return I2CBus_ReadRaw(TCA9548A_ADDR_7B, val, 1);
}

uint8_t TCA_GetCachedMask(void) { return s_mask; }
