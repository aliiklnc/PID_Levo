/**
  ******************************************************************************
  * @file    i2c_bus.c
  * @brief   I2C sarmalayici uygulamasi.
  ******************************************************************************
  */
#include "i2c_bus.h"

static volatile uint32_t s_err_count = 0;

/* ~1 us mertebesinde kaba bekleme (168 MHz, -O0..-O2 arasinda degisir).
   Yalnizca hat kurtarmada kullanilir, hassasiyeti kritik degildir. */
static void bus_delay(void)
{
  for (volatile uint32_t i = 0; i < 400U; i++) { __NOP(); }
}

void I2CBus_Init(void)
{
  s_err_count = 0;
}

uint32_t I2CBus_GetErrorCount(void) { return s_err_count; }
void     I2CBus_ClearErrorCount(void) { s_err_count = 0; }

/* Basarisiz erisimden sonra cevre birimini temiz duruma dondurur. */
static void handle_failure(HAL_StatusTypeDef st)
{
  s_err_count++;

  /* BERR / ARLO / AF gibi bayraklar kalirsa sonraki erisimler de patlar */
  if (HAL_I2C_GetError(APP_I2C) & HAL_I2C_ERROR_BERR)
  {
    I2CBus_Recover();
  }
  else if (st == HAL_BUSY || HAL_I2C_GetState(APP_I2C) == HAL_I2C_STATE_BUSY)
  {
    HAL_I2C_DeInit(APP_I2C);
    HAL_I2C_Init(APP_I2C);
  }
}

HAL_StatusTypeDef I2CBus_WriteRaw(uint8_t addr7, const uint8_t *buf, uint16_t len)
{
  HAL_StatusTypeDef st = HAL_ERROR;

  for (uint8_t attempt = 0; attempt <= APP_I2C_RETRY_COUNT; attempt++)
  {
    st = HAL_I2C_Master_Transmit(APP_I2C, (uint16_t)(addr7 << 1),
                                 (uint8_t *)buf, len, APP_I2C_TIMEOUT_MS);
    if (st == HAL_OK) { return HAL_OK; }
    handle_failure(st);
  }
  return st;
}

HAL_StatusTypeDef I2CBus_ReadRaw(uint8_t addr7, uint8_t *buf, uint16_t len)
{
  HAL_StatusTypeDef st = HAL_ERROR;

  for (uint8_t attempt = 0; attempt <= APP_I2C_RETRY_COUNT; attempt++)
  {
    st = HAL_I2C_Master_Receive(APP_I2C, (uint16_t)(addr7 << 1),
                                buf, len, APP_I2C_TIMEOUT_MS);
    if (st == HAL_OK) { return HAL_OK; }
    handle_failure(st);
  }
  return st;
}

HAL_StatusTypeDef I2CBus_WriteReg16(uint8_t addr7, uint16_t reg, uint8_t val)
{
  HAL_StatusTypeDef st = HAL_ERROR;

  for (uint8_t attempt = 0; attempt <= APP_I2C_RETRY_COUNT; attempt++)
  {
    st = HAL_I2C_Mem_Write(APP_I2C, (uint16_t)(addr7 << 1), reg,
                           I2C_MEMADD_SIZE_16BIT, &val, 1, APP_I2C_TIMEOUT_MS);
    if (st == HAL_OK) { return HAL_OK; }
    handle_failure(st);
  }
  return st;
}

HAL_StatusTypeDef I2CBus_ReadReg16(uint8_t addr7, uint16_t reg,
                                   uint8_t *buf, uint16_t len)
{
  HAL_StatusTypeDef st = HAL_ERROR;

  for (uint8_t attempt = 0; attempt <= APP_I2C_RETRY_COUNT; attempt++)
  {
    st = HAL_I2C_Mem_Read(APP_I2C, (uint16_t)(addr7 << 1), reg,
                          I2C_MEMADD_SIZE_16BIT, buf, len, APP_I2C_TIMEOUT_MS);
    if (st == HAL_OK) { return HAL_OK; }
    handle_failure(st);
  }
  return st;
}

uint8_t I2CBus_Scan(uint8_t *found, uint8_t max_found)
{
  uint8_t n = 0;

  for (uint8_t a = 0x08U; a <= 0x77U; a++)
  {
    /* IsDeviceReady dogrudan HAL uzerinden cagrilir: bulunamayan adresler
       normaldir, hata sayacini sisirmemeleri gerekir. */
    if (HAL_I2C_IsDeviceReady(APP_I2C, (uint16_t)(a << 1), 1, 2) == HAL_OK)
    {
      if (found != NULL && n < max_found) { found[n] = a; }
      n++;
    }
  }
  return n;
}

void I2CBus_Recover(void)
{
  GPIO_InitTypeDef g = {0};

  HAL_I2C_DeInit(APP_I2C);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  g.Pin   = APP_I2C_SCL_PIN | APP_I2C_SDA_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_OD;
  g.Pull  = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(APP_I2C_SCL_PORT, &g);

  HAL_GPIO_WritePin(APP_I2C_SDA_PORT, APP_I2C_SDA_PIN, GPIO_PIN_SET);

  /* 9 SCL darbesi: yarim kalmis bir transferde SDA'yi tutan slave birakir */
  for (uint8_t i = 0; i < 9U; i++)
  {
    HAL_GPIO_WritePin(APP_I2C_SCL_PORT, APP_I2C_SCL_PIN, GPIO_PIN_RESET);
    bus_delay();
    HAL_GPIO_WritePin(APP_I2C_SCL_PORT, APP_I2C_SCL_PIN, GPIO_PIN_SET);
    bus_delay();
  }

  /* STOP kosulu: SCL yuksekken SDA dusukten yuksege */
  HAL_GPIO_WritePin(APP_I2C_SDA_PORT, APP_I2C_SDA_PIN, GPIO_PIN_RESET);
  bus_delay();
  HAL_GPIO_WritePin(APP_I2C_SCL_PORT, APP_I2C_SCL_PIN, GPIO_PIN_SET);
  bus_delay();
  HAL_GPIO_WritePin(APP_I2C_SDA_PORT, APP_I2C_SDA_PIN, GPIO_PIN_SET);
  bus_delay();

  /* HAL_I2C_DeInit MspDeInit'i cagirdigi icin HAL_I2C_Init pinleri
     tekrar alternatif fonksiyona ayarlar. */
  HAL_I2C_Init(APP_I2C);
}
