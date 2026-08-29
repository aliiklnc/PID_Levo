/**
  ******************************************************************************
  * @file    i2c_bus.h
  * @brief   I2C sarmalayici: timeout, yeniden deneme, hata sayaci,
  *          takilan hattin kurtarilmasi ve adres taramasi.
  ******************************************************************************
  */
#ifndef I2C_BUS_H
#define I2C_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"

void     I2CBus_Init(void);
uint32_t I2CBus_GetErrorCount(void);
void     I2CBus_ClearErrorCount(void);

/* Register adresi olmadan ham erisim (TCA9548A bunu kullanir) */
HAL_StatusTypeDef I2CBus_WriteRaw(uint8_t addr7, const uint8_t *buf, uint16_t len);
HAL_StatusTypeDef I2CBus_ReadRaw (uint8_t addr7, uint8_t *buf, uint16_t len);

/* 16-bit register adresli erisim -- VL6180X BUNU kullanir.
   8-bit adresleme ile sensorden hicbir cevap alinamaz. */
HAL_StatusTypeDef I2CBus_WriteReg16(uint8_t addr7, uint16_t reg, uint8_t val);
HAL_StatusTypeDef I2CBus_ReadReg16 (uint8_t addr7, uint16_t reg, uint8_t *buf, uint16_t len);

/* 0x08..0x77 arasini tarar, bulunan 7-bit adresleri found[] icine yazar.
   Donus: bulunan cihaz sayisi. */
uint8_t I2CBus_Scan(uint8_t *found, uint8_t max_found);

/* SDA'yi asagida tutan bir slave'i 9 SCL darbesi ile serbest birakir,
   ardindan cevre birimini yeniden baslatir. */
void I2CBus_Recover(void);

#ifdef __cplusplus
}
#endif
#endif /* I2C_BUS_H */
