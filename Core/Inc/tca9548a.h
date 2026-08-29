/**
  ******************************************************************************
  * @file    tca9548a.h
  * @brief   TCA9548A 8 kanalli I2C coklayici surucusu.
  ******************************************************************************
  */
#ifndef TCA9548A_H
#define TCA9548A_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"

/* RESET darbesi verir ve tum kanallari kapatir. */
void TCA_Init(void);

/* Yalnizca verilen kanali (0..7) acar. Ayni kanal zaten aciksa I2C
   trafigi uretmez -- 4 sensorlu dongu icin bu onemli. */
HAL_StatusTypeDef TCA_SelectChannel(uint8_t ch);

HAL_StatusTypeDef TCA_DisableAll(void);

/* Coklayicinin kontrol registerini geri okur (teshis icin). */
HAL_StatusTypeDef TCA_ReadControl(uint8_t *val);

/* Yazilimin bildigi acik kanal maskesi; 0xFF = bilinmiyor. */
uint8_t TCA_GetCachedMask(void);

#ifdef __cplusplus
}
#endif
#endif /* TCA9548A_H */
