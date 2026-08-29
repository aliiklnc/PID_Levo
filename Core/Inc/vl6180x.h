/**
  ******************************************************************************
  * @file    vl6180x.h
  * @brief   VL6180X (TOF050C modulu) mesafe sensoru surucusu.
  *
  * Onemli noktalar:
  *  - Register adresleri 16 BIT'tir (MSB once).
  *  - I2C 7-bit adresi 0x29'dur (HAL'e 0x52 olarak verilir).
  *  - Guc verildikten sonra AN4545'te tanimli "SR03 settings" ozel register
  *    dizisi YAZILMAK ZORUNDADIR; yazilmazsa sensor kararsiz/yanlis olcer.
  ******************************************************************************
  */
#ifndef VL6180X_H
#define VL6180X_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"

#define VL6180X_ADDR_7B         0x29U
#define VL6180X_MODEL_ID        0xB4U

/* RESULT__RANGE_STATUS ust nibble'inda donen hata kodlari */
#define VL6180X_RS_NO_ERROR           0x00U
#define VL6180X_RS_VCSEL_CONT_TEST    0x01U
#define VL6180X_RS_VCSEL_WATCHDOG_TEST 0x02U
#define VL6180X_RS_VCSEL_WATCHDOG     0x03U
#define VL6180X_RS_PLL1_LOCK          0x04U
#define VL6180X_RS_PLL2_LOCK          0x05U
#define VL6180X_RS_EARLY_CONV_EST     0x06U
#define VL6180X_RS_MAX_CONV           0x07U   /* yakinsama zaman asimi   */
#define VL6180X_RS_NO_TARGET          0x08U   /* hedef yok / cok uzak    */
#define VL6180X_RS_MAX_SNR            0x0BU
#define VL6180X_RS_RAW_UNDERFLOW      0x0CU
#define VL6180X_RS_RAW_OVERFLOW       0x0DU
#define VL6180X_RS_RANGE_UNDERFLOW    0x0EU   /* hedef olu bolgede       */
#define VL6180X_RS_RANGE_OVERFLOW     0x0FU

typedef struct
{
  uint8_t  mux_channel;      /* bagli oldugu TCA9548A kanali            */
  uint8_t  present;          /* 1 = MODEL_ID dogrulandi                 */
  uint8_t  continuous;       /* 1 = surekli mod calisiyor               */
  uint8_t  last_range_mm;    /* son ham olcum [mm]                      */
  uint8_t  last_status;      /* son RESULT__RANGE_STATUS hata kodu      */
  uint32_t sample_count;     /* toplanan gecerli ornek sayisi           */
  uint32_t status_err_count; /* last_status != 0 olan ornek sayisi      */
  uint32_t io_err_count;     /* I2C seviyesinde basarisiz erisim sayisi */
  uint32_t last_sample_tick; /* son ornegin HAL_GetTick() zamani        */
} vl6180x_t;

/* Sensoru bulur, gerekiyorsa AN4545 tuning dizisini yazar ve varsayilan
   olcum ayarlarini yukler. Cagirmadan ONCE dogru mux kanali acilmis olmalidir. */
HAL_StatusTypeDef VL6180X_Init(vl6180x_t *dev, uint8_t mux_channel);

/* Surekli olcum modu. period_ms 10..2550 arasi, 10 ms adimlarla yuvarlanir.
   10 ms -> teorik 100 Hz. */
HAL_StatusTypeDef VL6180X_StartContinuous(vl6180x_t *dev, uint16_t period_ms);
HAL_StatusTypeDef VL6180X_StopContinuous(vl6180x_t *dev);

/* Hazir ornek varsa alir ve dev alanlarini gunceller.
   Donus:  1 = yeni ornek alindi
           0 = henuz hazir degil
          -1 = I2C hatasi */
int VL6180X_Poll(vl6180x_t *dev);

/* Tek atimlik blokleyici olcum (teshis/kalibrasyon icin). */
HAL_StatusTypeDef VL6180X_ReadSingle(vl6180x_t *dev, uint8_t *range_mm);

/* Hata kodunun insan okunur karsiligi (Live Expressions / log icin). */
const char *VL6180X_StatusStr(uint8_t status);

/* Teshis amacli dogrudan register erisimi. Dogru mux kanali acik olmalidir. */
HAL_StatusTypeDef VL6180X_ReadReg(uint16_t reg, uint8_t *val);
HAL_StatusTypeDef VL6180X_WriteReg(uint16_t reg, uint8_t val);

#ifdef __cplusplus
}
#endif
#endif /* VL6180X_H */
