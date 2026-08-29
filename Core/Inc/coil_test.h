/**
  ******************************************************************************
  * @file    coil_test.h
  * @brief   Elektromiknatis karakterizasyon testi: bobin direnci (R),
  *          enduktansi (L) ve BTS7960 IS pininin gercek bant genisligi.
  *
  * Yontem: cikis kapaliyken bir taban cizgisi alinir, sonra bobine tam
  * gerilim (duty %100) uygulanir ve akimin yukselme egrisi 100 kHz'de
  * RAM'e kaydedilir; ardindan cikis kesilip sonum egrisi de kaydedilir.
  *
  *   yukselme:  i(t) = (V/R) * (1 - exp(-t*R/L))
  *   sonum   :  i(t) = i0 * exp(-t*R/L)
  *
  * Iki egriden R ve L bagimsiz olarak cikarilabilir; ayrica ham dalga
  * formu IS pininin dalgalanmayi cozup cozemedigini dogrudan gosterir.
  *
  * GUVENLIK: darbe suresi COIL_TEST_MAX_ON_MS ile sinirlidir, akim
  * COIL_TEST_ABORT_AMP'i asarsa yakalama derhal durdurulur ve enable
  * pinleri LOW'a cekilir. Test acilista BIR KEZ calisir.
  ******************************************************************************
  */
#ifndef COIL_TEST_H
#define COIL_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"

/* 2048 tetik x 2 kanal (IS, VBUS) = 20.48 ms, kanal basina 100 kHz */
#define COIL_CAP_PAIRS      2048U
#define COIL_CAP_LEN        (COIL_CAP_PAIRS * 2U)

typedef enum
{
  COIL_TEST_IDLE = 0,
  COIL_TEST_RUNNING,
  COIL_TEST_DONE,
  COIL_TEST_ABORTED_OVERCURRENT,
  COIL_TEST_ERR_INIT
} coil_test_state_t;

typedef struct
{
  coil_test_state_t state;
  uint16_t idx_on;        /* cikisin acildigi ornek indeksi   */
  uint16_t idx_off;       /* cikisin kesildigi ornek indeksi  */
  uint16_t samples;       /* toplanan cift sayisi             */
  uint16_t adc_baseline;  /* cikis kapaliyken IS ham degeri   */
  uint16_t adc_peak;      /* darbe sonundaki IS ham degeri    */
  float    vbus_v;        /* darbe oncesi bus gerilimi        */
  float    vbus_min_v;    /* darbe sirasindaki en dusuk bus   */
  float    i_peak_a;      /* tepe akim (IS kalibrasyonuna gore) */
} coil_test_result_t;

/* Cevre birimlerini kurar (TIM1 PWM, TIM2 tetik, ADC1 + DMA2).
   CubeMX'ten bagimsizdir. */
HAL_StatusTypeDef CoilTest_Init(void);

/* Cikislari pasif ve guvenli duruma alir. Her cikis yolunda cagrilir. */
void CoilTest_SafeOff(void);

/* Tek atimlik yakalama. Blokleyicidir, ~25 ms surer. */
void CoilTest_Run(void);

/* Yakalanan ham ADC tamponu: cift indeksler IS, tek indeksler VBUS. */
extern volatile uint16_t g_coil_cap[COIL_CAP_LEN];
extern volatile coil_test_result_t g_coil_res;

#ifdef __cplusplus
}
#endif
#endif /* COIL_TEST_H */
