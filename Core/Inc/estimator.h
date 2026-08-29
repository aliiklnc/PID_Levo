/**
  ******************************************************************************
  * @file    estimator.h
  * @brief   4 kose hava araligi olcumunden heave / roll / pitch kestirimi.
  *
  * Sasi rijit kabul edilir; dort sensor ayni duzlemin dort noktasini olcer.
  * Bu yuzden olculen degerlere bir DUZLEM oturtulur:
  *
  *     gap(x, y) = heave + pitch_slope * x + roll_slope * y
  *
  *   x: ileri yon [mm], y: sol yon [mm], sasi merkezine gore
  *   heave       : merkezdeki hava araligi [mm]
  *   pitch_slope : dgap/dx  (pozitif = burun yukarida)
  *   roll_slope  : dgap/dy  (pozitif = sol taraf yukarida)
  *
  * Cozum en kucuk kareler ile yapilir; boylece UC gecerli sensor de yeterlidir
  * (uc nokta bir duzlemi tam belirler). Sartname 3.p acisindan onemli olan
  * budur: bir sensor devre disi kalinca sistem levitasyonu kesmek zorunda
  * kalmaz, kalan uc olcumle calismaya devam eder. Ikiden az gecerli olcumde
  * ya da noktalar ayni dogru uzerindeyken cozum yoktur ve valid = 0 doner.
  ******************************************************************************
  */
#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"

typedef struct
{
  uint8_t valid;         /* 1 = duzlem cozuldu                          */
  uint8_t n_used;        /* cozume giren sensor sayisi                  */
  uint8_t degraded;      /* 1 = 4'ten az sensorle calisiliyor           */
  float   heave_mm;      /* merkezdeki hava araligi                     */
  float   pitch_rad;     /* kucuk aci yaklasimi: egim ~ aci             */
  float   roll_rad;
  float   gap_fit_mm[GAP_CH_COUNT];  /* duzlemin her kose icin degeri   */
  float   residual_mm;   /* en buyuk |olculen - duzlem| (tutarlilik)    */
} est_state_t;

void EstimatorInit(void);

/* GapSensor_Task ile ayni hizda ya da kontrol dongusu hizinda cagrilir. */
void Estimator_Update(void);

const est_state_t *Estimator_Get(void);

#ifdef __cplusplus
}
#endif
#endif /* ESTIMATOR_H */
