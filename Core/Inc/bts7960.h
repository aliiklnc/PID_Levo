/**
  ******************************************************************************
  * @file    bts7960.h
  * @brief   4 kanalli BTS7960 H-koprusu surucu katmani.
  *
  * Kanal indeksleri GAP_CH_* ile ayni: 0=FL, 1=FR, 2=RL, 3=RR.
  *
  * Tek yonlu surus: RPWM = TIM1 CHx (PWM), LPWM = donanimda GND. Bobin
  * akiminin isareti onemsizdir -- kuvvet F = B^2*A/mu0 oldugundan her iki
  * yonde de cekme olur. Kartta ters surus yolu bulunmaz.
  *
  * FAIL-SAFE ilkesi (sartname 3.p / 3.s / 3.t):
  *   - Cikislar boot'ta LOW, EN pinlerinde donanim pull-down var.
  *   - BTS_AllOff() her hata yolunda cagrilir; kesme icinden guvenlidir.
  *   - Enable pinleri LOW iken duty ne olursa olsun koprude akim akmaz.
  ******************************************************************************
  */
#ifndef BTS7960_H
#define BTS7960_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"

/* MX_TIM1_Init'ten SONRA cagrilir. PWM'i baslatir, tum duty'leri sifirlar
   ve EN pinlerini LOW'a ceker. */
HAL_StatusTypeDef BTS_Init(void);

/* FAIL-SAFE cikis. Duty=0, EN=LOW. Kesme icinden cagrilabilir,
   blok etmez, hata dondurmez -- her kosulda calismak zorundadir. */
void BTS_AllOff(void);

/* Kopruleri etkinlestirir. Once tum duty'ler sifirlanir, sonra EN yukselir. */
void BTS_Enable(void);

/* Duty 0.0 .. BTS_DUTY_MAX arasina sinirlanir. Kopru pasifken de yazilabilir;
   deger saklanir ama BTS_Enable() cagrilana kadar etkisizdir. */
void  BTS_SetDuty(uint8_t ch, float duty);
float BTS_GetDuty(uint8_t ch);

/* Tum kanallara ayni duty (self-test ve dengeli yuk icin) */
void BTS_SetDutyAll(float duty);

uint8_t BTS_IsEnabled(void);

#ifdef __cplusplus
}
#endif
#endif /* BTS7960_H */
