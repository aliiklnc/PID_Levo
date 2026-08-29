/**
  ******************************************************************************
  * @file    current_sense.h
  * @brief   4 kanal bobin akimi + bus gerilimi olcumu (sartname 3.q / 3.r).
  *
  * ADC1, TIM2 TRGO ile 40 kHz'de tetiklenir; 5 kanallik tarama DMA2_Stream0
  * uzerinden dairesel bir tampona yazilir. Boylece son degerler her an
  * hazirdir, kontrol dongusu ADC'yi beklemez.
  *
  *   tampon indeksi:  0=IS_FL  1=IS_FR  2=IS_RL  3=IS_RR  4=VBUS
  *
  * BTS7960'in IS pini bir akim kaynagidir (I_yuk / 8500) ve 2.2k direnc
  * uzerinde gerilime cevrilir. Ofset (yuk yokken okunan deger) sifirlanabilir;
  * CS_Zero() cikislar kapaliyken cagrilmalidir.
  ******************************************************************************
  */
#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"

/* MX_ADC1_Init + MX_TIM2_Init sonrasi cagrilir. DMA'yi ve TIM2'yi baslatir. */
HAL_StatusTypeDef CS_Init(void);

/* Cikislar KAPALIYKEN cagrilmalidir: her kanalin sifir noktasini olcer. */
void CS_ZeroCalibrate(void);

/* Filtrelenmis degerleri gunceller. Kontrol dongusu hizinda cagrilir. */
void CS_Update(void);

/* DMA en az bir tam/yarim tampon yazdi ve veri halen taze mi? */
uint8_t  CS_IsFresh(void);
float    CS_GetCurrent(uint8_t ch);   /* amper, ofset dusulmus, filtreli */
float    CS_GetCurrentRaw(uint8_t ch);/* amper, filtresiz (trip icin)    */
float    CS_GetBusVoltage(void);      /* volt                            */
float    CS_GetPower(void);           /* watt, toplam (V * sum(I))       */
uint16_t CS_GetAdcRaw(uint8_t idx);   /* ham ADC, teshis icin            */

#ifdef __cplusplus
}
#endif
#endif /* CURRENT_SENSE_H */
