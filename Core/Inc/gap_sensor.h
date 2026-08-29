/**
  ******************************************************************************
  * @file    gap_sensor.h
  * @brief   4 kanalli hava araligi olcum katmani (TCA9548A + 4 x VL6180X).
  *
  * Her sensor kendi surekli modunda SERBEST kosar; MCU sirayla coklayici
  * kanalini degistirip hazir olan ornegi toplar. Boylece hicbir olcum
  * beklenmez ve bir sensorun yavaslamasi digerlerini etkilemez.
  *
  * Eksik sensor sorun degil: acilista bulunamayan kanallar "yok" olarak
  * isaretlenir ve yoklama dongusunde hic I2C trafigi uretmezler. Bu sayede
  * ayni kod 1, 2, 3 veya 4 sensorle calisir -- montaj tamamlandikca kod
  * degismeden kanallar devreye girer.
  *
  * Sartname 3.p: bir sensor cevap vermeyi birakirsa yalnizca O kanal
  * SENSOR_FAULT'a duser, digerleri kesintisiz devam eder. Kac kanalin
  * saglikli oldugu GapSensor_ValidCount() ile sorulur; kestirici uc gecerli
  * olcumle hala tam duzlem cozebildigi icin sistem levitasyonu kesmek
  * zorunda kalmaz.
  ******************************************************************************
  */
#ifndef GAP_SENSOR_H
#define GAP_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"
#include "vl6180x.h"

typedef struct
{
  uint8_t  present;        /* 1 = acilista bulundu ve yapilandirildi     */
  uint8_t  fault;          /* 1 = zaman asimi veya surekli olcum hatasi  */
  uint8_t  raw_mm;         /* sensorun ham okumasi [mm]                  */
  float    gap_mm;         /* kalibre edilmis hava araligi [mm]          */
  float    gap_med_mm;     /* medyan-3 filtreli (tek ornek gecikme)      */
  uint8_t  status;         /* son RESULT__RANGE_STATUS hata kodu         */
  uint32_t samples;        /* gecerli ornek sayaci                       */
  uint32_t status_errs;    /* olcum hatasi sayaci                        */
  uint32_t io_errs;        /* I2C hata sayaci                            */
  uint32_t last_tick;      /* son gecerli ornegin zamani [ms]            */
  uint16_t hz;             /* olculen gercek ornekleme hizi              */
} gap_ch_t;

/* Coklayiciyi ve bulunabilen tum sensorleri baslatir.
   Donen deger: bulunan sensor sayisi (0 olabilir). */
uint8_t GapSensor_Init(void);

/* Ana donguden periyodik cagrilir. Her cagrida SIRADAKI mevcut kanali
   yoklar (round-robin), bu yuzden blokleme suresi tek kanallik erisimle
   sinirlidir. GAP_POLL_STEP_MS periyodunda cagrilmalidir. */
void GapSensor_Task(void);

const gap_ch_t *GapSensor_Get(uint8_t ch);

/* Zaman asimi olmamis, olcum hatasi bulunmayan kanal sayisi. */
uint8_t GapSensor_ValidCount(void);
uint8_t GapSensor_IsValid(uint8_t ch);

/* Kanal basina sifir noktasi ayari: sensor montajlandiktan sonra bilinen
   bir hava araliginda cagrilir, o anki okumayi o degere esitler. */
void GapSensor_Calibrate(uint8_t ch, float true_gap_mm);
float GapSensor_GetOffset(uint8_t ch);
void  GapSensor_SetOffset(uint8_t ch, float offset_mm);

#ifdef __cplusplus
}
#endif
#endif /* GAP_SENSOR_H */
