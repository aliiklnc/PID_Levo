/**
  ******************************************************************************
  * @file    state_machine.h
  * @brief   Levitasyon emniyet ve calisma durumu yonetimi.
  *
  * Sartname karsiliklari:
  *   3.p  Bir modul devre disi kalirsa stabil kal ya da levitasyonu TAMAMEN
  *        kes  -> sensor/akim arizasinda LANDING -> SAFE_SHUTDOWN (mandalli)
  *   3.q  Kalkis ve stabil durumda akim olcumu -> current_sense + zirve kaydi
  *   3.s  Guc kesintisinde raya zarar vermeden guvenli inis
  *        -> dusuk gerilimde ani kesme degil, kontrollu duty rampasi (LANDING)
  *   3.t  FMEA / fail-safe -> hata bit maskesi, mandalli kapanma, IWDG
  *   3.u  Hava araligi >= 2 mm -> GAP_MIN ihlali aninda hata
  *   EK1-3.4  Yer istasyonu baglantisi koparsa guvenli duruma gec
  *        -> GS_TIMEOUT_MS icinde heartbeat gelmezse LANDING
  *
  * Durum gecisleri:
  *
  *   INIT -> SELFTEST -> IDLE
  *                        | SM_RequestLevitate(1)
  *                        v
  *                    SOFT_START --(hedef araliga ulasildi)--> LEVITATING
  *                        |                                        |
  *                        | zaman asimi                            | istek=0
  *                        v                                        v
  *                      FAULT <---------- hata ----------------- LANDING
  *                        |                                        |
  *                        v                                        v
  *                 SAFE_SHUTDOWN (mandalli, cikis icin reset gerekir)
  *
  * SAFE_SHUTDOWN mandallidir: yalnizca donanim reseti ile cikilir. Yarismada
  * "levitasyonu tamamen kes" istegi tam olarak bunu gerektiriyor.
  ******************************************************************************
  */
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"

typedef enum
{
  SM_INIT = 0,
  SM_SELFTEST,
  SM_IDLE,
  SM_SOFT_START,
  SM_LEVITATING,
  SM_LANDING,
  SM_FAULT,
  SM_SAFE_SHUTDOWN
} sm_state_t;

/* Hata bit maskesi -- FMEA tablosunun yazilim karsiligi (sartname 3.t).
   Bitler mandallidir; SM_ClearFaults() yalnizca IDLE/FAULT'ta islev gorur. */
#define SM_F_SENSOR_TIMEOUT   (1UL << 0)  /* taze hava araligi olcumu yok   */
#define SM_F_GAP_MIN          (1UL << 1)  /* aralik 2 mm altina indi (3.u)  */
#define SM_F_GAP_MAX          (1UL << 2)  /* yakalama araligi disina cikti  */
#define SM_F_OVERCURRENT      (1UL << 3)  /* bobin akimi limiti asti        */
#define SM_F_UNDERVOLTAGE     (1UL << 4)  /* batarya bitti / guc kesildi    */
#define SM_F_OVERVOLTAGE      (1UL << 5)
#define SM_F_GS_TIMEOUT       (1UL << 6)  /* yer istasyonu sessiz (EK1-3.4) */
#define SM_F_SOFTSTART_TMO    (1UL << 7)  /* kalkis suresinde askiya gecemedi */
#define SM_F_SELFTEST         (1UL << 8)  /* acilis testi basarisiz         */
#define SM_F_WATCHDOG_RESET   (1UL << 9)  /* onceki cevrim IWDG ile resetlendi */
#define SM_F_DRIVER_INIT      (1UL << 10) /* BTS7960 / ADC baslatilamadi    */

void SM_Init(void);

/* CTRL_PERIOD_MS periyodunda cagrilir. IWDG'yi de bu fonksiyon besler --
   dongu takilirsa donanim reseti olur ve EN pinleri pull-down ile LOW'a duser. */
void SM_Task(void);

/* Sensor katmanindan gelen guncelleme. valid=0 ise olcum kullanilmaz;
   SENSOR_TIMEOUT_MS boyunca gecerli olcum gelmezse hata uretilir. */
void SM_ReportGap(float gap_mm, uint8_t valid);

/* Yer istasyonundan heartbeat alindiginda cagrilir (EK1-3.4). */
void SM_ReportGroundStation(void);

/* Asama 4'te PID bu fonksiyonla duty yazacak. Yalnizca LEVITATING durumunda
   dikkate alinir; diger durumlarda sessizce yok sayilir. */
void SM_SetControlDuty(uint8_t ch, float duty);

void SM_RequestLevitate(uint8_t on);

/* Geri donusu olmayan kapanma. Cikislar kesilir, yalnizca reset ile cikilir. */
void SM_LatchShutdown(uint32_t fault_mask);

void SM_ClearFaults(void);

sm_state_t  SM_GetState(void);
uint32_t    SM_GetFaults(void);
const char *SM_StateName(sm_state_t s);

/* Sartname 3.q: kalkis transiyenti ve stabil durum icin zirve kayitlari */
float SM_GetPeakCurrent(void);
float SM_GetPeakPower(void);
void  SM_ResetPeaks(void);

#ifdef __cplusplus
}
#endif
#endif /* STATE_MACHINE_H */
