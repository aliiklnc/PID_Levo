/**
  ******************************************************************************
  * @file    gap_sensor.c
  * @brief   4 kanalli hava araligi olcum katmani.
  ******************************************************************************
  */
#include "gap_sensor.h"
#include "tca9548a.h"

static vl6180x_t s_dev[GAP_CH_COUNT];
static gap_ch_t  s_ch[GAP_CH_COUNT];
static float     s_offset[GAP_CH_COUNT];

/* Medyan-3 penceresi. Medyan secildi cunku ToF'un ana kusuru beyaz gurultu
   degil, arada bir gelen tek ornekluk sicramalar; medyan bunlari tek ornek
   gecikmeyle temizler. Hareketli ortalama daha az gurultu birakirdi ama
   getirdigi gecikme kararsiz tesisi kontrol edilemez hale getirir. */
static uint8_t s_hist[GAP_CH_COUNT][3];
static uint8_t s_hist_n[GAP_CH_COUNT];

static uint8_t s_rr;                 /* round-robin sirasi   */
static uint32_t s_hz_tick;           /* hiz penceresi baslangici */
static uint32_t s_hz_base[GAP_CH_COUNT];

/* TCA9548A kanal numarasi: dizi indeksiyle ayni tutuluyor (GAP_CH_*). */
static inline uint8_t mux_of(uint8_t ch) { return ch; }

static uint8_t median3(const uint8_t *v)
{
  uint8_t a = v[0], b = v[1], c = v[2], t;
  if (a > b) { t = a; a = b; b = t; }
  if (b > c) { t = b; b = c; c = t; }
  if (a > b) { b = a; }
  return b;
}

/* ------------------------------------------------------------------ */

uint8_t GapSensor_Init(void)
{
  uint8_t ch, found = 0U;

  TCA_Init();

  for (ch = 0U; ch < GAP_CH_COUNT; ch++)
  {
    s_offset[ch]  = GAP_SENSOR_OFFSET_MM;
    s_hist_n[ch]  = 0U;
    s_hz_base[ch] = 0U;
    s_ch[ch].present     = 0U;
    s_ch[ch].fault       = 1U;      /* kanit gelene kadar arizali say */
    s_ch[ch].raw_mm      = 0U;
    s_ch[ch].gap_mm      = 0.0f;
    s_ch[ch].gap_med_mm  = 0.0f;
    s_ch[ch].status      = 0U;
    s_ch[ch].samples     = 0U;
    s_ch[ch].status_errs = 0U;
    s_ch[ch].io_errs     = 0U;
    s_ch[ch].last_tick   = 0U;
    s_ch[ch].hz          = 0U;

    if (TCA_SelectChannel(mux_of(ch)) != HAL_OK) { continue; }

    if (VL6180X_Init(&s_dev[ch], mux_of(ch)) != HAL_OK)
    {
      /* Kanalda sensor yok ya da takilmis durumda. Sessizce atlanir:
         eksik sensorle calisabilmek kasitli bir tasarim tercihi. */
      continue;
    }
    if (VL6180X_StartContinuous(&s_dev[ch], 10U) != HAL_OK) { continue; }

    s_ch[ch].present = 1U;
    found++;
  }

  (void)TCA_DisableAll();
  s_rr      = 0U;
  s_hz_tick = HAL_GetTick();
  return found;
}

/* ------------------------------------------------------------------ */

static void poll_channel(uint8_t ch)
{
  int r;

  if (TCA_SelectChannel(mux_of(ch)) != HAL_OK)
  {
    s_ch[ch].io_errs++;
    return;
  }

  r = VL6180X_Poll(&s_dev[ch]);
  if (r != 1) { return; }             /* henuz yeni ornek yok */

  s_ch[ch].raw_mm = s_dev[ch].last_range_mm;
  s_ch[ch].status = s_dev[ch].last_status;

  if (s_dev[ch].last_status != VL6180X_RS_NO_ERROR)
  {
    s_ch[ch].status_errs++;
    return;                            /* hatali olcum yayimlanmaz */
  }

  s_ch[ch].samples++;
  s_ch[ch].last_tick = HAL_GetTick();
  s_ch[ch].gap_mm    = (float)s_dev[ch].last_range_mm - s_offset[ch];

  /* medyan-3 */
  if (s_hist_n[ch] < 3U)
  {
    s_hist[ch][s_hist_n[ch]] = s_dev[ch].last_range_mm;
    s_hist_n[ch]++;
    s_ch[ch].gap_med_mm = s_ch[ch].gap_mm;
  }
  else
  {
    s_hist[ch][0] = s_hist[ch][1];
    s_hist[ch][1] = s_hist[ch][2];
    s_hist[ch][2] = s_dev[ch].last_range_mm;
    s_ch[ch].gap_med_mm = (float)median3(s_hist[ch]) - s_offset[ch];
  }
}

void GapSensor_Task(void)
{
  const uint32_t now = HAL_GetTick();
  uint8_t tries;
  uint8_t ch;

  /* Sirada mevcut olan ilk kanali bul. Yok olan kanallar hic I2C
     trafigi uretmez, bu yuzden eksik sensor bant genisligi harcamaz. */
  for (tries = 0U; tries < GAP_CH_COUNT; tries++)
  {
    ch   = s_rr;
    s_rr = (uint8_t)((s_rr + 1U) % GAP_CH_COUNT);
    if (s_ch[ch].present != 0U)
    {
      poll_channel(ch);
      break;
    }
  }

  /* Zaman asimi denetimi ve 1 saniyelik hiz penceresi */
  for (ch = 0U; ch < GAP_CH_COUNT; ch++)
  {
    if (s_ch[ch].present == 0U) { s_ch[ch].fault = 1U; continue; }
    s_ch[ch].fault = ((now - s_ch[ch].last_tick) > SENSOR_TIMEOUT_MS) ? 1U : 0U;
  }

  if ((now - s_hz_tick) >= 1000U)
  {
    s_hz_tick = now;
    for (ch = 0U; ch < GAP_CH_COUNT; ch++)
    {
      s_ch[ch].hz  = (uint16_t)(s_ch[ch].samples - s_hz_base[ch]);
      s_hz_base[ch] = s_ch[ch].samples;
    }
  }
}

/* ------------------------------------------------------------------ */

const gap_ch_t *GapSensor_Get(uint8_t ch)
{
  return (ch < GAP_CH_COUNT) ? &s_ch[ch] : NULL;
}

uint8_t GapSensor_IsValid(uint8_t ch)
{
  if (ch >= GAP_CH_COUNT) { return 0U; }
  return (uint8_t)((s_ch[ch].present != 0U) && (s_ch[ch].fault == 0U));
}

uint8_t GapSensor_ValidCount(void)
{
  uint8_t ch, n = 0U;
  for (ch = 0U; ch < GAP_CH_COUNT; ch++) { n += GapSensor_IsValid(ch); }
  return n;
}

void GapSensor_Calibrate(uint8_t ch, float true_gap_mm)
{
  if (ch >= GAP_CH_COUNT) { return; }
  if (s_ch[ch].present == 0U) { return; }
  s_offset[ch] = (float)s_ch[ch].raw_mm - true_gap_mm;
}

float GapSensor_GetOffset(uint8_t ch)
{
  return (ch < GAP_CH_COUNT) ? s_offset[ch] : 0.0f;
}

void GapSensor_SetOffset(uint8_t ch, float offset_mm)
{
  if (ch < GAP_CH_COUNT) { s_offset[ch] = offset_mm; }
}
