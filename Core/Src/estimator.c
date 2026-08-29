/**
  ******************************************************************************
  * @file    estimator.c
  * @brief   Hava araligi olcumlerinden heave / roll / pitch kestirimi.
  ******************************************************************************
  */
#include "estimator.h"
#include "gap_sensor.h"
#include <math.h>

/* Sensor konumlari, sasi merkezine gore [mm].  +x ileri, +y sol.
   Sira: FL, FR, RL, RR -- GAP_CH_* ile ayni. */
static const float s_x[GAP_CH_COUNT] = {
  +GEOM_HALF_LENGTH_MM, +GEOM_HALF_LENGTH_MM,
  -GEOM_HALF_LENGTH_MM, -GEOM_HALF_LENGTH_MM
};
static const float s_y[GAP_CH_COUNT] = {
  +GEOM_HALF_WIDTH_MM, -GEOM_HALF_WIDTH_MM,
  +GEOM_HALF_WIDTH_MM, -GEOM_HALF_WIDTH_MM
};

static est_state_t s_est;

void EstimatorInit(void)
{
  uint8_t i;
  s_est.valid     = 0U;
  s_est.n_used    = 0U;
  s_est.degraded  = 1U;
  s_est.heave_mm  = 0.0f;
  s_est.pitch_rad = 0.0f;
  s_est.roll_rad  = 0.0f;
  s_est.residual_mm = 0.0f;
  for (i = 0U; i < GAP_CH_COUNT; i++) { s_est.gap_fit_mm[i] = 0.0f; }
}

/* z = a + b*x + c*y en kucuk kareler cozumu.
   Normal denklemler:
     [ n    Sx   Sy  ] [a]   [ Sz  ]
     [ Sx   Sxx  Sxy ] [b] = [ Sxz ]
     [ Sy   Sxy  Syy ] [c]   [ Syz ]
   3x3 oldugu icin Cramer kuralini dogrudan uyguluyoruz. */
static uint8_t solve_plane(const float *z, const uint8_t *use, uint8_t n,
                           float *a, float *b, float *c)
{
  float Sx = 0.0f, Sy = 0.0f, Sz = 0.0f;
  float Sxx = 0.0f, Syy = 0.0f, Sxy = 0.0f, Sxz = 0.0f, Syz = 0.0f;
  float m[3][3], r[3], det, d1, d2, d3;
  uint8_t i;

  if (n < 3U) { return 0U; }

  for (i = 0U; i < GAP_CH_COUNT; i++)
  {
    if (use[i] == 0U) { continue; }
    Sx  += s_x[i];              Sy  += s_y[i];              Sz  += z[i];
    Sxx += s_x[i] * s_x[i];     Syy += s_y[i] * s_y[i];
    Sxy += s_x[i] * s_y[i];
    Sxz += s_x[i] * z[i];       Syz += s_y[i] * z[i];
  }

  m[0][0] = (float)n; m[0][1] = Sx;  m[0][2] = Sy;
  m[1][0] = Sx;       m[1][1] = Sxx; m[1][2] = Sxy;
  m[2][0] = Sy;       m[2][1] = Sxy; m[2][2] = Syy;
  r[0] = Sz; r[1] = Sxz; r[2] = Syz;

  det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
      - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
      + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

  /* Noktalar ayni dogru uzerindeyse (ornegin yalnizca on eksendeki iki
     sensor artı biri) determinant sifira gider ve duzlem belirsizdir. */
  if (fabsf(det) < 1.0e-3f) { return 0U; }

  d1 = r[0]    * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
     - m[0][1] * (r[1]    * m[2][2] - m[1][2] * r[2])
     + m[0][2] * (r[1]    * m[2][1] - m[1][1] * r[2]);

  d2 = m[0][0] * (r[1]    * m[2][2] - m[1][2] * r[2])
     - r[0]    * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
     + m[0][2] * (m[1][0] * r[2]    - r[1]    * m[2][0]);

  d3 = m[0][0] * (m[1][1] * r[2]    - r[1]    * m[2][1])
     - m[0][1] * (m[1][0] * r[2]    - r[1]    * m[2][0])
     + r[0]    * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

  *a = d1 / det;
  *b = d2 / det;
  *c = d3 / det;
  return 1U;
}

void Estimator_Update(void)
{
  float   z[GAP_CH_COUNT];
  uint8_t use[GAP_CH_COUNT];
  uint8_t i, n = 0U;
  float   a, b, c, e, worst = 0.0f;

  for (i = 0U; i < GAP_CH_COUNT; i++)
  {
    const gap_ch_t *g = GapSensor_Get(i);
    use[i] = GapSensor_IsValid(i);
    z[i]   = (g != NULL) ? g->gap_med_mm : 0.0f;
    n     += use[i];
  }

  s_est.n_used   = n;
  s_est.degraded = (n < GAP_CH_COUNT) ? 1U : 0U;

  /* --- Iki sensorlu indirgenmis durum ---
     Iki nokta bir duzlem belirlemez, ama ayni eksen uzerindeyseler O eksenin
     egimi tam olarak cozulur. Dort sensor tamamlanana kadar tezgah testinin
     anlamli sayi uretebilmesi icin bu durum ayrica ele aliniyor.
     Capraz bir cift (or. FL + RR) roll ile pitch'i ayirt edemez; o durumda
     asagidaki genel yol calisir ve valid = 0 doner. */
  if (n == 2U)
  {
    uint8_t i0 = 0xFFU, i1 = 0xFFU;
    for (i = 0U; i < GAP_CH_COUNT; i++)
    {
      if (use[i] == 0U) { continue; }
      if (i0 == 0xFFU) { i0 = i; } else { i1 = i; }
    }
    if (i1 != 0xFFU)
    {
      float dx = s_x[i1] - s_x[i0];
      float dy = s_y[i1] - s_y[i0];

      if ((fabsf(dx) < 1.0f) && (fabsf(dy) > 1.0f))
      {
        /* Ayni x: sol-sag cift -> roll cozulur, pitch bilinmiyor */
        c = (z[i1] - z[i0]) / dy;
        b = 0.0f;
        a = z[i0] - (c * s_y[i0]) - (b * s_x[i0]);
      }
      else if ((fabsf(dy) < 1.0f) && (fabsf(dx) > 1.0f))
      {
        /* Ayni y: on-arka cift -> pitch cozulur, roll bilinmiyor */
        b = (z[i1] - z[i0]) / dx;
        c = 0.0f;
        a = z[i0] - (b * s_x[i0]);
      }
      else
      {
        a = 0.0f; b = 0.0f; c = 0.0f;
        i1 = 0xFFU;   /* capraz cift: cozulemez */
      }

      if (i1 != 0xFFU)
      {
        s_est.heave_mm  = a;
        s_est.pitch_rad = b;
        s_est.roll_rad  = c;
        for (i = 0U; i < GAP_CH_COUNT; i++)
        {
          s_est.gap_fit_mm[i] = a + (b * s_x[i]) + (c * s_y[i]);
        }
        s_est.residual_mm = 0.0f;   /* iki nokta her zaman tam oturur */
        /* valid = 1, ama degraded = 1 kaliyor: kontrol katmani bir eksenin
           bilinmedigini n_used'a bakarak anlamali. */
        s_est.valid = 1U;
        return;
      }
    }
  }

  if (solve_plane(z, use, n, &a, &b, &c) == 0U)
  {
    s_est.valid = 0U;
    /* Duzlem cozulemedi. Yine de tek bir sayi vermek gerekirse gecerli
       olcumlerin ortalamasi en makul heave tahminidir; kontrol katmani
       valid = 0 gordugunde buna guvenmemelidir. */
    if (n > 0U)
    {
      float sum = 0.0f;
      for (i = 0U; i < GAP_CH_COUNT; i++) { if (use[i] != 0U) { sum += z[i]; } }
      s_est.heave_mm = sum / (float)n;
    }
    s_est.pitch_rad = 0.0f;
    s_est.roll_rad  = 0.0f;
    return;
  }

  s_est.heave_mm  = a;
  s_est.pitch_rad = b;      /* kucuk aci: egim ~ aci [rad] */
  s_est.roll_rad  = c;

  for (i = 0U; i < GAP_CH_COUNT; i++)
  {
    s_est.gap_fit_mm[i] = a + (b * s_x[i]) + (c * s_y[i]);
    if (use[i] != 0U)
    {
      e = fabsf(z[i] - s_est.gap_fit_mm[i]);
      if (e > worst) { worst = e; }
    }
  }
  s_est.residual_mm = worst;
  s_est.valid = 1U;
}

const est_state_t *Estimator_Get(void)
{
  return &s_est;
}
