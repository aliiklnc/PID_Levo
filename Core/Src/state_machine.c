/**
  ******************************************************************************
  * @file    state_machine.c
  * @brief   Levitasyon emniyet ve calisma durumu yonetimi.
  ******************************************************************************
  */
#include "state_machine.h"
#include "bts7960.h"
#include "current_sense.h"

/* ============================ IWDG ==================================
   HAL_IWDG modulu .ioc'de acik degil; IWDG uc register'dan ibaret oldugu
   icin dogrudan yaziyoruz. Boylece CubeMX'e tekrar donmek gerekmiyor.
   LSI ~32 kHz, PR=3 -> bolucu 32 -> 1 tik ~= 1 ms.                    */
#define IWDG_KEY_RELOAD   0x0000AAAAUL
#define IWDG_KEY_ENABLE   0x0000CCCCUL
#define IWDG_KEY_UNLOCK   0x00005555UL
#define IWDG_PR_DIV32     3UL

static void iwdg_start(void)
{
  /* Hata ayiklayici bagliysa bekci kopegini cekirdek durdugunda dondur.
     Aksi halde her breakpoint 250 ms sonra karti resetler. Kosul, prob
     debug'i gercekten etkinlestirdiginde saglanir; normal calismada
     (prob yokken) bu satir atlanir ve bekci kopegi tam yetkiyle calisir. */
  if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0UL)
  {
    __HAL_DBGMCU_FREEZE_IWDG();
  }

  /* SIRALAMA KRITIK: LSI osilatoru ancak IWDG etkinlestirildiginde (0xCCCC)
     calismaya baslar. Once PR/RLR yazip SR'nin temizlenmesini beklemek
     sonsuz donguye girer, cunku SR yalnizca LSI saat alanindan temizlenir. */
  IWDG->KR  = IWDG_KEY_ENABLE;   /* baslat -> LSI calisir  */
  IWDG->KR  = IWDG_KEY_UNLOCK;   /* PR/RLR yazmaya izin ver */
  IWDG->PR  = IWDG_PR_DIV32;
  IWDG->RLR = IWDG_RELOAD_TICKS;

  /* Sinirli bekleme: LSI beklenmedik bir sekilde calismazsa acilis
     kilitlenmesin. Zaman asiminda bekci kopegi varsayilan periyotla
     (PR=0, RLR=0xFFF -> ~512 ms) calismaya devam eder; korumasiz kalmayiz. */
  {
    uint32_t guard = 200000UL;
    while ((IWDG->SR != 0UL) && (guard != 0UL)) { guard--; }
  }
  IWDG->KR  = IWDG_KEY_RELOAD;
}

static inline void iwdg_refresh(void)
{
  IWDG->KR = IWDG_KEY_RELOAD;
}

/* ============================ durum ================================= */

static sm_state_t s_state;
static uint32_t   s_faults;
static uint32_t   s_state_ms;        /* duruma girildigi an              */
static uint8_t    s_request;         /* levitasyon istegi                */

static float      s_gap_mm;
static uint8_t    s_gap_valid;
static uint32_t   s_gap_ms;          /* son gecerli olcum ani            */
static uint32_t   s_gs_ms;           /* son yer istasyonu heartbeat'i    */
static uint8_t    s_gs_seen;         /* hic heartbeat geldi mi           */

static float      s_ramp_duty;       /* SOFT_START / LANDING rampasi     */
static float      s_ctrl_duty[BTS_CH_COUNT];
static uint8_t    s_ctrl_fresh;

static uint32_t   s_oc_since;        /* asiri akimin basladigi an, 0=yok */
static uint32_t   s_uv_since;
static uint32_t   s_settle_since;

static float      s_peak_i;
static float      s_peak_p;

/* ------------------------------------------------------------------ */

static void enter(sm_state_t st)
{
  s_state    = st;
  s_state_ms = HAL_GetTick();
}

static void fault(uint32_t mask)
{
  s_faults |= mask;
  BTS_AllOff();
  enter(SM_FAULT);
}

/* Hem FAULT hem SAFE_SHUTDOWN'da cikis kesilidir; fark, SAFE_SHUTDOWN'dan
   yalnizca reset ile cikilabilmesidir (sartname 3.p "tamamen kes"). */
void SM_LatchShutdown(uint32_t fault_mask)
{
  s_faults |= fault_mask;
  BTS_AllOff();
  enter(SM_SAFE_SHUTDOWN);
}

/* ------------------------------------------------------------------ */

void SM_Init(void)
{
  uint8_t ch;

  s_state      = SM_INIT;
  s_faults     = 0UL;
  s_request    = 0U;
  s_gap_mm     = 0.0f;
  s_gap_valid  = 0U;
  s_gap_ms     = 0UL;
  s_gs_ms      = 0UL;
  s_gs_seen    = 0U;
  s_ramp_duty  = 0.0f;
  s_ctrl_fresh = 0U;
  s_oc_since   = 0UL;
  s_uv_since   = 0UL;
  s_settle_since = 0UL;
  s_peak_i     = 0.0f;
  s_peak_p     = 0.0f;
  for (ch = 0U; ch < BTS_CH_COUNT; ch++) { s_ctrl_duty[ch] = 0.0f; }

  /* Onceki cevrim bekci kopegi ile mi resetlendi? Bu, kontrol dongusunun
     takildigi anlamina gelir -- kaydedip raporluyoruz (sartname 3.t). */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    s_faults |= SM_F_WATCHDOG_RESET;
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();

  BTS_AllOff();
  enter(SM_INIT);
  iwdg_start();
}

void SM_ReportGap(float gap_mm, uint8_t valid)
{
  if (valid != 0U)
  {
    s_gap_mm    = gap_mm;
    s_gap_valid = 1U;
    s_gap_ms    = HAL_GetTick();
  }
}

void SM_ReportGroundStation(void)
{
  s_gs_ms   = HAL_GetTick();
  s_gs_seen = 1U;
}

void SM_SetControlDuty(uint8_t ch, float duty)
{
  if (ch >= BTS_CH_COUNT) { return; }
  s_ctrl_duty[ch] = duty;
  s_ctrl_fresh    = 1U;
}

void SM_RequestLevitate(uint8_t on)
{
  s_request = (on != 0U) ? 1U : 0U;
}

void SM_ClearFaults(void)
{
  /* Mandalli kapanma temizlenmez. */
  if (s_state == SM_SAFE_SHUTDOWN) { return; }
  s_faults = 0UL;
  s_oc_since = 0UL;
  s_uv_since = 0UL;
  if (s_state == SM_FAULT) { enter(SM_IDLE); }
}

/* ------------------------------------------------------------------ */

/* Cikis sürülürken her cevrimde bakilan emniyet kontrolleri.
   Trip karari FILTRESIZ akimdan verilir; filtre gecikmesi emniyeti
   yavaslatmamali. */
static uint32_t check_hazards(uint32_t now, uint8_t outputs_live)
{
  uint32_t f = 0UL;
  uint8_t  ch;
  float    i, v;

  /* --- sensor tazeligi (3.p) --- */
  if ((s_gap_valid == 0U) || ((now - s_gap_ms) > SENSOR_TIMEOUT_MS))
  {
    f |= SM_F_SENSOR_TIMEOUT;
  }
  else if (outputs_live != 0U)
  {
    /* --- hava araligi limitleri (3.u) --- */
    if (s_gap_mm < GAP_MIN_MM)          { f |= SM_F_GAP_MIN; }
    if (s_gap_mm > (GAP_TARGET_MM * 4.0f)) { f |= SM_F_GAP_MAX; }
  }

  /* --- ADC/DMA tazeligi ---
     Eski ama sayisal olarak makul bir ADC degeriyle guvenlik karari
     verilmez. Akim ve gerilim denetimleri yalniz taze veriyle yapilir. */
  if (CS_IsFresh() == 0U)
  {
    f |= SM_F_ADC_TIMEOUT;
    s_oc_since = 0UL;
    s_uv_since = 0UL;
  }

  /* --- akim (3.q) ---
     Zirve kaydi TUM kanallar icin yapilir; asim zamanlayicisi ise "su an
     herhangi bir kanal limitin ustunde mi" bilgisine bakar. Zamanlayiciyi
     hata bitine bakarak sifirlamak, trip suresi dolmadan her cevrimde
     sayaci sifirlar ve asim hicbir zaman tetiklenmezdi. */
  if ((f & SM_F_ADC_TIMEOUT) == 0UL)
  {
    uint8_t over = 0U;
    for (ch = 0U; ch < BTS_CH_COUNT; ch++)
    {
      i = CS_GetCurrentRaw(ch);
      if (i > s_peak_i) { s_peak_i = i; }
      if (i > COIL_CURRENT_MAX_A) { over = 1U; }
    }
    if (over != 0U)
    {
      if (s_oc_since == 0UL) { s_oc_since = now; }
      else if ((now - s_oc_since) >= COIL_CURRENT_TRIP_MS)
      {
        f |= SM_F_OVERCURRENT;
      }
    }
    else { s_oc_since = 0UL; }
  }

  /* --- bus gerilimi (3.s) --- */
  if ((f & SM_F_ADC_TIMEOUT) == 0UL)
  {
    v = CS_GetBusVoltage();
    if ((v < VBUS_MIN_V) || (v > VBUS_MAX_V))
    {
      if (s_uv_since == 0UL) { s_uv_since = now; }
      else if ((now - s_uv_since) >= VBUS_TRIP_MS)
      {
        f |= (v < VBUS_MIN_V) ? SM_F_UNDERVOLTAGE : SM_F_OVERVOLTAGE;
      }
    }
    else { s_uv_since = 0UL; }
  }

  /* --- yer istasyonu (EK1-3.4) --- */
  if ((s_gs_seen != 0U) && ((now - s_gs_ms) > GS_TIMEOUT_MS))
  {
    f |= SM_F_GS_TIMEOUT;
  }

  return f;
}

/* Guc sinifi hatalar: hemen kes. Bunlar icin kontrollu inis yapilamaz,
   cunku kopruye guvenemeyiz. */
#define SM_F_HARD   (SM_F_OVERCURRENT | SM_F_OVERVOLTAGE | SM_F_DRIVER_INIT | SM_F_ADC_TIMEOUT)

/* Yumusak hatalar: kontrollu inis denenir (sartname 3.s / 3.p). */
#define SM_F_SOFT   (SM_F_SENSOR_TIMEOUT | SM_F_GAP_MIN | SM_F_GAP_MAX | \
                     SM_F_UNDERVOLTAGE   | SM_F_GS_TIMEOUT)

void SM_Task(void)
{
  const uint32_t now = HAL_GetTick();
  const float    dt  = (float)CTRL_PERIOD_MS * 0.001f;
  uint32_t       hz;
  uint8_t        ch;
  uint8_t        any_live;
  float          duty;
  float          p;

  iwdg_refresh();
  CS_Update();

  p = CS_GetPower();
  if (p > s_peak_p) { s_peak_p = p; }

  switch (s_state)
  {
    case SM_INIT:
      enter(SM_SELFTEST);
      break;

    case SM_SELFTEST:
      /* Cikislar kapaliyken bus gerilimi makul mu, ADC donuyor mu?
         Bobin surmeden yapilabilecek tek anlamli test bu. */
      if ((now - s_state_ms) >= 100UL)
      {
        float v = CS_GetBusVoltage();
        if (CS_IsFresh() == 0U)
        {
          s_faults |= SM_F_SELFTEST | SM_F_ADC_TIMEOUT;
          enter(SM_FAULT);
        }
        else if ((v < VBUS_MIN_V) || (v > VBUS_MAX_V))
        {
          s_faults |= SM_F_SELFTEST |
                      ((v < VBUS_MIN_V) ? SM_F_UNDERVOLTAGE
                                        : SM_F_OVERVOLTAGE);
          enter(SM_FAULT);
        }
        else
        {
          enter(SM_IDLE);
        }
      }
      break;

    case SM_IDLE:
      BTS_AllOff();
      s_ramp_duty = 0.0f;
      if ((s_request != 0U) && (s_faults == 0UL))
      {
        /* Kalkis akimini olcebilmek icin zirveleri sifirla (3.q) */
        SM_ResetPeaks();
        BTS_Enable();
        s_settle_since = 0UL;
        enter(SM_SOFT_START);
      }
      break;

    case SM_SOFT_START:
      hz = check_hazards(now, 1U);
      if ((hz & SM_F_HARD) != 0UL)      { fault(hz & SM_F_HARD); break; }
      if ((hz & SM_F_SOFT) != 0UL)      { s_faults |= (hz & SM_F_SOFT);
                                          enter(SM_LANDING); break; }
      if (s_request == 0U)              { enter(SM_LANDING); break; }

      s_ramp_duty += SOFT_START_RATE * dt;
      if (s_ramp_duty > BTS_DUTY_MAX) { s_ramp_duty = BTS_DUTY_MAX; }
      BTS_SetDutyAll(s_ramp_duty);

      /* Hedef bandin icinde yeterince kalirsak askidayiz demektir. */
      if ((s_gap_mm > (GAP_TARGET_MM - SOFT_START_BAND_MM)) &&
          (s_gap_mm < (GAP_TARGET_MM + SOFT_START_BAND_MM)))
      {
        if (s_settle_since == 0UL) { s_settle_since = now; }
        else if ((now - s_settle_since) >= SOFT_START_SETTLE_MS)
        {
          enter(SM_LEVITATING);
        }
      }
      else { s_settle_since = 0UL; }

      if ((now - s_state_ms) > SOFT_START_TIMEOUT_MS)
      {
        s_faults |= SM_F_SOFTSTART_TMO;
        enter(SM_LANDING);
      }
      break;

    case SM_LEVITATING:
      hz = check_hazards(now, 1U);
      if ((hz & SM_F_HARD) != 0UL)      { fault(hz & SM_F_HARD); break; }
      if ((hz & SM_F_SOFT) != 0UL)      { s_faults |= (hz & SM_F_SOFT);
                                          enter(SM_LANDING); break; }
      if (s_request == 0U)              { enter(SM_LANDING); break; }

      /* Asama 4'te PID SM_SetControlDuty() ile yazacak. Kontrol katmani
         henuz yoksa duty rampanin bittigi degerde sabit kalir -- bu
         yalnizca tezgah testi icindir, gercek askilama saglamaz. */
      if (s_ctrl_fresh != 0U)
      {
        for (ch = 0U; ch < BTS_CH_COUNT; ch++)
        {
          BTS_SetDuty(ch, s_ctrl_duty[ch]);
        }
        s_ctrl_fresh = 0U;
      }
      break;

    case SM_LANDING:
      /* Kontrollu inis: duty'yi rampayla sifira indir, sonra kes.
         Asiri akim gibi sert bir hata bu sirada gelirse hemen kesilir. */
      hz = check_hazards(now, 1U);
      if ((hz & SM_F_HARD) != 0UL) { fault(hz & SM_F_HARD); break; }

      /* Her kanali kendi mevcut duty degerinden indir. LEVITATING ileride
         kanal basina PID duty yazdiginda ortak s_ramp_duty degerine donmek
         ani bir cikis sicramasi olustururdu. */
      any_live = 0U;
      for (ch = 0U; ch < BTS_CH_COUNT; ch++)
      {
        duty = BTS_GetDuty(ch) - (LANDING_RATE * dt);
        if (duty <= 0.0f) { duty = 0.0f; }
        else              { any_live = 1U; }
        BTS_SetDuty(ch, duty);
      }

      if (any_live == 0U)
      {
        s_ramp_duty = 0.0f;
        BTS_AllOff();
        /* Hata yuzunden iniyorsak IDLE'a donmek yanlis olur: operator
           mudahale edene kadar cikis kapali kalmali. */
        if (s_faults != 0UL) { enter(SM_FAULT); }
        else                 { enter(SM_IDLE);  }
        break;
      }
      if ((now - s_state_ms) > LANDING_TIMEOUT_MS)
      {
        SM_LatchShutdown(0UL);   /* rampa takildi: guvenli tarafta kal */
      }
      break;

    case SM_FAULT:
      BTS_AllOff();
      s_ramp_duty = 0.0f;
      /* Ayni hata siniflari SAFE_SHUTDOWN'a mandallanir: sensor arizasi
         veya asiri akim varken tekrar kalkis denemesine izin verilmez
         (sartname 3.p). */
      if ((s_faults & (SM_F_OVERCURRENT | SM_F_OVERVOLTAGE |
                       SM_F_SENSOR_TIMEOUT | SM_F_DRIVER_INIT |
                       SM_F_ADC_TIMEOUT)) != 0UL)
      {
        SM_LatchShutdown(0UL);
      }
      break;

    case SM_SAFE_SHUTDOWN:
    default:
      BTS_AllOff();
      break;
  }
}

/* ------------------------------------------------------------------ */

sm_state_t SM_GetState(void)  { return s_state;  }
uint32_t   SM_GetFaults(void) { return s_faults; }
float      SM_GetPeakCurrent(void) { return s_peak_i; }
float      SM_GetPeakPower(void)   { return s_peak_p; }

void SM_ResetPeaks(void)
{
  s_peak_i = 0.0f;
  s_peak_p = 0.0f;
}

const char *SM_StateName(sm_state_t s)
{
  switch (s)
  {
    case SM_INIT:          return "INIT";
    case SM_SELFTEST:      return "SELFTEST";
    case SM_IDLE:          return "IDLE";
    case SM_SOFT_START:    return "SOFT_START";
    case SM_LEVITATING:    return "LEVITATING";
    case SM_LANDING:       return "LANDING";
    case SM_FAULT:         return "FAULT";
    case SM_SAFE_SHUTDOWN: return "SAFE_SHUTDOWN";
    default:               return "?";
  }
}
