/**
  ******************************************************************************
  * @file    vl6180x.c
  * @brief   VL6180X surucusu (TOF050C modulu).
  ******************************************************************************
  */
#include "vl6180x.h"
#include "i2c_bus.h"

/* ------------------------- Register adresleri ------------------------- */
#define REG_IDENTIFICATION_MODEL_ID       0x0000U
#define REG_SYSTEM_MODE_GPIO1             0x0011U
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO  0x0014U
#define REG_SYSTEM_INTERRUPT_CLEAR        0x0015U
#define REG_SYSTEM_FRESH_OUT_OF_RESET     0x0016U
#define REG_SYSRANGE_START                0x0018U
#define REG_SYSRANGE_INTERMEAS_PERIOD     0x001BU
#define REG_SYSRANGE_MAX_CONVERGENCE_TIME 0x001CU
#define REG_SYSRANGE_VHV_RECALIBRATE      0x002EU
#define REG_SYSRANGE_VHV_REPEAT_RATE      0x0031U
#define REG_SYSALS_ANALOGUE_GAIN          0x003FU
#define REG_SYSALS_INTEGRATION_PERIOD     0x0040U
#define REG_RESULT_RANGE_STATUS           0x004DU
#define REG_RESULT_INTERRUPT_STATUS_GPIO  0x004FU
#define REG_RESULT_RANGE_VAL              0x0062U
#define REG_READOUT_AVERAGING_SAMPLE_PER  0x010AU

/* RESULT__INTERRUPT_STATUS_GPIO icindeki menzil bitleri */
#define INT_STATUS_RANGE_MASK             0x07U
#define INT_STATUS_RANGE_NEW_SAMPLE       0x04U

/* SYSRANGE__START */
#define SYSRANGE_START_SINGLE             0x01U
#define SYSRANGE_START_CONTINUOUS         0x03U

/* ------------------ AN4545 SR03 settings tuning dizisi ------------------
   Bu ozel (private) registerlar ST tarafindan zorunlu tutulur. Sensore guc
   verildiginde SYSTEM__FRESH_OUT_OF_RESET = 1 olur; bu dizi yazilmadan
   yapilan olcumler kararsiz ve hatali olur. */
typedef struct { uint16_t reg; uint8_t val; } vl_reg_t;

static const vl_reg_t k_tuning[] =
{
  {0x0207, 0x01}, {0x0208, 0x01}, {0x0096, 0x00}, {0x0097, 0xFD},
  {0x00E3, 0x00}, {0x00E4, 0x04}, {0x00E5, 0x02}, {0x00E6, 0x01},
  {0x00E7, 0x03}, {0x00F5, 0x02}, {0x00D9, 0x05}, {0x00DB, 0xCE},
  {0x00DC, 0x03}, {0x00DD, 0xF8}, {0x009F, 0x00}, {0x00A3, 0x3C},
  {0x00B7, 0x00}, {0x00BB, 0x3C}, {0x00B2, 0x09}, {0x00CA, 0x09},
  {0x0198, 0x01}, {0x01B0, 0x17}, {0x01AD, 0x00}, {0x00FF, 0x05},
  {0x0100, 0x05}, {0x0199, 0x05}, {0x01A6, 0x1B}, {0x01AC, 0x3E},
  {0x01A7, 0x1F}, {0x0030, 0x00},
};

/* Varsayilan olcum ayarlari. Levitasyon icin hiz onceliklidir:
   - MAX_CONVERGENCE_TIME dusuruldu (varsayilan 49 ms -> 10 ms)
   - INTERMEASUREMENT_PERIOD en dusuk degere cekilir (StartContinuous icinde)
   Kisa mesafede (25-40 mm) parlak hedefte yakinsama zaten hizli olur. */
static const vl_reg_t k_defaults[] =
{
  {REG_SYSTEM_MODE_GPIO1,             0x10}, /* GPIO1 = sample ready cikisi   */
  {REG_READOUT_AVERAGING_SAMPLE_PER,  0x18}, /* olculen hizli/kararli ayar     */
  {REG_SYSALS_ANALOGUE_GAIN,          0x46},
  {REG_SYSRANGE_VHV_REPEAT_RATE,      0xFF}, /* periyodik otomatik VHV kalib. */
  {REG_SYSALS_INTEGRATION_PERIOD,     0x63},
  {REG_SYSRANGE_VHV_RECALIBRATE,      0x01}, /* bir kez sicaklik kalibrasyonu */
  {REG_SYSRANGE_MAX_CONVERGENCE_TIME, 0x0A},
  {REG_SYSTEM_INTERRUPT_CONFIG_GPIO,  0x04}, /* menzil: yeni ornek hazir      */
};

/* ---------------------------------------------------------------------- */

static HAL_StatusTypeDef write_table(const vl_reg_t *t, uint32_t n)
{
  for (uint32_t i = 0; i < n; i++)
  {
    if (I2CBus_WriteReg16(VL6180X_ADDR_7B, t[i].reg, t[i].val) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

HAL_StatusTypeDef VL6180X_Init(vl6180x_t *dev, uint8_t mux_channel)
{
  uint8_t v = 0;

  if (dev == NULL) { return HAL_ERROR; }

  dev->mux_channel      = mux_channel;
  dev->present          = 0;
  dev->continuous       = 0;
  dev->last_range_mm    = 0;
  dev->last_status      = 0;
  dev->sample_count     = 0;
  dev->status_err_count = 0;
  dev->io_err_count     = 0;
  dev->last_sample_tick = 0;

  /* Guc verildikten sonra sensorun ic acilis suresi */
  HAL_Delay(5);

  if (I2CBus_ReadReg16(VL6180X_ADDR_7B, REG_IDENTIFICATION_MODEL_ID, &v, 1) != HAL_OK)
  {
    dev->io_err_count++;
    return HAL_ERROR;
  }
  if (v != VL6180X_MODEL_ID)
  {
    return HAL_ERROR;   /* yanlis cihaz veya bozuk hat */
  }
  dev->present = 1;

  /* --- Sensoru bilinen bir duruma cek ---
     VL6180X, MCU resetinden ETKILENMEZ: onceki cevrimden surekli modda
     kalmis olabilir. Cihaz olcum yaparken yapilandirma yazmalarini yok
     sayar; bu durumda init ayarlari yazdigini sanir ama sensor dinlemez ve
     ardindan gelen tek atimlik olcum de cakisip zaman asimina ugrar.
     Bu yuzden once varsa surekli modu durdurup cihazin hazir olmasini
     bekliyoruz. SYSRANGE__START bit0'in TOGGLE oldugunu unutmayin:
     durdurmak icin 0x01 yazilir, 0x03 degil. */
  if (I2CBus_ReadReg16(VL6180X_ADDR_7B, REG_SYSRANGE_START, &v, 1) == HAL_OK)
  {
    if ((v & 0x02U) != 0U)
    {
      (void)I2CBus_WriteReg16(VL6180X_ADDR_7B, REG_SYSRANGE_START, 0x01U);
    }
  }
  (void)I2CBus_WriteReg16(VL6180X_ADDR_7B, REG_SYSTEM_INTERRUPT_CLEAR, 0x07U);

  {
    uint32_t t0 = HAL_GetTick();
    uint8_t  rdy = 0;
    do
    {
      if (I2CBus_ReadReg16(VL6180X_ADDR_7B, REG_RESULT_RANGE_STATUS, &v, 1) == HAL_OK)
      {
        rdy = (uint8_t)(v & 0x01U);     /* bit0 = Device_Ready */
      }
    } while ((rdy == 0U) && ((HAL_GetTick() - t0) < 200U));

    if (rdy == 0U)
    {
      /* Cihaz 200 ms icinde hazir olmadi: yazilimla kurtarilamaz bir
         durumda (genellikle guc cevrimi gerektirir). Cagirana bildir. */
      dev->io_err_count++;
      return HAL_ERROR;
    }
  }

  /* Tazeyse zorunlu tuning dizisini yaz */
  if (I2CBus_ReadReg16(VL6180X_ADDR_7B, REG_SYSTEM_FRESH_OUT_OF_RESET, &v, 1) != HAL_OK)
  {
    dev->io_err_count++;
    return HAL_ERROR;
  }
  if (v == 0x01U)
  {
    if (write_table(k_tuning, sizeof(k_tuning) / sizeof(k_tuning[0])) != HAL_OK)
    {
      dev->io_err_count++;
      return HAL_ERROR;
    }
    if (I2CBus_WriteReg16(VL6180X_ADDR_7B, REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00) != HAL_OK)
    {
      dev->io_err_count++;
      return HAL_ERROR;
    }
  }

  if (write_table(k_defaults, sizeof(k_defaults) / sizeof(k_defaults[0])) != HAL_OK)
  {
    dev->io_err_count++;
    return HAL_ERROR;
  }

  /* Bekleyen kesme bayraklarini temizle */
  (void)I2CBus_WriteReg16(VL6180X_ADDR_7B, REG_SYSTEM_INTERRUPT_CLEAR, 0x07);

  return HAL_OK;
}

HAL_StatusTypeDef VL6180X_StartContinuous(vl6180x_t *dev, uint16_t period_ms)
{
  if (dev == NULL || !dev->present) { return HAL_ERROR; }

  /* period = (register_degeri + 1) * 10 ms  ->  0 = 10 ms = 100 Hz */
  uint16_t steps = (period_ms < 20U) ? 0U : (uint16_t)((period_ms / 10U) - 1U);
  if (steps > 254U) { steps = 254U; }

  if (I2CBus_WriteReg16(VL6180X_ADDR_7B, REG_SYSRANGE_INTERMEAS_PERIOD,
                        (uint8_t)steps) != HAL_OK)
  {
    dev->io_err_count++;
    return HAL_ERROR;
  }
  (void)I2CBus_WriteReg16(VL6180X_ADDR_7B, REG_SYSTEM_INTERRUPT_CLEAR, 0x07);

  if (I2CBus_WriteReg16(VL6180X_ADDR_7B, REG_SYSRANGE_START,
                        SYSRANGE_START_CONTINUOUS) != HAL_OK)
  {
    dev->io_err_count++;
    return HAL_ERROR;
  }

  /* Dogrula: calisan surekli modda START registeri bit1 (mode) set okunur.
     Toggle semantigi yuzunden yanlislikla durdurmus olma ihtimaline karsi
     bir kez daha denenir. */
  uint8_t rb = 0;
  if (I2CBus_ReadReg16(VL6180X_ADDR_7B, REG_SYSRANGE_START, &rb, 1) == HAL_OK)
  {
    if ((rb & 0x02U) == 0U)
    {
      (void)I2CBus_WriteReg16(VL6180X_ADDR_7B, REG_SYSRANGE_START,
                              SYSRANGE_START_CONTINUOUS);
    }
  }

  dev->continuous = 1;
  return HAL_OK;
}

HAL_StatusTypeDef VL6180X_StopContinuous(vl6180x_t *dev)
{
  if (dev == NULL) { return HAL_ERROR; }

  /* DIKKAT: SYSRANGE__START bit0 surekli modda bir TOGGLE'dir. 0x03 yazmak
     calisan modu durdurur ama DURAN modu baslatir. Bu yuzden durdurmak icin
     0x03 degil, tek atim mod secimi ile 0x01 yazilir -- bu her durumda
     surekli modu sonlandirir ve bilinen bir duruma getirir. */
  HAL_StatusTypeDef st = I2CBus_WriteReg16(VL6180X_ADDR_7B, REG_SYSRANGE_START,
                                           SYSRANGE_START_SINGLE);
  (void)I2CBus_WriteReg16(VL6180X_ADDR_7B, REG_SYSTEM_INTERRUPT_CLEAR, 0x07);
  dev->continuous = 0;
  return st;
}

int VL6180X_Poll(vl6180x_t *dev)
{
  uint8_t irq = 0, range = 0, status = 0;

  if (dev == NULL || !dev->present) { return -1; }

  if (I2CBus_ReadReg16(VL6180X_ADDR_7B, REG_RESULT_INTERRUPT_STATUS_GPIO, &irq, 1) != HAL_OK)
  {
    dev->io_err_count++;
    return -1;
  }
  if ((irq & INT_STATUS_RANGE_MASK) != INT_STATUS_RANGE_NEW_SAMPLE)
  {
    return 0;   /* henuz hazir degil */
  }

  if (I2CBus_ReadReg16(VL6180X_ADDR_7B, REG_RESULT_RANGE_VAL, &range, 1) != HAL_OK ||
      I2CBus_ReadReg16(VL6180X_ADDR_7B, REG_RESULT_RANGE_STATUS, &status, 1) != HAL_OK)
  {
    dev->io_err_count++;
    return -1;
  }

  if (I2CBus_WriteReg16(VL6180X_ADDR_7B, REG_SYSTEM_INTERRUPT_CLEAR, 0x07) != HAL_OK)
  {
    dev->io_err_count++;
    return -1;
  }

  dev->last_range_mm    = range;
  dev->last_status      = (uint8_t)(status >> 4);
  dev->last_sample_tick = HAL_GetTick();
  dev->sample_count++;
  if (dev->last_status != VL6180X_RS_NO_ERROR) { dev->status_err_count++; }

  return 1;
}

HAL_StatusTypeDef VL6180X_ReadSingle(vl6180x_t *dev, uint8_t *range_mm)
{
  uint8_t irq = 0;
  uint32_t t0;

  if (dev == NULL || !dev->present || range_mm == NULL) { return HAL_ERROR; }

  if (I2CBus_WriteReg16(VL6180X_ADDR_7B, REG_SYSRANGE_START,
                        SYSRANGE_START_SINGLE) != HAL_OK)
  {
    dev->io_err_count++;
    return HAL_ERROR;
  }

  t0 = HAL_GetTick();
  do
  {
    if (I2CBus_ReadReg16(VL6180X_ADDR_7B, REG_RESULT_INTERRUPT_STATUS_GPIO,
                         &irq, 1) != HAL_OK)
    {
      dev->io_err_count++;
      return HAL_ERROR;
    }
    if ((HAL_GetTick() - t0) > 100U) { return HAL_TIMEOUT; }
  } while ((irq & INT_STATUS_RANGE_MASK) != INT_STATUS_RANGE_NEW_SAMPLE);

  if (VL6180X_Poll(dev) != 1) { return HAL_ERROR; }

  *range_mm = dev->last_range_mm;
  return HAL_OK;
}

HAL_StatusTypeDef VL6180X_ReadReg(uint16_t reg, uint8_t *val)
{
  return I2CBus_ReadReg16(VL6180X_ADDR_7B, reg, val, 1);
}

HAL_StatusTypeDef VL6180X_WriteReg(uint16_t reg, uint8_t val)
{
  return I2CBus_WriteReg16(VL6180X_ADDR_7B, reg, val);
}

const char *VL6180X_StatusStr(uint8_t status)
{
  switch (status)
  {
    case VL6180X_RS_NO_ERROR:            return "OK";
    case VL6180X_RS_VCSEL_CONT_TEST:     return "VCSEL continuity";
    case VL6180X_RS_VCSEL_WATCHDOG_TEST: return "VCSEL wdog test";
    case VL6180X_RS_VCSEL_WATCHDOG:      return "VCSEL wdog";
    case VL6180X_RS_PLL1_LOCK:           return "PLL1 lock";
    case VL6180X_RS_PLL2_LOCK:           return "PLL2 lock";
    case VL6180X_RS_EARLY_CONV_EST:      return "Early conv est";
    case VL6180X_RS_MAX_CONV:            return "Max convergence";
    case VL6180X_RS_NO_TARGET:           return "No target";
    case VL6180X_RS_MAX_SNR:             return "Max SNR";
    case VL6180X_RS_RAW_UNDERFLOW:       return "Raw underflow";
    case VL6180X_RS_RAW_OVERFLOW:        return "Raw overflow";
    case VL6180X_RS_RANGE_UNDERFLOW:     return "Range underflow (too close)";
    case VL6180X_RS_RANGE_OVERFLOW:      return "Range overflow (too far)";
    default:                             return "Unknown";
  }
}
