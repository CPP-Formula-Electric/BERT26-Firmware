#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <EEPROM.h>
#include <math.h>

// ============================================================
// ENUMS / STRUCTS — must be first for Arduino prototype gen
// ============================================================
enum BmsMode  { MODE_UNKNOWN, MODE_DISCHARGE, MODE_CHARGE };
enum BmsState {
  STATE_INIT, STATE_WAIT_SDC, STATE_PRECHARGE,
  STATE_CLOSE_POSITIVE, STATE_CHARGE_SETTLE,
  STATE_RUNNING, STATE_CHARGE_ACTIVE, STATE_CHARGE_DONE,
  STATE_FAULT
};
enum FaultCode {
  FAULT_NONE = 0, FAULT_CELL_UV, FAULT_CELL_OV,
  FAULT_PACK_UV, FAULT_PACK_OV, FAULT_OVER_TEMP,
  FAULT_SPREAD, FAULT_PRECHARGE_TIMEOUT, FAULT_PRECHARGE_FAST,
  FAULT_PACK_VOLT_SENSE, FAULT_CHARGER_COMMS, FAULT_BQ_INIT, FAULT_BQ_COMMS,
  FAULT_CHARGE_TIMEOUT
};
struct Stats { float mn, mx, spread, avg, std_dev; };

// ============================================================
// PACK PARAMETERS — Molicel P50B 95s4p plus one monitored busbar channel
// ============================================================
static constexpr float   CELL_V_MIN         = 2.50f;   // V per cell, UV cutoff
static constexpr float   CELL_V_MAX         = 4.20f;   // V per cell, OV cutoff (both modes)
static constexpr float   CELL_TEMP_MAX_C    = 60.0f;   // °C, discharge & charge
static constexpr uint8_t CELLS_SERIES       = 95;
static constexpr uint8_t CELL_CHANNELS      = 96;      // BQ/CAN layout remains 6 × 16
static constexpr uint8_t BYPASSED_CELL_INDEX = 91;     // zero-based: pack cell 92 = IC6 Cell12
static constexpr float   CELL_V_CHARGE_TARGET = 400.0f / 96.0f; // preserve the original 4.1667 V/cell target
static constexpr float   CELL_V_PACK_OV       = 402.0f / 96.0f; // preserve the original 4.1875 V/cell pack fault point
static constexpr float   PACK_V_MIN         = CELLS_SERIES * CELL_V_MIN;           // 237.5 V
static constexpr float   PACK_V_CHARGE_MAX  = CELLS_SERIES * CELL_V_CHARGE_TARGET; // 395.833 V
static constexpr float   PACK_V_CHARGE_OV   = CELLS_SERIES * CELL_V_PACK_OV;       // 397.813 V; below 95 × 4.20 V
static constexpr uint8_t CELLS_PARALLEL     = 4;
static constexpr float   PACK_CAPACITY_AH   = 20.0f;   // 4 × 5 Ah
// Spread limit = DCIR × I_cell_max = 0.014Ω × (80kW / 342V / 4P) = ~0.82V
static constexpr float   CELL_DCIR_OHM      = 0.014f;
static constexpr float   PACK_POWER_MAX_W   = 80000.0f;
static constexpr float   PACK_V_NOM         = CELLS_SERIES * 3.6f; // 342.0 V

// Charger (HK-MF OBC, Elcon protocol)
static constexpr float   CHARGER_V_REQ      = PACK_V_CHARGE_MAX; // 395.8 V requested (0.1 V protocol resolution)
static constexpr float   CHARGER_I_REQ      = 10.0f;   // A (max for 312V platform)
static constexpr float   CHARGER_DONE_A     = 0.5f;    // A current-taper termination
static constexpr uint32_t CV_TIMEOUT_MS     = 5400000; // 90 min — max CV taper time; safety cutoff if current never falls below CHARGER_DONE_A

// Timing (ms)
static constexpr uint32_t INIT_DELAY_MS        = 2000;
static constexpr uint32_t SDC_SETTLE_MS        = 1000;
static constexpr uint32_t PRECHARGE_TIMEOUT_MS = 10000;  // RC=528ms, t90=1216ms, EV.5.6.2 max(2x1216,1216+500)=2432ms
static constexpr uint32_t PRECHARGE_MIN_MS     = 200;  // discharge only
static constexpr uint32_t PRECHARGE_POLL_MS    = 2;
static constexpr uint32_t IMD_POLL_MS          = 100;
static constexpr uint32_t CLOSE_SETTLE_MS      = 100;
static constexpr uint32_t CHARGE_SETTLE_MS     = 2000;
static constexpr uint32_t TEMP_INTERVAL_MS     = 10000;
static constexpr uint32_t FIRST_8_OVERTEMP_DELAY_MS = 30000;
static constexpr uint32_t VCU_BROADCAST_MS     = 500;
static constexpr uint32_t CHARGER_MSG_MS       = 1000;
static constexpr uint32_t CHARGER_TIMEOUT_MS   = 5000;
static constexpr uint32_t SOC_SAVE_MS          = 30000;
static constexpr uint32_t RUNNING_VOLT_MS      = 200;
static constexpr uint32_t CELL_DUMP_GAP_MS     = 5;
static constexpr uint32_t IMD_DISAGREE_ABS     = 100;   // V absolute threshold
static constexpr float    IMD_DISAGREE_PCT     = 0.40f;

static constexpr int EEPROM_SOC_ADDR      = 0;  // float, 4 bytes
static constexpr int EEPROM_CAL_GAIN_ADDR = 4;  // float, 4 bytes
static constexpr int EEPROM_CAL_OFFS_ADDR = 8;  // float, 4 bytes

// ============================================================
// CAN BUSES
// Charger: CAN1, pins 22(TX)/23(RX), 250 kbps, extended 29-bit
// Vehicle/IMD: CAN3, pins 30(TX)/31(RX), 500 kbps, standard 11-bit
// ============================================================
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CanCharger;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CanVehicle;

// Vehicle CAN message IDs (11-bit)
static constexpr uint32_t ID_BMS_STATUS    = 0x310;
static constexpr uint32_t ID_BMS_TEMP      = 0x311;
static constexpr uint32_t ID_CELL_REQUEST  = 0x312;
static constexpr uint32_t ID_CELL_DATA     = 0x313;
static constexpr uint32_t ID_SET_SOC       = 0x314; // bytes 0-1: SOC uint16 0.1%/bit
static constexpr uint32_t ID_CURRENT_CAL   = 0x315; // byte 0: pt(1/2), bytes 1-2: actual A (int16, 0.1A/bit)
// IMD IDs (on vehicle CAN)
static constexpr uint32_t ID_IMD_CYCLIC    = 0x37;
static constexpr uint32_t ID_IMD_RESPONSE  = 0x23;
static constexpr uint32_t ID_IMD_REQUEST   = 0x22;
static constexpr uint8_t  IDX_HV_VOLTAGE   = 0x5E;
// Charger IDs (29-bit extended, Elcon/J1939)
static constexpr uint32_t ID_BMS_TO_CHARGER  = 0x1806E5F4; // BMS → Charger
static constexpr uint32_t ID_CHARGER_BCAST   = 0x18FF50E5; // Charger → BMS

// ============================================================
// PINS
// ============================================================
#define PIN_CHAN_A      14  // A0 — battery-side iso amp
#define PIN_CURRENT     16  // A2 — LEM DHAB S/124 Ch2 ±500A
#define PIN_NEG_SIGNAL  20  // SDC: rising=close, falling=shutdown
#define PIN_PRECHARGE   24  // precharge relay low-side
#define PIN_POS_AIR     25  // positive AIR low-side
#define PIN_BMS_STATUS  41  // HIGH=nominal, LOW=hardware fault

// ============================================================
// BQ79616 CONFIG (verbatim from bq79616_parametricFinal_LimitedPolling.ino)
// ============================================================
static const uint8_t  NUM_STACK_DEVS = 6;
static const uint8_t  NUM_DEV        = NUM_STACK_DEVS + 1;
static const uint8_t  ID_BASE        = 0;
static const uint8_t  ID_TOP         = NUM_STACK_DEVS;
static const uint16_t PACK_CELLS     = (uint16_t)NUM_STACK_DEVS * 16u; // = 96
static_assert(PACK_CELLS == CELL_CHANNELS, "BQ channel count must match the CAN cell layout");
static_assert(BYPASSED_CELL_INDEX < PACK_CELLS, "Bypassed cell index is outside the BQ channel array");
static constexpr uint8_t BYPASSED_BQ_DEVICE = (BYPASSED_CELL_INDEX / 16u) + 1u; // ID 6
static constexpr uint8_t BYPASSED_BQ_CELL   = (BYPASSED_CELL_INDEX % 16u) + 1u; // Cell 12
static_assert(BYPASSED_BQ_CELL >= 9u, "This configuration expects BBVC_POSN1 (cells 9-16)");
static constexpr uint8_t BYPASSED_BBVC_MASK = 1u << (BYPASSED_BQ_CELL - 9u);    // bit 3 = 0x08

static const uint32_t UART_BAUD = 1000000;
HardwareSerial& BMS_UART = Serial1;

#define ACTIVE_CELL     0x0003
#define BBVC_POSN1      0x0005
#define ADC_CONF1       0x0007
#define GPIO_CONF1      0x000E
#define GPIO_CONF2      0x000F
#define GPIO_CONF3      0x0010
#define GPIO_CONF4      0x0011
#define DIR0_ADDR       0x0306
#define COMM_CTRL       0x0308
#define CONTROL1        0x0309
#define CONTROL2        0x030A
#define ADC_CTRL1       0x030D
#define COMM_TIMEOUT_CONF 0x0019
#define OTP_ECC_DATAIN1 0x0343
#define OTP_ECC_DATAIN8 0x034A
#define PARTID          0x0500
#define VCELL16_HI      0x0568
#define TSREF_HI        0x058C
#define GPIO1_HI        0x058E

static inline uint16_t gpio_reg(uint8_t gpio) {
  return (uint16_t)(GPIO1_HI + 2u * (uint16_t)(gpio - 1u));
}
#define GM_OFF 0b000
#define GM_ADC 0b010
#define GM_HI  0b100
#define GM_LO  0b101

enum : uint8_t { FRMWRT_SGL_W=0x90, FRMWRT_STK_W=0xB0, FRMWRT_ALL_W=0xD0 };
enum : uint8_t { FRMWRT_SGL_R=0x00, FRMWRT_STK_R=0x20, FRMWRT_ALL_R=0x40 };

static const float NTC_R25  = 10000.0f;
static const float NTC_BETA = 3380.0f;
static const float T0_K     = 298.15f;
static const float RPULL    = 10000.0f;
static const float CORR_A   = -5.7e-6f;
static const float CORR_B   =  0.846f;
static const float CORR_C   =  180.0f;

// ============================================================
// RUNTIME STATE
// ============================================================
BmsMode   g_mode       = MODE_UNKNOWN;
BmsState  g_state      = STATE_INIT;
FaultCode g_fault_code = FAULT_NONE;
bool      g_contactors_closed = false;

volatile bool g_charger_can_seen = false; // set by canSniff2 during INIT
volatile bool g_vehicle_can_seen = false; // set by canSniff3 during INIT (non-IMD frames)

volatile bool g_sdc_state   = false; // true = SDC high (neg contactor closed)
volatile bool g_sdc_changed = false; // set on any edge, cleared by state handlers

volatile bool  g_imd_valid   = false;
volatile float g_imd_voltage = 0.0f;

volatile float   g_charger_out_v    = 0.0f;
volatile float   g_charger_out_a    = 0.0f;
volatile uint8_t g_charger_status   = 0;
volatile uint32_t g_charger_last_rx = 0;

static float g_volts[PACK_CELLS];
static float g_temps[PACK_CELLS];
static float g_bypass_drop_v = NAN; // raw VC12 measurement retained for diagnostics
static Stats g_vstats, g_tstats;
static float g_spread_limit = 0.81f; // computed at init

static float g_soc       = 50.0f; // %
static float g_current_A = 0.0f;  // A, positive=discharge, negative=charge/regen

// Current sensor calibration (two-point linear fit)
static float g_cal_gain        = 1.0f;
static float g_cal_offset      = 0.0f;
static float g_cal_pt1_raw     = 0.0f, g_cal_pt1_actual = 0.0f;
static float g_cal_pt2_raw     = 0.0f, g_cal_pt2_actual = 0.0f;
static bool  g_cal_pt1_set     = false, g_cal_pt2_set   = false;

static uint8_t g_dump_type    = 0; // 0=idle, 1=voltages, 2=temps
static uint8_t g_dump_frame   = 0;
static bool    g_dump_on_can2 = false;

// Timers
elapsedMillis initTimer;
elapsedMillis sdcSettleTimer;
elapsedMillis prechargeTimer;
elapsedMillis prechargeVoltTimer;
elapsedMillis imdPollTimer;
elapsedMillis closeSettleTimer;
elapsedMillis chargeSettleTimer;
elapsedMillis tempTimer;
elapsedMillis vcuBroadcastTimer;
elapsedMillis chargerMsgTimer;
elapsedMillis chargerTimeoutTimer;
elapsedMillis socSaveTimer;
elapsedMillis runningVoltTimer;
elapsedMillis cellDumpTimer;
elapsedMillis postPrechargeTimer;
elapsedMillis chargeCvTimer;          // CV-phase taper watchdog
bool          g_cv_phase = false;     // latched true once charger reaches CV plateau

// ============================================================
// ISR
// ============================================================
void pin20ISR() {
  g_sdc_state   = (bool)digitalRead(PIN_NEG_SIGNAL);
  g_sdc_changed = true;
}

// ============================================================
// BQ79616 FUNCTIONS — verbatim from bq79616_parametricFinal_LimitedPolling.ino
// ============================================================
static uint16_t crc16(const uint8_t* d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

static void bq_tx(uint8_t wrType, uint8_t addr, uint16_t reg, const uint8_t* data, uint8_t n) {
  uint8_t buf[16]; uint8_t k = 0;
  buf[k++] = 0x80 | (wrType + (n - 1));
  if (wrType == FRMWRT_SGL_W) buf[k++] = addr;
  buf[k++] = reg >> 8; buf[k++] = reg & 0xFF;
  for (uint8_t i = 0; i < n; i++) buf[k++] = data[i];
  uint16_t c = crc16(buf, k);
  buf[k++] = c & 0xFF; buf[k++] = c >> 8;
  BMS_UART.write(buf, k); BMS_UART.flush();
}

static void bq_rx_req(uint8_t rdType, uint8_t addr, uint16_t reg, uint8_t nBytes) {
  uint8_t buf[8]; uint8_t k = 0;
  buf[k++] = 0x80 | rdType;
  if (rdType == FRMWRT_SGL_R) buf[k++] = addr;
  buf[k++] = reg >> 8; buf[k++] = reg & 0xFF;
  buf[k++] = nBytes - 1;
  uint16_t c = crc16(buf, k);
  buf[k++] = c & 0xFF; buf[k++] = c >> 8;
  BMS_UART.write(buf, k); BMS_UART.flush();
}

static bool bq_rx_wait(uint8_t* dst, size_t n, uint32_t to_ms) {
  uint32_t t0 = millis(); size_t got = 0;
  while (got < n) {
    while (BMS_UART.available() && got < n) dst[got++] = BMS_UART.read();
    if (got == n) return true;
    if (millis() - t0 >= to_ms) return false;
  }
  return true;
}

static bool bq_reg_read8(uint8_t id, uint16_t reg, uint8_t& val) {
  uint8_t buf[7] = {};
  bq_rx_req(FRMWRT_SGL_R, id, reg, 1);
  if (!bq_rx_wait(buf, 7, 30)) return false;
  val = buf[4]; return true;
}

static bool bq_reg_read16(uint8_t id, uint16_t reg, uint16_t& val) {
  uint8_t buf[8] = {};
  bq_rx_req(FRMWRT_SGL_R, id, reg, 2);
  if (!bq_rx_wait(buf, 8, 30)) return false;
  val = ((uint16_t)buf[4] << 8) | buf[5]; return true;
}

static void bq_write_all(uint16_t reg, uint8_t val) { bq_tx(FRMWRT_ALL_W, 0, reg, &val, 1); }
static void bq_write_sgl(uint8_t id, uint16_t reg, uint8_t val) { bq_tx(FRMWRT_SGL_W, id, reg, &val, 1); }

static void bq_ping(uint32_t us) {
  BMS_UART.end();
  pinMode(1, OUTPUT); digitalWrite(1, LOW);
  delayMicroseconds(us);
  digitalWrite(1, HIGH);
  BMS_UART.begin(UART_BAUD, SERIAL_8N1);
  delayMicroseconds(200);
}

static void bq_hwrst() { bq_ping(36000); delay(100); }
static void bq_wake()  { bq_ping(2500); }
static void bq_sta()   { bq_ping(250); }

static void bq_auto_address() {
  for (uint16_t r = OTP_ECC_DATAIN1; r <= OTP_ECC_DATAIN8; r++) {
    uint8_t z = 0; bq_tx(FRMWRT_STK_W, 0, r, &z, 1);
  }
  bq_write_all(CONTROL1, 0x01);
  for (uint8_t a = 0; a < NUM_DEV; a++) bq_write_all(DIR0_ADDR, a);
  bq_write_all(COMM_CTRL, 0x02);
  bq_write_sgl(ID_TOP,  COMM_CTRL, 0x03);
  bq_write_sgl(ID_BASE, COMM_CTRL, 0x00);
}

static bool bq_configure_busbar_channel() {
  // Cell 92 is physically bypassed, but it remains the twelfth measurement
  // channel on IC6. ACTIVE_CELL must stay at 16 because it only selects a
  // contiguous range starting at Cell1; BBVC_POSN1 identifies the interior gap.
  bq_write_sgl(BYPASSED_BQ_DEVICE, BBVC_POSN1, BYPASSED_BBVC_MASK);
  delay(2);

  uint8_t rb = 0;
  if (!bq_reg_read8(BYPASSED_BQ_DEVICE, BBVC_POSN1, rb)) {
    Serial.println("[BQ] busbar configuration readback failed");
    return false;
  }
  if (rb != BYPASSED_BBVC_MASK) {
    Serial.printf("[BQ] busbar configuration mismatch: id%u reg=0x%02X expected=0x%02X\n",
                  BYPASSED_BQ_DEVICE, rb, BYPASSED_BBVC_MASK);
    return false;
  }

  Serial.printf("[BQ] busbar configured: pack index %u = id%u Cell%u (BBVC_POSN1=0x%02X)\n",
                BYPASSED_CELL_INDEX, BYPASSED_BQ_DEVICE,
                BYPASSED_BQ_CELL, BYPASSED_BBVC_MASK);
  return true;
}

static void bq_adc_init() {
  bq_write_all(ACTIVE_CELL, 0x0A); // all 16 positions active; BBVC marks the interior busbar
  bq_write_all(ADC_CONF1,   0x02);
  bq_write_all(ADC_CTRL1,   0x0E);
  delay(50);
}

static void bq_enable_tsref(uint8_t id) {
  uint8_t v = 0; bq_reg_read8(id, CONTROL2, v);
  bq_write_sgl(id, CONTROL2, v | 0x01); delay(3);
}

static bool bq_read_voltages(uint8_t id, float dst[16]) {
  uint8_t buf[38] = {};
  bq_rx_req(FRMWRT_SGL_R, id, VCELL16_HI, 32);
  if (!bq_rx_wait(buf, 38, 60)) return false;
  for (int i = 0; i < 16; i++) {
    uint16_t raw = ((uint16_t)buf[4 + i * 2] << 8) | buf[4 + i * 2 + 1];
    dst[15 - i] = (int16_t)raw * 190.73e-6f;
  }
  return true;
}

static float bq_ntc_to_c(float Rmeas) {
  float R = CORR_A * Rmeas * Rmeas + CORR_B * Rmeas + CORR_C;
  if (R <= 0.0f) return NAN;
  float invT = (1.0f / T0_K) + logf(R / NTC_R25) / NTC_BETA;
  if (invT <= 0.0f) return NAN;
  return 1.0f / invT - 273.15f;
}

static void bq_mux_set(uint8_t id, uint8_t bank, uint8_t chan) {
  chan &= 0x7;
  uint8_t a0 = (chan & 1) ? GM_HI : GM_LO;
  uint8_t a1 = (chan & 2) ? GM_HI : GM_LO;
  uint8_t a2 = (chan & 4) ? GM_HI : GM_LO;
  uint8_t en0 = (bank == 0) ? GM_HI : GM_OFF;
  uint8_t en1 = (bank == 1) ? GM_HI : GM_OFF;
  bq_write_sgl(id, GPIO_CONF1, (uint8_t)((a1 << 3) | a0));
  bq_write_sgl(id, GPIO_CONF2, (uint8_t)((en0 << 3) | a2));
  bq_write_sgl(id, GPIO_CONF3, (uint8_t)((en1 << 3) | GM_ADC));
  bq_write_sgl(id, GPIO_CONF4, (uint8_t)((GM_OFF << 3) | GM_ADC));
  bq_write_sgl(id, ADC_CTRL1, 0x0E);
}

static bool bq_mux_read(uint8_t id, uint8_t gpioNum, float& tC) {
  uint16_t tsref_raw = 0, gpio_raw = 0;
  if (!bq_reg_read16(id, TSREF_HI,         tsref_raw)) return false;
  if (!bq_reg_read16(id, gpio_reg(gpioNum), gpio_raw)) return false;
  int16_t tsref = (int16_t)tsref_raw, code = (int16_t)gpio_raw;
  if (tsref <= 0 || tsref == (int16_t)0x8000) return false;
  if (code  <= 0 || code  == (int16_t)0x8000) return false;
  float ratio = (float)code / (float)tsref;
  if (ratio <= 0.001f || ratio >= 0.999f) return false;
  float Rmeas = RPULL * ratio / (1.0f - ratio);
  tC = bq_ntc_to_c(Rmeas);
  return !isnan(tC);
}

static bool bq_read_temps(uint8_t id, float dst[16]) {
  const uint32_t SETTLE_US = 35000;
  bool ok = true;
  for (uint8_t cell = 0; cell < 16; cell++) {
    uint8_t bank = (cell < 8) ? 0 : 1;
    uint8_t chan = cell & 0x7;
    uint8_t gpioOut = (bank == 0) ? 5 : 7;
    bq_mux_set(id, bank, chan);
    if (cell == 0 || cell == 8) delayMicroseconds(SETTLE_US);
    delayMicroseconds(SETTLE_US);
    float tC = NAN;
    if (!bq_mux_read(id, gpioOut, tC)) {
      Serial.printf("[BQ] temp read failed: id%d cell%d\n", id, cell + 1);
      ok = false;
    }
    dst[cell] = tC;
  }
  return ok;
}

static void maskBypassedCellVoltage() {
  // Preserve the busbar measurement for diagnostics, then remove it from all
  // cell statistics, UV/OV checks, pack sums, SOC initialization, and CAN data.
  g_bypass_drop_v = g_volts[BYPASSED_CELL_INDEX];
  g_volts[BYPASSED_CELL_INDEX] = NAN;
}

static uint16_t calc_stats(const float* arr, uint16_t len, Stats& s) {
  s = {NAN, NAN, NAN, NAN, NAN};
  uint16_t n = 0; double sum = 0;
  for (uint16_t i = 0; i < len; i++) {
    if (isnan(arr[i])) continue;
    if (n == 0 || arr[i] < s.mn) s.mn = arr[i];
    if (n == 0 || arr[i] > s.mx) s.mx = arr[i];
    sum += arr[i]; n++;
  }
  if (!n) return 0;
  s.avg = (float)(sum / n);
  s.spread = s.mx - s.mn;
  double ss = 0;
  for (uint16_t i = 0; i < len; i++) {
    if (isnan(arr[i])) continue;
    double d = arr[i] - s.avg; ss += d * d;
  }
  s.std_dev = (float)sqrt(ss / n);
  return n;
}

static bool bq_init() {
  BMS_UART.begin(UART_BAUD, SERIAL_8N1);
  delay(500);
  bq_hwrst(); delay(10);
  bq_wake(); delay(15);
  bq_wake(); delay(15);
  bq_write_sgl(ID_BASE, CONTROL1, 0x20); delay(100); // BQ79600 SEND_WAKE to BQ79616 stack
  bq_sta();  delay(5);
  bq_auto_address(); delay(5);

  for (uint8_t s = 0; s < NUM_STACK_DEVS; s++) {
    uint8_t id = s + 1; uint16_t pid = 0;
    if (!bq_reg_read16(id, PARTID, pid)) {
      Serial.printf("[BQ] id%d no response\n", id);
      return false;
    }
    Serial.printf("[BQ] id%d PARTID=0x%04X\n", id, pid);
  }
  for (uint8_t s = 0; s < NUM_STACK_DEVS; s++) bq_enable_tsref(s + 1);
  if (!bq_configure_busbar_channel()) return false;
  bq_adc_init();

  // Auto-shutdown after 1 min of no valid comms
  // CTL_ACT=1 (SHUTDOWN), CTL_TIME=100b (1 min), CTS_TIME=000 disabled → 0x0C
  for (uint8_t s = 0; s < NUM_STACK_DEVS; s++) {
    uint8_t id = s + 1;
    bq_write_sgl(id, COMM_TIMEOUT_CONF, 0x0C);
    delay(2);
    uint8_t rb = 0;
    if (bq_reg_read8(id, COMM_TIMEOUT_CONF, rb)) {
      Serial.printf("[BQ] id%d COMM_TIMEOUT_CONF=0x%02X %s\n",
                    id, rb, rb == 0x0C ? "OK" : "MISMATCH");
    } else {
      Serial.printf("[BQ] id%d COMM_TIMEOUT_CONF read FAIL\n", id);
    }
  }

  for (uint16_t i = 0; i < PACK_CELLS; i++) g_temps[i] = NAN;
  Serial.println("[BQ] init done");
  return true;
}

// ============================================================
// CURRENT SENSOR — LEM DHAB S/124, Channel 2 ±500A
//   Uout = S*IP + UO  (S=4mV/A, UO=2.5V at UC=5V)
//   Voltage divider gain = 0.6875 before Teensy pin 16
// ============================================================
static float readPackCurrentRaw_A() {
  float v_adc    = analogRead(PIN_CURRENT) * (3.3f / 4095.0f);
  float v_sensor = v_adc / 0.6875f;
  return (v_sensor - 2.5f) / 0.004f; // S = 4 mV/A
}

static float readPackCurrent_A() {
  return g_cal_gain * readPackCurrentRaw_A() + g_cal_offset;
}

// ============================================================
// SOC
// ============================================================
static float ocvToSoc(float cell_v) {
  static const float v[]   = {2.50f,3.00f,3.25f,3.40f,3.50f,3.60f,3.70f,3.80f,3.90f,4.00f,4.10f,4.20f};
  static const float soc[] = { 0.0f, 5.0f,10.0f,20.0f,30.0f,40.0f,50.0f,60.0f,70.0f,80.0f,90.0f,100.0f};
  static const int N = 12;
  if (cell_v <= v[0]) return 0.0f;
  if (cell_v >= v[N-1]) return 100.0f;
  for (int i = 0; i < N-1; i++) {
    if (cell_v <= v[i+1]) {
      float t = (cell_v - v[i]) / (v[i+1] - v[i]);
      return soc[i] + t * (soc[i+1] - soc[i]);
    }
  }
  return 100.0f;
}

static void initSOC() {
  float stored = 0.0f;
  EEPROM.get(EEPROM_SOC_ADDR, stored);
  if (!isnan(stored) && stored >= 0.0f && stored <= 100.0f) {
    g_soc = stored;
    Serial.printf("[SOC] Loaded from EEPROM: %.1f%%\n", g_soc);
  } else {
    // Fall back to OCV estimate from average cell voltage
    uint16_t n = calc_stats(g_volts, PACK_CELLS, g_vstats);
    g_soc = (n > 0) ? ocvToSoc(g_vstats.avg) : 50.0f;
    Serial.printf("[SOC] OCV estimate: %.1f%%\n", g_soc);
  }
}

static void saveSOC() {
  EEPROM.put(EEPROM_SOC_ADDR, g_soc);
}

static void saveCalibration() {
  EEPROM.put(EEPROM_CAL_GAIN_ADDR, g_cal_gain);
  EEPROM.put(EEPROM_CAL_OFFS_ADDR, g_cal_offset);
}

static void loadCalibration() {
  float g = 0.0f, o = 0.0f;
  EEPROM.get(EEPROM_CAL_GAIN_ADDR, g);
  EEPROM.get(EEPROM_CAL_OFFS_ADDR, o);
  if (!isnan(g) && !isnan(o) && g > 0.01f && g < 100.0f) {
    g_cal_gain = g; g_cal_offset = o;
    Serial.printf("[CAL] Loaded: gain=%.4f  offset=%.4fA\n", g_cal_gain, g_cal_offset);
  } else {
    Serial.println("[CAL] No valid calibration in EEPROM, using defaults (gain=1 offset=0)");
  }
}

static void recordCalPoint(uint8_t pt, float actual_A) {
  float raw = readPackCurrentRaw_A();
  if (pt == 1) {
    g_cal_pt1_raw = raw; g_cal_pt1_actual = actual_A; g_cal_pt1_set = true;
    Serial.printf("[CAL] Point 1: raw=%.3fA  actual=%.3fA\n", raw, actual_A);
  } else {
    g_cal_pt2_raw = raw; g_cal_pt2_actual = actual_A; g_cal_pt2_set = true;
    Serial.printf("[CAL] Point 2: raw=%.3fA  actual=%.3fA\n", raw, actual_A);
  }
  if (g_cal_pt1_set && g_cal_pt2_set) {
    float denom = g_cal_pt2_raw - g_cal_pt1_raw;
    if (fabsf(denom) < 0.5f) {
      Serial.println("[CAL] ERROR: cal points too close — apply different currents and retry");
      return;
    }
    g_cal_gain   = (g_cal_pt2_actual - g_cal_pt1_actual) / denom;
    g_cal_offset = g_cal_pt1_actual - g_cal_gain * g_cal_pt1_raw;
    saveCalibration();
    Serial.printf("[CAL] Applied: gain=%.4f  offset=%.4fA\n", g_cal_gain, g_cal_offset);
  }
}

static void setSOC(float soc) {
  g_soc = fmaxf(0.0f, fminf(100.0f, soc));
  saveSOC();
  Serial.printf("[SOC] Manually set to %.1f%%\n", g_soc);
}

static void updateSOC(float dt_s) {
  // Positive current = discharge (SOC drops), negative = charge/regen (SOC rises)
  g_soc -= (g_current_A * dt_s) / (3600.0f * PACK_CAPACITY_AH) * 100.0f;
  g_soc = fmaxf(0.0f, fminf(100.0f, g_soc));
}

// ============================================================
// GENERAL HELPERS
// ============================================================
static inline uint16_t u16le(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

static float decodeIMDVoltage(uint16_t raw) {
  return ((float)raw - 32128.0f) * 0.05f;
}

static float adcToVolts(int adc) {
  // Chan A iso amp: V = (ADC - 1.83) / 5.2957
  return ((float)adc - 1.83f) / 5.2957f;
}

static float readVoltage_A() {
  long sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(PIN_CHAN_A);
  return adcToVolts((int)(sum / 16));
}

static void sendIMDGet(uint8_t index) {
  CAN_message_t tx{};
  tx.id = ID_IMD_REQUEST; tx.len = 8; tx.buf[0] = index;
  for (int i = 1; i < 8; i++) tx.buf[i] = 0xFF;
  CanVehicle.write(tx);
}

// ============================================================
// FAULT HANDLING
// ============================================================
static FaultCode checkCellFaults() {
  static bool first_8_overtemp = false;
  static uint32_t first_8_overtemp_start_ms = 0;
  float pack_sum = 0.0f;
  for (uint16_t i = 0; i < PACK_CELLS; i++) {
    if (isnan(g_volts[i])) continue;
    if (g_volts[i] < CELL_V_MIN) return FAULT_CELL_UV;
    if (g_volts[i] > CELL_V_MAX) return FAULT_CELL_OV;
    pack_sum += g_volts[i];
  }
  if (pack_sum < PACK_V_MIN) return FAULT_PACK_UV;
  if (g_state == STATE_CHARGE_ACTIVE && pack_sum > PACK_V_CHARGE_OV) return FAULT_PACK_OV;
  if (!isnan(g_vstats.spread) && g_vstats.spread > g_spread_limit) return FAULT_SPREAD;
  bool first_8_overtemp_now = false;
  for (uint16_t i = 0; i < PACK_CELLS; i++) {
    if (isnan(g_temps[i]) || g_temps[i] < CELL_TEMP_MAX_C) continue;
    if (i < 8) first_8_overtemp_now = true;
    else return FAULT_OVER_TEMP;
  }
  if (first_8_overtemp_now) {
    if (!first_8_overtemp) {
      first_8_overtemp = true;
      first_8_overtemp_start_ms = millis();
    } else if ((uint32_t)(millis() - first_8_overtemp_start_ms) >= FIRST_8_OVERTEMP_DELAY_MS) {
      return FAULT_OVER_TEMP;
    }
  } else {
    first_8_overtemp = false;
  }
  return FAULT_NONE;
}

static void enterFault(FaultCode code) {
  digitalWrite(PIN_PRECHARGE,  LOW);
  digitalWrite(PIN_POS_AIR,    LOW);
  digitalWrite(PIN_BMS_STATUS, LOW);  // open SDC
  g_contactors_closed = false;
  g_fault_code = code;
  g_state = STATE_FAULT;

  // Stop charger if in charge mode
  if (g_mode == MODE_CHARGE) {
    CAN_message_t tx{};
    tx.id = ID_BMS_TO_CHARGER; tx.flags.extended = 1; tx.len = 8;
    uint16_t vraw = (uint16_t)(CHARGER_V_REQ * 10.0f);
    uint16_t iraw = (uint16_t)(CHARGER_I_REQ * 10.0f);
    tx.buf[0] = vraw >> 8; tx.buf[1] = vraw & 0xFF;
    tx.buf[2] = iraw >> 8; tx.buf[3] = iraw & 0xFF;
    tx.buf[4] = 0x01;  // control = stop
    CanCharger.write(tx);
  }

  saveSOC();

  const char* names[] = {"none","cell_UV","cell_OV","pack_UV","pack_OV",
                         "over_temp","spread","precharge_timeout",
                         "precharge_fast","pack_volt_sense","charger_comms","BQ_init","BQ_comms",
                         "charge_timeout"};
  Serial.printf("[FAULT] %s — power cycle to reset\n", names[(int)code]);
}

static void handleSDCShutdown() {
  // SDC fell LOW — clean shutdown, NOT a fault. BQ auto-shuts via COMM_TIMEOUT_CONF.
  digitalWrite(PIN_PRECHARGE, LOW);
  digitalWrite(PIN_POS_AIR,   LOW);
  g_contactors_closed = false;
  saveSOC();
  g_state = STATE_WAIT_SDC;
  Serial.println("[BMS] SDC shutdown — waiting for next close");
}

static void printCellData() {
  Serial.printf("[CELLS] SOC=%.1f%%  PackCurrent=%.1fA\n", g_soc, g_current_A);
  Serial.printf("[CELLS] Volt  min=%.3fV  max=%.3fV  avg=%.3fV  spread=%.3fV\n",
                g_vstats.mn, g_vstats.mx, g_vstats.avg, g_vstats.spread);
  Serial.printf("[CELLS] Busbar index %u (IC%u Cell%u) drop=%.4fV\n",
                BYPASSED_CELL_INDEX, BYPASSED_BQ_DEVICE,
                BYPASSED_BQ_CELL, g_bypass_drop_v);
  Serial.printf("[CELLS] Temp  min=%.1fC  max=%.1fC  avg=%.1fC\n",
                g_tstats.mn, g_tstats.mx, g_tstats.avg);
  for (uint8_t s = 0; s < NUM_STACK_DEVS; s++) {
    Serial.printf("[CELLS] IC%u V:", s + 1);
    for (uint8_t c = 0; c < 16; c++)
      Serial.printf(" %.3f", g_volts[s * 16 + c]);
    Serial.println();
    Serial.printf("[CELLS] IC%u T:", s + 1);
    for (uint8_t c = 0; c < 16; c++) {
      if (isnan(g_temps[s * 16 + c])) Serial.print(" ---");
      else Serial.printf(" %.1f", g_temps[s * 16 + c]);
    }
    Serial.println();
  }
}

// Matches parametric file: voltages every call, temps every TEMP_INTERVAL_MS
static void tickMonitor() {
  bool all_v_ok = true;
  for (uint8_t s = 0; s < NUM_STACK_DEVS; s++) {
    if (!bq_read_voltages(s + 1, g_volts + s * 16)) {
      Serial.printf("[BQ] voltage read failed: id%d\n", s + 1);
      all_v_ok = false;
    }
  }
  if (!all_v_ok) { enterFault(FAULT_BQ_COMMS); return; }
  maskBypassedCellVoltage();
  uint16_t valid_v = calc_stats(g_volts, PACK_CELLS, g_vstats);
  if (valid_v != CELLS_SERIES) {
    Serial.printf("[BQ] expected %u valid cell voltages, got %u\n", CELLS_SERIES, valid_v);
    enterFault(FAULT_BQ_COMMS);
    return;
  }

  if (tempTimer >= TEMP_INTERVAL_MS) {
    tempTimer = 0;
    bool all_t_ok = true;
    // Module 1 was the last device touched by the voltage read burst above.
    // Its ADC needs extra time to leave cell-voltage mode before the GPIO/NTC
    // mux reads start — otherwise the first cell reads ~45°C from residual charge.
    delay(100);
    for (uint8_t s = 0; s < NUM_STACK_DEVS; s++)
      if (!bq_read_temps(s + 1, g_temps + s * 16)) all_t_ok = false;
    if (!all_t_ok) { enterFault(FAULT_BQ_COMMS); return; }
    calc_stats(g_temps, PACK_CELLS, g_tstats);
    bq_adc_init();
    printCellData();
  }
}

// ============================================================
// CAN TRANSMIT HELPERS
// ============================================================
static void sendChargerMsg(bool stop) {
  CAN_message_t tx{};
  tx.id = ID_BMS_TO_CHARGER; tx.flags.extended = 1; tx.len = 8;
  uint16_t vraw = (uint16_t)(CHARGER_V_REQ * 10.0f);  // 3958 = 395.8 V
  uint16_t iraw = stop ? 0u : (uint16_t)(CHARGER_I_REQ * 10.0f); // 100 = 10.0A
  tx.buf[0] = vraw >> 8; tx.buf[1] = vraw & 0xFF;
  tx.buf[2] = iraw >> 8; tx.buf[3] = iraw & 0xFF;
  tx.buf[4] = stop ? 0x01 : 0x00;
  CanCharger.write(tx);
}

// Send 0x310 + 0x311 on the specified bus
static void sendVcuStatus(bool on_can2) {
  uint16_t vn = calc_stats(g_volts, PACK_CELLS, g_vstats);
  uint16_t tn = calc_stats(g_temps, PACK_CELLS, g_tstats);

  // Status frame 0x310
  CAN_message_t s{};
  s.id = ID_BMS_STATUS; s.len = 8;
  s.buf[0] = (g_fault_code != FAULT_NONE ? 0x01 : 0x00)
           | (g_contactors_closed       ? 0x02 : 0x00)
           | (g_mode == MODE_CHARGE     ? 0x04 : 0x00);
  uint16_t vmin = vn ? (uint16_t)(g_vstats.mn  * 1000.0f) : 0;
  uint16_t vmax = vn ? (uint16_t)(g_vstats.mx  * 1000.0f) : 0;
  uint16_t vavg = vn ? (uint16_t)(g_vstats.avg * 1000.0f) : 0;
  s.buf[1] = vmin >> 8; s.buf[2] = vmin & 0xFF;
  s.buf[3] = vmax >> 8; s.buf[4] = vmax & 0xFF;
  s.buf[5] = vavg >> 8; s.buf[6] = vavg & 0xFF;
  s.buf[7] = (uint8_t)constrain((int)g_soc, 0, 100);
  if (on_can2) CanCharger.write(s); else CanVehicle.write(s);

  // Temp frame 0x311
  CAN_message_t t{};
  t.id = ID_BMS_TEMP; t.len = 8;
  t.buf[0] = tn ? (uint8_t)constrain((int)(g_tstats.mn  + 40.0f), 0, 255) : 0xFF;
  t.buf[1] = tn ? (uint8_t)constrain((int)(g_tstats.mx  + 40.0f), 0, 255) : 0xFF;
  t.buf[2] = tn ? (uint8_t)constrain((int)(g_tstats.avg + 40.0f), 0, 255) : 0xFF;
  t.buf[3] = (uint8_t)g_fault_code;
  int16_t cur_raw = (int16_t)(g_current_A * 10.0f + 3000.0f);
  t.buf[4] = cur_raw >> 8; t.buf[5] = cur_raw & 0xFF;
  t.buf[6] = 0xFF; t.buf[7] = 0xFF;
  if (on_can2) CanCharger.write(t); else CanVehicle.write(t);
}

// Send one cell data frame; called every CELL_DUMP_GAP_MS from loop
static void tickCellDump() {
  if (g_dump_type == 0) return;
  if (cellDumpTimer < CELL_DUMP_GAP_MS) return;
  cellDumpTimer = 0;

  CAN_message_t f{};
  f.id = ID_CELL_DATA; f.len = 8;
  f.buf[0] = g_dump_frame;

  if (g_dump_type == 1) {
    // Voltages: 96 × uint16, 3.5 cells per frame (7 bytes / 2)
    // Packing: 3 cells per frame (6 bytes), last byte padding
    uint8_t cells_per_frame = 3;
    uint16_t base = (uint16_t)g_dump_frame * cells_per_frame;
    for (uint8_t i = 0; i < cells_per_frame; i++) {
      uint16_t idx = base + i;
      uint16_t mv = (idx < PACK_CELLS && !isnan(g_volts[idx]))
                    ? (uint16_t)(g_volts[idx] * 1000.0f) : 0xFFFF;
      f.buf[1 + i * 2] = mv >> 8;
      f.buf[2 + i * 2] = mv & 0xFF;
    }
    f.buf[7] = 0xFF;
    uint16_t total_frames = (PACK_CELLS + cells_per_frame - 1) / cells_per_frame; // 32
    g_dump_frame++;
    if (g_dump_frame >= total_frames) g_dump_type = 0;

  } else {
    // Temps: 96 × uint8 (7 per frame)
    uint16_t base = (uint16_t)g_dump_frame * 7;
    for (uint8_t i = 0; i < 7; i++) {
      uint16_t idx = base + i;
      f.buf[1 + i] = (idx < PACK_CELLS && !isnan(g_temps[idx]))
                     ? (uint8_t)constrain((int)(g_temps[idx] + 40.0f), 0, 254)
                     : 0xFF;
    }
    uint16_t total_frames = (PACK_CELLS + 6) / 7; // 14
    g_dump_frame++;
    if (g_dump_frame >= total_frames) g_dump_type = 0;
  }

  if (g_dump_on_can2) CanCharger.write(f); else CanVehicle.write(f);
}

// ============================================================
// CAN CALLBACKS
// ============================================================
void canSniff3(const CAN_message_t& msg) {
  // Vehicle / IMD CAN (CAN3)
  if (msg.id == ID_IMD_RESPONSE && msg.buf[0] == IDX_HV_VOLTAGE) {
    uint16_t raw = u16le(msg.buf[1], msg.buf[2]);
    if (raw == 0xFFFF) { g_imd_valid = false; }
    else { g_imd_voltage = decodeIMDVoltage(raw); g_imd_valid = true; }
    return;
  }
  // Mode detection: any non-IMD frame on vehicle CAN → discharge
  if (g_state == STATE_INIT && msg.id != ID_IMD_CYCLIC && msg.id != ID_IMD_RESPONSE)
    g_vehicle_can_seen = true;

  // Cell data request from VCU
  if (msg.id == ID_CELL_REQUEST && msg.len >= 1) {
    g_dump_type    = msg.buf[0]; // 1=voltages, 2=temps
    g_dump_frame   = 0;
    g_dump_on_can2 = false;
    cellDumpTimer  = CELL_DUMP_GAP_MS; // fire immediately
  }
  // SOC set: bytes 0-1 = SOC as uint16, 0.1%/bit (e.g. 725 = 72.5%)
  if (msg.id == ID_SET_SOC && msg.len >= 2) {
    setSOC(((uint16_t)(msg.buf[0] << 8) | msg.buf[1]) * 0.1f);
  }
  // Current cal point: byte 0 = point (1/2), bytes 1-2 = actual current int16 0.1A/bit
  if (msg.id == ID_CURRENT_CAL && msg.len >= 3) {
    int16_t raw_a = (int16_t)((msg.buf[1] << 8) | msg.buf[2]);
    recordCalPoint(msg.buf[0], raw_a * 0.1f);
  }
}

void canSniff2(const CAN_message_t& msg) {
  // Charger CAN (CAN1)
  if (g_state == STATE_INIT) g_charger_can_seen = true;

  if (msg.flags.extended && msg.id == ID_CHARGER_BCAST && msg.len >= 5) {
    g_charger_out_v    = ((uint16_t)(msg.buf[0] << 8) | msg.buf[1]) * 0.1f;
    g_charger_out_a    = ((uint16_t)(msg.buf[2] << 8) | msg.buf[3]) * 0.1f;
    g_charger_status   = msg.buf[4];
    g_charger_last_rx  = millis();
  }
  // Cell data request over charger CAN
  if (msg.id == ID_CELL_REQUEST && msg.len >= 1) {
    g_dump_type    = msg.buf[0];
    g_dump_frame   = 0;
    g_dump_on_can2 = true;
    cellDumpTimer  = CELL_DUMP_GAP_MS;
  }
  if (msg.id == ID_SET_SOC && msg.len >= 2) {
    setSOC(((uint16_t)(msg.buf[0] << 8) | msg.buf[1]) * 0.1f);
  }
  if (msg.id == ID_CURRENT_CAL && msg.len >= 3) {
    int16_t raw_a = (int16_t)((msg.buf[1] << 8) | msg.buf[2]);
    recordCalPoint(msg.buf[0], raw_a * 0.1f);
  }
}

// ============================================================
// STATE HANDLERS
// ============================================================
void handleInit() {
  if (initTimer >= INIT_DELAY_MS) {
    // Determine mode from CAN traffic observed during init
    if (g_charger_can_seen) {
      g_mode = MODE_CHARGE;
      Serial.println("[BMS] Mode: CHARGE");
    } else {
      g_mode = MODE_DISCHARGE;
      Serial.println("[BMS] Mode: DISCHARGE");
    }
    g_spread_limit = CELL_DCIR_OHM * (PACK_POWER_MAX_W / PACK_V_NOM / (float)CELLS_PARALLEL);
    Serial.printf("[BMS] Spread limit: %.3f V\n", g_spread_limit);
    initSOC();
    digitalWrite(PIN_BMS_STATUS, HIGH);
    g_state = STATE_WAIT_SDC;
    Serial.println("[BMS] INIT -> WAIT_SDC | Pin 41 HIGH");
  }
}

void handleWaitSDC() {
  static bool settling = false;
  if (g_sdc_changed) {
    g_sdc_changed = false;
    if (g_sdc_state && !settling) {
      settling = true;
      sdcSettleTimer = 0;
      Serial.println("[BMS] Neg contactor closed — 200ms settle...");
    } else {
      settling = false; // falling edge while waiting — abort any settle in progress
    }
  }
  if (settling && sdcSettleTimer >= SDC_SETTLE_MS) {
    settling = false;
    g_imd_valid   = false; // discard stale reading — must get fresh poll before checking
    g_imd_voltage = 0.0f;
    digitalWrite(PIN_PRECHARGE, HIGH);
    prechargeTimer     = 0;
    prechargeVoltTimer = 0;
    imdPollTimer       = 0;
    g_state = STATE_PRECHARGE;
    Serial.println("[BMS] -> PRECHARGE | Pin 24 HIGH");
  }
}

void handlePrecharge() {
  bool charge_mode = (g_mode == MODE_CHARGE);

  if (imdPollTimer >= IMD_POLL_MS) {
    imdPollTimer = 0;
    sendIMDGet(IDX_HV_VOLTAGE);
  }
  if (prechargeVoltTimer < PRECHARGE_POLL_MS) return;
  prechargeVoltTimer = 0;

  float v_batt    = readVoltage_A();
  uint32_t elapsedMs = prechargeTimer;

  if (v_batt < 10.0f) {
    enterFault(FAULT_PRECHARGE_TIMEOUT); // treat as timeout — no battery seen
    return;
  }
  if (elapsedMs > PRECHARGE_TIMEOUT_MS) {
    enterFault(FAULT_PRECHARGE_TIMEOUT);
    return;
  }
  if (!g_imd_valid) {
    Serial.printf("[PRECHARGE] %5lu ms | Batt=%.2fV | IMD=pending\n", elapsedMs, v_batt);
    return;
  }

  float v_inv = g_imd_voltage;
  float pct   = (v_batt > 0.1f) ? (v_inv / v_batt * 100.0f) : 0.0f;
  Serial.printf("[PRECHARGE] %5lu ms | Batt=%.2fV | IMD=%.2fV | %.1f%%\n",
                elapsedMs, v_batt, v_inv, pct);

  if (v_inv >= 0.90f * v_batt) {
    if (!charge_mode && elapsedMs < PRECHARGE_MIN_MS) {
      enterFault(FAULT_PRECHARGE_FAST);
      return;
    }
    digitalWrite(PIN_POS_AIR, HIGH);
    closeSettleTimer = 0;
    g_state = STATE_CLOSE_POSITIVE;
    Serial.printf("[BMS] Precharge done in %lu ms (%.1f%%) -> CLOSE_POSITIVE | Pin 25 HIGH\n",
                  elapsedMs, pct);
  }

  if (g_sdc_changed && !g_sdc_state) { g_sdc_changed = false; handleSDCShutdown(); }
}

void handleClosePositive() {
  if (closeSettleTimer >= CLOSE_SETTLE_MS) {
    digitalWrite(PIN_PRECHARGE, LOW);
    g_contactors_closed = true;
    if (g_mode == MODE_CHARGE) {
      chargeSettleTimer = 0;
      g_state = STATE_CHARGE_SETTLE;
      Serial.println("[BMS] -> CHARGE_SETTLE (2s before requesting charge)");
    } else {
      runningVoltTimer    = 0;
      imdPollTimer        = 0;
      socSaveTimer        = 0;
      postPrechargeTimer  = 0;
      g_state = STATE_RUNNING;
      Serial.println("[BMS] -> RUNNING");
    }
  }
  if (g_sdc_changed && !g_sdc_state) { g_sdc_changed = false; handleSDCShutdown(); }
}

void handleChargeSettle() {
  if (chargeSettleTimer >= CHARGE_SETTLE_MS) {
    chargerMsgTimer     = 0;
    chargerTimeoutTimer = 0;
    socSaveTimer        = 0;
    g_cv_phase          = false;  // arm CV watchdog fresh each session
    g_charger_last_rx   = millis();
    g_state = STATE_CHARGE_ACTIVE;
    Serial.println("[BMS] -> CHARGE_ACTIVE");
  }
  if (g_sdc_changed && !g_sdc_state) { g_sdc_changed = false; handleSDCShutdown(); }
}

void handleRunning() {
  static uint32_t last_soc_update_ms = 0;

  // IMD poll
  if (imdPollTimer >= IMD_POLL_MS) {
    imdPollTimer = 0;
    sendIMDGet(IDX_HV_VOLTAGE);
  }

  // SOC update + fault check (BQ reads handled by tickMonitor in main loop)
  if (runningVoltTimer >= RUNNING_VOLT_MS) {
    uint32_t now = millis();
    float dt_s = (now - last_soc_update_ms) / 1000.0f;
    last_soc_update_ms = now;
    runningVoltTimer = 0;

    g_current_A = readPackCurrent_A();
    updateSOC(dt_s);

    // Pack voltage sense cross-check — 5 consecutive hits required before faulting,
    // suppressed for 4s after precharge to avoid false triggers during settle.
    if (g_imd_valid && postPrechargeTimer >= 4000) {
      static uint8_t imd_disagree_count = 0;
      float v_batt_a  = readVoltage_A();
      float threshold = fmaxf((float)IMD_DISAGREE_ABS, IMD_DISAGREE_PCT * v_batt_a);
      float diff      = fabsf(g_imd_voltage - v_batt_a);
      if (diff > threshold) {
        imd_disagree_count++;
        Serial.printf("[PACK_VOLT] DISAGREE %u/5 — IMD=%.1fV  IsoAmp=%.1fV  diff=%.1fV  thresh=%.1fV\n",
                      imd_disagree_count, g_imd_voltage, v_batt_a, diff, threshold);
        if (imd_disagree_count >= 5) {
          delay(200); // let serial output flush before fault
          enterFault(FAULT_PACK_VOLT_SENSE); return;
        }
      } else {
        imd_disagree_count = 0;
      }
    }

    FaultCode fc = checkCellFaults();
    if (fc != FAULT_NONE) { enterFault(fc); return; }
  }

  // SOC save
  if (socSaveTimer >= SOC_SAVE_MS) { socSaveTimer = 0; saveSOC(); }

  if (g_sdc_changed && !g_sdc_state) { g_sdc_changed = false; handleSDCShutdown(); }
}

void handleChargeActive() {
  static uint32_t last_soc_update_ms = 0;

  // Send charger control message every 1s (spec requires <5s or charger stops)
  if (chargerMsgTimer >= CHARGER_MSG_MS) {
    chargerMsgTimer = 0;
    sendChargerMsg(false);
  }

  // Charger comms watchdog
  if (millis() - g_charger_last_rx > CHARGER_TIMEOUT_MS) {
    enterFault(FAULT_CHARGER_COMMS); return;
  }

  // SOC update + fault check (BQ reads handled by tickMonitor in main loop)
  if (runningVoltTimer >= RUNNING_VOLT_MS) {
    uint32_t now = millis();
    float dt_s = (now - last_soc_update_ms) / 1000.0f;
    last_soc_update_ms = now;
    runningVoltTimer = 0;
    g_current_A = readPackCurrent_A();  // negative = charging
    updateSOC(dt_s);

    FaultCode fc = checkCellFaults();
    if (fc != FAULT_NONE) { enterFault(fc); return; }

    // Arm the CV watchdog once the charger reaches the CV plateau (output
    // voltage at/near the configured 95s setpoint). From there the current must taper
    // below CHARGER_DONE_A within CV_TIMEOUT_MS or we fault out — guards
    // against a charger that never reports a falling current.
    if (!g_cv_phase && g_charger_out_v >= (PACK_V_CHARGE_MAX - 5.0f)) {
      g_cv_phase = true;
      chargeCvTimer = 0;
      Serial.printf("[BMS] CV phase entered at %.1fV — taper watchdog armed (%lu min)\n",
                    g_charger_out_v, (unsigned long)(CV_TIMEOUT_MS / 60000UL));
    }
    if (g_cv_phase && chargeCvTimer >= CV_TIMEOUT_MS) {
      enterFault(FAULT_CHARGE_TIMEOUT); return;
    }

    // CC/CV charge: the charger holds CV at the configured setpoint once the pack
    // reaches it, and output current tapers as the cells fill. Terminate only
    // when charger output current drops below the taper threshold (CV done).
    // Reaching CELL_V_MAX is the CC->CV transition, not the end of charge;
    // per-cell over-voltage stays a hard safety fault via checkCellFaults().
    bool tapered = (g_charger_out_a > 0.0f && g_charger_out_a < CHARGER_DONE_A);
    if (tapered) {
      g_soc = 100.0f; saveSOC();
      sendChargerMsg(true);
      digitalWrite(PIN_POS_AIR,   LOW);
      digitalWrite(PIN_PRECHARGE, LOW);
      g_contactors_closed = false;
      bq_hwrst();
      g_state = STATE_CHARGE_DONE;
      Serial.printf("[BMS] Charge done (current tapered to %.2f A < %.2f A) -> CHARGE_DONE\n",
                    g_charger_out_a, CHARGER_DONE_A);
      return;
    }
  }

  if (socSaveTimer >= SOC_SAVE_MS) { socSaveTimer = 0; saveSOC(); }
  if (g_sdc_changed && !g_sdc_state) { g_sdc_changed = false; handleSDCShutdown(); }
}

void handleChargeDone() {
  // Re-init BQ and return to WAIT_SDC for next charge session
  static bool done_init = false;
  if (!done_init) { done_init = true; bq_init(); }
  if (g_sdc_changed && !g_sdc_state) { g_sdc_changed = false; done_init = false; handleSDCShutdown(); }
  if (g_sdc_changed && g_sdc_state)  { g_sdc_changed = false; done_init = false; g_state = STATE_WAIT_SDC; }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("[BMS] Full firmware — Teensy 4.1");

  delay(500); // power-up settle before touching pins or attaching interrupts

  // Outputs — safe-low
  pinMode(PIN_BMS_STATUS, OUTPUT); digitalWrite(PIN_BMS_STATUS, LOW);
  pinMode(PIN_PRECHARGE,  OUTPUT); digitalWrite(PIN_PRECHARGE,  LOW);
  pinMode(PIN_POS_AIR,    OUTPUT); digitalWrite(PIN_POS_AIR,    LOW);

  // Inputs
  analogReadResolution(12);
  pinMode(PIN_CHAN_A,     INPUT);
  pinMode(PIN_CURRENT,   INPUT);
  pinMode(PIN_NEG_SIGNAL, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_NEG_SIGNAL), pin20ISR, CHANGE);

  // Charger CAN (CAN1, pins 22/23, 250 kbps, extended frames)
  CanCharger.setTX(22); CanCharger.setRX(23);
  CanCharger.begin(); CanCharger.setBaudRate(250000);
  CanCharger.setMaxMB(16); CanCharger.enableFIFO();
  CanCharger.enableFIFOInterrupt(); CanCharger.onReceive(canSniff2);

  // Vehicle/IMD CAN (CAN3, pins 30/31, 500 kbps)
  CanVehicle.setTX(30); CanVehicle.setRX(31);
  CanVehicle.begin(); CanVehicle.setBaudRate(500000);
  CanVehicle.setMaxMB(16); CanVehicle.enableFIFO();
  CanVehicle.enableFIFOInterrupt(); CanVehicle.onReceive(canSniff3);

  loadCalibration();
  tempTimer = TEMP_INTERVAL_MS; // fire first temp read immediately on loop entry
  // Start init timer BEFORE bq_init so 3s includes BQ startup time
  initTimer = 0;

  // Init BQ79616 stack — up to 10 attempts before faulting
  {
    bool bq_ok = false;
    for (int attempt = 1; attempt <= 10 && !bq_ok; attempt++) {
      if (attempt > 1) {
        Serial.printf("[BQ] Init failed — retry %d/10, waiting 200ms...\n", attempt);
        delay(200);
      }
      bq_ok = bq_init();
    }
    if (!bq_ok) {
      enterFault(FAULT_BQ_INIT);
      return;
    }
  }

  // Baseline voltage read for OCV-based SOC (bq_init pre-fills g_volts to 0, do a real read)
  for (uint8_t s = 0; s < NUM_STACK_DEVS; s++)
    bq_read_voltages(s + 1, g_volts + s * 16);
  maskBypassedCellVoltage();

  Serial.println("[BMS] Waiting for 3s INIT window (mode detection active)...");
}

// ============================================================
// UART COMMAND PARSER
// Commands (send with newline):
//   SOC=XX.X      — set state of charge (%)
//   CAL1=XX.X     — record cal point 1: apply known current, send its value in amps
//   CAL2=XX.X     — record cal point 2: apply a different known current, send its value
// ============================================================
static void tickSerial() {
  static char buf[32];
  static uint8_t idx = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (idx == 0) return;
      buf[idx] = '\0'; idx = 0;
      if (strncmp(buf, "SOC=", 4) == 0) {
        setSOC(atof(buf + 4));
      } else if (strncmp(buf, "CAL1=", 5) == 0) {
        recordCalPoint(1, atof(buf + 5));
      } else if (strncmp(buf, "CAL2=", 5) == 0) {
        recordCalPoint(2, atof(buf + 5));
      } else {
        Serial.printf("[CMD] Unknown: %s\n", buf);
      }
    } else if (idx < (uint8_t)(sizeof(buf) - 1)) {
      buf[idx++] = c;
    }
  }
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  CanVehicle.events();
  CanCharger.events();

  tickCellDump();
  tickMonitor();
  tickSerial();

  // 0x310+0x311 broadcast in every state, including FAULT
  if (vcuBroadcastTimer >= VCU_BROADCAST_MS) {
    vcuBroadcastTimer = 0;
    sendVcuStatus(false);
    if (g_mode == MODE_CHARGE) sendVcuStatus(true);
  }

  switch (g_state) {
    case STATE_INIT:           handleInit();          break;
    case STATE_WAIT_SDC:       handleWaitSDC();       break;
    case STATE_PRECHARGE:      handlePrecharge();     break;
    case STATE_CLOSE_POSITIVE: handleClosePositive(); break;
    case STATE_CHARGE_SETTLE:  handleChargeSettle();  break;
    case STATE_RUNNING:        handleRunning();       break;
    case STATE_CHARGE_ACTIVE:  handleChargeActive();  break;
    case STATE_CHARGE_DONE:    handleChargeDone();    break;
    case STATE_FAULT:          /* latched */          break;
  }
}
