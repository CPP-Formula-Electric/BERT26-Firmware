#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <Wire.h>
#include <IntervalTimer.h>
#include <ST7796_t3.h>
#include <XPT2046_Touchscreen.h>
#include <SD.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/*
  BERT26 VCU integrated sketch

  Functions:
    - Drives the ST7796/XPT2046 touchscreen dashboard.
    - Plays a buzzer chirp when BMS status becomes active.
    - Plays a different buzzer pattern when the RTD button on GPIO 23 is pressed.
    - Reads the dual analog accelerator pedal sensor.
    - Sends Cascadia PM100DX torque commands at 100 Hz over CAN.
    - Logs binary telemetry sessions (LOGnnnn.BIN) to the built-in SD card;
      decode to CSV on a PC with tools/decode_vcu_log.py.
*/

// ---------------- Display / touch pins ----------------
static const int TFT_MISO = 12;
static const int TFT_MOSI = 11;
static const int TFT_SCK  = 13;
static const int TFT_DC   = 8;
static const int TFT_CS   = 10;
static const int TFT_RST  = 9;
static const int TFT_LED  = 29;

static const int TOUCH_CS   = 7;
static const int TOUCH_IRQ  = 6;

ST7796_t3 tft(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);

// ---------------- Vehicle IO ----------------
static const int READY_BUTTON_PIN = 23;
static const int BUZZER_PIN       = 22;    // Change here if the buzzer is wired elsewhere.
static const int BRAKE_LIGHT_PIN  = 5;     // HIGH = brake light on
static const int FAN_1_PIN        = 1;
static const int FAN_2_PIN        = 2;

// Shutdown-circuit status inputs: HIGH = signal OK (green), LOW = tripped (red).
static const int BSPD_STATUS_PIN  = 24;
static const int IMD_STATUS_PIN   = 25;
static const int BMS_STATUS_PIN   = 28;

// HV battery current sensor analog input; sampled into the fast data log.
static const int HV_CURRENT_SENSE_PIN = 14;

// MPU6050 IMU on the default Wire pins (18=SDA, 19=SCL). The sensor's INT pin
// is wired to 33 and configured as data-ready, but samples are polled at the
// fast-log rate so the interrupt is informational only.
static const int IMU_INT_PIN = 33;
static constexpr uint8_t MPU6050_I2C_ADDR = 0x68;
static constexpr uint32_t IMU_I2C_CLOCK_HZ = 400000;
static constexpr uint8_t MPU6050_REG_SMPLRT_DIV   = 0x19;
static constexpr uint8_t MPU6050_REG_CONFIG       = 0x1A;
static constexpr uint8_t MPU6050_REG_GYRO_CONFIG  = 0x1B;
static constexpr uint8_t MPU6050_REG_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t MPU6050_REG_INT_PIN_CFG  = 0x37;
static constexpr uint8_t MPU6050_REG_INT_ENABLE   = 0x38;
static constexpr uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;
static constexpr uint8_t MPU6050_REG_PWR_MGMT_1   = 0x6B;
static constexpr uint8_t MPU6050_REG_WHO_AM_I     = 0x75;
static constexpr uint8_t MPU6050_WHO_AM_I_VALUE   = 0x68;
// ±4 g accel (8192 LSB/g) and ±500 dps gyro (65.5 LSB/dps); raw counts are
// logged and scaled to g / deg/s in the decoder.
static constexpr uint8_t MPU6050_ACCEL_FS_4G      = 0x08;
static constexpr uint8_t MPU6050_GYRO_FS_500DPS   = 0x08;
static constexpr uint8_t MPU6050_DLPF_44HZ        = 0x03;
static constexpr uint32_t IMU_RETRY_PERIOD_MS = 5000;

static constexpr uint32_t FAN_PWM_FREQUENCY_HZ = 20000;
static constexpr uint8_t FAN_PWM_WRITE_MAX = 255;

// Fan duty tracks the hotter of motor/inverter temp: 20% at room temp,
// ramping linearly to 100% by 40 C (system max is 50 C).
static const float FAN_TEMP_MIN_C = 25.0f;
static const float FAN_TEMP_MAX_C = 40.0f;
static const int FAN_DUTY_MIN_PERCENT = 20;
static const int FAN_DUTY_MAX_PERCENT = 100;

// EURO-XPD analog input pins from VCU_Pedal_Read_Test.
static const int APPS_1_PIN = 38;         // Inverted channel
static const int APPS_2_PIN = 41;         // Normal channel
static const int BSE_1_PIN  = 17;
static const int BSE_2_PIN  = 16;

static const int ADC_RESOLUTION_BITS = 12;
static const int ADC_MAX_COUNTS      = (1 << ADC_RESOLUTION_BITS) - 1;
static const float ADC_REFERENCE_VOLTAGE = 3.3f;

// Raw ADC calibration ranges from the BERT26 pedal test sketch.
static const int APPS_1_RAW_MIN = 325;
static const int APPS_1_RAW_MAX = 2400;
static const int APPS_2_RAW_MIN = 2130;
static const int APPS_2_RAW_MAX = 4013;

static const float APPS_EMA_ALPHA          = 0.35f;
static const float APPS_IMPLAUSIBLE_PCT    = 10.0f;
// T.4.2.5 allows up to 100 ms of >10% channel deviation before it counts as
// an implausibility, so brief sensor/connector noise no longer drops RTD.
static const uint32_t APPS_IMPLAUSIBILITY_PERSIST_MS = 100;
// A raw count this far outside a channel's calibrated span means a wiring or
// sensor fault (short/open), not pedal travel.
static const int APPS_RAW_FAULT_MARGIN = 150;
static const float PEDAL_ZERO_DEADBAND_PCT = 7.0f;
static const float MAX_TORQUE_NM           = 170.0f;
static const float BSPC_APPS_THRESHOLD_PCT = 25.0f;
// Per EV.4.7/T.4.7 the BSPC latch only releases once the accelerator returns
// below 5% travel; releasing the brake alone must NOT restore torque.
static const float BSPC_APPS_RESET_PCT     = 5.0f;

// ---- Power derate -------------------------------------------------------
// When any protection condition trips, commanded torque is capped so output
// power stays at or below the active power limit (kW). The cap is the torque
// that produces that power at the current shaft speed:
//   T[Nm] = 9549 * kW / rpm (mechanical power form; ignores drivetrain/inverter
// efficiency, so the electrical bus power will be slightly higher than this).
//
// Low cell voltage derates *linearly* on the rolling-average minimum cell:
// at POWER_DERATE_CELL_FULL_MV the limit is POWER_DERATE_CELL_MAX_KW, falling
// to 0 kW at POWER_DERATE_CELL_ZERO_MV. Motor/battery-temperature trips use the
// fixed POWER_DERATE_KW cap. When more than one condition is active the lowest
// (most restrictive) limit wins.
static const float    POWER_DERATE_KW           = 5.0f;
static const float    TORQUE_PER_KW_PER_RPM      = 9549.0f;  // 60000 / (2*pi)
// Below this rpm the power cap exceeds MAX_TORQUE_NM anyway; the guard also
// avoids dividing by zero / near-zero rpm.
static const int      POWER_DERATE_MIN_RPM       = 50;
// Linear low-voltage derate endpoints (rolling-average minimum cell):
//   >= CELL_FULL_MV -> no voltage derate; == CELL_FULL_MV -> CELL_MAX_KW;
//   <= CELL_ZERO_MV -> 0 kW; linear in between.
static const uint16_t POWER_DERATE_CELL_FULL_MV  = 3600;   // 60 kW point
static const uint16_t POWER_DERATE_CELL_ZERO_MV  = 2500;   // 0 kW point
static const float    POWER_DERATE_CELL_MAX_KW   = 60.0f;
// Derate trip points. Battery temperature is evaluated on a 15 s rolling
// average; motor temperature is instantaneous.
static const float    POWER_DERATE_MOTOR_TEMP_C  = 105.0f; // >  trips (instantaneous)
static const int16_t  POWER_DERATE_BATT_TEMP_C   = 50;     // >  trips (rolling avg)
static const uint32_t POWER_DERATE_SAMPLE_MS     = 250;    // rolling-average sample period
static const uint32_t POWER_DERATE_WINDOW_MS     = 15000;  // rolling-average window
static const uint8_t  POWER_DERATE_WINDOW_SAMPLES =
    (uint8_t)(POWER_DERATE_WINDOW_MS / POWER_DERATE_SAMPLE_MS);  // 60 samples

// Fixed-cadence rolling average over POWER_DERATE_WINDOW_SAMPLES samples. Defined
// here (ahead of any function) so the .ino auto-generated prototypes can see it.
struct RollingWindow {
  float    buf[POWER_DERATE_WINDOW_SAMPLES];
  float    sum;
  uint8_t  count;
  uint8_t  head;
};

static RollingWindow minCellMvWindow = {};
static RollingWindow battMaxTempWindow = {};
bool powerDerateActive = false;
// Active power limit (kW) while powerDerateActive; the lowest cap across all
// tripped conditions. Updated each sample by servicePowerDerate().
float powerDerateKw = POWER_DERATE_KW;

// Calculated from voltage divider at input. R1 = 5.6k, R2 = 15k, Gain = 0.728
// This comment was written by a human, me
static const float BSE_SENSOR_MIN_VOLTAGE  = 0.364f;
static const float BSE_SENSOR_MAX_VOLTAGE  = 3.277f;
static const int BSE_1_BRAKING_RAW_THRESHOLD = 600;
static const int BSE_2_BRAKING_RAW_THRESHOLD = 800;
static const int BSE_1_HARD_BRAKING_RAW_THRESHOLD = 2000;
static const int BSE_2_HARD_BRAKING_RAW_THRESHOLD = 2100;

// ---------------- CAN ----------------
// GPIO 23 is used by the RTD button, so this sketch uses the Teensy 4.1 CAN3
// peripheral on pins 30=CAN RX and 31=CAN TX. These pins connect to the TXD/RXD
// side of an external CAN transceiver module; the module's CANH/CANL side
// connects to the vehicle CAN bus.
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> VehicleCan;

static constexpr uint32_t CAN_BAUD = 500000;
static constexpr bool CAN_LOG_SERIAL_ENABLED = true;
static constexpr bool CAN_TRANSLATION_SERIAL_ENABLED = true;
static constexpr bool APPS_SERIAL_LOG_ENABLED = true;

// Set false to bypass brake-press requirement when bench testing without pedals.
static constexpr bool REQUIRE_BRAKE_FOR_RTD = true;
static constexpr uint32_t APPS_SERIAL_LOG_PERIOD_MS = 100;

// Cascadia PM100DX command frame from BERT25.
static constexpr uint32_t PM100_CMD_ID       = 0x0C0;
static constexpr uint32_t PM100_TEMPERATURE_1_FRAME_ID = 0x0A0;
static constexpr uint32_t PM100_TEMPERATURE_3_FRAME_ID = 0x0A2;
static constexpr uint32_t PM100_RPM_FRAME_ID = 0x0A5;
static constexpr uint32_t PM100_CURRENT_INFO_FRAME_ID = 0x0A6;
static constexpr uint32_t PM100_VOLTAGE_INFO_FRAME_ID = 0x0A7;
static constexpr uint32_t PM100_INTERNAL_VOLTAGES_FRAME_ID = 0x0A9;
static constexpr uint32_t PM100_FAULT_CODES_FRAME_ID = 0x0AB;
static constexpr uint32_t PM100_TORQUE_CAPABILITY_FRAME_ID = 0x0B1;
static constexpr uint32_t PM100_PARAM_COMMAND_ID  = 0x0C1;
static constexpr uint32_t PM100_PARAM_RESPONSE_ID = 0x0C2;
// EEPROM parameter 148 = "CAN Active Messages Lo Word"; bit 0x0200 enables the
// 0x0A9 Internal Voltages broadcast that carries the GLV (12V) voltage.
static constexpr uint16_t PM100_PARAM_CAN_ACTIVE_MSGS = 148;
static constexpr uint32_t PM100_BROADCAST_ENABLE_DELAY_MS = 5000;
static constexpr uint32_t PM100_BROADCAST_ENABLE_RETRY_MS = 5000;
static constexpr uint8_t PM100_BROADCAST_ENABLE_MAX_ATTEMPTS = 3;
// Parameter 20 = "Fault Clear"; writing 0 tells the inverter to clear latched
// fault codes. Sent automatically when the inverter is faulted but the BMS
// reports contactors closed and the DC bus is back at pack voltage, so a
// transient fault (e.g. under-voltage from an HV cycle or pack sag) doesn't
// require power-cycling the inverter. 240 V = 96 series cells at the 2.5 V
// minimum, so any bus above that means the pack is genuinely back on the bus.
static constexpr uint16_t PM100_PARAM_FAULT_CLEAR = 20;
static constexpr float PM100_FAULT_CLEAR_MIN_BUS_V = 240.0f;
static constexpr uint32_t PM100_FAULT_CLEAR_RETRY_MS = 2000;
static constexpr uint8_t PM100_FAULT_CLEAR_MAX_ATTEMPTS = 3;
static constexpr uint16_t TORQUE_SCALE       = 10;      // 0.1 Nm per LSB
static constexpr uint32_t CMD_PERIOD_MS      = 10;      // 100 Hz
static constexpr uint32_t PM100_CMD_PERIOD_US = CMD_PERIOD_MS * 1000u;
static constexpr float MPH_PER_RPM           = 0.014005f;

static constexpr uint32_t BMS_TIMEOUT_MS = 10000;
static constexpr uint32_t SCREEN_PERIOD_MS = 100;
static constexpr uint32_t DASHBOARD_FULL_REDRAW_MS = 120000;  // periodic anti-glitch repaint
static constexpr uint32_t STARTUP_BMS_IMD_FAULT_DELAY_MS = 12000;
static constexpr uint32_t FAULT_DISPLAY_CYCLE_MS = 1500;
static constexpr uint32_t FAULT_BANNER_DURATION_MS = 5000;
// GLV battery sits around 12-13 V; show the readout red once it sags below this.
static constexpr float GLV_LOW_VOLTAGE_WARN_V = 11.5f;
static constexpr uint8_t DASHBOARD_MAX_FAULT_TEXTS = 48;
static constexpr uint32_t RTD_BLOCKED_LOG_PERIOD_MS = 1000;

// BERT26 vehicle CAN definitions for human-readable log translations.
static constexpr uint32_t BMS_STATUS_2026_ID        = 0x310;
static constexpr uint32_t BMS_TEMPERATURE_2026_ID   = 0x311;
static constexpr uint32_t BMS_CELL_REQUEST_2026_ID  = 0x312;
static constexpr uint32_t BMS_CELL_RESPONSE_2026_ID = 0x313;
static constexpr uint8_t CELL_DATA_REQUEST_UNKNOWN      = 0x00;
static constexpr uint8_t CELL_DATA_REQUEST_VOLTAGES     = 0x01;
static constexpr uint8_t CELL_DATA_REQUEST_TEMPERATURES = 0x02;
static constexpr int16_t CELL_TEMP_UNKNOWN_C = -1000;

// ---- Formula SAE competition Energy Meter (EM_User_Manual_2026) ----
// The Energy Meter shares this 500 kbit/s vehicle bus. It broadcasts three
// 11-bit standard frames; all signals are little-endian. The VCU only listens
// and decodes Voltage, the module-temperature summary, and the status bits for
// the dashboard.
static constexpr uint32_t EM_MEASUREMENT_ID = 0x10D;  // 20 ms:  Current[A], Voltage[V] floats
static constexpr uint32_t EM_STATUS_ID      = 0x40D;  // 100 ms: status bits + Energy[Whr] float
static constexpr uint32_t EM_TEMPERATURE_ID = 0x60D;  // 250 ms: muxed module temperatures
// 0x40D status byte 0 bit masks.
static constexpr uint8_t EM_STATUS_VIOLATION    = 1u << 0;
static constexpr uint8_t EM_STATUS_LOGGING      = 1u << 1;
static constexpr uint8_t EM_STATUS_FAULT_ACTIVE = 1u << 2;
static constexpr uint8_t EM_STATUS_FAULT_PREV   = 1u << 3;
static constexpr float EM_TEMP_SCALE_C = 0.5f;        // degC = raw * 0.5
// The fastest EM frame is 20 ms; flag a comms fault if nothing arrives for this long.
static constexpr uint32_t EM_TIMEOUT_MS = 1000;

// Bender iso175 IMD/TSSI frames from VCU_TSSI_CODE_5_31_26.ino.
static constexpr uint32_t IMD_INFO_GENERAL_ID = 0x37;
static constexpr uint32_t IMD_INFO_ISOLATION_DETAIL_ID = 0x38;
static constexpr uint32_t IMD_INFO_VOLTAGE_ID = 0x39;
static constexpr uint32_t IMD_INFO_IT_SYSTEM_ID = 0x3A;
static constexpr uint32_t IMD_RESPONSE_ID     = 0x23;
static constexpr uint32_t IMD_REQUEST_ID      = 0x22;
static constexpr uint32_t IMD_FALLBACK_REQUEST_ID = 0x0FF;
static constexpr uint8_t IDX_WARN_ALARMS      = 0x6C;
static constexpr uint16_t IMD_SNV_WORD        = 0xFFFF;
static constexpr uint8_t IMD_SNV_BYTE         = 0xFF;
static constexpr int32_t IMD_VOLTAGE_OFFSET   = 32128;
static constexpr float IMD_VOLTAGE_SCALE      = 0.05f;

// TSSI LED pins: low-side switches, HIGH = transistor on = LED on.
static const int TSSI_LED_RED   = 3;
static const int TSSI_LED_GREEN = 4;

static constexpr uint32_t TSSI_BLINK_PERIOD_MS    = 500;  // 2 Hz
static constexpr uint32_t TSSI_BLINK_ON_MS        = 250;
static constexpr uint32_t IMD_HEARTBEAT_TIMEOUT_MS = 5000;
static constexpr uint32_t IMD_GET_REQ_PERIOD_MS    = 500;
 
static constexpr uint16_t IMD_FAULT_BITS_MASK = (1u << 0) | (1u << 1) | (1u << 2) |
                                                (1u << 3) | (1u << 4) | (1u << 7) |
                                                (1u << 8) | (1u << 9) | (1u << 10);

enum RtdState {
  RTD_STATE_FAULT_BLOCKED,
  RTD_STATE_READY_FOR_BUTTON,
  RTD_STATE_READY_TO_DRIVE
};

// ---------------- Data logging ----------------
// Binary session logs. A new LOGnnnn.BIN is created on the built-in SD card
// at every power-up, holding fast (10 ms), medium (100 ms), slow (1 s) and
// IMU (10 ms) records. Files are decoded to CSV on a computer with
// tools/decode_vcu_log.py; nothing is converted on the VCU.
static constexpr uint32_t LOG_FAST_PERIOD_MS   = 10;
static constexpr uint32_t LOG_MEDIUM_PERIOD_MS = 100;
static constexpr uint32_t LOG_SLOW_PERIOD_MS   = 1000;
static constexpr uint32_t LOG_SD_FLUSH_PERIOD_MS = 1000;
static constexpr uint8_t LOG_RECORD_FAST   = 0x01;
static constexpr uint8_t LOG_RECORD_MEDIUM = 0x02;
static constexpr uint8_t LOG_RECORD_SLOW   = 0x03;
static constexpr uint8_t LOG_RECORD_IMU    = 0x04;
static constexpr char LOG_FILE_MAGIC[8] = {'V','C','U','L','O','G','0','1'};

struct __attribute__((packed)) LogFileHeader {
  char magic[8];
  uint32_t session;
  uint8_t fastSize;
  uint8_t mediumSize;
  uint8_t slowSize;
  uint8_t imuSize;          // 0 in logs from firmware without an IMU
};

// 25 bytes @ 100 Hz. Raw ADC counts are logged (not scaled floats) so pedal
// faults can be reconstructed exactly; scaling happens in the decoder.
struct __attribute__((packed)) LogFastRecord {
  uint8_t type;
  uint32_t ms;
  uint16_t rawApps1;
  uint16_t rawApps2;
  uint16_t rawBse1;
  uint16_t rawBse2;
  uint16_t rawHvCurrent;
  int16_t commandedTorqueDeciNm;
  int16_t motorRpm;
  int16_t busVoltageDeciV;
  int16_t busCurrentDeciA;
  uint16_t statusFlags;     // see logStatusFlags()
};

// 39 bytes @ 10 Hz.
struct __attribute__((packed)) LogMediumRecord {
  uint8_t type;
  uint32_t ms;
  int16_t apps1PctX10;
  int16_t apps2PctX10;
  int16_t pedalPctX10;
  int16_t desiredTorqueDeciNm;
  int16_t powerDeciKw;
  int16_t speedMph;
  uint16_t glvCentiV;
  int16_t inverterTempTenthsC;
  int16_t motorTempTenthsC;
  uint8_t socPct;
  uint8_t batteryTempC;
  uint8_t bmsFlags;
  uint8_t bms2026FaultCode;
  uint16_t imdWarnAlarms;
  uint32_t pm100PostFaults;
  uint32_t pm100RunFaults;
  uint8_t fanDutyPct;
  uint8_t rtdState;
};

// 21 bytes @ 1 Hz.
struct __attribute__((packed)) LogSlowRecord {
  uint8_t type;
  uint32_t ms;
  uint16_t minCellMv;
  uint16_t maxCellMv;
  uint16_t avgCellMv;
  int16_t minCellTempC;
  int16_t maxCellTempC;
  int16_t avgCellTempC;
  uint16_t imdRIsoKohm;
  uint8_t imdRIsoStatus;
  uint8_t validFlags;       // bit0 cell V stats, bit1 cell T stats, bit2 GLV
};

// 19 bytes @ 100 Hz. Raw MPU6050 counts; the decoder scales to g and deg/s.
struct __attribute__((packed)) LogImuRecord {
  uint8_t type;
  uint32_t ms;
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
  int16_t tempRaw;          // degC = raw / 340 + 36.53
};

// ---------------- Runtime state ----------------
char driveMode = 'N';  // RTD state machine sets D only while ready to drive.

float apps1Pct = 0.0f;
float apps2Pct = 0.0f;
float pedalPct = 0.0f;
float desiredTorqueNm = 0.0f;
// Torque actually loaded into the PM100 command frame this cycle; this is
// what the dashboard shows so the display always matches the bus.
float commandedTorqueNm = 0.0f;
bool appsPlausible = false;
bool appsDisagreeing = false;
uint32_t appsDisagreeStartMs = 0;
// Names the kind of APPS problem (channel disagreement vs. out-of-range
// sensor) for the dash fault texts; only meaningful while not plausible.
const char *appsFaultText = "APPS Disagree";
// True while brake + accelerator >25% travel are applied together; commands
// zero torque only for as long as the condition holds (no latch).
bool bspcActive = false;
bool appsFilterInitialized = false;
int rawApps1Last = 0;
int rawApps2Last = 0;
int rawBse1Last = 0;
int rawBse2Last = 0;
float bse1Voltage = 0.0f;
float bse2Voltage = 0.0f;
float bse1Pct = 0.0f;
float bse2Pct = 0.0f;
bool braking = false;
bool hardBraking = false;
int rawHvCurrentLast = 0;

// IMU state; imuPresent reflects the last init/read outcome so a loose
// connector shows up as missing records, never as stale repeated data.
bool imuPresent = false;
uint32_t imuLastInitAttemptMs = 0;
int16_t imuAccelRaw[3] = {0, 0, 0};
int16_t imuGyroRaw[3] = {0, 0, 0};
int16_t imuTempRaw = 0;

// Data logging state. Records are staged in RAM and written in 512-byte
// chunks so the loop only pays for an SD write every couple hundred ms.
File sdLogFile;
bool sdCardStarted = false;
bool sdLogActive = false;
uint32_t logSessionNumber = 0;
uint8_t sdLogBuffer[512];
size_t sdLogBufferLen = 0;

// Duty cycles are set automatically from motor/inverter temps unless a
// manual override is entered over serial (digits = override, 'a' = auto).
int fan1DutyPercent = FAN_DUTY_MIN_PERCENT;
int fan2DutyPercent = FAN_DUTY_MIN_PERCENT;
int fan1LastDutyPercent = -1;
int fan2LastDutyPercent = -1;
bool fanManualOverride = false;

float packVoltage = 0.0f;
float packCurrent = 0.0f;
float inverterDcBusVoltage = 0.0f;
float inverterDcBusCurrent = 0.0f;
bool haveInverterVoltageInfo = false;
bool haveInverterCurrentInfo = false;
// GLV (12V system) voltage reported by the PM100 Internal Voltages frame.
float glvVoltage = 0.0f;
bool haveGlvVoltage = false;
// If the inverter is talking but 0x0A9 never arrives, its broadcast mask has
// Internal Voltages disabled; we write parameter 148 once to turn it back on.
bool havePm100Traffic = false;
uint32_t firstPm100FrameMs = 0;
uint8_t pm100BroadcastEnableAttempts = 0;
uint32_t pm100BroadcastEnableLastTxMs = 0;
float powerKw = 0.0f;
uint8_t stateOfChargePct = 0;
uint8_t batteryTempC = 0;
uint16_t minCellVoltageMv = 0;
uint16_t maxCellVoltageMv = 0;
uint16_t avgCellVoltageMv = 0;
bool haveCellVoltageStats = false;
int16_t minCellTempC = CELL_TEMP_UNKNOWN_C;
int16_t maxCellTempC = CELL_TEMP_UNKNOWN_C;
int16_t avgCellTempC = CELL_TEMP_UNKNOWN_C;
bool haveCellTempStats = false;
int16_t inverterTempTenthsC = 0;
int16_t pm100MotorTempTenthsC = 0;
bool haveInverterTemp = false;
bool havePm100MotorTemp = false;
uint32_t pm100PostFaults = 0;
uint32_t pm100RunFaults = 0;
bool havePm100FaultStatus = false;
bool pm100Fault = false;
// Automatic fault-clear bookkeeping; attempts reset when the fault clears or
// the contactors open, so each HV cycle gets a fresh set of retries.
uint8_t pm100FaultClearAttempts = 0;
uint32_t pm100FaultClearLastTxMs = 0;
int16_t motorRpm = 0;
int speedMph = 0;

// ---- Energy Meter decoded state ----
// Populated from 0x10D / 0x40D / 0x60D. "have" flags gate the dashboard so an
// absent meter shows "--"/"NO COMMS" instead of stale zeros.
float emCurrentA = 0.0f;
float emVoltageV = 0.0f;
bool haveEmMeasurement = false;
uint8_t emStatusByte = 0;
bool haveEmStatus = false;
uint8_t emNumSensors = 0;
uint8_t emMinTempRaw = 0;
uint8_t emMaxTempRaw = 0;
bool haveEmTempSummary = false;
bool haveEmTemps = false;
uint32_t emLastRxMs = 0;
bool emTimedOut = true;

bool bmsActive = false;
bool bmsFault = true;
bool bmsTimedOut = true;
bool haveBmsStatus = false;
uint8_t bmsFlags = 0;
uint8_t bms2026FaultCode = 0;
bool haveBms2026FaultCode = false;
uint32_t lastBmsRxMs = 0;
uint8_t lastCellDataRequestType = CELL_DATA_REQUEST_UNKNOWN;

bool imdFault = true;
uint16_t imdWarnAlarms = 0;
uint16_t imdRIsoKohm = 0xFFFF;
uint8_t imdRIsoStatus = 0;
uint8_t imdDeviceActivity = 0;
uint32_t lastImdRxMs = 0;
bool imdCanTimedOut = true;

uint32_t tssiBlinkTimerMs = 0;
bool tssiRedOn = false;
// Once the TSSI has blinked red for any fault it stays latched in the red-blink
// state until the board is power-cycled (this flag clears on reset).
bool tssiFaultLatched = false;
uint32_t lastImdGetReqMs = 0;

RtdState rtdState = RTD_STATE_FAULT_BLOCKED;
// Compatibility flag used by the display and PM100 gate; kept in sync with rtdState.
bool readyToDriveLatched = false;

bool readyButtonLast = false;

// Fault banner: a red strip over the drive-button row naming the most recent
// fault, shown for FAULT_BANNER_DURATION_MS after each fault event.
char faultBannerText[48] = "";
uint32_t faultBannerUntilMs = 0;
bool faultBannerVisible = false;
bool faultBannerDirty = false;

int backlightBrightness = 255;
bool screenDirty = true;
bool dashboardNeedsFullRedraw = true;
uint32_t startupFaultDelayStartMs = 0;

IntervalTimer pm100CommandTimer;
volatile int16_t pm100CommandTorqueRaw = 0;
volatile uint8_t pm100CommandDirection = 0x00;
volatile uint8_t pm100CommandEnable = 0x00;

static constexpr uint8_t PM100_TX_LOG_QUEUE_SIZE = 192;
CAN_message_t pm100TxLogQueue[PM100_TX_LOG_QUEUE_SIZE];
volatile uint8_t pm100TxLogHead = 0;
volatile uint8_t pm100TxLogTail = 0;
volatile uint32_t pm100TxLogDropped = 0;

// ---------------- Buzzer sequencer ----------------
struct ToneStep {
  uint16_t frequencyHz;
  uint16_t onMs;
  uint16_t offMs;
};

static const ToneStep TS_READY_CHIME_PATTERN[] = {
  {1568, 200, 0},
  {1047, 100, 0},
  {1319, 300, 1500}
};

// EV.10.2: ready-to-drive sound, 1-3 seconds. Solid continuous tone for the
// first 1.25 s (satisfies the >=1 s continuous requirement), then a short
// playful flourish. Step on+off times sum to 2000 ms total.
static const ToneStep RTD_PATTERN[] = {
  {2093, 1250,  0},  // C7 - continuous main sound (1.25 s)
  {2637,  120, 40},  // E7
  {3136,  120, 40},  // G7
  {2637,  120, 40},  // E7
  {3136,  270,  0}   // G7 - final flourish
};

const ToneStep *activeTonePattern = nullptr;
uint8_t activeToneLength = 0;
uint8_t activeToneIndex = 0;
bool activeToneInGap = false;
bool activeToneRepeats = false;
uint32_t activeToneDeadlineMs = 0;

typedef void (*CanTranslationPrinter)(const CAN_message_t &msg);

struct CanFrameTranslation {
  uint32_t id;
  const char *name;
  CanTranslationPrinter print;
};

struct DashboardFaultList {
  const char *texts[DASHBOARD_MAX_FAULT_TEXTS];
  uint8_t count;
};

// ---------------- Function declarations ----------------
void setupDisplay();
void resetDisplay();
void handleSerialConsoleInput();
void handleSerialConsoleCommand(const char *cmd);

void setupDataLogging();
void serviceDataLogging();
void writeFastLogRecord(uint32_t nowMs);
void writeMediumLogRecord(uint32_t nowMs);
void writeSlowLogRecord(uint32_t nowMs);
void writeImuLogRecord(uint32_t nowMs);
void setupImu();
bool initImu();
bool readImuSample();
bool writeImuRegister(uint8_t reg, uint8_t value);
uint16_t logStatusFlags();
void appendSdLog(const void *data, size_t len);
void flushSdLogBuffer();
uint32_t nextLogSessionNumber();
uint32_t maxLogSessionInDir(File dir);
void listLogFiles();
void listLogDir(File dir);
void dumpLogFile(uint32_t session);
void handleTouch();
char checkIfTouchedRaw(int x, int y);
void drawDashboard();
void drawDriveButtons(bool force);
void drawDashboardLabel(int x, int y, const char *label);
void drawTileFrame(int x, int y, const char *label);
void drawDerateBanner(bool force);
uint16_t motorTempColor(float tempC, bool valid);
uint16_t batteryTempColor(int tempC, bool valid);
uint16_t cellSpreadColor(int spreadMv, bool valid);
void drawCachedDashboardText(int x, int y, int w, int h, uint8_t textSize,
                             const char *value, uint16_t valueColor,
                             char *lastValue, size_t lastValueSize,
                             uint16_t &lastColor, bool force);
void formatCellVoltageText(char *buffer, size_t bufferSize, uint16_t millivolts, bool valid);
void formatCellTempText(char *buffer, size_t bufferSize, int16_t tempC, bool valid);
void formatTemperatureTenthsText(char *buffer, size_t bufferSize, int16_t tempTenthsC, bool valid);
const char *dashboardFaultText();
void collectDashboardFaults(DashboardFaultList &faults);
void appendDashboardFault(DashboardFaultList &faults, const char *text);
void appendTssiFaultNames(DashboardFaultList &faults, uint16_t bits);
void appendPm100FaultNames(DashboardFaultList &faults, uint32_t postFaults, uint32_t runFaults);
const char *firstTssiFaultName(uint16_t bits);
const char *firstPm100FaultName(uint32_t postFaults, uint32_t runFaults);
const char *bmsTemperatureFaultCodeName(uint8_t faultCode);
const char *pm100PostFaultName(uint8_t bit);
const char *pm100RunFaultName(uint8_t bit);
void drawStatusText(int x, int y, const char *label, const char *value, uint16_t valueColor);
void drawStatusText(int x, int y, const char *label, int value, const char *units, uint16_t valueColor);
void drawShutdownStatusIndicators(bool force);
void drawShutdownStatusIndicator(int x, int y, const char *label, bool high,
                                 int &lastState, bool force);

void setupFans();
void updateFanDutyFromTemps();
void serviceFans();
uint8_t dutyPercentToFanPwmWrite(int dutyPercent);

void setupVehicleCan();
void receiveCanFrames();
void dispatchReceivedCanFrame(const CAN_message_t &msg);
void sendCanFrame(CAN_message_t &msg);
void logCanFrame(const char *direction, const CAN_message_t &msg);
void logCanFrameToSerial(const char *direction, const CAN_message_t &msg);
void setupPm100CommandTimer();
void pm100CommandTimerIsr();
void updatePm100CommandState();
void queuePm100TxLogFromIsr(const CAN_message_t &msg);
bool dequeuePm100TxLog(CAN_message_t &msg);
void servicePm100TxLogging();
void updateCanTranslationState(const char *direction, const CAN_message_t &msg);
void logCanTranslationToSerial(const char *direction, const CAN_message_t &msg);
void printBmsStatus2026Translation(const CAN_message_t &msg);
void printBmsTemperature2026Translation(const CAN_message_t &msg);
void printCellDataRequest2026Translation(const CAN_message_t &msg);
void printCellDataResponse2026Translation(const CAN_message_t &msg);
void printImdRequestTranslation(const CAN_message_t &msg);
void printImdResponseTranslation(const CAN_message_t &msg);
void printImdInfoGeneralTranslation(const CAN_message_t &msg);
void printImdInfoIsolationDetailTranslation(const CAN_message_t &msg);
void printImdInfoVoltageTranslation(const CAN_message_t &msg);
void printImdInfoItSystemTranslation(const CAN_message_t &msg);
void printPm100CommandTranslation(const CAN_message_t &msg);
void printPm100Temperatures1Translation(const CAN_message_t &msg);
void printPm100Temperatures3Translation(const CAN_message_t &msg);
void printPm100MotorPositionTranslation(const CAN_message_t &msg);
void printPm100CurrentInfoTranslation(const CAN_message_t &msg);
void printPm100VoltageInfoTranslation(const CAN_message_t &msg);
void printPm100InternalVoltagesTranslation(const CAN_message_t &msg);
void printPm100FaultCodesTranslation(const CAN_message_t &msg);
void printPm100TorqueCapabilityTranslation(const CAN_message_t &msg);
void printRawPayloadFrom(const CAN_message_t &msg, uint8_t startIndex);
void printBmsTemperatureFaultCode(uint8_t faultCode);
void printCellDataRequestName(uint8_t requestType);
void printImdRequestIndexName(uint8_t requestIndex);
void printImdResponseErrorName(uint8_t errorCode);
void printImdInfoMessageSelectorName(uint8_t selector);
void printImdProfileName(uint8_t profile);
void printImdRIsoStatusName(uint8_t status);
void printImdDeviceActivityName(uint8_t activity);
void printImdLockStateName(uint8_t state);
void printImdEarthliftStateName(uint8_t state);
void printImdVoltageModeName(uint8_t mode);
void printImdBaudRateName(uint8_t baudRate);
void printImdAsciiPayloadFrom(const CAN_message_t &msg, uint8_t startIndex);
void printImdWordValue(uint16_t value);
void printImdResistanceValue(uint16_t kohms);
void printImdVoltageValue(uint16_t raw);
void printImdCapacityValue(uint16_t raw);
void printImdPercentValue(uint8_t value);
void printImdCounterValue(uint8_t value);
void printCellVoltageValue(uint16_t millivolts);
void printCanHexByte(uint8_t value);
bool tssiFaultFromWarnBits(uint16_t warnBits);
bool bmsImdFaultDetectionArmed();
bool anyFaultActive();
bool contactorsOpen();
void handleBms2026StatusFrame(const CAN_message_t &msg);
void handleBms2026TempFrame(const CAN_message_t &msg);
void handleImdFrame(const CAN_message_t &msg);
void handlePm100Temperatures1Frame(const CAN_message_t &msg);
void handlePm100Temperatures3Frame(const CAN_message_t &msg);
void handlePm100RpmFrame(const CAN_message_t &msg);
void handlePm100CurrentInfoFrame(const CAN_message_t &msg);
void handlePm100VoltageInfoFrame(const CAN_message_t &msg);
void handlePm100InternalVoltagesFrame(const CAN_message_t &msg);
void handlePm100FaultCodesFrame(const CAN_message_t &msg);
void handlePm100ParamResponseFrame(const CAN_message_t &msg);
void handleEmMeasurementFrame(const CAN_message_t &msg);
void handleEmStatusFrame(const CAN_message_t &msg);
void handleEmTemperatureFrame(const CAN_message_t &msg);
void serviceEnergyMeterTimeout(uint32_t now);
const char *emStatusSummaryText(uint16_t &colorOut);
void servicePm100BroadcastEnable();
void sendPm100BroadcastEnable();
void servicePm100FaultClear();
void sendPm100FaultClear();
void setupTssi();
void serviceTssi();
void requestImdWarnings();
void updateTssiLed(bool faultActive, uint32_t now);
void setTssiGreenOn();
void setTssiRedOn();
void setTssiRedOff();
void setTssiAllOff();
void printTssiWarnBits(uint16_t bits);

void updatePedal();
void updateBrakePressureSensors();
void logAppsTelemetry();
float scalePercentNormal(int rawValue, int rawMin, int rawMax);
float scalePercentInverted(int rawValue, int rawMin, int rawMax);
float adcCountsToVoltage(int rawValue);
float scaleBrakePressurePercent(float voltage);

void serviceReadyToDrive();
bool readReadyButtonPressed();
bool rtdInputsReady();
bool readyForRtdButton();
void refreshRtdStateFromInputs();
void logRtdBlockedReason();
void handleRtdButtonPressed();
void setRtdState(RtdState newState);
void dropReadyToDrive(const char *reason);
const char *rtdDropCauseText();
void showFaultBanner(const char *text);
void serviceFaultBanner();
void drawFaultBanner(bool force);
const char *rtdStateName(RtdState state);
void setDriveMode(char newDriveMode);
uint8_t pm100DirectionByteForMode(char mode);

void playTonePattern(const ToneStep *pattern, uint8_t length, bool repeats);
void serviceBuzzer();
void stopBuzzer();

static const CanFrameTranslation CAN_FRAME_TRANSLATIONS[] = {
  {BMS_STATUS_2026_ID,        "BMS Status",        printBmsStatus2026Translation},
  {BMS_TEMPERATURE_2026_ID,   "BMS Temperature",   printBmsTemperature2026Translation},
  {BMS_CELL_REQUEST_2026_ID,  "Cell Data Request", printCellDataRequest2026Translation},
  {BMS_CELL_RESPONSE_2026_ID, "Cell Data Response", printCellDataResponse2026Translation},
  {IMD_REQUEST_ID, "IMD Request", printImdRequestTranslation},
  {IMD_FALLBACK_REQUEST_ID, "IMD Fallback Request", printImdRequestTranslation},
  {IMD_RESPONSE_ID, "IMD Response", printImdResponseTranslation},
  {IMD_INFO_GENERAL_ID, "IMD Info General", printImdInfoGeneralTranslation},
  {IMD_INFO_ISOLATION_DETAIL_ID, "IMD Info Isolation Detail", printImdInfoIsolationDetailTranslation},
  {IMD_INFO_VOLTAGE_ID, "IMD Info Voltage", printImdInfoVoltageTranslation},
  {IMD_INFO_IT_SYSTEM_ID, "IMD Info IT-System", printImdInfoItSystemTranslation},
  {PM100_CMD_ID, "PM100 Command", printPm100CommandTranslation},
  {PM100_TEMPERATURE_1_FRAME_ID, "PM100 Temperatures #1", printPm100Temperatures1Translation},
  {PM100_TEMPERATURE_3_FRAME_ID, "PM100 Temperatures #3", printPm100Temperatures3Translation},
  {PM100_RPM_FRAME_ID, "PM100 Motor Position", printPm100MotorPositionTranslation},
  {PM100_CURRENT_INFO_FRAME_ID, "PM100 Current Information", printPm100CurrentInfoTranslation},
  {PM100_VOLTAGE_INFO_FRAME_ID, "PM100 Voltage Information", printPm100VoltageInfoTranslation},
  {PM100_INTERNAL_VOLTAGES_FRAME_ID, "PM100 Internal Voltages", printPm100InternalVoltagesTranslation},
  {PM100_FAULT_CODES_FRAME_ID, "PM100 Fault Codes", printPm100FaultCodesTranslation},
  {PM100_TORQUE_CAPABILITY_FRAME_ID, "PM100 Torque Capability", printPm100TorqueCapabilityTranslation}
};

static constexpr uint8_t CAN_FRAME_TRANSLATION_COUNT =
    sizeof(CAN_FRAME_TRANSLATIONS) / sizeof(CAN_FRAME_TRANSLATIONS[0]);

static inline uint16_t u16Le(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

static inline uint16_t u16Be(uint8_t hi, uint8_t lo) {
  return ((uint16_t)hi << 8) | (uint16_t)lo;
}

static inline uint32_t u32Le(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
  return (uint32_t)b0 | ((uint32_t)b1 << 8) |
         ((uint32_t)b2 << 16) | ((uint32_t)b3 << 24);
}

static inline int16_t s16Le(uint8_t lo, uint8_t hi) {
  return (int16_t)u16Le(lo, hi);
}

// IEEE-754 32-bit float, little-endian on the wire. The Teensy 4.1 is also
// little-endian, so the four bytes copy straight into a float. Used for the
// Energy Meter Current / Voltage / Energy signals.
static inline float f32Le(const uint8_t *b) {
  float value;
  memcpy(&value, b, sizeof(value));
  return value;
}

// ---------------- Setup ----------------
void setup() {
  startupFaultDelayStartMs = millis();

  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  setupFans();

  pinMode(READY_BUTTON_PIN, INPUT);
  readyButtonLast = readReadyButtonPressed();
  refreshRtdStateFromInputs();

  analogReadResolution(ADC_RESOLUTION_BITS);
  pinMode(APPS_1_PIN, INPUT);
  pinMode(APPS_2_PIN, INPUT);
  pinMode(BSE_1_PIN, INPUT);
  pinMode(BSE_2_PIN, INPUT);
  pinMode(HV_CURRENT_SENSE_PIN, INPUT);

  pinMode(BRAKE_LIGHT_PIN, OUTPUT);
  digitalWrite(BRAKE_LIGHT_PIN, LOW);

  // Pulldowns so a disconnected status line reads LOW and shows red.
  pinMode(BSPD_STATUS_PIN, INPUT_PULLDOWN);
  pinMode(IMD_STATUS_PIN, INPUT_PULLDOWN);
  pinMode(BMS_STATUS_PIN, INPUT_PULLDOWN);

  setupTssi();
  setupVehicleCan();
  setupPm100CommandTimer();

  while (!Serial && millis() < 1500) {}

  Serial.println("BERT26 VCU starting");
  Serial.print("[Startup] BMS/IMD fault detection delayed for ");
  Serial.print(STARTUP_BMS_IMD_FAULT_DELAY_MS);
  Serial.println(" ms");

  setupImu();

  setupDataLogging();

  setupDisplay();

  drawDashboard();
}

// ---------------- Power derate ----------------
void rollingWindowPush(RollingWindow &w, float value) {
  if (w.count == POWER_DERATE_WINDOW_SAMPLES) {
    w.sum -= w.buf[w.head];           // evict oldest before overwriting
  } else {
    w.count++;
  }
  w.buf[w.head] = value;
  w.sum += value;
  w.head = (uint8_t)((w.head + 1) % POWER_DERATE_WINDOW_SAMPLES);
}

bool rollingWindowAverage(const RollingWindow &w, float &out) {
  if (w.count == 0) return false;
  out = w.sum / (float)w.count;
  return true;
}

// Linear low-voltage power limit (kW) for a rolling-average minimum cell of
// avgMv: POWER_DERATE_CELL_MAX_KW at CELL_FULL_MV, 0 kW at CELL_ZERO_MV, and
// linear in between. At or above CELL_FULL_MV there is no voltage derate, so
// this returns CELL_MAX_KW (callers treat >= full-voltage as "no cell trip").
float cellDerateKw(float avgMv) {
  if (avgMv >= (float)POWER_DERATE_CELL_FULL_MV) return POWER_DERATE_CELL_MAX_KW;
  if (avgMv <= (float)POWER_DERATE_CELL_ZERO_MV) return 0.0f;
  float frac = (avgMv - (float)POWER_DERATE_CELL_ZERO_MV) /
               (float)(POWER_DERATE_CELL_FULL_MV - POWER_DERATE_CELL_ZERO_MV);
  return POWER_DERATE_CELL_MAX_KW * frac;
}

// Torque that produces powerDerateKw at the given shaft speed, clamped to the
// normal max. Below POWER_DERATE_MIN_RPM the cap exceeds MAX_TORQUE_NM, so the
// limiter has no effect and we simply return the unrestricted max.
float powerDerateTorqueCapNm(int16_t rpm) {
  if (powerDerateKw <= 0.0f) return 0.0f;
  int absRpm = abs((int)rpm);
  if (absRpm < POWER_DERATE_MIN_RPM) return MAX_TORQUE_NM;
  float cap = TORQUE_PER_KW_PER_RPM * powerDerateKw / (float)absRpm;
  if (cap > MAX_TORQUE_NM) cap = MAX_TORQUE_NM;
  return cap;
}

// Samples the protection signals on a fixed cadence and updates the rolling
// averages, then sets powerDerateActive when any trip condition is met.
void servicePowerDerate(uint32_t nowMs) {
  static uint32_t lastSampleMs = 0;
  if (nowMs - lastSampleMs < POWER_DERATE_SAMPLE_MS) return;
  lastSampleMs = nowMs;

  if (haveCellVoltageStats) rollingWindowPush(minCellMvWindow, (float)minCellVoltageMv);
  if (haveCellTempStats)    rollingWindowPush(battMaxTempWindow, (float)maxCellTempC);

  bool derate = false;
  float capKw = POWER_DERATE_CELL_MAX_KW;  // lowest cap across active trips

  // Low cell voltage: linear derate on the rolling-average minimum cell.
  float avgCellMv;
  if (rollingWindowAverage(minCellMvWindow, avgCellMv) &&
      avgCellMv < (float)POWER_DERATE_CELL_FULL_MV) {
    derate = true;
    float kw = cellDerateKw(avgCellMv);
    if (kw < capKw) capKw = kw;
  }

  if (havePm100MotorTemp &&
      (float)pm100MotorTempTenthsC / 10.0f > POWER_DERATE_MOTOR_TEMP_C) {
    derate = true;
    if (POWER_DERATE_KW < capKw) capKw = POWER_DERATE_KW;
  }

  float avgBattTemp;
  if (rollingWindowAverage(battMaxTempWindow, avgBattTemp) &&
      avgBattTemp > (float)POWER_DERATE_BATT_TEMP_C) {
    derate = true;
    if (POWER_DERATE_KW < capKw) capKw = POWER_DERATE_KW;
  }

  powerDerateKw = derate ? capKw : POWER_DERATE_CELL_MAX_KW;

  if (derate != powerDerateActive) {
    powerDerateActive = derate;
    if (derate) {
      Serial.print("[Power] Derate active; capping to ");
      Serial.print(powerDerateKw, 1);
      Serial.println(" kW");
    } else {
      Serial.println("[Power] Derate cleared; full power restored");
    }
  }
}

// ---------------- Main loop ----------------
void loop() {
  handleSerialConsoleInput();
  updateFanDutyFromTemps();
  serviceFans();
  receiveCanFrames();
  updatePedal();
  updateBrakePressureSensors();
  digitalWrite(BRAKE_LIGHT_PIN, braking ? HIGH : LOW);
  logAppsTelemetry();
  serviceReadyToDrive();
  serviceBuzzer();
  serviceTssi();
  serviceFaultBanner();
  handleTouch();

  uint32_t now = millis();
  if (bmsImdFaultDetectionArmed() && haveBmsStatus && (now - lastBmsRxMs > BMS_TIMEOUT_MS)) {
    if (!bmsTimedOut) {
      Serial.println("[BMS] Status timeout"); //,disabling torque
      showFaultBanner("BMS Timeout");
      screenDirty = true;
    }
    bmsTimedOut = true;
    bmsActive = false;
    bmsFault = true;
    dropReadyToDrive("BMS Timeout");
  }

  serviceEnergyMeterTimeout(now);

  if (haveInverterVoltageInfo && haveInverterCurrentInfo) {
    powerKw = (inverterDcBusVoltage * inverterDcBusCurrent) / 1000.0f;
  } else {
    powerKw = 0.0f;
  }
  speedMph = (int)roundf((float)motorRpm * MPH_PER_RPM);
  servicePowerDerate(now);
  updatePm100CommandState();
  servicePm100TxLogging();
  servicePm100BroadcastEnable();
  servicePm100FaultClear();
  serviceDataLogging();

  // Rewrite the entire screen every 2 minutes to wipe any accumulated SPI
  // glitches that the per-cell incremental redraw would otherwise leave behind.
  static uint32_t lastFullRedrawMs = 0;
  if (now - lastFullRedrawMs >= DASHBOARD_FULL_REDRAW_MS) {
    lastFullRedrawMs = now;
    dashboardNeedsFullRedraw = true;
    screenDirty = true;
  }

  static uint32_t lastDrawMs = 0;
  if (screenDirty || now - lastDrawMs >= SCREEN_PERIOD_MS) {
    lastDrawMs = now;
    screenDirty = false;
    drawDashboard();
  }
}

// ---------------- Fans ----------------
void setupFans() {
  pinMode(FAN_1_PIN, OUTPUT);
  pinMode(FAN_2_PIN, OUTPUT);
  analogWriteFrequency(FAN_1_PIN, FAN_PWM_FREQUENCY_HZ);
  analogWriteFrequency(FAN_2_PIN, FAN_PWM_FREQUENCY_HZ);
  serviceFans();
}

void updateFanDutyFromTemps() {
  if (fanManualOverride) {
    return;
  }

  // Drive the fans from the hotter of the PM100 motor and inverter temps.
  bool haveTemp = false;
  float tempC = 0.0f;
  if (haveInverterTemp) {
    tempC = (float)inverterTempTenthsC / 10.0f;
    haveTemp = true;
  }
  if (havePm100MotorTemp) {
    float motorTempC = (float)pm100MotorTempTenthsC / 10.0f;
    if (!haveTemp || motorTempC > tempC) {
      tempC = motorTempC;
    }
    haveTemp = true;
  }

  int duty;
  if (!haveTemp || tempC <= FAN_TEMP_MIN_C) {
    duty = FAN_DUTY_MIN_PERCENT;
  } else if (tempC >= FAN_TEMP_MAX_C) {
    duty = FAN_DUTY_MAX_PERCENT;
  } else {
    float span = FAN_TEMP_MAX_C - FAN_TEMP_MIN_C;
    float fraction = (tempC - FAN_TEMP_MIN_C) / span;
    duty = FAN_DUTY_MIN_PERCENT +
           (int)(fraction * (float)(FAN_DUTY_MAX_PERCENT - FAN_DUTY_MIN_PERCENT) + 0.5f);
  }

  fan1DutyPercent = duty;
  fan2DutyPercent = duty;
}

void serviceFans() {
  int duty1 = constrain(fan1DutyPercent, 0, 100);
  int duty2 = constrain(fan2DutyPercent, 0, 100);

  if (duty1 != fan1LastDutyPercent) {
    fan1LastDutyPercent = duty1;
    analogWrite(FAN_1_PIN, dutyPercentToFanPwmWrite(duty1));
  }

  if (duty2 != fan2LastDutyPercent) {
    fan2LastDutyPercent = duty2;
    analogWrite(FAN_2_PIN, dutyPercentToFanPwmWrite(duty2));
  }
}

uint8_t dutyPercentToFanPwmWrite(int dutyPercent) {
  int duty = constrain(dutyPercent, 0, 100);
  return (uint8_t)((duty * FAN_PWM_WRITE_MAX + 50) / 100);
}

// ---------------- Data logging ----------------
void setupDataLogging() {
  sdCardStarted = SD.begin(BUILTIN_SDCARD);
  if (!sdCardStarted) {
    Serial.println("[Log] No SD card found; data logging disabled");
    return;
  }

  logSessionNumber = nextLogSessionNumber();

  char name[16];
  snprintf(name, sizeof(name), "LOG%04lu.BIN", (unsigned long)logSessionNumber);

  LogFileHeader header;
  memcpy(header.magic, LOG_FILE_MAGIC, sizeof(header.magic));
  header.session = logSessionNumber;
  header.fastSize = sizeof(LogFastRecord);
  header.mediumSize = sizeof(LogMediumRecord);
  header.slowSize = sizeof(LogSlowRecord);
  header.imuSize = sizeof(LogImuRecord);

  sdLogFile = SD.open(name, FILE_WRITE);
  if (sdLogFile) {
    sdLogFile.write(&header, sizeof(header));
    sdLogFile.flush();
    sdLogActive = true;
    Serial.print("[Log] Logging to ");
    Serial.println(name);
  } else {
    Serial.print("[Log] Failed to create ");
    Serial.println(name);
  }
}

// One file per power cycle: next session = highest existing LOGnnnn.BIN + 1.
uint32_t nextLogSessionNumber() {
  uint32_t maxSession = 0;

  File root = SD.open("/");
  if (root) {
    maxSession = maxLogSessionInDir(root);
    root.close();
  }

  return maxSession + 1;
}

uint32_t maxLogSessionInDir(File dir) {
  uint32_t maxSession = 0;

  for (File entry = dir.openNextFile(); entry; entry = dir.openNextFile()) {
    unsigned int session = 0;
    if (sscanf(entry.name(), "LOG%u.BIN", &session) == 1 &&
        (uint32_t)session > maxSession) {
      maxSession = session;
    }
    entry.close();
  }

  return maxSession;
}

void serviceDataLogging() {
  if (!sdLogActive) {
    return;
  }

  static uint32_t lastFastMs = 0;
  static uint32_t lastMediumMs = 0;
  static uint32_t lastSlowMs = 0;
  static uint32_t lastSdFlushMs = 0;

  uint32_t now = millis();

  if (now - lastFastMs >= LOG_FAST_PERIOD_MS) {
    lastFastMs = now;
    writeFastLogRecord(now);
    writeImuLogRecord(now);
  }

  if (now - lastMediumMs >= LOG_MEDIUM_PERIOD_MS) {
    lastMediumMs = now;
    writeMediumLogRecord(now);
  }

  if (now - lastSlowMs >= LOG_SLOW_PERIOD_MS) {
    lastSlowMs = now;
    writeSlowLogRecord(now);
  }

  // Periodic flush bounds data loss at power-off to about one second.
  if (sdLogActive && now - lastSdFlushMs >= LOG_SD_FLUSH_PERIOD_MS) {
    lastSdFlushMs = now;
    flushSdLogBuffer();
    if (sdLogActive) {
      sdLogFile.flush();
    }
  }
}

uint16_t logStatusFlags() {
  uint16_t flags = 0;
  if (braking)                              flags |= 1u << 0;
  if (hardBraking)                          flags |= 1u << 1;
  if (bspcActive)                           flags |= 1u << 2;
  if (appsPlausible)                        flags |= 1u << 3;
  if (readyToDriveLatched)                  flags |= 1u << 4;
  if (driveMode == 'D')                     flags |= 1u << 5;
  if (bmsFault)                             flags |= 1u << 6;
  if (imdFault)                             flags |= 1u << 7;
  if (pm100Fault)                           flags |= 1u << 8;
  if (bmsTimedOut)                          flags |= 1u << 9;
  if (imdCanTimedOut)                       flags |= 1u << 10;
  if (!contactorsOpen())                    flags |= 1u << 11;
  if (digitalRead(BSPD_STATUS_PIN) == HIGH) flags |= 1u << 12;
  if (digitalRead(IMD_STATUS_PIN) == HIGH)  flags |= 1u << 13;
  if (digitalRead(BMS_STATUS_PIN) == HIGH)  flags |= 1u << 14;
  return flags;
}

void writeFastLogRecord(uint32_t nowMs) {
  rawHvCurrentLast = analogRead(HV_CURRENT_SENSE_PIN);

  LogFastRecord r;
  r.type = LOG_RECORD_FAST;
  r.ms = nowMs;
  r.rawApps1 = (uint16_t)rawApps1Last;
  r.rawApps2 = (uint16_t)rawApps2Last;
  r.rawBse1 = (uint16_t)rawBse1Last;
  r.rawBse2 = (uint16_t)rawBse2Last;
  r.rawHvCurrent = (uint16_t)rawHvCurrentLast;
  r.commandedTorqueDeciNm = (int16_t)roundf(commandedTorqueNm * 10.0f);
  r.motorRpm = motorRpm;
  r.busVoltageDeciV = (int16_t)roundf(inverterDcBusVoltage * 10.0f);
  r.busCurrentDeciA = (int16_t)roundf(inverterDcBusCurrent * 10.0f);
  r.statusFlags = logStatusFlags();

  appendSdLog(&r, sizeof(r));
}

void writeMediumLogRecord(uint32_t nowMs) {
  LogMediumRecord r;
  r.type = LOG_RECORD_MEDIUM;
  r.ms = nowMs;
  r.apps1PctX10 = (int16_t)roundf(apps1Pct * 10.0f);
  r.apps2PctX10 = (int16_t)roundf(apps2Pct * 10.0f);
  r.pedalPctX10 = (int16_t)roundf(pedalPct * 10.0f);
  r.desiredTorqueDeciNm = (int16_t)roundf(desiredTorqueNm * 10.0f);
  r.powerDeciKw = (int16_t)roundf(powerKw * 10.0f);
  r.speedMph = (int16_t)speedMph;
  r.glvCentiV = haveGlvVoltage ? (uint16_t)roundf(glvVoltage * 100.0f) : 0;
  r.inverterTempTenthsC = inverterTempTenthsC;
  r.motorTempTenthsC = pm100MotorTempTenthsC;
  r.socPct = stateOfChargePct;
  r.batteryTempC = batteryTempC;
  r.bmsFlags = bmsFlags;
  r.bms2026FaultCode = haveBms2026FaultCode ? bms2026FaultCode : 0;
  r.imdWarnAlarms = imdWarnAlarms;
  r.pm100PostFaults = pm100PostFaults;
  r.pm100RunFaults = pm100RunFaults;
  r.fanDutyPct = (uint8_t)constrain(fan1DutyPercent, 0, 100);
  r.rtdState = (uint8_t)rtdState;

  appendSdLog(&r, sizeof(r));
}

void writeSlowLogRecord(uint32_t nowMs) {
  LogSlowRecord r;
  r.type = LOG_RECORD_SLOW;
  r.ms = nowMs;
  r.minCellMv = minCellVoltageMv;
  r.maxCellMv = maxCellVoltageMv;
  r.avgCellMv = avgCellVoltageMv;
  r.minCellTempC = minCellTempC;
  r.maxCellTempC = maxCellTempC;
  r.avgCellTempC = avgCellTempC;
  r.imdRIsoKohm = imdRIsoKohm;
  r.imdRIsoStatus = imdRIsoStatus;
  r.validFlags = (haveCellVoltageStats ? 0x01 : 0) |
                 (haveCellTempStats ? 0x02 : 0) |
                 (haveGlvVoltage ? 0x04 : 0);

  appendSdLog(&r, sizeof(r));
}

// ---------------- IMU (MPU6050) ----------------
void setupImu() {
  Wire.begin();
  Wire.setClock(IMU_I2C_CLOCK_HZ);
  pinMode(IMU_INT_PIN, INPUT);

  imuPresent = initImu();
  imuLastInitAttemptMs = millis();
  Serial.println(imuPresent
                     ? "[IMU] MPU6050 detected; logging accel/gyro at 100 Hz"
                     : "[IMU] MPU6050 not responding; IMU records disabled");
}

bool initImu() {
  Wire.beginTransmission(MPU6050_I2C_ADDR);
  Wire.write(MPU6050_REG_WHO_AM_I);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom((int)MPU6050_I2C_ADDR, 1) != 1) {
    return false;
  }
  uint8_t whoAmI = Wire.read();
  if (whoAmI != MPU6050_WHO_AM_I_VALUE) {
    Serial.print("[IMU] Unexpected WHO_AM_I 0x");
    printCanHexByte(whoAmI);
    Serial.println();
    return false;
  }

  // Wake from sleep with the gyro X PLL as clock, then 1 kHz sampling behind
  // the 44/42 Hz DLPF so every 100 Hz log read sees fresh, filtered data.
  return writeImuRegister(MPU6050_REG_PWR_MGMT_1, 0x01) &&
         writeImuRegister(MPU6050_REG_SMPLRT_DIV, 0) &&
         writeImuRegister(MPU6050_REG_CONFIG, MPU6050_DLPF_44HZ) &&
         writeImuRegister(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_500DPS) &&
         writeImuRegister(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_4G) &&
         writeImuRegister(MPU6050_REG_INT_PIN_CFG, 0x10) &&  // clear INT on any read
         writeImuRegister(MPU6050_REG_INT_ENABLE, 0x01);     // data-ready out on pin 33
}

bool writeImuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

// Burst-reads accel, temp and gyro (registers 0x3B-0x48, big-endian pairs).
bool readImuSample() {
  Wire.beginTransmission(MPU6050_I2C_ADDR);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom((int)MPU6050_I2C_ADDR, 14) != 14) {
    return false;
  }

  uint8_t buf[14];
  for (uint8_t i = 0; i < sizeof(buf); i++) {
    buf[i] = (uint8_t)Wire.read();
  }

  for (uint8_t axis = 0; axis < 3; axis++) {
    imuAccelRaw[axis] = (int16_t)u16Be(buf[axis * 2], buf[axis * 2 + 1]);
    imuGyroRaw[axis] = (int16_t)u16Be(buf[8 + axis * 2], buf[9 + axis * 2]);
  }
  imuTempRaw = (int16_t)u16Be(buf[6], buf[7]);
  return true;
}

void writeImuLogRecord(uint32_t nowMs) {
  if (!imuPresent) {
    // Retry init occasionally so a sensor that comes back mid-session
    // (loose connector, late power) resumes logging without a reboot.
    if (nowMs - imuLastInitAttemptMs >= IMU_RETRY_PERIOD_MS) {
      imuLastInitAttemptMs = nowMs;
      imuPresent = initImu();
      if (imuPresent) {
        Serial.println("[IMU] MPU6050 detected; logging accel/gyro at 100 Hz");
      }
    }
    if (!imuPresent) {
      return;
    }
  }

  if (!readImuSample()) {
    Serial.println("[IMU] Read failed; will retry init");
    imuPresent = false;
    imuLastInitAttemptMs = nowMs;
    return;
  }

  LogImuRecord r;
  r.type = LOG_RECORD_IMU;
  r.ms = nowMs;
  r.accelX = imuAccelRaw[0];
  r.accelY = imuAccelRaw[1];
  r.accelZ = imuAccelRaw[2];
  r.gyroX = imuGyroRaw[0];
  r.gyroY = imuGyroRaw[1];
  r.gyroZ = imuGyroRaw[2];
  r.tempRaw = imuTempRaw;

  appendSdLog(&r, sizeof(r));
}

void appendSdLog(const void *data, size_t len) {
  if (!sdLogActive) {
    return;
  }

  if (sdLogBufferLen + len > sizeof(sdLogBuffer)) {
    flushSdLogBuffer();
    if (!sdLogActive) {
      return;
    }
  }

  memcpy(sdLogBuffer + sdLogBufferLen, data, len);
  sdLogBufferLen += len;
}

void flushSdLogBuffer() {
  if (!sdLogActive || sdLogBufferLen == 0) {
    sdLogBufferLen = 0;
    return;
  }

  size_t written = sdLogFile.write(sdLogBuffer, sdLogBufferLen);
  if (written != sdLogBufferLen) {
    Serial.println("[Log] SD write failed; SD logging stopped");
    sdLogFile.close();
    sdLogActive = false;
  }
  sdLogBufferLen = 0;
}

void listLogFiles() {
  if (!sdCardStarted) {
    Serial.println("[Log] SD card not available");
    return;
  }

  Serial.println("[Log] SD card:");
  File root = SD.open("/");
  if (root) {
    listLogDir(root);
    root.close();
  }
}

void listLogDir(File dir) {
  for (File entry = dir.openNextFile(); entry; entry = dir.openNextFile()) {
    if (!entry.isDirectory()) {
      Serial.print("  ");
      Serial.print(entry.name());
      Serial.print("  ");
      Serial.print((unsigned long)entry.size());
      Serial.println(" bytes");
    }
    entry.close();
  }
}

// Hex-dumps a session file over serial so logs can be exported without
// pulling the SD card; decode with tools/decode_vcu_log.py --hex.
void dumpLogFile(uint32_t session) {
  if (!sdCardStarted) {
    Serial.println("[Log] SD card not available");
    return;
  }

  char name[16];
  snprintf(name, sizeof(name), "LOG%04lu.BIN", (unsigned long)session);

  // Make sure everything buffered for the current session is on the card.
  flushSdLogBuffer();
  if (sdLogActive) {
    sdLogFile.flush();
  }

  File f = SD.open(name, FILE_READ);
  if (!f) {
    Serial.print("[Log] ");
    Serial.print(name);
    Serial.println(" not found");
    return;
  }

  Serial.print("[Log] DUMP BEGIN ");
  Serial.print(name);
  Serial.print(" ");
  Serial.println((unsigned long)f.size());

  uint8_t buf[32];
  int n;
  while ((n = f.read(buf, sizeof(buf))) > 0) {
    for (int i = 0; i < n; i++) {
      printCanHexByte(buf[i]);
    }
    Serial.println();
  }
  f.close();

  Serial.println("[Log] DUMP END");
}

// ---------------- Display / touch ----------------
void setupDisplay() {
  SPI.setMOSI(TFT_MOSI);
  SPI.setMISO(TFT_MISO);
  SPI.setSCK(TFT_SCK);

  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, HIGH);

  pinMode(TFT_LED, OUTPUT);
  analogWriteFrequency(TFT_LED, 20000);
  analogWrite(TFT_LED, backlightBrightness);

  resetDisplay();

  ts.begin();
  ts.setRotation(1);

  tft.init(320, 480);
  tft.setRotation(135);
  tft.fillScreen(ST7735_BLACK);
  dashboardNeedsFullRedraw = true;
  screenDirty = true;
}

void resetDisplay() {
  digitalWrite(TFT_RST, HIGH);
  delay(20);
  digitalWrite(TFT_RST, LOW);
  delay(20);
  digitalWrite(TFT_RST, HIGH);
  delay(150);
}

// Line-based serial console: 0-100 = manual fan duty, 'a' = auto fans,
// 'l' = list log files, 'd<n>' = hex-dump log session n for export.
void handleSerialConsoleInput() {
  static char input[12];
  static uint8_t index = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (index > 0) {
        input[index] = '\0';
        index = 0;
        handleSerialConsoleCommand(input);
      }
    } else if (index < sizeof(input) - 1) {
      input[index++] = c;
    } else {
      index = 0;
    }
  }
}

void handleSerialConsoleCommand(const char *cmd) {
  if (strcmp(cmd, "a") == 0 || strcmp(cmd, "A") == 0) {
    fanManualOverride = false;
    Serial.println("[Fans] Automatic temperature control resumed");
    return;
  }

  if (strcmp(cmd, "l") == 0 || strcmp(cmd, "L") == 0) {
    listLogFiles();
    return;
  }

  if ((cmd[0] == 'd' || cmd[0] == 'D') && cmd[1] >= '0' && cmd[1] <= '9') {
    dumpLogFile((uint32_t)atoi(cmd + 1));
    return;
  }

  bool allDigits = cmd[0] != '\0';
  for (const char *p = cmd; *p; p++) {
    if (*p < '0' || *p > '9') {
      allDigits = false;
      break;
    }
  }

  if (allDigits) {
    int value = atoi(cmd);
    // Manual override: enter 0-100 to force both fan PWM duties.
    if (value >= 0 && value <= 100) {
      fanManualOverride = true;
      fan1DutyPercent = value;
      fan2DutyPercent = value;
      Serial.print("[Fans] Manual override duty set to ");
      Serial.print(value);
      Serial.println("% (send 'a' for auto)");
      return;
    }
  }

  Serial.println("[Console] Commands: 0-100 fan duty, 'a' auto fans, 'l' list logs, 'd<n>' dump log n");
}

void handleTouch() {
  // RTD owns driveMode: D while ready to drive, N in every other RTD state.
  return;

  if (!ts.touched()) {
    return;
  }

  TS_Point p = ts.getPoint();
  char newMode = checkIfTouchedRaw(p.x, p.y);
  if (newMode && newMode != driveMode) {
    setDriveMode(newMode);

    Serial.print("[Touch] Drive mode ");
    Serial.println(driveMode);
  }
}

char checkIfTouchedRaw(int x, int y) {
  const int yMin = 2650;
  const int yMax = 3500;

  if (x > 950 && x < 1550 && y > yMin && y < yMax) return 'D';
  if (x > 1900 && x < 2500 && y > yMin && y < yMax) return 'N';
  if (x > 2900 && x < 3500 && y > yMin && y < yMax) return 'R';

  return 0;
}

void drawDashboard() {
  // Primary-tile value caches (Zone B).
  static char lastSpeed[12] = "";   static uint16_t lastSpeedColor = ST7735_WHITE;
  static char lastBatT[12] = "";    static uint16_t lastBatTColor = ST7735_WHITE;
  static char lastMtrT[12] = "";    static uint16_t lastMtrTColor = ST7735_WHITE;
  static char lastSpread[12] = "";  static uint16_t lastSpreadColor = ST7735_WHITE;
  static char lastPackV[12] = "";   static uint16_t lastPackVColor = ST7735_WHITE;
  static char lastMode[12] = "";    static uint16_t lastModeColor = ST7735_WHITE;

  // Diagnostics-strip caches (Zone C).
  static char lastPedal[16] = "";   static uint16_t lastPedalColor = ST7735_WHITE;
  static char lastTorque[16] = "";  static uint16_t lastTorqueColor = ST7735_WHITE;
  static char lastPower[16] = "";   static uint16_t lastPowerColor = ST7735_WHITE;
  static char lastSoc[16] = "";     static uint16_t lastSocColor = ST7735_WHITE;
  static char lastRpm[16] = "";     static uint16_t lastRpmColor = ST7735_WHITE;
  static char lastBms[24] = "";     static uint16_t lastBmsColor = ST7735_WHITE;
  static char lastInv[24] = "";     static uint16_t lastInvColor = ST7735_WHITE;
  static char lastCtr[16] = "";     static uint16_t lastCtrColor = ST7735_WHITE;
  static char lastVmn[16] = "";     static uint16_t lastVmnColor = ST7735_WHITE;
  static char lastVmx[16] = "";     static uint16_t lastVmxColor = ST7735_WHITE;
  static char lastTmn[16] = "";     static uint16_t lastTmnColor = ST7735_WHITE;
  static char lastTmx[16] = "";     static uint16_t lastTmxColor = ST7735_WHITE;
  static char lastGlv[16] = "";     static uint16_t lastGlvColor = ST7735_WHITE;
  static char lastPack[16] = "";    static uint16_t lastPackColor = ST7735_WHITE;
  static char lastInvt[16] = "";    static uint16_t lastInvtColor = ST7735_WHITE;
  static char lastFan[16] = "";     static uint16_t lastFanColor = ST7735_WHITE;
  static char lastEmV[24] = "";     static uint16_t lastEmVColor = ST7735_WHITE;
  static char lastEmT[32] = "";     static uint16_t lastEmTColor = ST7735_WHITE;
  static char lastEmF[24] = "";     static uint16_t lastEmFColor = ST7735_WHITE;
  static char lastFault[24] = "";   static uint16_t lastFaultColor = ST7735_WHITE;

  bool force = dashboardNeedsFullRedraw;

  if (force) {
    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(2);
    tft.setCursor(6, 6);
    tft.print("BERT26");

    // Six primary tiles: 3 columns x 2 rows.
    drawTileFrame(2, 32, "SPEED");
    drawTileFrame(162, 32, "BAT T");
    drawTileFrame(322, 32, "MTR T");
    drawTileFrame(2, 136, "SPREAD");
    drawTileFrame(162, 136, "PACK V");
    drawTileFrame(322, 136, "MODE");

    dashboardNeedsFullRedraw = false;
  }

  char value[32];

  // ---- Header: power-derate banner ----
  drawDerateBanner(force);

  // ---- Zone B: large primary tiles (value cell = tileX+6, tileY+34) ----
  snprintf(value, sizeof(value), "%d", speedMph);
  drawCachedDashboardText(8, 66, 144, 48, 5, value, ST7735_WHITE,
                          lastSpeed, sizeof(lastSpeed), lastSpeedColor, force);

  bool battTempValid = haveCellTempStats && maxCellTempC != CELL_TEMP_UNKNOWN_C;
  if (battTempValid) {
    snprintf(value, sizeof(value), "%dC", (int)maxCellTempC);
  } else {
    snprintf(value, sizeof(value), "--");
  }
  drawCachedDashboardText(168, 66, 144, 48, 4, value,
                          batteryTempColor((int)maxCellTempC, battTempValid),
                          lastBatT, sizeof(lastBatT), lastBatTColor, force);

  {
    float motorC = (float)pm100MotorTempTenthsC / 10.0f;
    if (havePm100MotorTemp) {
      snprintf(value, sizeof(value), "%.0fC", motorC);
    } else {
      snprintf(value, sizeof(value), "--");
    }
    drawCachedDashboardText(328, 66, 144, 48, 4, value,
                            motorTempColor(motorC, havePm100MotorTemp),
                            lastMtrT, sizeof(lastMtrT), lastMtrTColor, force);
  }

  {
    int spread = (int)maxCellVoltageMv - (int)minCellVoltageMv;
    if (haveCellVoltageStats) {
      snprintf(value, sizeof(value), "%dmV", spread);
    } else {
      snprintf(value, sizeof(value), "--");
    }
    drawCachedDashboardText(8, 170, 144, 48, 4, value,
                            cellSpreadColor(spread, haveCellVoltageStats),
                            lastSpread, sizeof(lastSpread), lastSpreadColor, force);
  }

  if (haveInverterVoltageInfo) {
    snprintf(value, sizeof(value), "%.0fV", inverterDcBusVoltage);
  } else {
    snprintf(value, sizeof(value), "--");
  }
  drawCachedDashboardText(168, 170, 144, 48, 4, value,
                          haveInverterVoltageInfo ? ST7735_WHITE : ST7735_YELLOW,
                          lastPackV, sizeof(lastPackV), lastPackVColor, force);

  {
    bool inDrive = (driveMode == 'D');
    const char *modeText = inDrive ? "DRIVE" : "NEUTRAL";
    drawCachedDashboardText(328, 172, 144, 28, 3, modeText,
                            inDrive ? ST7735_GREEN : ST7735_WHITE,
                            lastMode, sizeof(lastMode), lastModeColor, force);
  }

  // ---- Zone C: diagnostics strip (size-1 grid) ----
  const int dc0 = 4, dc1 = 124, dc2 = 244, dc3 = 364;
  const int dw = 116, dh = 9;
  const int r0 = 242, r1 = 254, r2 = 266, r3 = 278, r4 = 290, r5 = 303;

  bool faultDetectionArmed = bmsImdFaultDetectionArmed();
  bool bms2026DisplayFault = haveBms2026FaultCode && bms2026FaultCode != 0;

  // r0: pedal, commanded torque, power, SOC
  snprintf(value, sizeof(value), "PED:%.0f%%", pedalPct);
  drawCachedDashboardText(dc0, r0, dw, dh, 1, value, appsPlausible ? ST7735_GREEN : ST7735_RED,
                          lastPedal, sizeof(lastPedal), lastPedalColor, force);

  snprintf(value, sizeof(value), "CMD:%.1f", commandedTorqueNm);
  drawCachedDashboardText(dc1, r0, dw, dh, 1, value, ST7735_WHITE,
                          lastTorque, sizeof(lastTorque), lastTorqueColor, force);

  bool haveInverterPower = haveInverterVoltageInfo && haveInverterCurrentInfo;
  if (haveInverterPower) {
    snprintf(value, sizeof(value), "PWR:%dkW", (int)roundf(powerKw));
  } else {
    snprintf(value, sizeof(value), "PWR:--");
  }
  drawCachedDashboardText(dc2, r0, dw, dh, 1, value, haveInverterPower ? ST7735_WHITE : ST7735_YELLOW,
                          lastPower, sizeof(lastPower), lastPowerColor, force);

  snprintf(value, sizeof(value), "SOC:%d%%", stateOfChargePct);
  drawCachedDashboardText(dc3, r0, dw, dh, 1, value, ST7735_GREEN,
                          lastSoc, sizeof(lastSoc), lastSocColor, force);

  // r1: RPM, BMS status, inverter status, contactor
  snprintf(value, sizeof(value), "RPM:%d", motorRpm);
  drawCachedDashboardText(dc0, r1, dw, dh, 1, value, ST7735_WHITE,
                          lastRpm, sizeof(lastRpm), lastRpmColor, force);

  const char *bmsText = "WAIT";
  uint16_t bmsColor = ST7735_YELLOW;
  if (!faultDetectionArmed) {
    bmsText = "STARTUP"; bmsColor = ST7735_YELLOW;
  } else if (bmsTimedOut) {
    bmsText = "TIMEOUT"; bmsColor = ST7735_RED;
  } else if (bmsFault || bms2026DisplayFault) {
    bmsText = "FAULT"; bmsColor = ST7735_RED;
  } else if (bmsActive) {
    bmsText = "ACTIVE"; bmsColor = ST7735_GREEN;
  }
  snprintf(value, sizeof(value), "BMS:%s", bmsText);
  drawCachedDashboardText(dc1, r1, dw, dh, 1, value, bmsColor,
                          lastBms, sizeof(lastBms), lastBmsColor, force);

  {
    const char *invText;
    uint16_t invColor;
    if (!havePm100FaultStatus) {
      invText = "--"; invColor = ST7735_YELLOW;
    } else if (pm100Fault) {
      invText = firstPm100FaultName(pm100PostFaults, pm100RunFaults); invColor = ST7735_RED;
    } else {
      invText = "OK"; invColor = ST7735_GREEN;
    }
    snprintf(value, sizeof(value), "INV:%s", invText);
    drawCachedDashboardText(dc2, r1, dw, dh, 1, value, invColor,
                            lastInv, sizeof(lastInv), lastInvColor, force);
  }

  {
    bool ctrClosed = haveBmsStatus && (bmsFlags & 0x02) != 0;
    const char *ctrText = !haveBmsStatus ? "--" : (ctrClosed ? "CLOSED" : "OPEN");
    uint16_t ctrColor = !haveBmsStatus ? ST7735_YELLOW : (ctrClosed ? ST7735_GREEN : ST7735_RED);
    snprintf(value, sizeof(value), "CTR:%s", ctrText);
    drawCachedDashboardText(dc3, r1, dw, dh, 1, value, ctrColor,
                            lastCtr, sizeof(lastCtr), lastCtrColor, force);
  }

  // r2: cell voltage min/max, cell temp min/max
  if (haveCellVoltageStats) snprintf(value, sizeof(value), "Vmn:%.3f", minCellVoltageMv / 1000.0f);
  else snprintf(value, sizeof(value), "Vmn:--");
  drawCachedDashboardText(dc0, r2, dw, dh, 1, value, haveCellVoltageStats ? ST7735_WHITE : ST7735_YELLOW,
                          lastVmn, sizeof(lastVmn), lastVmnColor, force);

  if (haveCellVoltageStats) snprintf(value, sizeof(value), "Vmx:%.3f", maxCellVoltageMv / 1000.0f);
  else snprintf(value, sizeof(value), "Vmx:--");
  drawCachedDashboardText(dc1, r2, dw, dh, 1, value, haveCellVoltageStats ? ST7735_WHITE : ST7735_YELLOW,
                          lastVmx, sizeof(lastVmx), lastVmxColor, force);

  if (haveCellTempStats && minCellTempC != CELL_TEMP_UNKNOWN_C) snprintf(value, sizeof(value), "Tmn:%dC", (int)minCellTempC);
  else snprintf(value, sizeof(value), "Tmn:--");
  drawCachedDashboardText(dc2, r2, dw, dh, 1, value, haveCellTempStats ? ST7735_WHITE : ST7735_YELLOW,
                          lastTmn, sizeof(lastTmn), lastTmnColor, force);

  if (haveCellTempStats && maxCellTempC != CELL_TEMP_UNKNOWN_C) snprintf(value, sizeof(value), "Tmx:%dC", (int)maxCellTempC);
  else snprintf(value, sizeof(value), "Tmx:--");
  drawCachedDashboardText(dc3, r2, dw, dh, 1, value, haveCellTempStats ? ST7735_WHITE : ST7735_YELLOW,
                          lastTmx, sizeof(lastTmx), lastTmxColor, force);

  // r3: GLV, BMS pack voltage, inverter temp, fan
  {
    uint16_t glvColor;
    if (!haveGlvVoltage) {
      snprintf(value, sizeof(value), "GLV:--"); glvColor = ST7735_YELLOW;
    } else {
      snprintf(value, sizeof(value), "GLV:%.1fV", glvVoltage);
      glvColor = (glvVoltage < GLV_LOW_VOLTAGE_WARN_V) ? ST7735_RED : ST7735_WHITE;
    }
    drawCachedDashboardText(dc0, r3, dw, dh, 1, value, glvColor,
                            lastGlv, sizeof(lastGlv), lastGlvColor, force);
  }

  if (haveCellVoltageStats) snprintf(value, sizeof(value), "PACK:%.0fV", packVoltage);
  else snprintf(value, sizeof(value), "PACK:--");
  drawCachedDashboardText(dc1, r3, dw, dh, 1, value, haveCellVoltageStats ? ST7735_WHITE : ST7735_YELLOW,
                          lastPack, sizeof(lastPack), lastPackColor, force);

  if (haveInverterTemp) snprintf(value, sizeof(value), "INVT:%.1fC", inverterTempTenthsC / 10.0f);
  else snprintf(value, sizeof(value), "INVT:--");
  drawCachedDashboardText(dc2, r3, dw, dh, 1, value, haveInverterTemp ? ST7735_WHITE : ST7735_YELLOW,
                          lastInvt, sizeof(lastInvt), lastInvtColor, force);

  {
    int fan1Duty = constrain(fan1DutyPercent, 0, 100);
    int fan2Duty = constrain(fan2DutyPercent, 0, 100);
    snprintf(value, sizeof(value), "FAN:%d/%d", fan1Duty, fan2Duty);
    drawCachedDashboardText(dc3, r3, dw, dh, 1, value, ST7735_WHITE,
                            lastFan, sizeof(lastFan), lastFanColor, force);
  }

  // r4: energy meter V / T / status (shutdown chips occupy the far right)
  {
    bool emComms = (haveEmMeasurement || haveEmStatus || haveEmTemps) && !emTimedOut;
    uint16_t liveColor = emComms ? ST7735_WHITE : ST7735_YELLOW;

    if (haveEmMeasurement) snprintf(value, sizeof(value), "EMV:%.1fV", emVoltageV);
    else snprintf(value, sizeof(value), "EMV:--");
    drawCachedDashboardText(dc0, r4, dw, dh, 1, value, liveColor,
                            lastEmV, sizeof(lastEmV), lastEmVColor, force);

    if (haveEmTempSummary) {
      snprintf(value, sizeof(value), "EMT:%u %d-%dC", emNumSensors,
               (int)((float)emMinTempRaw * EM_TEMP_SCALE_C),
               (int)((float)emMaxTempRaw * EM_TEMP_SCALE_C));
    } else {
      snprintf(value, sizeof(value), "EMT:--");
    }
    drawCachedDashboardText(dc1, r4, dw, dh, 1, value, liveColor,
                            lastEmT, sizeof(lastEmT), lastEmTColor, force);

    uint16_t emColor;
    const char *emText = emStatusSummaryText(emColor);
    snprintf(value, sizeof(value), "EM:%s", emText);
    drawCachedDashboardText(dc2, r4, 112, dh, 1, value, emColor,
                            lastEmF, sizeof(lastEmF), lastEmFColor, force);
  }

  // r5: wide fault text (left of the shutdown chips)
  {
    const char *faultText = dashboardFaultText();
    bool faultActive = pm100Fault ||
                       (faultDetectionArmed &&
                        (anyFaultActive() || bmsTimedOut || imdCanTimedOut || bms2026DisplayFault));
    uint16_t faultColor = faultActive ? ST7735_RED : (faultDetectionArmed ? ST7735_GREEN : ST7735_YELLOW);
    snprintf(value, sizeof(value), "FAULT:%s", faultText);
    drawCachedDashboardText(dc0, r5, 350, dh, 1, value, faultColor,
                            lastFault, sizeof(lastFault), lastFaultColor, force);
  }

  drawShutdownStatusIndicators(force);

  if (faultBannerVisible) {
    drawFaultBanner(force);
  }
}

void formatCellVoltageText(char *buffer, size_t bufferSize, uint16_t millivolts, bool valid) {
  if (!valid) {
    snprintf(buffer, bufferSize, "--");
    return;
  }

  snprintf(buffer, bufferSize, "%.3fV", (float)millivolts / 1000.0f);
}

void formatCellTempText(char *buffer, size_t bufferSize, int16_t tempC, bool valid) {
  if (!valid || tempC == CELL_TEMP_UNKNOWN_C) {
    snprintf(buffer, bufferSize, "--");
    return;
  }

  snprintf(buffer, bufferSize, "%dC", (int)tempC);
}

void formatTemperatureTenthsText(char *buffer, size_t bufferSize, int16_t tempTenthsC, bool valid) {
  if (!valid) {
    snprintf(buffer, bufferSize, "--");
    return;
  }

  snprintf(buffer, bufferSize, "%.1fC", (float)tempTenthsC / 10.0f);
}

const char *dashboardFaultText() {
  DashboardFaultList faults;
  collectDashboardFaults(faults);

  if (faults.count == 0) {
    if (!bmsImdFaultDetectionArmed()) {
      return "Startup Wait";
    }
    return "No Fault";
  }

  if (faults.count == 1) {
    return faults.texts[0];
  }

  uint8_t faultIndex = (millis() / FAULT_DISPLAY_CYCLE_MS) % faults.count;
  return faults.texts[faultIndex];
}

void collectDashboardFaults(DashboardFaultList &faults) {
  faults.count = 0;
  bool faultDetectionArmed = bmsImdFaultDetectionArmed();

  if (faultDetectionArmed && bmsTimedOut) {
    appendDashboardFault(faults, "BMS Timeout");
  }

  if (faultDetectionArmed && imdCanTimedOut) {
    appendDashboardFault(faults, "IMD Timeout");
  }

  if (faultDetectionArmed) {
    if (haveBms2026FaultCode && bms2026FaultCode != 0) {
      appendDashboardFault(faults, bmsTemperatureFaultCodeName(bms2026FaultCode));
    } else if (bmsFault && haveBmsStatus && !haveBms2026FaultCode) {
      appendDashboardFault(faults, "BMS Fault");
    }
  }

  if (faultDetectionArmed && imdFault && (imdWarnAlarms != 0 || !imdCanTimedOut)) {
    uint8_t beforeCount = faults.count;
    appendTssiFaultNames(faults, imdWarnAlarms);
    if (faults.count == beforeCount) {
      appendDashboardFault(faults, "IMD Fault");
    }
  }

  if (havePm100FaultStatus && pm100Fault) {
    uint8_t beforeCount = faults.count;
    appendPm100FaultNames(faults, pm100PostFaults, pm100RunFaults);
    if (faults.count == beforeCount) {
      appendDashboardFault(faults, "Inv Fault");
    }
  }

  if (!appsPlausible) {
    appendDashboardFault(faults, appsFaultText);
  }

  if (bspcActive) {
    appendDashboardFault(faults, "BSPC Zero Trq");
  }
}

void appendDashboardFault(DashboardFaultList &faults, const char *text) {
  if (faults.count >= DASHBOARD_MAX_FAULT_TEXTS || text == nullptr || text[0] == '\0') {
    return;
  }

  faults.texts[faults.count++] = text;
}

void appendTssiFaultNames(DashboardFaultList &faults, uint16_t bits) {
  if (bits & (1u << 0)) appendDashboardFault(faults, "DeviceError");
  if (bits & (1u << 1)) appendDashboardFault(faults, "HV+ConnFail");
  if (bits & (1u << 2)) appendDashboardFault(faults, "HV-ConnFail");
  if (bits & (1u << 3)) appendDashboardFault(faults, "EarthConnFail");
  if (bits & (1u << 4)) appendDashboardFault(faults, "IsoAlarm");
  if (bits & (1u << 7)) appendDashboardFault(faults, "Unbalance");
  if (bits & (1u << 8)) appendDashboardFault(faults, "Undervolt");
  if (bits & (1u << 9)) appendDashboardFault(faults, "UnsafeStart");
  if (bits & (1u << 10)) appendDashboardFault(faults, "EarthliftOpen");
  if (bits & (1u << 5)) appendDashboardFault(faults, "IsoWarning");
  if (bits & (1u << 6)) appendDashboardFault(faults, "IsoOutdated");
}

void appendPm100FaultNames(DashboardFaultList &faults, uint32_t postFaults, uint32_t runFaults) {
  for (uint8_t bit = 0; bit < 32; bit++) {
    if (postFaults & (1UL << bit)) {
      appendDashboardFault(faults, pm100PostFaultName(bit));
    }
  }

  for (uint8_t bit = 0; bit < 32; bit++) {
    if (runFaults & (1UL << bit)) {
      appendDashboardFault(faults, pm100RunFaultName(bit));
    }
  }
}

const char *firstTssiFaultName(uint16_t bits) {
  if (bits & (1u << 0)) return "DeviceError";
  if (bits & (1u << 1)) return "HV+ConnFail";
  if (bits & (1u << 2)) return "HV-ConnFail";
  if (bits & (1u << 3)) return "EarthConnFail";
  if (bits & (1u << 4)) return "IsoAlarm";
  if (bits & (1u << 7)) return "Unbalance";
  if (bits & (1u << 8)) return "Undervolt";
  if (bits & (1u << 9)) return "UnsafeStart";
  if (bits & (1u << 10)) return "EarthliftOpen";
  if (bits & (1u << 5)) return "IsoWarning";
  if (bits & (1u << 6)) return "IsoOutdated";
  return "Fault";
}

const char *firstPm100FaultName(uint32_t postFaults, uint32_t runFaults) {
  for (uint8_t bit = 0; bit < 32; bit++) {
    if (postFaults & (1UL << bit)) return pm100PostFaultName(bit);
  }
  for (uint8_t bit = 0; bit < 32; bit++) {
    if (runFaults & (1UL << bit)) return pm100RunFaultName(bit);
  }
  return "No Fault";
}

const char *bmsTemperatureFaultCodeName(uint8_t faultCode) {
  switch (faultCode) {
    case 0:  return "No Fault";
    case 1:  return "Cell UV";
    case 2:  return "Cell OV";
    case 3:  return "Pack UV";
    case 4:  return "Pack OV";
    case 5:  return "Over Temp";
    case 6:  return "Cell Spread";
    case 7:  return "Prchg Timeout";
    case 8:  return "Prchg Fast";
    case 9:  return "Pack V Sense";
    case 10: return "Chgr Comms";
    case 11: return "BQ Init Fail";
    case 12: return "BQ Comms Fail";
    default: return "BMS Fault";
  }
}

const char *pm100PostFaultName(uint8_t bit) {
  switch (bit) {
    case 0: return "POST GateDesat";
    case 1: return "POST OverCur";
    case 2: return "POST AccelShrt";
    case 3: return "POST AccelOpen";
    case 4: return "POST CurSensLo";
    case 5: return "POST CurSensHi";
    case 6: return "POST ModTempLo";
    case 7: return "POST ModTempHi";
    case 8: return "POST CtrlTmpLo";
    case 9: return "POST CtrlTmpHi";
    case 10: return "POST GateTmpLo";
    case 11: return "POST GateTmpHi";
    case 12: return "POST 5V Low";
    case 13: return "POST 5V High";
    case 14: return "POST 12V Low";
    case 15: return "POST 12V High";
    case 16: return "POST 2.5V Lo";
    case 17: return "POST 2.5V Hi";
    case 18: return "POST 1.5V Lo";
    case 19: return "POST 1.5V Hi";
    case 20: return "POST DCBusHi";
    case 21: return "POST DCBusLo";
    case 22: return "POST PrechTmo";
    case 23: return "POST PrechVolt";
    case 24: return "POST EEPROMSum";
    case 25: return "POST EEPROMRg";
    case 26: return "POST EEPROMUpd";
    case 27: return "POST HWBusOV";
    case 28: return "POST GateInit";
    case 29: return "POST Reserved";
    case 30: return "POST BrakeShrt";
    case 31: return "POST BrakeOpen";
    default: return "POST Fault";
  }
}

const char *pm100RunFaultName(uint8_t bit) {
  switch (bit) {
    case 0: return "Mtr Overspeed";
    case 1: return "Inv Overcur";
    case 2: return "Inv Overvolt";
    case 3: return "Inv Overtemp";
    case 4: return "Accel Short";
    case 5: return "Accel Open";
    case 6: return "Dir Cmd Fault";
    case 7: return "Inv Resp Tmo";
    case 8: return "Gate/Desat";
    case 9: return "HW Overcur";
    case 10: return "Undervolt";
    case 11: return "CAN Cmd Lost";
    case 12: return "Mtr Overtemp";
    case 13: return "Run Reserved45";
    case 14: return "Run Reserved46";
    case 15: return "Run Reserved47";
    case 16: return "Brake Short";
    case 17: return "Brake Open";
    case 18: return "ModA Overtemp";
    case 19: return "ModB Overtemp";
    case 20: return "ModC Overtemp";
    case 21: return "PCB Overtemp";
    case 22: return "Gate1 Overtemp";
    case 23: return "Gate2 Overtemp";
    case 24: return "Gate3 Overtemp";
    case 25: return "Cur Sensor";
    case 26: return "Gate Overvolt";
    case 27: return "HWBusOV Gen3";
    case 28: return "HWBusOV Gen5";
    case 29: return "Run Reserved61";
    case 30: return "Resolver Disc";
    case 31: return "Run Reserved63";
    default: return "Inv Fault";
  }
}

void drawDashboardLabel(int x, int y, const char *label) {
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(x, y);
  tft.print(label);
  tft.print(": ");
}

// One 156x100 primary tile: rounded border + size-2 title in the top-left.
// Drawn only on a full redraw; the value cell inside is repainted incrementally.
void drawTileFrame(int x, int y, const char *label) {
  tft.drawRoundRect(x, y, 156, 100, 6, ST7735_WHITE);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(x + 6, y + 6);
  tft.print(label);
}

// Red "DERATE <n>kW" banner in the header while the power limiter is active;
// cleared to black when not. Custom drawn (filled box, not the text cache).
// Redraws on the active/inactive transition and whenever the shown (rounded)
// kW changes, so the linear voltage derate is reflected live.
void drawDerateBanner(bool force) {
  static bool lastDerate = false;
  static int  lastShownKw = -1;
  static bool initialized = false;
  int shownKw = (int)roundf(powerDerateKw);
  if (!force && initialized && powerDerateActive == lastDerate &&
      (!powerDerateActive || shownKw == lastShownKw)) {
    return;
  }
  initialized = true;
  lastDerate = powerDerateActive;
  lastShownKw = shownKw;

  const int x = 300, y = 2, w = 176, h = 24;
  if (powerDerateActive) {
    char banner[20];
    snprintf(banner, sizeof(banner), "DERATE %dkW", shownKw);
    tft.fillRect(x, y, w, h, ST7735_RED);
    tft.setTextSize(2);
    tft.setTextColor(ST7735_BLACK);
    tft.setCursor(x + 8, y + 5);
    tft.print(banner);
    tft.setTextColor(ST7735_WHITE);
  } else {
    tft.fillRect(x, y, w, h, ST7735_BLACK);
  }
}

// Color bands (user-specified). Invalid/no-data is shown yellow.
uint16_t motorTempColor(float tempC, bool valid) {
  if (!valid) return ST7735_YELLOW;
  if (tempC >= 95.0f) return ST7735_RED;
  if (tempC >= 65.0f) return ST7735_YELLOW;
  return ST7735_BLUE;
}

uint16_t batteryTempColor(int tempC, bool valid) {
  if (!valid) return ST7735_YELLOW;
  if (tempC > 45) return ST7735_RED;
  if (tempC >= 30) return ST7735_YELLOW;
  return ST7735_BLUE;
}

uint16_t cellSpreadColor(int spreadMv, bool valid) {
  if (!valid) return ST7735_YELLOW;
  if (spreadMv > 200) return ST7735_RED;
  if (spreadMv >= 100) return ST7735_YELLOW;
  return ST7735_GREEN;
}

void drawCachedDashboardText(int x, int y, int w, int h, uint8_t textSize,
                             const char *value, uint16_t valueColor,
                             char *lastValue, size_t lastValueSize,
                             uint16_t &lastColor, bool force) {
  if (!force && strcmp(value, lastValue) == 0 && valueColor == lastColor) {
    return;
  }

  tft.fillRect(x, y, w, h, ST7735_BLACK);
  tft.setTextSize(textSize);
  tft.setTextColor(valueColor);
  tft.setCursor(x, y);
  tft.print(value);
  tft.setTextColor(ST7735_WHITE);

  strncpy(lastValue, value, lastValueSize - 1);
  lastValue[lastValueSize - 1] = '\0';
  lastColor = valueColor;
}

void drawDriveButtons(bool force) {
  static char lastDriveMode = 0;

  if (!force && driveMode == lastDriveMode) {
    return;
  }

  int y = (int)(tft.height() * 0.72f);
  int buttonW = 72;
  int buttonH = 54;
  int startX = 42;
  int gap = 24;

  // Clear only the button band; the cell-stats column at x>=335 shares this row.
  tft.fillRect(0, y - 8, 320, buttonH + 16, ST7735_BLACK);

  const char modes[2] = {'D', 'N'};
  for (int i = 0; i < 2; i++) {
    int x = startX + i * (buttonW + gap);
    bool selected = (driveMode == modes[i]);
    uint16_t color = selected ? ST7735_RED : ST7735_WHITE;

    tft.drawRoundRect(x, y, buttonW, buttonH, 6, color);
    tft.setTextSize(4);
    tft.setTextColor(color);
    tft.setCursor(x + 24, y + 12);
    tft.print(modes[i]);
  }

  tft.setTextColor(ST7735_WHITE);
  lastDriveMode = driveMode;
}

void drawStatusText(int x, int y, const char *label, const char *value, uint16_t valueColor) {
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(x, y);
  tft.print(label);
  tft.print(": ");
  tft.setTextColor(valueColor);
  tft.print(value);
  tft.setTextColor(ST7735_WHITE);
}

void drawStatusText(int x, int y, const char *label, int value, const char *units, uint16_t valueColor) {
  char text[16];
  snprintf(text, sizeof(text), "%d%s", value, units);
  drawStatusText(x, y, label, text, valueColor);
}

// Shutdown-circuit status lights: GPIO HIGH = green, LOW = red.
void drawShutdownStatusIndicators(bool force) {
  static int lastBspd = -1;
  static int lastImd = -1;
  static int lastBms = -1;

  drawShutdownStatusIndicator(362, 291, "BSPD", digitalRead(BSPD_STATUS_PIN) == HIGH,
                              lastBspd, force);
  drawShutdownStatusIndicator(402, 291, "IMD", digitalRead(IMD_STATUS_PIN) == HIGH,
                              lastImd, force);
  drawShutdownStatusIndicator(442, 291, "BMS", digitalRead(BMS_STATUS_PIN) == HIGH,
                              lastBms, force);
}

void drawShutdownStatusIndicator(int x, int y, const char *label, bool high,
                                 int &lastState, bool force) {
  if (!force && (int)high == lastState) {
    return;
  }
  lastState = (int)high;

  const int w = 36;
  const int h = 16;
  uint16_t color = high ? ST7735_GREEN : ST7735_RED;
  tft.fillRoundRect(x, y, w, h, 4, color);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_BLACK);
  int textW = (int)strlen(label) * 6;
  tft.setCursor(x + (w - textW) / 2, y + 4);
  tft.print(label);
  tft.setTextColor(ST7735_WHITE);
}

// ---------------- CAN transport ----------------
void setupVehicleCan() {
  VehicleCan.begin();
  VehicleCan.setBaudRate(CAN_BAUD);
  VehicleCan.setMaxMB(16);
  VehicleCan.enableFIFO();

  Serial.println("[CAN] CAN3 started at 500 kbit/s on pins 30=RX, 31=TX");
}

void sendCanFrame(CAN_message_t &msg) {
  noInterrupts();
  VehicleCan.write(msg);
  interrupts();
  logCanFrame("TX", msg);
}

void receiveCanFrames() {
  CAN_message_t rx;
  while (VehicleCan.read(rx)) {
    logCanFrame("RX", rx);
    dispatchReceivedCanFrame(rx);
  }
}

void dispatchReceivedCanFrame(const CAN_message_t &msg) {
  if (!havePm100Traffic && msg.id >= PM100_TEMPERATURE_1_FRAME_ID &&
      msg.id <= PM100_TORQUE_CAPABILITY_FRAME_ID) {
    havePm100Traffic = true;
    firstPm100FrameMs = millis();
  }

  switch (msg.id) {
    case IMD_INFO_GENERAL_ID:
    case IMD_RESPONSE_ID:
      handleImdFrame(msg);
      break;

    case PM100_TEMPERATURE_1_FRAME_ID:
      handlePm100Temperatures1Frame(msg);
      break;

    case PM100_TEMPERATURE_3_FRAME_ID:
      handlePm100Temperatures3Frame(msg);
      break;

    case PM100_RPM_FRAME_ID:
      handlePm100RpmFrame(msg);
      break;

    case PM100_CURRENT_INFO_FRAME_ID:
      handlePm100CurrentInfoFrame(msg);
      break;

    case PM100_VOLTAGE_INFO_FRAME_ID:
      handlePm100VoltageInfoFrame(msg);
      break;

    case PM100_INTERNAL_VOLTAGES_FRAME_ID:
      handlePm100InternalVoltagesFrame(msg);
      break;

    case PM100_FAULT_CODES_FRAME_ID:
      handlePm100FaultCodesFrame(msg);
      break;

    case PM100_PARAM_RESPONSE_ID:
      handlePm100ParamResponseFrame(msg);
      break;

    case BMS_STATUS_2026_ID:
      handleBms2026StatusFrame(msg);
      break;

    case BMS_TEMPERATURE_2026_ID:
      handleBms2026TempFrame(msg);
      break;

    case EM_MEASUREMENT_ID:
      handleEmMeasurementFrame(msg);
      break;

    case EM_STATUS_ID:
      handleEmStatusFrame(msg);
      break;

    case EM_TEMPERATURE_ID:
      handleEmTemperatureFrame(msg);
      break;

    default:
      break;
  }
}

// ---------------- CAN logging ----------------
void logCanFrame(const char *direction, const CAN_message_t &msg) {
  updateCanTranslationState(direction, msg);

  // Future SD-card logging should mirror this raw record and the decoded record here.
  if (CAN_LOG_SERIAL_ENABLED) {
    logCanFrameToSerial(direction, msg);
    logCanTranslationToSerial(direction, msg);
  }
}

void logCanFrameToSerial(const char *direction, const CAN_message_t &msg) {
  Serial.print("[CAN ");
  Serial.print(direction);
  Serial.print("] t=");
  Serial.print(millis());
  Serial.print(" id=0x");
  Serial.print(msg.id, HEX);
  Serial.print(msg.flags.extended ? " ext" : " std");
  Serial.print(" len=");
  Serial.print(msg.len);
  Serial.print(" data=");

  for (uint8_t i = 0; i < msg.len; i++) {
    printCanHexByte(msg.buf[i]);
    if (i + 1 < msg.len) {
      Serial.print(' ');
    }
  }

  Serial.println();
}

// ---------------- CAN decoded logging ----------------
void updateCanTranslationState(const char *direction, const CAN_message_t &msg) {
  (void)direction;

  if (msg.flags.extended || msg.id != BMS_CELL_REQUEST_2026_ID || msg.len < 1) {
    return;
  }

  lastCellDataRequestType = msg.buf[0];
}

void logCanTranslationToSerial(const char *direction, const CAN_message_t &msg) {
  if (!CAN_TRANSLATION_SERIAL_ENABLED) {
    return;
  }

  for (uint8_t i = 0; i < CAN_FRAME_TRANSLATION_COUNT; i++) {
    const CanFrameTranslation &translation = CAN_FRAME_TRANSLATIONS[i];
    if (msg.id != translation.id) {
      continue;
    }

    Serial.print("[CAN ");
    Serial.print(direction);
    Serial.print(" DECODED] ");
    Serial.print(translation.name);
    Serial.print(": ");
    translation.print(msg);
    Serial.println();
    return;
  }
}

void printBmsStatus2026Translation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  uint8_t flags = msg.buf[0];
  uint16_t minCellMv = u16Be(msg.buf[1], msg.buf[2]);
  uint16_t maxCellMv = u16Be(msg.buf[3], msg.buf[4]);
  uint16_t avgCellMv = u16Be(msg.buf[5], msg.buf[6]);

  Serial.print("flags=0x");
  printCanHexByte(flags);
  Serial.print(" fault=");
  Serial.print((flags & (1u << 0)) ? "YES" : "NO");
  Serial.print(" posContactorClosed=");
  Serial.print((flags & (1u << 1)) ? "YES" : "NO");
  Serial.print(" chargeMode=");
  Serial.print((flags & (1u << 2)) ? "YES" : "NO");
  Serial.print(" minCell=");
  Serial.print(minCellMv);
  Serial.print("mV maxCell=");
  Serial.print(maxCellMv);
  Serial.print("mV avgCell=");
  Serial.print(avgCellMv);
  Serial.print("mV soc=");
  if (msg.buf[7] == 0xFF) {
    Serial.print("unknown");
  } else {
    Serial.print(msg.buf[7]);
    Serial.print('%');
  }
}

void printBmsTemperature2026Translation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  uint8_t faultCode = msg.buf[3];
  uint16_t packCurrentRaw = u16Be(msg.buf[4], msg.buf[5]);
  float packCurrentA = ((int32_t)packCurrentRaw - 3000) * 0.1f;

  Serial.print("minCellTemp=");
  Serial.print((int)msg.buf[0] - 40);
  Serial.print("C maxCellTemp=");
  Serial.print((int)msg.buf[1] - 40);
  Serial.print("C avgCellTemp=");
  Serial.print((int)msg.buf[2] - 40);
  Serial.print("C faultCode=");
  Serial.print(faultCode);
  Serial.print('(');
  printBmsTemperatureFaultCode(faultCode);
  Serial.print(") packCurrent=");
  Serial.print(packCurrentA, 1);
  Serial.print("A reserved=");
  printCanHexByte(msg.buf[6]);
  Serial.print(' ');
  printCanHexByte(msg.buf[7]);
}

void printCellDataRequest2026Translation(const CAN_message_t &msg) {
  if (msg.len < 1) {
    Serial.print("short frame, expected request byte");
    return;
  }

  Serial.print("request=0x");
  printCanHexByte(msg.buf[0]);
  Serial.print(" (");
  printCellDataRequestName(msg.buf[0]);
  Serial.print(')');
}

void printCellDataResponse2026Translation(const CAN_message_t &msg) {
  if (msg.len < 1) {
    Serial.print("short frame, expected frame index");
    return;
  }

  uint8_t frameIndex = msg.buf[0];

  Serial.print("frameIndex=");
  Serial.print(frameIndex);

  if (lastCellDataRequestType == CELL_DATA_REQUEST_VOLTAGES) {
    uint8_t availablePairs = (msg.len > 1) ? ((msg.len - 1) / 2) : 0;
    if (availablePairs > 3) {
      availablePairs = 3;
    }

    Serial.print(" mode=voltages cells=");
    for (uint8_t i = 0; i < availablePairs; i++) {
      if (i > 0) {
        Serial.print(", ");
      }
      Serial.print('#');
      Serial.print((uint16_t)frameIndex * 3u + i + 1u);
      Serial.print('=');
      printCellVoltageValue(u16Be(msg.buf[1 + (i * 2)], msg.buf[2 + (i * 2)]));
    }
    if (availablePairs == 0) {
      Serial.print("none");
    }
    if (msg.len > 7) {
      Serial.print(" extra=");
      printCanHexByte(msg.buf[7]);
    }
  } else if (lastCellDataRequestType == CELL_DATA_REQUEST_TEMPERATURES) {
    uint8_t availableTemps = (msg.len > 1) ? (msg.len - 1) : 0;
    if (availableTemps > 7) {
      availableTemps = 7;
    }

    Serial.print(" mode=temperatures cells=");
    for (uint8_t i = 0; i < availableTemps; i++) {
      if (i > 0) {
        Serial.print(", ");
      }
      Serial.print('#');
      Serial.print((uint16_t)frameIndex * 7u + i + 1u);
      Serial.print('=');
      Serial.print((int)msg.buf[1 + i] - 40);
      Serial.print('C');
    }
    if (availableTemps == 0) {
      Serial.print("none");
    }
  } else {
    Serial.print(" mode=unknown lastRequest=0x");
    printCanHexByte(lastCellDataRequestType);
    Serial.print(" payload=");
    printRawPayloadFrom(msg, 1);
  }
}

void printImdRequestTranslation(const CAN_message_t &msg) {
  if (msg.len < 1) {
    Serial.print("short frame, expected request index");
    return;
  }

  Serial.print("index=0x");
  printCanHexByte(msg.buf[0]);
  Serial.print(" (");
  printImdRequestIndexName(msg.buf[0]);
  Serial.print(") data=");
  printRawPayloadFrom(msg, 1);
}

void printImdResponseTranslation(const CAN_message_t &msg) {
  if (msg.len < 1) {
    Serial.print("short frame, expected response index");
    return;
  }

  uint8_t index = msg.buf[0];

  if (index == 0xFF) {
    Serial.print("error");
    if (msg.len >= 2) {
      Serial.print(" code=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdResponseErrorName(msg.buf[1]);
      Serial.print(')');
    }
    if (msg.len >= 3) {
      Serial.print(" requestedIndex=0x");
      printCanHexByte(msg.buf[2]);
      Serial.print('(');
      printImdRequestIndexName(msg.buf[2]);
      Serial.print(')');
    }
    return;
  }

  Serial.print("index=0x");
  printCanHexByte(index);
  Serial.print('(');
  printImdRequestIndexName(index);
  Serial.print(") ");

  if (msg.len < 2) {
    Serial.print("missing response data");
    return;
  }

  uint16_t word = (msg.len >= 3) ? u16Le(msg.buf[1], msg.buf[2]) : msg.buf[1];

  switch (index) {
    case 0x0A:
    case 0x0C:
    case 0x0E:
    case 0x1E:
    case 0x20:
    case 0x22:
      Serial.print("value=");
      printImdWordValue(word);
      break;

    case 0x10:
    case 0x12:
    case 0x14:
    case 0x16:
    case 0x18:
    case 0x1A:
    case 0x1C:
      Serial.print("ascii=");
      printImdAsciiPayloadFrom(msg, 1);
      break;

    case 0x2A:
    case 0x2E:
      Serial.print("percent=");
      printImdPercentValue(msg.buf[1]);
      break;

    case 0x2C:
    case 0x36:
    case 0x54:
    case 0x5C:
      Serial.print("counter=");
      printImdCounterValue(msg.buf[1]);
      break;

    case 0x30:
      Serial.print("selfHoldingAlarm=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      switch (msg.buf[1]) {
        case 0xFC: Serial.print("automatic alarm reset"); break;
        case 0xFD: Serial.print("self-holding alarm"); break;
        case 0xFE: Serial.print("reserved"); break;
        case IMD_SNV_BYTE: Serial.print("SNV"); break;
        default: Serial.print("unknown"); break;
      }
      Serial.print(')');
      break;

    case 0x38:
    case 0x3A:
      Serial.print("profile=");
      Serial.print(msg.buf[1]);
      Serial.print('(');
      printImdProfileName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x3E:
      Serial.print("quality=");
      printImdPercentValue(msg.buf[1]);
      break;

    case 0x40:
    case 0x42:
    case 0x46:
    case 0x4A:
    case 0x4C:
    case 0x4E:
    case 0x7E:
      Serial.print("resistance=");
      printImdResistanceValue(word);
      break;

    case 0x44:
      Serial.print("rIsoStatus=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdRIsoStatusName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x48:
    case 0x50:
      Serial.print("seconds=");
      printImdWordValue(word);
      break;

    case 0x52:
      Serial.print("capacity=");
      printImdCapacityValue(word);
      break;

    case 0x58:
      Serial.print("selfTestPeriod=");
      if (word == 0) {
        Serial.print("disabled");
      } else if (word == IMD_SNV_WORD) {
        Serial.print("SNV");
      } else {
        Serial.print((uint32_t)word * 10u);
        Serial.print("s");
      }
      break;

    case 0x5E:
    case 0x60:
    case 0x62:
      Serial.print("voltage=");
      printImdVoltageValue(word);
      break;

    case 0x64:
      Serial.print("mode=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdVoltageModeName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x66:
    case 0x72:
      Serial.print("voltage=");
      printImdWordValue(word);
      Serial.print("V");
      break;

    case 0x68:
      Serial.print("deviceActivity=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdDeviceActivityName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x6A:
    case 0x6B:
      Serial.print("lock=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdLockStateName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x6C:
      Serial.print("warningsAndAlarms=0x");
      Serial.print(word, HEX);
      printTssiWarnBits(word);
      break;

    case 0x70:
    case 0x71:
      Serial.print("earthlift=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdEarthliftStateName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x74:
      Serial.print("maxVoltageDifference=");
      if (word == IMD_SNV_WORD) {
        Serial.print("SNV");
      } else {
        Serial.print((float)word * 0.01f, 2);
        Serial.print("V");
      }
      break;

    case 0x76:
    case 0x77:
      Serial.print("message=");
      if (msg.len >= 2) {
        printImdInfoMessageSelectorName(msg.buf[1]);
      } else {
        Serial.print("missing");
      }
      Serial.print(" canId=");
      if (msg.len >= 3) {
        Serial.print("0x");
        printCanHexByte(msg.buf[2]);
      } else {
        Serial.print("missing");
      }
      break;

    case 0x78:
    case 0x79:
      Serial.print("message=");
      if (msg.len >= 2) {
        printImdInfoMessageSelectorName(msg.buf[1]);
      } else {
        Serial.print("missing");
      }
      Serial.print(" period=");
      if (msg.len >= 3) {
        if (msg.buf[2] == 0) {
          Serial.print("deactivated");
        } else if (msg.buf[2] == IMD_SNV_BYTE) {
          Serial.print("SNV");
        } else {
          Serial.print((uint16_t)msg.buf[2] * 100u);
          Serial.print("ms");
        }
      } else {
        Serial.print("missing");
      }
      break;

    case 0x7B:
      Serial.print("baudRate=");
      printImdBaudRateName(msg.buf[1]);
      break;

    default:
      Serial.print("payload=");
      printRawPayloadFrom(msg, 1);
      break;
  }
}

void printImdInfoGeneralTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  uint16_t rIsoCorrected = u16Le(msg.buf[0], msg.buf[1]);
  uint8_t rIsoStatus = msg.buf[2];
  uint8_t isolationCounter = msg.buf[3];
  uint8_t warningsLowByte = msg.buf[4];
  uint8_t reservedByte = msg.buf[5];
  uint8_t deviceActivity = msg.buf[6];

  Serial.print("rIsoCorrected=");
  printImdResistanceValue(rIsoCorrected);
  Serial.print(" rIsoStatus=0x");
  printCanHexByte(rIsoStatus);
  Serial.print('(');
  printImdRIsoStatusName(rIsoStatus);
  Serial.print(") isolationCounter=");
  printImdCounterValue(isolationCounter);
  Serial.print(" warningsLow=0x");
  printCanHexByte(warningsLowByte);
  printTssiWarnBits(warningsLowByte);
  Serial.print(" reserved5=0x");
  printCanHexByte(reservedByte);
  Serial.print(" deviceActivity=0x");
  printCanHexByte(deviceActivity);
  Serial.print('(');
  printImdDeviceActivityName(deviceActivity);
  Serial.print(") byte7=0x");
  printCanHexByte(msg.buf[7]);
}

void printImdInfoIsolationDetailTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  Serial.print("rIsoNeg=");
  printImdResistanceValue(u16Le(msg.buf[0], msg.buf[1]));
  Serial.print(" rIsoPos=");
  printImdResistanceValue(u16Le(msg.buf[2], msg.buf[3]));
  Serial.print(" rIsoOriginal=");
  printImdResistanceValue(u16Le(msg.buf[4], msg.buf[5]));
  Serial.print(" isolationCounter=");
  printImdCounterValue(msg.buf[6]);
  Serial.print(" quality=");
  printImdPercentValue(msg.buf[7]);
}

void printImdInfoVoltageTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  Serial.print("hvSystem=");
  printImdVoltageValue(u16Le(msg.buf[0], msg.buf[1]));
  Serial.print(" hvNegToEarth=");
  printImdVoltageValue(u16Le(msg.buf[2], msg.buf[3]));
  Serial.print(" hvPosToEarth=");
  printImdVoltageValue(u16Le(msg.buf[4], msg.buf[5]));
  Serial.print(" voltageCounter=");
  printImdCounterValue(msg.buf[6]);
  Serial.print(" byte7=0x");
  printCanHexByte(msg.buf[7]);
}

void printImdInfoItSystemTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  Serial.print("capacity=");
  printImdCapacityValue(u16Le(msg.buf[0], msg.buf[1]));
  Serial.print(" capacityCounter=");
  printImdCounterValue(msg.buf[2]);
  Serial.print(" unbalance=");
  printImdPercentValue(msg.buf[3]);
  Serial.print(" unbalanceCounter=");
  printImdCounterValue(msg.buf[4]);
  Serial.print(" byte5=0x");
  printCanHexByte(msg.buf[5]);
  Serial.print(" byte6=0x");
  printCanHexByte(msg.buf[6]);
  Serial.print(" byte7=0x");
  printCanHexByte(msg.buf[7]);
}

void printPm100CommandTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t torqueRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t speedCommandRpm = s16Le(msg.buf[2], msg.buf[3]);
  uint8_t directionCommand = msg.buf[4];
  uint8_t inverterEnable = msg.buf[5];
  uint8_t inverterDischarge = msg.buf[6];
  uint8_t speedModeEnable = msg.buf[7];

  Serial.print("torqueCommand=");
  Serial.print((float)torqueRaw / 10.0f, 1);
  Serial.print("Nm speedCommand=");
  Serial.print(speedCommandRpm);
  Serial.print("rpm direction=0x");
  printCanHexByte(directionCommand);
  Serial.print('(');
  if (directionCommand == 0x00) {
    Serial.print("reverse/clockwise");
  } else if (directionCommand == 0x01) {
    Serial.print("forward/counterclockwise");
  } else {
    Serial.print("unknown");
  }
  Serial.print(") inverterEnable=");
  Serial.print(inverterEnable ? "ON" : "OFF");
  Serial.print(" inverterEnableRaw=0x");
  printCanHexByte(inverterEnable);
  Serial.print(" inverterDischarge=0x");
  printCanHexByte(inverterDischarge);
  Serial.print(" speedModeEnable=0x");
  printCanHexByte(speedModeEnable);
}

void printPm100Temperatures1Translation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t moduleATempRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t moduleBTempRaw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t moduleCTempRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t gateDriverTempRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("moduleA=");
  Serial.print((float)moduleATempRaw / 10.0f, 1);
  Serial.print("C moduleB=");
  Serial.print((float)moduleBTempRaw / 10.0f, 1);
  Serial.print("C moduleC=");
  Serial.print((float)moduleCTempRaw / 10.0f, 1);
  Serial.print("C gateDriver=");
  Serial.print((float)gateDriverTempRaw / 10.0f, 1);
  Serial.print("C");
}

void printPm100Temperatures3Translation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t coolantTempRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t hotSpotTempRaw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t motorTempRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t torqueShudderRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("coolant=");
  Serial.print((float)coolantTempRaw / 10.0f, 1);
  Serial.print("C hotSpot=");
  Serial.print((float)hotSpotTempRaw / 10.0f, 1);
  Serial.print("C motor=");
  Serial.print((float)motorTempRaw / 10.0f, 1);
  Serial.print("C torqueShudder=");
  Serial.print((float)torqueShudderRaw / 10.0f, 1);
  Serial.print("Nm");
}

void printPm100MotorPositionTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t motorAngleRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t motorSpeedRpm = s16Le(msg.buf[2], msg.buf[3]);
  int16_t electricalFrequencyRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t deltaResolverRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("motorAngle=");
  Serial.print((float)motorAngleRaw / 10.0f, 1);
  Serial.print("deg motorSpeed=");
  Serial.print(motorSpeedRpm);
  Serial.print("rpm electricalFrequency=");
  Serial.print((float)electricalFrequencyRaw / 10.0f, 1);
  Serial.print("Hz deltaResolver=");
  Serial.print((float)deltaResolverRaw / 10.0f, 1);
  Serial.print("deg");
}

void printPm100CurrentInfoTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t phaseACurrentRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t phaseBCurrentRaw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t phaseCCurrentRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t dcBusCurrentRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("phaseA=");
  Serial.print((float)phaseACurrentRaw / 10.0f, 1);
  Serial.print("A phaseB=");
  Serial.print((float)phaseBCurrentRaw / 10.0f, 1);
  Serial.print("A phaseC=");
  Serial.print((float)phaseCCurrentRaw / 10.0f, 1);
  Serial.print("A dcBus=");
  Serial.print((float)dcBusCurrentRaw / 10.0f, 1);
  Serial.print("A");
}

void printPm100VoltageInfoTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t dcBusVoltageRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t outputVoltageRaw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t vabVdVoltageRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t vbcVqVoltageRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("dcBus=");
  Serial.print((float)dcBusVoltageRaw / 10.0f, 1);
  Serial.print("V output=");
  Serial.print((float)outputVoltageRaw / 10.0f, 1);
  Serial.print("V vabOrVd=");
  Serial.print((float)vabVdVoltageRaw / 10.0f, 1);
  Serial.print("V vbcOrVq=");
  Serial.print((float)vbcVqVoltageRaw / 10.0f, 1);
  Serial.print("V");
}

void printPm100InternalVoltagesTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t ref1V5Raw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t ref2V5Raw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t ref5V0Raw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t glvRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("ref1V5=");
  Serial.print((float)ref1V5Raw / 100.0f, 2);
  Serial.print("V ref2V5=");
  Serial.print((float)ref2V5Raw / 100.0f, 2);
  Serial.print("V ref5V0=");
  Serial.print((float)ref5V0Raw / 100.0f, 2);
  Serial.print("V glv12V=");
  Serial.print((float)glvRaw / 100.0f, 2);
  Serial.print("V");
}

void printPm100FaultCodesTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  uint32_t postFaults = u32Le(msg.buf[0], msg.buf[1], msg.buf[2], msg.buf[3]);
  uint32_t runFaults = u32Le(msg.buf[4], msg.buf[5], msg.buf[6], msg.buf[7]);

  Serial.print("post=0x");
  Serial.print((unsigned long)postFaults, HEX);
  Serial.print(" run=0x");
  Serial.print((unsigned long)runFaults, HEX);
  Serial.print(" active=");

  bool printedFault = false;
  for (uint8_t bit = 0; bit < 32; bit++) {
    if (postFaults & (1UL << bit)) {
      if (printedFault) Serial.print(',');
      Serial.print(pm100PostFaultName(bit));
      printedFault = true;
    }
  }

  for (uint8_t bit = 0; bit < 32; bit++) {
    if (runFaults & (1UL << bit)) {
      if (printedFault) Serial.print(',');
      Serial.print(pm100RunFaultName(bit));
      printedFault = true;
    }
  }

  if (!printedFault) {
    Serial.print("none");
  }
}

void printPm100TorqueCapabilityTranslation(const CAN_message_t &msg) {
  if (msg.len < 2) {
    Serial.print("short frame, expected at least 2 bytes");
    return;
  }

  int16_t positiveTorqueRaw = s16Le(msg.buf[0], msg.buf[1]);

  Serial.print("positiveTorqueCapability=");
  Serial.print((float)positiveTorqueRaw / 10.0f, 1);
  Serial.print("Nm");

  if (msg.len >= 4) {
    int16_t negativeTorqueRaw = s16Le(msg.buf[2], msg.buf[3]);
    Serial.print(" possibleNegativeTorqueCapability=");
    Serial.print((float)negativeTorqueRaw / 10.0f, 1);
    Serial.print("Nm");
  }

  if (msg.len > 4) {
    Serial.print(" reserved=");
    printRawPayloadFrom(msg, 4);
  }
}

void printRawPayloadFrom(const CAN_message_t &msg, uint8_t startIndex) {
  if (startIndex >= msg.len) {
    Serial.print("none");
    return;
  }

  for (uint8_t i = startIndex; i < msg.len; i++) {
    if (i > startIndex) {
      Serial.print(' ');
    }
    printCanHexByte(msg.buf[i]);
  }
}

void printBmsTemperatureFaultCode(uint8_t faultCode) {
  Serial.print(bmsTemperatureFaultCodeName(faultCode));
}

void printCellDataRequestName(uint8_t requestType) {
  switch (requestType) {
    case CELL_DATA_REQUEST_VOLTAGES: Serial.print("cell voltages"); break;
    case CELL_DATA_REQUEST_TEMPERATURES: Serial.print("cell temperatures"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdRequestIndexName(uint8_t requestIndex) {
  switch (requestIndex) {
    case 0x0A: Serial.print("bootloader build number"); break;
    case 0x0C: Serial.print("bootloader D-number"); break;
    case 0x0E: Serial.print("bootloader version"); break;
    case 0x10: Serial.print("hardware AH history"); break;
    case 0x12: Serial.print("hardware AH number"); break;
    case 0x14: Serial.print("hardware AH number part B"); break;
    case 0x16: Serial.print("hardware item number"); break;
    case 0x18: Serial.print("hardware article number part B"); break;
    case 0x1A: Serial.print("serial number"); break;
    case 0x1C: Serial.print("serial number part B"); break;
    case 0x1E: Serial.print("software build number"); break;
    case 0x20: Serial.print("software D-number"); break;
    case 0x22: Serial.print("software version"); break;
    case 0x2A: Serial.print("unbalance measured value"); break;
    case 0x2C: Serial.print("unbalance measurement counter"); break;
    case 0x2E:
    case 0x2F: Serial.print("unbalance threshold"); break;
    case 0x30:
    case 0x31: Serial.print("self-holding alarm activation"); break;
    case 0x33: Serial.print("reset alarm"); break;
    case 0x36: Serial.print("isolation measurement counter"); break;
    case 0x38:
    case 0x39: Serial.print("isolation active profile"); break;
    case 0x3A:
    case 0x3B: Serial.print("isolation power-on profile"); break;
    case 0x3E: Serial.print("isolation quality"); break;
    case 0x40: Serial.print("R_iso_neg"); break;
    case 0x42: Serial.print("R_iso_pos"); break;
    case 0x44: Serial.print("R_iso_status"); break;
    case 0x46:
    case 0x47: Serial.print("isolation threshold error"); break;
    case 0x48:
    case 0x49: Serial.print("isolation measurement timeout threshold"); break;
    case 0x4A:
    case 0x4B: Serial.print("isolation threshold warning"); break;
    case 0x4C: Serial.print("R_iso_corrected"); break;
    case 0x4E: Serial.print("R_iso_original"); break;
    case 0x50: Serial.print("time since last isolation measurement"); break;
    case 0x52: Serial.print("capacity measured value"); break;
    case 0x54: Serial.print("capacity measurement counter"); break;
    case 0x57: Serial.print("trigger self test"); break;
    case 0x58:
    case 0x59: Serial.print("self test period"); break;
    case 0x5C: Serial.print("voltage measurement counter"); break;
    case 0x5E: Serial.print("HV system voltage"); break;
    case 0x60: Serial.print("HV negative to earth voltage"); break;
    case 0x62: Serial.print("HV positive to earth voltage"); break;
    case 0x64:
    case 0x65: Serial.print("voltage mode"); break;
    case 0x66:
    case 0x67: Serial.print("undervoltage threshold"); break;
    case 0x68: Serial.print("device activity"); break;
    case 0x6A:
    case 0x6B: Serial.print("parameter lock"); break;
    case IDX_WARN_ALARMS: Serial.print("warnings and alarms"); break;
    case 0x6F: Serial.print("factory reset"); break;
    case 0x70:
    case 0x71: Serial.print("earthlift status"); break;
    case 0x72:
    case 0x73: Serial.print("threshold first reference estimation"); break;
    case 0x74:
    case 0x75: Serial.print("pre-estimation max difference"); break;
    case 0x76:
    case 0x77: Serial.print("interface CAN-ID"); break;
    case 0x78:
    case 0x79: Serial.print("interface periodic cycle time"); break;
    case 0x7B: Serial.print("interface baudrate"); break;
    case 0x7E:
    case 0x7F: Serial.print("isolation initial value"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdResponseErrorName(uint8_t errorCode) {
  switch (errorCode) {
    case 0x23: Serial.print("unknown or invalid request"); break;
    case 0x24: Serial.print("set command failed, parameter locked"); break;
    case 0x25: Serial.print("data1 range overflow"); break;
    case 0x26: Serial.print("data2 range overflow"); break;
    case 0x27: Serial.print("CAN-ID already in use"); break;
    case 0x28: Serial.print("write failed"); break;
    case 0x29: Serial.print("read failed"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdInfoMessageSelectorName(uint8_t selector) {
  switch (selector) {
    case 0x00: Serial.print("Request"); break;
    case 0x01: Serial.print("Response"); break;
    case 0x02: Serial.print("IMD_Info_General"); break;
    case 0x03: Serial.print("IMD_Info_IsolationDetail"); break;
    case 0x04: Serial.print("IMD_Info_Voltage"); break;
    case 0x05: Serial.print("IMD_Info_IT-System"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdProfileName(uint8_t profile) {
  switch (profile) {
    case 1: Serial.print("standard fast startup"); break;
    case 2: Serial.print("standard"); break;
    case 3: Serial.print("high capacity fast startup"); break;
    case 4: Serial.print("high capacity"); break;
    case 5: Serial.print("disturbed"); break;
    case 6: Serial.print("service"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdRIsoStatusName(uint8_t status) {
  switch (status) {
    case 0xFC: Serial.print("startup estimated"); break;
    case 0xFD: Serial.print("startup first measurement"); break;
    case 0xFE: Serial.print("normal operation"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdDeviceActivityName(uint8_t activity) {
  switch (activity) {
    case 0: Serial.print("initialization"); break;
    case 1: Serial.print("normal operation"); break;
    case 2: Serial.print("self test"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdLockStateName(uint8_t state) {
  switch (state) {
    case 0xFC: Serial.print("parameter write enable"); break;
    case 0xFD: Serial.print("parameter write disable"); break;
    case 0xFE: Serial.print("reserved"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdEarthliftStateName(uint8_t state) {
  switch (state) {
    case 0:
    case 0xFC: Serial.print("earth connection closed"); break;
    case 1:
    case 0xFD: Serial.print("earth connection open"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdVoltageModeName(uint8_t mode) {
  switch (mode) {
    case 0xFD: Serial.print("AC"); break;
    case 0xFE: Serial.print("DC"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdBaudRateName(uint8_t baudRate) {
  switch (baudRate) {
    case 0x01: Serial.print("1 Mbit/s"); break;
    case 0x02: Serial.print("800 kbit/s"); break;
    case 0x03: Serial.print("666 kbit/s"); break;
    case 0x04: Serial.print("500 kbit/s"); break;
    case 0x05: Serial.print("250 kbit/s"); break;
    case 0x06: Serial.print("125 kbit/s"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdAsciiPayloadFrom(const CAN_message_t &msg, uint8_t startIndex) {
  Serial.print('"');
  for (uint8_t i = startIndex; i < msg.len; i++) {
    uint8_t value = msg.buf[i];
    if (value == IMD_SNV_BYTE) {
      break;
    }
    if (value >= 32 && value <= 126) {
      Serial.write(value);
    } else {
      Serial.print('.');
    }
  }
  Serial.print('"');
}

void printImdWordValue(uint16_t value) {
  if (value == IMD_SNV_WORD) {
    Serial.print("SNV");
  } else {
    Serial.print(value);
  }
}

void printImdResistanceValue(uint16_t kohms) {
  if (kohms == IMD_SNV_WORD) {
    Serial.print("SNV");
  } else {
    Serial.print(kohms);
    Serial.print("kOhm");
  }
}

void printImdVoltageValue(uint16_t raw) {
  if (raw == IMD_SNV_WORD) {
    Serial.print("SNV");
  } else {
    float volts = ((int32_t)raw - IMD_VOLTAGE_OFFSET) * IMD_VOLTAGE_SCALE;
    Serial.print(volts, 2);
    Serial.print("V");
  }
}

void printImdCapacityValue(uint16_t raw) {
  if (raw == IMD_SNV_WORD) {
    Serial.print("SNV");
  } else {
    Serial.print((float)raw * 0.1f, 1);
    Serial.print("uF");
  }
}

void printImdPercentValue(uint8_t value) {
  if (value == IMD_SNV_BYTE) {
    Serial.print("SNV");
  } else {
    Serial.print(value);
    Serial.print('%');
  }
}

void printImdCounterValue(uint8_t value) {
  Serial.print(value);
}

void printCellVoltageValue(uint16_t millivolts) {
  if (millivolts == 0xFFFF) {
    Serial.print("pad");
  } else {
    Serial.print(millivolts);
    Serial.print("mV");
  }
}

void printCanHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

bool tssiFaultFromWarnBits(uint16_t warnBits) {
  return (warnBits & IMD_FAULT_BITS_MASK) != 0;
}

bool bmsImdFaultDetectionArmed() {
  return (millis() - startupFaultDelayStartMs) >= STARTUP_BMS_IMD_FAULT_DELAY_MS;
}

bool anyFaultActive() {
  if (!bmsImdFaultDetectionArmed()) {
    return false;
  }

  return imdFault || bmsFault;
}

// Contactors open is a normal pre-RTD condition, not a fault: it should drop/
// block RTD (back to neutral) without lighting the fault indicator.
bool contactorsOpen() {
  return haveBmsStatus && !(bmsFlags & 0x02);
}

// ---------------- CAN receive handlers ----------------
void handleBms2026StatusFrame(const CAN_message_t &msg) {
  if (msg.len < 8) {
    return;
  }

  bool wasActive = bmsActive;
  bool previousFault = bmsFault;
  bool faultDetectionArmed = bmsImdFaultDetectionArmed();

  uint8_t flags = msg.buf[0];
  minCellVoltageMv = u16Be(msg.buf[1], msg.buf[2]);
  maxCellVoltageMv = u16Be(msg.buf[3], msg.buf[4]);
  avgCellVoltageMv = u16Be(msg.buf[5], msg.buf[6]);
  haveCellVoltageStats = true;

  haveBmsStatus    = true;
  bmsTimedOut      = false;
  lastBmsRxMs      = millis();
  bmsFlags         = flags;
  bmsFault         = (flags & 0x01) != 0;
  bmsActive        = !bmsFault;
  stateOfChargePct = msg.buf[7];
  packVoltage      = (avgCellVoltageMv / 1000.0f) * 96.0f;  // 96 cells in series

  if (bmsActive && !wasActive) {
    Serial.println("[BMS] Active");
  }

  if (faultDetectionArmed && bmsFault != previousFault) {
    Serial.print("[BMS] 2026 flags=0x");
    Serial.print(flags, HEX);
    Serial.print(" fault=");
    Serial.println(bmsFault ? "YES" : "NO");
    if (bmsFault) {
      showFaultBanner((haveBms2026FaultCode && bms2026FaultCode != 0)
                          ? bmsTemperatureFaultCodeName(bms2026FaultCode)
                          : "BMS Fault");
    }
  }

  if (anyFaultActive() || contactorsOpen()) {
    dropReadyToDrive(rtdDropCauseText());
  }

  screenDirty = true;
}

void handleBms2026TempFrame(const CAN_message_t &msg) {
  if (msg.len < 6) {
    return;
  }

  // byte 1 = max cell temp encoded as (°C + 40); 0xFF means no valid reading
  uint8_t previousFaultCode = haveBms2026FaultCode ? bms2026FaultCode : 0;
  bms2026FaultCode = msg.buf[3];
  haveBms2026FaultCode = true;
  if (bmsImdFaultDetectionArmed() && bms2026FaultCode != 0 &&
      bms2026FaultCode != previousFaultCode) {
    showFaultBanner(bmsTemperatureFaultCodeName(bms2026FaultCode));
  }

  haveCellTempStats = (msg.buf[0] != 0xFF && msg.buf[1] != 0xFF && msg.buf[2] != 0xFF);
  if (haveCellTempStats) {
    minCellTempC = (int16_t)msg.buf[0] - 40;
    maxCellTempC = (int16_t)msg.buf[1] - 40;
    avgCellTempC = (int16_t)msg.buf[2] - 40;
  } else {
    minCellTempC = CELL_TEMP_UNKNOWN_C;
    maxCellTempC = CELL_TEMP_UNKNOWN_C;
    avgCellTempC = CELL_TEMP_UNKNOWN_C;
  }

  if (msg.buf[1] != 0xFF) {
    batteryTempC = (uint8_t)constrain((int)msg.buf[1] - 40, 0, 255);
  }

  // bytes 4-5 = pack current, big-endian, raw = A * 10 + 3000
  int16_t curRaw = (int16_t)(((uint16_t)msg.buf[4] << 8) | msg.buf[5]);
  packCurrent = ((float)curRaw - 3000.0f) / 10.0f;

  screenDirty = true;
}

void handleImdFrame(const CAN_message_t &msg) {
  bool wasTimedOut = imdCanTimedOut;
  uint16_t previousWarnings = imdWarnAlarms;
  bool previousFault = imdFault;
  bool faultDetectionArmed = bmsImdFaultDetectionArmed();

  if (msg.id == IMD_INFO_GENERAL_ID && msg.len >= 5) {
    imdRIsoKohm = u16Le(msg.buf[0], msg.buf[1]);
    imdRIsoStatus = msg.buf[2];
    imdWarnAlarms = (imdWarnAlarms & 0xFF00u) | msg.buf[4];
    imdDeviceActivity = (msg.len >= 7) ? msg.buf[6] : IMD_SNV_BYTE;
    lastImdRxMs = millis();
    imdCanTimedOut = false;
    imdFault = tssiFaultFromWarnBits(imdWarnAlarms);
  } else if (msg.id == IMD_RESPONSE_ID && msg.len >= 3 && msg.buf[0] == IDX_WARN_ALARMS) {
    imdWarnAlarms = u16Le(msg.buf[1], msg.buf[2]);
    lastImdRxMs = millis();
    imdCanTimedOut = false;
    imdFault = tssiFaultFromWarnBits(imdWarnAlarms);
  } else {
    return;
  }

  if (wasTimedOut) {
    Serial.println("[TSSI] IMD CAN restored");
  }

  if (faultDetectionArmed && (imdWarnAlarms != previousWarnings || imdFault != previousFault)) {
    Serial.print("[TSSI] IMD warn=0x");
    Serial.print(imdWarnAlarms, HEX);
    printTssiWarnBits(imdWarnAlarms);
    Serial.print(" => ");
    Serial.println(imdFault ? "FAULT, red blink" : "OK, green on");
    if (imdFault && !previousFault) {
      showFaultBanner(firstTssiFaultName(imdWarnAlarms));
    }
  }
}

void handlePm100Temperatures1Frame(const CAN_message_t &msg) {
  if (msg.len < 8) {
    return;
  }

  int16_t moduleATempRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t moduleBTempRaw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t moduleCTempRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t gateDriverTempRaw = s16Le(msg.buf[6], msg.buf[7]);

  int16_t hottestTempRaw = moduleATempRaw;
  if (moduleBTempRaw > hottestTempRaw) hottestTempRaw = moduleBTempRaw;
  if (moduleCTempRaw > hottestTempRaw) hottestTempRaw = moduleCTempRaw;
  if (gateDriverTempRaw > hottestTempRaw) hottestTempRaw = gateDriverTempRaw;

  inverterTempTenthsC = hottestTempRaw;
  haveInverterTemp = true;
  screenDirty = true;
}

void handlePm100Temperatures3Frame(const CAN_message_t &msg) {
  if (msg.len < 6) {
    return;
  }

  pm100MotorTempTenthsC = s16Le(msg.buf[4], msg.buf[5]);
  havePm100MotorTemp = true;
  screenDirty = true;
}

void handlePm100RpmFrame(const CAN_message_t &msg) {
  if (msg.len < 4) {
    return;
  }

  motorRpm = s16Le(msg.buf[2], msg.buf[3]);
  screenDirty = true;
}

void handlePm100CurrentInfoFrame(const CAN_message_t &msg) {
  if (msg.len < 8) {
    return;
  }

  inverterDcBusCurrent = (float)s16Le(msg.buf[6], msg.buf[7]) / 10.0f;
  haveInverterCurrentInfo = true;
  screenDirty = true;
}

void handlePm100VoltageInfoFrame(const CAN_message_t &msg) {
  if (msg.len < 2) {
    return;
  }

  inverterDcBusVoltage = (float)s16Le(msg.buf[0], msg.buf[1]) / 10.0f;
  haveInverterVoltageInfo = true;
  screenDirty = true;
}

void handlePm100InternalVoltagesFrame(const CAN_message_t &msg) {
  if (msg.len < 8) {
    return;
  }

  // Bytes 6-7 = 12V System voltage, "Low Voltage" format (volts * 100).
  glvVoltage = (float)s16Le(msg.buf[6], msg.buf[7]) / 100.0f;
  haveGlvVoltage = true;
  screenDirty = true;
}

void handlePm100ParamResponseFrame(const CAN_message_t &msg) {
  if (msg.len < 3) {
    return;
  }

  uint16_t address = u16Le(msg.buf[0], msg.buf[1]);
  if (address == PM100_PARAM_CAN_ACTIVE_MSGS) {
    Serial.print("[PM100] CAN Active Messages parameter write ");
    Serial.println(msg.buf[2] ? "succeeded" : "FAILED");
  } else if (address == PM100_PARAM_FAULT_CLEAR) {
    Serial.print("[PM100] Fault clear command ");
    Serial.println(msg.buf[2] ? "accepted" : "REJECTED");
  }
}

// ---------------- Energy Meter ----------------
// 0x10D Measurement: Current[A] float (bytes 0-3), Voltage[V] float (bytes 4-7).
void handleEmMeasurementFrame(const CAN_message_t &msg) {
  emLastRxMs = millis();
  emTimedOut = false;
  if (msg.len < 8) {
    return;
  }
  emCurrentA = f32Le(&msg.buf[0]);
  emVoltageV = f32Le(&msg.buf[4]);
  haveEmMeasurement = true;
  screenDirty = true;
}

// 0x40D Status: byte 0 holds the status bits (bytes 1-4 are Energy[Whr], unused
// on the dashboard).
void handleEmStatusFrame(const CAN_message_t &msg) {
  emLastRxMs = millis();
  emTimedOut = false;
  if (msg.len < 1) {
    return;
  }
  emStatusByte = msg.buf[0];
  haveEmStatus = true;
  screenDirty = true;
}

// 0x60D Temperature: byte 0 bits 0-2 = multiplexor, bits 3-7 = sensor count.
// The mux-0 frame carries the count plus min/max in bytes 1/2 (degC = raw*0.5),
// which is all the dashboard needs.
void handleEmTemperatureFrame(const CAN_message_t &msg) {
  emLastRxMs = millis();
  emTimedOut = false;
  if (msg.len < 8) {
    return;
  }
  if ((msg.buf[0] & 0x07) == 0) {
    emNumSensors = msg.buf[0] >> 3;
    emMinTempRaw = msg.buf[1];
    emMaxTempRaw = msg.buf[2];
    haveEmTempSummary = true;
  }
  haveEmTemps = true;
  screenDirty = true;
}

// Flags the EM comms-lost condition once nothing has arrived for EM_TIMEOUT_MS.
void serviceEnergyMeterTimeout(uint32_t now) {
  if (!haveEmMeasurement && !haveEmStatus && !haveEmTemps) {
    return;  // never seen the meter yet; stays "NO COMMS" without a timeout edge
  }
  emTimedOut = (now - emLastRxMs > EM_TIMEOUT_MS);
}

// One-glance EM health string for the main dashboard, with a colour out-param.
const char *emStatusSummaryText(uint16_t &colorOut) {
  if (emTimedOut || (!haveEmStatus && !haveEmMeasurement)) {
    colorOut = ST7735_YELLOW;
    return "NO COMMS";
  }
  if (haveEmStatus && (emStatusByte & EM_STATUS_FAULT_ACTIVE)) {
    colorOut = ST7735_RED;
    return "FAULT";
  }
  if (haveEmStatus && (emStatusByte & EM_STATUS_VIOLATION)) {
    colorOut = ST7735_RED;
    return "VIOLATION";
  }
  if (haveEmStatus && (emStatusByte & EM_STATUS_LOGGING)) {
    colorOut = ST7735_GREEN;
    return "LOGGING";
  }
  if (haveEmStatus && (emStatusByte & EM_STATUS_FAULT_PREV)) {
    colorOut = ST7735_YELLOW;
    return "IDLE/PFLT";
  }
  colorOut = ST7735_YELLOW;
  return "IDLE";
}

// The 0x0A9 Internal Voltages broadcast (GLV voltage source) can be disabled
// in the inverter's EEPROM broadcast mask. If the inverter is alive but 0x0A9
// stays silent, write parameter 148 to enable all broadcast messages.
// EEPROM parameters are only writable while the motor is not running.
void servicePm100BroadcastEnable() {
  if (haveGlvVoltage || !havePm100Traffic || readyToDriveLatched) {
    return;
  }

  if (pm100BroadcastEnableAttempts >= PM100_BROADCAST_ENABLE_MAX_ATTEMPTS) {
    return;
  }

  uint32_t now = millis();
  if (now - firstPm100FrameMs < PM100_BROADCAST_ENABLE_DELAY_MS) {
    return;
  }

  if (pm100BroadcastEnableAttempts > 0 &&
      now - pm100BroadcastEnableLastTxMs < PM100_BROADCAST_ENABLE_RETRY_MS) {
    return;
  }

  pm100BroadcastEnableAttempts++;
  pm100BroadcastEnableLastTxMs = now;

  Serial.print("[PM100] No 0x0A9 Internal Voltages frames; enabling broadcasts, attempt ");
  Serial.println(pm100BroadcastEnableAttempts);
  sendPm100BroadcastEnable();
}

void sendPm100BroadcastEnable() {
  CAN_message_t tx = {};
  tx.id = PM100_PARAM_COMMAND_ID;
  tx.len = 8;
  tx.flags.extended = 0;
  tx.buf[0] = (uint8_t)(PM100_PARAM_CAN_ACTIVE_MSGS & 0xFF);
  tx.buf[1] = (uint8_t)(PM100_PARAM_CAN_ACTIVE_MSGS >> 8);
  tx.buf[2] = 1;     // write
  tx.buf[3] = 0;     // reserved
  tx.buf[4] = 0xFF;  // CAN Active Messages Lo Word: enable all broadcasts
  tx.buf[5] = 0xFF;
  tx.buf[6] = 0xFF;  // Hi Word: keep command/BMS/OBD2 mailboxes enabled
  tx.buf[7] = 0xFF;
  sendCanFrame(tx);
}

// Automatically clears latched PM100 faults over CAN once the conditions that
// caused them are gone: the BMS must report contactors closed and the inverter
// must see pack voltage on its DC bus again. Without this, an under-voltage
// fault latched during an HV cycle or pack sag blocks torque until someone
// power-cycles the inverter. Fault Clear is a command parameter (not an EEPROM
// write), so it is safe to send any time; a fault whose cause is still present
// simply latches again.
void servicePm100FaultClear() {
  if (!havePm100FaultStatus || !pm100Fault) {
    return;
  }

  if (!haveBmsStatus || contactorsOpen()) {
    pm100FaultClearAttempts = 0;
    return;
  }

  if (!haveInverterVoltageInfo ||
      inverterDcBusVoltage < PM100_FAULT_CLEAR_MIN_BUS_V) {
    return;
  }

  if (pm100FaultClearAttempts >= PM100_FAULT_CLEAR_MAX_ATTEMPTS) {
    return;
  }

  uint32_t now = millis();
  if (pm100FaultClearAttempts > 0 &&
      now - pm100FaultClearLastTxMs < PM100_FAULT_CLEAR_RETRY_MS) {
    return;
  }

  pm100FaultClearAttempts++;
  pm100FaultClearLastTxMs = now;

  Serial.print("[PM100] Fault latched with contactors closed and bus at ");
  Serial.print(inverterDcBusVoltage, 1);
  Serial.print(" V; sending fault clear, attempt ");
  Serial.println(pm100FaultClearAttempts);
  sendPm100FaultClear();
}

void sendPm100FaultClear() {
  CAN_message_t tx = {};
  tx.id = PM100_PARAM_COMMAND_ID;
  tx.len = 8;
  tx.flags.extended = 0;
  tx.buf[0] = (uint8_t)(PM100_PARAM_FAULT_CLEAR & 0xFF);
  tx.buf[1] = (uint8_t)(PM100_PARAM_FAULT_CLEAR >> 8);
  tx.buf[2] = 1;  // write
  tx.buf[3] = 0;  // reserved
  tx.buf[4] = 0;  // data 0 = clear all faults
  tx.buf[5] = 0;
  tx.buf[6] = 0;
  tx.buf[7] = 0;
  sendCanFrame(tx);
}

void handlePm100FaultCodesFrame(const CAN_message_t &msg) {
  if (msg.len < 8) {
    return;
  }

  bool previousFault = havePm100FaultStatus && pm100Fault;
  pm100PostFaults = u32Le(msg.buf[0], msg.buf[1], msg.buf[2], msg.buf[3]);
  pm100RunFaults = u32Le(msg.buf[4], msg.buf[5], msg.buf[6], msg.buf[7]);
  havePm100FaultStatus = true;
  pm100Fault = (pm100PostFaults != 0 || pm100RunFaults != 0);
  if (pm100Fault && !previousFault) {
    showFaultBanner(firstPm100FaultName(pm100PostFaults, pm100RunFaults));
  }
  if (!pm100Fault) {
    if (previousFault && pm100FaultClearAttempts > 0) {
      Serial.println("[PM100] Faults cleared");
    }
    pm100FaultClearAttempts = 0;
  }
  screenDirty = true;
}

void setupTssi() {
  pinMode(TSSI_LED_RED, OUTPUT);
  pinMode(TSSI_LED_GREEN, OUTPUT);
  setTssiAllOff();

  imdFault = true;
  imdCanTimedOut = true;
  lastImdRxMs = millis();
  tssiBlinkTimerMs = millis();
  lastImdGetReqMs = 0;
}

void serviceTssi() {
  uint32_t now = millis();
  bool faultDetectionArmed = bmsImdFaultDetectionArmed();

  if (faultDetectionArmed && now - lastImdRxMs > IMD_HEARTBEAT_TIMEOUT_MS) {
    if (!imdCanTimedOut) {
      Serial.println("[TSSI] IMD heartbeat lost, forcing FAULT");
      showFaultBanner("IMD Timeout");
    }
    imdCanTimedOut = true;
    imdFault = true;
  }

  if ((anyFaultActive() || contactorsOpen()) && readyToDriveLatched) {
    dropReadyToDrive(rtdDropCauseText());
  }

  if (now - lastImdGetReqMs >= IMD_GET_REQ_PERIOD_MS) {
    lastImdGetReqMs = now;
    requestImdWarnings();
  }

  // Latch the red blink the first time a fault is seen; once latched the TSSI
  // stays red until power cycle, even if the underlying fault clears.
  if (faultDetectionArmed && anyFaultActive()) {
    tssiFaultLatched = true;
  }

  if (tssiFaultLatched) {
    updateTssiLed(true, now);
  } else if (!faultDetectionArmed) {
    setTssiGreenOn();
  } else {
    updateTssiLed(anyFaultActive(), now);
  }
}

void requestImdWarnings() {
  CAN_message_t tx = {};
  tx.id = IMD_REQUEST_ID;
  tx.len = 8;
  tx.flags.extended = 0;
  tx.buf[0] = IDX_WARN_ALARMS;
  for (int i = 1; i < 8; i++) {
    tx.buf[i] = 0xFF;
  }
  sendCanFrame(tx);
}

void updateTssiLed(bool faultActive, uint32_t now) {
  if (faultActive) {
    uint32_t elapsed = now - tssiBlinkTimerMs;
    if (elapsed >= TSSI_BLINK_PERIOD_MS) {
      tssiBlinkTimerMs += TSSI_BLINK_PERIOD_MS;
      elapsed = now - tssiBlinkTimerMs;
    }

    if (elapsed < TSSI_BLINK_ON_MS) {
      if (!tssiRedOn) {
        setTssiRedOn();
      }
    } else if (tssiRedOn) {
      setTssiRedOff();
    }
  } else {
    setTssiGreenOn();
  }
}

void setTssiGreenOn() {
  digitalWrite(TSSI_LED_RED, LOW);
  digitalWrite(TSSI_LED_GREEN, HIGH);
  tssiRedOn = false;
}

void setTssiRedOn() {
  digitalWrite(TSSI_LED_GREEN, LOW);
  digitalWrite(TSSI_LED_RED, HIGH);
  tssiRedOn = true;
}

void setTssiRedOff() {
  digitalWrite(TSSI_LED_RED, LOW);
  tssiRedOn = false;
}

void setTssiAllOff() {
  digitalWrite(TSSI_LED_GREEN, LOW);
  digitalWrite(TSSI_LED_RED, LOW);
  tssiRedOn = false;
}

void printTssiWarnBits(uint16_t bits) {
  if (bits & (1u << 0)) Serial.print(" [DeviceError]");
  if (bits & (1u << 1)) Serial.print(" [HV+ConnFail]");
  if (bits & (1u << 2)) Serial.print(" [HV-ConnFail]");
  if (bits & (1u << 3)) Serial.print(" [EarthConnFail]");
  if (bits & (1u << 4)) Serial.print(" [IsoAlarm]");
  if (bits & (1u << 5)) Serial.print(" [IsoWarning]");
  if (bits & (1u << 6)) Serial.print(" [IsoOutdated]");
  if (bits & (1u << 7)) Serial.print(" [UnbalanceAlarm]");
  if (bits & (1u << 8)) Serial.print(" [Undervoltage]");
  if (bits & (1u << 9)) Serial.print(" [UnsafeToStart]");
  if (bits & (1u << 10)) Serial.print(" [EarthliftOpen]");
}

// ---------------- PM100 CAN command output ----------------
void setupPm100CommandTimer() {
  updatePm100CommandState();
  pm100CommandTimerIsr();
  pm100CommandTimer.begin(pm100CommandTimerIsr, PM100_CMD_PERIOD_US);
  Serial.println("[PM100] 100 Hz command timer started");
}

void pm100CommandTimerIsr() {
  CAN_message_t tx = {};
  tx.id = PM100_CMD_ID;
  tx.len = 8;
  tx.flags.extended = 0;

  int16_t torqueRaw = pm100CommandTorqueRaw;
  tx.buf[0] = (uint8_t)(torqueRaw & 0xFF);
  tx.buf[1] = (uint8_t)((torqueRaw >> 8) & 0xFF);
  tx.buf[4] = pm100CommandDirection;
  tx.buf[5] = pm100CommandEnable;

  VehicleCan.write(tx);
  queuePm100TxLogFromIsr(tx);
}

void updatePm100CommandState() {
  int16_t torqueRaw = 0;
  uint8_t direction = 0x00;
  uint8_t inverterEnable = 0x00;

  bool driveSelected = readyToDriveLatched && (driveMode == 'D');
  // APPS implausibility and an active BSPC keep the inverter enabled but command
  // zero torque: updatePedal already forces desiredTorqueNm to 0 for both.
  bool torqueAllowed = driveSelected && !anyFaultActive() && !contactorsOpen();

  if (torqueAllowed) {
    // Apply the power derate cap. desiredTorqueNm stays the raw pedal request
    // (logged as desired); commandedTorqueNm below reflects the capped value.
    float commandTorqueNm = desiredTorqueNm;
    if (powerDerateActive) {
      float cap = powerDerateTorqueCapNm(motorRpm);
      if (commandTorqueNm > cap) commandTorqueNm = cap;
    }
    torqueRaw = (int16_t)roundf(commandTorqueNm * (float)TORQUE_SCALE);
    direction = pm100DirectionByteForMode(driveMode);
    inverterEnable = 0x01;
  }

  noInterrupts();
  pm100CommandTorqueRaw = torqueRaw;
  pm100CommandDirection = direction;
  pm100CommandEnable = inverterEnable;
  interrupts();

  commandedTorqueNm = (float)torqueRaw / (float)TORQUE_SCALE;
}

void queuePm100TxLogFromIsr(const CAN_message_t &msg) {
  uint8_t nextHead = (uint8_t)((pm100TxLogHead + 1u) % PM100_TX_LOG_QUEUE_SIZE);
  if (nextHead == pm100TxLogTail) {
    pm100TxLogDropped++;
    return;
  }

  pm100TxLogQueue[pm100TxLogHead] = msg;
  pm100TxLogHead = nextHead;
}

bool dequeuePm100TxLog(CAN_message_t &msg) {
  bool haveMessage = false;

  noInterrupts();
  if (pm100TxLogTail != pm100TxLogHead) {
    msg = pm100TxLogQueue[pm100TxLogTail];
    pm100TxLogTail = (uint8_t)((pm100TxLogTail + 1u) % PM100_TX_LOG_QUEUE_SIZE);
    haveMessage = true;
  }
  interrupts();

  return haveMessage;
}

void servicePm100TxLogging() {
  CAN_message_t tx;
  while (dequeuePm100TxLog(tx)) {
    logCanFrame("TX", tx);
  }

  static uint32_t lastDroppedCount = 0;
  uint32_t droppedCount;

  noInterrupts();
  droppedCount = pm100TxLogDropped;
  interrupts();

  if (droppedCount != lastDroppedCount) {
    Serial.print("[CAN TX LOG] PM100 command log dropped=");
    Serial.println(droppedCount);
    lastDroppedCount = droppedCount;
  }
}

// ---------------- Pedal ----------------
void updatePedal() {
  int rawApps1 = analogRead(APPS_1_PIN);
  int rawApps2 = analogRead(APPS_2_PIN);
  rawApps1Last = rawApps1;
  rawApps2Last = rawApps2;

  float newApps1Pct = scalePercentInverted(rawApps1, APPS_1_RAW_MIN, APPS_1_RAW_MAX);
  float newApps2Pct = scalePercentNormal(rawApps2, APPS_2_RAW_MIN, APPS_2_RAW_MAX);

  if (!appsFilterInitialized) {
    apps1Pct = newApps1Pct;
    apps2Pct = newApps2Pct;
    appsFilterInitialized = true;
  } else {
    apps1Pct = APPS_EMA_ALPHA * newApps1Pct + (1.0f - APPS_EMA_ALPHA) * apps1Pct;
    apps2Pct = APPS_EMA_ALPHA * newApps2Pct + (1.0f - APPS_EMA_ALPHA) * apps2Pct;
  }

  bool apps1InRange = rawApps1 >= APPS_1_RAW_MIN - APPS_RAW_FAULT_MARGIN &&
                      rawApps1 <= APPS_1_RAW_MAX + APPS_RAW_FAULT_MARGIN;
  bool apps2InRange = rawApps2 >= APPS_2_RAW_MIN - APPS_RAW_FAULT_MARGIN &&
                      rawApps2 <= APPS_2_RAW_MAX + APPS_RAW_FAULT_MARGIN;

  float percentError = fabsf(apps1Pct - apps2Pct);
  bool wasPlausible = appsPlausible;
  bool appsAbnormal = !apps1InRange || !apps2InRange ||
                      percentError > APPS_IMPLAUSIBLE_PCT;

  if (appsAbnormal) {
    if (!apps1InRange && !apps2InRange) {
      appsFaultText = "APPS1+2 Range";
    } else if (!apps1InRange) {
      appsFaultText = "APPS1 Range";
    } else if (!apps2InRange) {
      appsFaultText = "APPS2 Range";
    } else {
      appsFaultText = "APPS Disagree";
    }
  }

  if (!appsAbnormal) {
    appsDisagreeing = false;
    appsPlausible = true;
  } else {
    uint32_t now = millis();
    if (!appsDisagreeing) {
      appsDisagreeing = true;
      appsDisagreeStartMs = now;
    }
    if (now - appsDisagreeStartMs > APPS_IMPLAUSIBILITY_PERSIST_MS) {
      appsPlausible = false;
    }
  }

  float pedalTravelPct = (apps1Pct + apps2Pct) * 0.5f;

  pedalPct = pedalTravelPct;
  if (pedalPct <= PEDAL_ZERO_DEADBAND_PCT) {
    pedalPct = 0.0f;
  } else {
    // Rescale the remaining travel back up to 0-100% so output ramps
    // smoothly from zero instead of jumping past the deadband.
    pedalPct = (pedalPct - PEDAL_ZERO_DEADBAND_PCT) * 100.0f / (100.0f - PEDAL_ZERO_DEADBAND_PCT);
  }

  // BSPC: zero torque once the brake (light-braking threshold) and
  // accelerator >25% travel are applied together. Per EV.4.7/T.4.7 this latches:
  // once tripped, torque stays suppressed until the accelerator is released
  // below 5% travel, regardless of brake state. Releasing the brake alone does
  // not restore torque.
  bool bspcWasActive = bspcActive;
  if (bspcActive) {
    // Latched: only the accelerator returning below 5% clears the cut.
    if (pedalPct < BSPC_APPS_RESET_PCT) {
      bspcActive = false;
    }
  } else if (braking && pedalPct > BSPC_APPS_THRESHOLD_PCT) {
    bspcActive = true;
  }
  if (bspcActive != bspcWasActive) {
    Serial.println(bspcActive ? "[BSPC] Active; commanding zero torque"
                              : "[BSPC] Cleared");
  }

  if (appsPlausible) {
    desiredTorqueNm = bspcActive ? 0.0f : (pedalPct / 100.0f) * MAX_TORQUE_NM;
  } else {
    desiredTorqueNm = 0.0f;
    if (wasPlausible) {
      Serial.print("[APPS] Implausible (");
      Serial.print(appsFaultText);
      Serial.print("): raw1=");
      Serial.print(rawApps1);
      Serial.print(" raw2=");
      Serial.print(rawApps2);
      Serial.print(" deviation=");
      Serial.print(percentError, 1);
      Serial.print("% for >");
      Serial.print(APPS_IMPLAUSIBILITY_PERSIST_MS);
      Serial.println(" ms");
      showFaultBanner(appsFaultText);
    }
    // T.4.2.10 only requires zero torque on implausibility; the car stays in
    // drive and torque returns once the channels agree again.
  }
}

void logAppsTelemetry() {
  if (!APPS_SERIAL_LOG_ENABLED) {
    return;
  }

  static uint32_t lastPrintMs = 0;
  uint32_t now = millis();
  if (now - lastPrintMs < APPS_SERIAL_LOG_PERIOD_MS) {
    return;
  }
  lastPrintMs = now;

  Serial.print("[APPS] raw1=");
  Serial.print(rawApps1Last);
  Serial.print(" raw2=");
  Serial.print(rawApps2Last);
  Serial.print(" apps1=");
  Serial.print(apps1Pct, 1);
  Serial.print("% apps2=");
  Serial.print(apps2Pct, 1);
  Serial.print("% pedal=");
  Serial.print(pedalPct, 1);
  Serial.print("% torque=");
  Serial.print(desiredTorqueNm, 1);
  Serial.print("Nm cmd=");
  Serial.print(commandedTorqueNm, 1);
  Serial.print("Nm bspc=");
  Serial.print(bspcActive ? "ACTIVE" : "clear");
  Serial.print(" plausible=");
  Serial.println(appsPlausible ? "YES" : "NO");
}

void updateBrakePressureSensors() {
  rawBse1Last = analogRead(BSE_1_PIN);
  rawBse2Last = analogRead(BSE_2_PIN);

  braking = (rawBse1Last > BSE_1_BRAKING_RAW_THRESHOLD) ||
            (rawBse2Last > BSE_2_BRAKING_RAW_THRESHOLD);
  hardBraking = (rawBse1Last > BSE_1_HARD_BRAKING_RAW_THRESHOLD) ||
                (rawBse2Last > BSE_2_HARD_BRAKING_RAW_THRESHOLD);

  bse1Voltage = adcCountsToVoltage(rawBse1Last);
  bse2Voltage = adcCountsToVoltage(rawBse2Last);

  bse1Pct = scaleBrakePressurePercent(bse1Voltage);
  bse2Pct = scaleBrakePressurePercent(bse2Voltage);
}

float scalePercentNormal(int rawValue, int rawMin, int rawMax) {
  float scaled = (float)(rawValue - rawMin) * 100.0f / (float)(rawMax - rawMin);
  return constrain(scaled, 0.0f, 100.0f);
}

float scalePercentInverted(int rawValue, int rawMin, int rawMax) {
  float scaled = 100.0f - ((float)(rawValue - rawMin) * 100.0f / (float)(rawMax - rawMin));
  return constrain(scaled, 0.0f, 100.0f);
}

float adcCountsToVoltage(int rawValue) {
  return ((float)rawValue * ADC_REFERENCE_VOLTAGE) / (float)ADC_MAX_COUNTS;
}

float scaleBrakePressurePercent(float voltage) {
  float scaled = (voltage - BSE_SENSOR_MIN_VOLTAGE) * 100.0f /
                 (BSE_SENSOR_MAX_VOLTAGE - BSE_SENSOR_MIN_VOLTAGE);
  scaled = constrain(scaled, 0.0f, 100.0f);
  return roundf(scaled * 100.0f) / 100.0f;
}

// ---------------- Ready to drive state machine ----------------
void serviceReadyToDrive() {
  refreshRtdStateFromInputs();

  bool pressed = readReadyButtonPressed();

  if (pressed && !readyButtonLast) {
    handleRtdButtonPressed();
    screenDirty = true;
  }

  readyButtonLast = pressed;
}

bool readReadyButtonPressed() {
  return digitalRead(READY_BUTTON_PIN) == HIGH;
}

bool rtdInputsReady() {
  if (!bmsImdFaultDetectionArmed()) {
    return false;
  }

  // APPS implausibility intentionally does not gate RTD; it zeroes the torque
  // command instead (see updatePedal/updatePm100CommandState).
  return !anyFaultActive() && !contactorsOpen();
}

bool readyForRtdButton() {
  return rtdState == RTD_STATE_READY_FOR_BUTTON;
}

void refreshRtdStateFromInputs() {
  if (rtdState == RTD_STATE_READY_TO_DRIVE) {
    if (!rtdInputsReady()) {
      dropReadyToDrive(rtdDropCauseText());
    }
    return;
  }

  bool inputsReady = rtdInputsReady();
  if (!inputsReady) {
    logRtdBlockedReason();
  }

  setRtdState(inputsReady ? RTD_STATE_READY_FOR_BUTTON : RTD_STATE_FAULT_BLOCKED);
}

// Periodically prints why RTD_STATE_FAULT_BLOCKED hasn't cleared, e.g. so a
// stuck contactor/fault/IMD flag after an HV power cycle shows up in the
// serial log instead of just leaving the dashboard stuck on "N".
void logRtdBlockedReason() {
  static uint32_t lastLogMs = 0;
  uint32_t now = millis();
  if (now - lastLogMs < RTD_BLOCKED_LOG_PERIOD_MS) {
    return;
  }
  lastLogMs = now;

  if (!bmsImdFaultDetectionArmed()) {
    Serial.println("[RTD] Blocked: startup fault-detection delay active");
    return;
  }

  Serial.print("[RTD] Blocked:");
  if (imdFault)            Serial.print(" imdFault");
  if (bmsFault)            Serial.print(" bmsFault");
  if (contactorsOpen())    Serial.print(" contactorsOpen");
  if (imdCanTimedOut)      Serial.print(" imdCanTimedOut");
  if (bmsTimedOut)         Serial.print(" bmsTimedOut");
  Serial.print(" bmsFlags=0x");
  printCanHexByte(bmsFlags);
  Serial.print(" imdWarnAlarms=0x");
  Serial.print(imdWarnAlarms, HEX);
  Serial.println();
}

void handleRtdButtonPressed() {
  Serial.print("[RTD] Button pressed, state=");
  Serial.println(rtdStateName(rtdState));

  switch (rtdState) {
    case RTD_STATE_READY_FOR_BUTTON:
      if (REQUIRE_BRAKE_FOR_RTD && !braking) {
        Serial.println("[RTD] Not latched; brake pedal must be pressed");
        break;
      }
      setRtdState(RTD_STATE_READY_TO_DRIVE);
      playTonePattern(RTD_PATTERN, sizeof(RTD_PATTERN) / sizeof(RTD_PATTERN[0]), false);
      Serial.println("[RTD] Ready to drive latched");
      break;

    case RTD_STATE_READY_TO_DRIVE:
      dropReadyToDrive("RTD button pressed");
      break;

    case RTD_STATE_FAULT_BLOCKED:
    default:
      setRtdState(RTD_STATE_FAULT_BLOCKED);
      Serial.println(anyFaultActive() ? "[RTD] Not latched; anyFault is active"
                                       : "[RTD] Not latched; contactors open");
      break;
  }
}

void setRtdState(RtdState newState) {
  char newDriveMode = (newState == RTD_STATE_READY_TO_DRIVE) ? 'D' : 'N';

  if (rtdState == newState) {
    readyToDriveLatched = (newState == RTD_STATE_READY_TO_DRIVE);
    if (driveMode != newDriveMode) {
      setDriveMode(newDriveMode);
      screenDirty = true;
    }
    return;
  }

  rtdState = newState;
  readyToDriveLatched = (newState == RTD_STATE_READY_TO_DRIVE);
  setDriveMode(newDriveMode);
  screenDirty = true;
}

void dropReadyToDrive(const char *reason) {
  bool wasLatched = (rtdState == RTD_STATE_READY_TO_DRIVE) || readyToDriveLatched;

  setRtdState(rtdInputsReady() ? RTD_STATE_READY_FOR_BUTTON : RTD_STATE_FAULT_BLOCKED);

  if (wasLatched && reason != nullptr) {
    Serial.print("[RTD] Dropped; ");
    Serial.println(reason);
    if (strcmp(reason, "RTD button pressed") != 0) {
      showFaultBanner(reason);
    }
  }
}

// Names the specific condition currently blocking RTD, for serial logs and
// the dashboard fault banner.
const char *rtdDropCauseText() {
  if (bmsTimedOut && haveBmsStatus) return "BMS Timeout";
  if (imdCanTimedOut) return "IMD Timeout";
  if (bmsFault) {
    return (haveBms2026FaultCode && bms2026FaultCode != 0)
               ? bmsTemperatureFaultCodeName(bms2026FaultCode)
               : "BMS Fault";
  }
  if (imdFault) return firstTssiFaultName(imdWarnAlarms);
  if (contactorsOpen()) return "Contactors Open";
  return "Fault";
}

void showFaultBanner(const char *text) {
  if (text == nullptr || text[0] == '\0') {
    return;
  }

  if (!faultBannerVisible || strcmp(text, faultBannerText) != 0) {
    strncpy(faultBannerText, text, sizeof(faultBannerText) - 1);
    faultBannerText[sizeof(faultBannerText) - 1] = '\0';
    faultBannerDirty = true;
  }
  faultBannerVisible = true;
  faultBannerUntilMs = millis() + FAULT_BANNER_DURATION_MS;
  screenDirty = true;
}

void serviceFaultBanner() {
  if (!faultBannerVisible) {
    return;
  }

  if ((int32_t)(millis() - faultBannerUntilMs) >= 0) {
    faultBannerVisible = false;
    faultBannerText[0] = '\0';
    dashboardNeedsFullRedraw = true;
    screenDirty = true;
  }
}

// Draws the fault banner over the drive-button row; the row is repainted via
// a full redraw when the banner expires.
void drawFaultBanner(bool force) {
  if (!force && !faultBannerDirty) {
    return;
  }
  faultBannerDirty = false;

  int y = (int)(tft.height() * 0.72f) - 8;
  int h = 54 + 16;

  tft.fillRect(0, y, tft.width(), h, ST7735_RED);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(12, y + 8);
  tft.print("FAULT");
  tft.setTextSize(3);
  tft.setCursor(12, y + 32);
  tft.print(faultBannerText);
  tft.setTextColor(ST7735_WHITE);
}

const char *rtdStateName(RtdState state) {
  switch (state) {
    case RTD_STATE_FAULT_BLOCKED:
      return "FAULT_BLOCKED";
    case RTD_STATE_READY_FOR_BUTTON:
      return "READY_FOR_BUTTON";
    case RTD_STATE_READY_TO_DRIVE:
      return "READY_TO_DRIVE";
    default:
      return "UNKNOWN";
  }
}

void setDriveMode(char newDriveMode) {
  if (newDriveMode != 'D' && newDriveMode != 'N') {
    return;
  }

  if (driveMode == newDriveMode) {
    return;
  }

  driveMode = newDriveMode;
  screenDirty = true;
}

uint8_t pm100DirectionByteForMode(char mode) {
  (void)mode;
  return 0x00;
}

// ---------------- Buzzer ----------------
void playTonePattern(const ToneStep *pattern, uint8_t length, bool repeats) {
  if (pattern == nullptr || length == 0) {
    return;
  }

  activeTonePattern = pattern;
  activeToneLength = length;
  activeToneIndex = 0;
  activeToneInGap = false;
  activeToneRepeats = repeats;
  activeToneDeadlineMs = millis() + activeTonePattern[0].onMs;
  tone(BUZZER_PIN, activeTonePattern[0].frequencyHz);
}

void serviceBuzzer() {
  // 3-tone RTD-ready chime disabled for testing.
  // bool shouldChimeForRtd = readyForRtdButton();
  //
  // if (activeTonePattern != RTD_PATTERN) {
  //   if (shouldChimeForRtd) {
  //     if (activeTonePattern != TS_READY_CHIME_PATTERN) {
  //       playTonePattern(TS_READY_CHIME_PATTERN,
  //                       sizeof(TS_READY_CHIME_PATTERN) / sizeof(TS_READY_CHIME_PATTERN[0]),
  //                       true);
  //     }
  //   } else if (activeTonePattern == TS_READY_CHIME_PATTERN) {
  //     stopBuzzer();
  //     return;
  //   }
  // }

  if (activeTonePattern == nullptr) {
    return;
  }

  uint32_t now = millis();
  if ((int32_t)(now - activeToneDeadlineMs) < 0) {
    return;
  }

  const ToneStep &step = activeTonePattern[activeToneIndex];

  if (!activeToneInGap && step.offMs > 0) {
    noTone(BUZZER_PIN);
    activeToneInGap = true;
    activeToneDeadlineMs = now + step.offMs;
    return;
  }

  activeToneIndex++;
  activeToneInGap = false;

  if (activeToneIndex >= activeToneLength) {
    if (activeToneRepeats) {
      activeToneIndex = 0;
      activeToneInGap = false;
      tone(BUZZER_PIN, activeTonePattern[activeToneIndex].frequencyHz);
      activeToneDeadlineMs = now + activeTonePattern[activeToneIndex].onMs;
    } else {
      stopBuzzer();
    }
    return;
  }

  tone(BUZZER_PIN, activeTonePattern[activeToneIndex].frequencyHz);
  activeToneDeadlineMs = now + activeTonePattern[activeToneIndex].onMs;
}

void stopBuzzer() {
  noTone(BUZZER_PIN);
  activeTonePattern = nullptr;
  activeToneLength = 0;
  activeToneIndex = 0;
  activeToneInGap = false;
  activeToneRepeats = false;
  activeToneDeadlineMs = 0;
}
