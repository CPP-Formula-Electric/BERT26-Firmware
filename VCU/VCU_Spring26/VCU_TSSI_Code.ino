#include <Arduino.h>
#include <FlexCAN_T4.h>

// CAN settings
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

static constexpr uint32_t BMS_TO_ECU_ID = 0x100;

// LED pins
static const int TSSI_GREEN = 36;
static const int GPIO_GREEN = 8;
static const int TSSI_RED   = 35;
static const int GPIO_RED   = 9;

// Blink timing for fault on RED: 3 Hz, 50% duty cycle
static constexpr unsigned long BLINK_PERIOD_MS = 333;
static constexpr unsigned long BLINK_ON_MS     = 166;

// Internal state
volatile bool imdFault = false;
volatile uint16_t lastWarnBits = 0;
volatile unsigned long lastCanRxMs = 0;
static constexpr unsigned long HEARTBEAT_TIMEOUT_MS = 500;

unsigned long blinkTimer = 0;
bool redOn = false;
bool canTimedOut = false;

// ---------- Helper: print warning bits ----------
void printWarnBits(uint16_t bits) {
  if (bits & (1 << 0))  Serial.print(" [DeviceError]");
  if (bits & (1 << 1))  Serial.print(" [HV+ConnFail]");
  if (bits & (1 << 2))  Serial.print(" [HV-ConnFail]");
  if (bits & (1 << 3))  Serial.print(" [EarthConnFail]");
  if (bits & (1 << 4))  Serial.print(" [IsoAlarm]");
  if (bits & (1 << 5))  Serial.print(" [IsoWarning]");
  if (bits & (1 << 6))  Serial.print(" [IsoOutdated]");
  if (bits & (1 << 7))  Serial.print(" [UnbalanceAlarm]");
  if (bits & (1 << 8))  Serial.print(" [Undervoltage]");
  if (bits & (1 << 9))  Serial.print(" [UnsafeToStart]");
  if (bits & (1 << 10)) Serial.print(" [EarthliftOpen]");
}

// ---------- CAN receive callback ----------
void canRx(const CAN_message_t &msg) {
  if (msg.id == BMS_TO_ECU_ID && msg.len >= 3) {
    uint8_t flags    = msg.buf[0];
    uint8_t warn_lsb = msg.buf[1];
    uint8_t warn_msb = msg.buf[2];
    uint16_t warnbits = (uint16_t)warn_lsb | ((uint16_t)warn_msb << 8);

    imdFault = (flags & 0x01) != 0;
    lastWarnBits = warnbits;
    lastCanRxMs = millis();

    if (canTimedOut) {
      Serial.println("[ECU] CAN communication restored");
      canTimedOut = false;
    }

    Serial.print("[ECU RX] ID=0x");
    Serial.print(msg.id, HEX);
    Serial.print(" LEN=");
    Serial.print(msg.len);
    Serial.print(" DATA=");
    for (int i = 0; i < msg.len; i++) {
      if (msg.buf[i] < 0x10) Serial.print("0");
      Serial.print(msg.buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    Serial.print("         fault=");
    Serial.print(imdFault ? "YES" : "NO");
    Serial.print(" warnBits=0x");
    Serial.print(lastWarnBits, HEX);
    printWarnBits(lastWarnBits);
    Serial.println();

    Serial.print("         LED state -> ");
    if (imdFault) Serial.println("RED BLINK");
    else          Serial.println("GREEN ON");

    Serial.println();
  }
}

// ---------- LED control ----------
void setGreenOn() {
  digitalWrite(TSSI_RED, LOW);
  digitalWrite(TSSI_GREEN, HIGH);
  redOn = false;
}

void setRedOn() {
  digitalWrite(TSSI_GREEN, LOW);
  digitalWrite(TSSI_RED, HIGH);
  redOn = true;
}

void setRedOff() {
  digitalWrite(TSSI_RED, LOW);
  redOn = false;
}

void setAllOff() {
  digitalWrite(TSSI_GREEN, LOW);
  digitalWrite(TSSI_RED, LOW);
  redOn = false;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  Serial.println("ECU Teensy CAN LED controller starting...");

  pinMode(TSSI_GREEN, OUTPUT);
  pinMode(TSSI_RED, OUTPUT);
  pinMode(GPIO_GREEN, INPUT_PULLUP);
  pinMode(GPIO_RED, INPUT_PULLUP);

  setAllOff();

  Can3.begin();
  Can3.setBaudRate(500000);
  Can3.setMaxMB(16);
  Can3.enableFIFO();
  Can3.enableFIFOInterrupt();
  Can3.onReceive(canRx);

  blinkTimer = millis();
  lastCanRxMs = millis();

  Serial.print("Listening for BMS status on 0x");
  Serial.println(BMS_TO_ECU_ID, HEX);
  Serial.println();
}

// ---------- Main loop ----------
void loop() {
  Can3.events();

  unsigned long now = millis();

  // If heartbeat lost, force fault so RED keeps blinking
  if ((now - lastCanRxMs) > HEARTBEAT_TIMEOUT_MS) {
    if (!canTimedOut) {
      Serial.println("[ECU] CAN heartbeat lost -> forcing RED BLINK");
      canTimedOut = true;
    }
    imdFault = true;
  }

  if (imdFault) {
    unsigned long elapsed = now - blinkTimer;
    if (elapsed >= BLINK_PERIOD_MS) {
      blinkTimer += BLINK_PERIOD_MS;
      elapsed = now - blinkTimer;
    }

    if (elapsed < BLINK_ON_MS) {
      if (!redOn) setRedOn();
    } else {
      if (redOn) setRedOff();
    }
  } else {
    setGreenOn();
  }

  delay(2);
}
