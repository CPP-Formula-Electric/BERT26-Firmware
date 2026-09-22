#include <ST7796_t3.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>

/* TFT & touch pins */
#define TFT_MISO   12
#define TFT_MOSI   11
#define TFT_SCK    13
#define TFT_DC      8
#define TFT_CS     10
#define TFT_RST     9      // Display reset controlled by Teensy pin 9
#define TFT_LED    29      // Display LED/backlight controlled by Teensy pin 29 (PWM)

#define CS_PIN      7
#define TIRQ_PIN    6

/* Display & Touch objects */
ST7796_t3            tft(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen  ts(CS_PIN, TIRQ_PIN);

char drive_mode = 'N';

/* Touch debug */
uint32_t lastTouchPrint = 0;
bool lastTouched = false;

/* Backlight brightness: 0-255 */
int backlightBrightness = 255;

/* Helpers */
char checkIfTouchedRaw(int x, int y);
void drawScreen();
void drawDriveButtons(char mode);
void drawTouchDebug(bool touched, int rawX, int rawY);
void drawPressedMode(char mode);
void resetDisplay();
void handleSerialBrightness();

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 1500) {}
  Serial.println("Touchscreen D/N/R test starting...");
  Serial.println("Enter backlight brightness 0-255 in Serial Monitor.");

  /* SPI pins */
  SPI.setMOSI(TFT_MOSI);
  SPI.setMISO(TFT_MISO);
  SPI.setSCK(TFT_SCK);

  /* Control TFT reset pin from Teensy */
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, HIGH);

  /* Control TFT LED/backlight from Teensy using PWM */
  pinMode(TFT_LED, OUTPUT);
  analogWriteFrequency(TFT_LED, 20000);   // 20 kHz PWM
  analogWrite(TFT_LED, backlightBrightness);

  /* Hardware reset pulse */
  resetDisplay();

  ts.begin();
  ts.setRotation(1);

  tft.init(320, 480);
  tft.setRotation(135);
  tft.fillScreen(ST7735_BLACK);

  drawScreen();
}

void loop() {
  handleSerialBrightness();

  bool touched = ts.touched();

  if (touched) {
    TS_Point p = ts.getPoint();

    int rawX = p.x;
    int rawY = p.y;

    char newMode = checkIfTouchedRaw(rawX, rawY);
    if (newMode && newMode != drive_mode) {
      drive_mode = newMode;
      drawDriveButtons(drive_mode);
      drawPressedMode(drive_mode);

      Serial.print("Button pressed: ");
      Serial.print(drive_mode);
      Serial.print("   rawX=");
      Serial.print(rawX);
      Serial.print(" rawY=");
      Serial.println(rawY);
    }

    if (millis() - lastTouchPrint >= 100) {
      lastTouchPrint = millis();
      drawTouchDebug(true, rawX, rawY);

      Serial.print("Touch raw: x=");
      Serial.print(rawX);
      Serial.print(" y=");
      Serial.println(rawY);
    }

    lastTouched = true;
  } else {
    if (lastTouched) {
      drawTouchDebug(false, 0, 0);
      lastTouched = false;
    }
  }

  delay(20);
}

/* Read brightness from Serial Monitor */
void handleSerialBrightness() {
  if (Serial.available()) {
    int value = Serial.parseInt();

    if (value >= 0 && value <= 255) {
      backlightBrightness = value;
      analogWrite(TFT_LED, backlightBrightness);

      Serial.print("Brightness set to: ");
      Serial.println(backlightBrightness);
    } else {
      Serial.println("Enter value 0-255");
    }

    while (Serial.available()) {
      Serial.read();
    }
  }
}

/* Manual hardware reset using Teensy GPIO 9 */
void resetDisplay() {
  digitalWrite(TFT_RST, HIGH);
  delay(20);
  digitalWrite(TFT_RST, LOW);
  delay(20);
  digitalWrite(TFT_RST, HIGH);
  delay(150);
}

/* Match your original raw touch regions */
char checkIfTouchedRaw(int x, int y) {
  const int y_min = 2650;
  const int y_max = 3500;

  if (x >  950 && x < 1550 && y > y_min && y < y_max) return 'D';
  if (x > 1900 && x < 2500 && y > y_min && y < y_max) return 'N';
  if (x > 2900 && x < 3500 && y > y_min && y < y_max) return 'R';

  return 0;
}

void drawScreen() {
  tft.fillScreen(ST7735_BLACK);

  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(3);
  tft.setCursor(20, 20);
  tft.print("Touch Test");

  tft.setTextSize(2);
  tft.setCursor(20, 60);
  tft.print("Tap D, N, or R");

  tft.drawRect(15, 95, 290, 70, ST7735_WHITE);
  tft.setCursor(25, 110);
  tft.setTextSize(2);
  tft.print("Raw: ");

  drawPressedMode(drive_mode);
  drawDriveButtons(drive_mode);
}

void drawPressedMode(char mode) {
  tft.fillRect(20, 180, 280, 35, ST7735_BLACK);
  tft.setCursor(20, 185);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(2);
  tft.print("Selected: ");
  tft.print(mode);
}

void drawTouchDebug(bool touched, int rawX, int rawY) {
  tft.fillRect(85, 108, 200, 40, ST7735_BLACK);
  tft.setCursor(85, 110);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(2);

  if (touched) {
    tft.print(rawX);
    tft.print(", ");
    tft.print(rawY);
  } else {
    tft.print("not touched");
  }
}

/* Draw D/N/R exactly in the style of your original code */
void drawDriveButtons(char mode) {
  int y = (int)(tft.height() * 0.70);

  /* Clear lower area first */
  tft.fillRect(0, y - 10, tft.width(), 80, ST7735_BLACK);

  tft.setTextSize(5);

  tft.setCursor(tft.width() / 4, y);
  tft.setTextColor((mode == 'D') ? ST7735_RED : ST7735_WHITE);
  tft.print("D");

  tft.setCursor(tft.width() / 2, y);
  tft.setTextColor((mode == 'N') ? ST7735_RED : ST7735_WHITE);
  tft.print("N");

  tft.setCursor((int)(tft.width() * 0.75), y);
  tft.setTextColor((mode == 'R') ? ST7735_RED : ST7735_WHITE);
  tft.print("R");

  tft.setTextColor(ST7735_WHITE);
}
