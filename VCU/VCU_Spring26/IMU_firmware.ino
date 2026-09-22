#include <Wire.h>
#include <math.h>

#define I2CBUS Wire   // Use Wire for pins 18 (SDA) and 19 (SCL)

const uint8_t MPU = 0x68;

int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int AcXcal, AcYcal, AcZcal, GyXcal, GyYcal, GyZcal, tcal;

double t, tx, tf;
double pitch, roll;

void getAngle(int16_t ax, int16_t ay, int16_t az);

void getAngle(int16_t ax, int16_t ay, int16_t az) {
  double x = (double)(ax + AcXcal);
  double y = (double)(ay + AcYcal);
  double z = (double)(az + AcZcal);

  roll  = atan2(y, z) * 180.0 / M_PI;
  pitch = atan2(-x, sqrt(y * y + z * z)) * 180.0 / M_PI;
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  I2CBUS.begin();            // Start I2C on pins 18/19
  I2CBUS.setClock(100000);   // 100 kHz

  // Wake up MPU6050
  I2CBUS.beginTransmission(MPU);
  I2CBUS.write(0x6B);
  I2CBUS.write(0x00);
  uint8_t err = I2CBUS.endTransmission(true);

  if (err == 0) {
    Serial.println("MPU6050 connected");
  } else {
    Serial.print("MPU6050 init failed, error = ");
    Serial.println(err);
  }
}

void loop() {
  // Calibration values
  AcXcal = -950;
  AcYcal = -300;
  AcZcal = 0;
  tcal   = -1600;
  GyXcal = 480;
  GyYcal = 170;
  GyZcal = 210;

  I2CBUS.beginTransmission(MPU);
  I2CBUS.write(0x3B);

  if (I2CBUS.endTransmission(false) != 0) {
    Serial.println("I2C write failed");
    delay(1000);
    return;
  }

  uint8_t bytesReceived = I2CBUS.requestFrom(MPU, (uint8_t)14, (uint8_t)true);

  if (bytesReceived == 14) {
    AcX = (I2CBUS.read() << 8) | I2CBUS.read();
    AcY = (I2CBUS.read() << 8) | I2CBUS.read();
    AcZ = (I2CBUS.read() << 8) | I2CBUS.read();
    Tmp = (I2CBUS.read() << 8) | I2CBUS.read();
    GyX = (I2CBUS.read() << 8) | I2CBUS.read();
    GyY = (I2CBUS.read() << 8) | I2CBUS.read();
    GyZ = (I2CBUS.read() << 8) | I2CBUS.read();

    tx = Tmp + tcal;
    t  = tx / 340.0 + 36.53;
    tf = (t * 9.0 / 5.0) + 32.0;

    getAngle(AcX, AcY, AcZ);

    Serial.print("Angle: Pitch = ");
    Serial.print(pitch);
    Serial.print(" Roll = ");
    Serial.println(roll);

    Serial.print("Accelerometer: X = ");
    Serial.print(AcX + AcXcal);
    Serial.print(" Y = ");
    Serial.print(AcY + AcYcal);
    Serial.print(" Z = ");
    Serial.println(AcZ + AcZcal);

    Serial.print("Temperature C = ");
    Serial.print(t);
    Serial.print(" F = ");
    Serial.println(tf);

    Serial.print("Gyroscope: X = ");
    Serial.print(GyX + GyXcal);
    Serial.print(" Y = ");
    Serial.print(GyY + GyYcal);
    Serial.print(" Z = ");
    Serial.println(GyZ + GyZcal);

    Serial.println();
  } else {
    Serial.println("I2C read failed");
  }

  delay(1000);
}
