//===========================
//   MT6701 TEST READ
//===========================
#include <Wire.h>

#define MT6701_ADDR  0x06
#define ANGLE_REG_H  0x03
#define ANGLE_REG_L  0x04

void setup() {
  Serial.begin(115200);

  Wire.begin();

  // I2C timeout: 3000 milliseconds
  Wire.setWireTimeout(3000, true);

  Serial.println("MT6701 I2C Encoder");
  Serial.println("------------------");
}

bool readMT6701(uint16_t &angle) {
  uint8_t highByte, lowByte;

  // Read register 0x03
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(ANGLE_REG_H);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MT6701_ADDR, (uint8_t)1) != 1) {
    return false;
  }

  highByte = Wire.read();

  // Read register 0x04
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(ANGLE_REG_L);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MT6701_ADDR, (uint8_t)1) != 1) {
    return false;
  }

  lowByte = Wire.read();

  // Combine 14-bit angle
  angle = ((uint16_t)highByte << 6) | (lowByte >> 2);

  return true;
}

void loop() {
  uint16_t rawAngle;

  if (readMT6701(rawAngle)) {
    float angleDegrees = rawAngle * 360.0 / 16384.0;

    Serial.print("Raw: ");
    Serial.print(rawAngle);

    Serial.print(" | Angle: ");
    Serial.print(angleDegrees, 2);
    Serial.println(" degrees");
  }
  else {
    Serial.println("ERROR: Cannot read MT6701!");
  }

  delay(100);
}
