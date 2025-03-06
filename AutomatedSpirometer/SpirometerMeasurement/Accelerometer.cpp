#include "Accelerometer.h"
#include <Arduino.h>  // For Serial and math functions

Accelerometer::Accelerometer(uint8_t address, float tiltThreshold)
  : i2cAddress(address), tiltAngleThreshold(tiltThreshold), refX(0), refY(0), refZ(0) {}

void Accelerometer::initialize() {
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x00);  // Device ID register
  Wire.endTransmission();
  Wire.requestFrom(i2cAddress, (uint8_t)1, (bool)true);

  uint8_t deviceId = Wire.read();
  if (deviceId == 0xE5) {  // Check ADXL345 device ID
    Serial.println("ADXL345 initialization successful!");
  } else {
    Serial.print("ADXL345 initialization failed. Device ID: ");
    Serial.println(deviceId, HEX);
    return;  // Exit if initialization fails
  }

  // Put the sensor into measurement mode
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x2D);  // Power control register
  Wire.write(0x08);  // Measurement mode
  Wire.endTransmission();
}

void Accelerometer::saveReferenceOrientation() {
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x32);  // Start with DATAX0 register
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddress, 6, true);

  refX = Wire.read() | (Wire.read() << 8);
  refY = Wire.read() | (Wire.read() << 8);
  refZ = Wire.read() | (Wire.read() << 8);

  Serial.print("Reference Orientation - X: ");
  Serial.print(refX);
  Serial.print(", Y: ");
  Serial.print(refY);
  Serial.print(", Z: ");
  Serial.println(refZ);

  // Ensure reference orientation is non-zero
  if (refX == 0 && refY == 0 && refZ == 0) {
    Serial.println("Warning: Reference orientation is zero. Check sensor data.");
  }
}

bool Accelerometer::detectTilt() {
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x32);  // Start with DATAX0 register
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddress, 6, true);

  if (Wire.available() < 6) {
    Serial.println("Error: Insufficient data received from accelerometer.");
    return false;
  }

  int16_t x = Wire.read() | (Wire.read() << 8);
  int16_t y = Wire.read() | (Wire.read() << 8);
  int16_t z = Wire.read() | (Wire.read() << 8);

  Serial.print("Current Orientation - X: ");
  Serial.print(x);
  Serial.print(", Y: ");
  Serial.print(y);
  Serial.print(", Z: ");
  Serial.println(z);

  float dotProduct = (x * refX) + (y * refY) + (z * refZ);
  float magnitudeCurrent = sqrt((float)(x * x + y * y + z * z));
  float magnitudeRef = sqrt((float)(refX * refX + refY * refY + refZ * refZ));

  if (magnitudeCurrent == 0 || magnitudeRef == 0) {
    Serial.println("Error: Magnitude is zero!");
    return false;
  }

  float cosTheta = dotProduct / (magnitudeCurrent * magnitudeRef);
  cosTheta = constrain(cosTheta, -1.0, 1.0);
  float angle = acos(cosTheta) * 180.0 / PI;

  Serial.print("Delta Angle: ");
  Serial.println(angle);

  return (angle > tiltAngleThreshold);
}
