#include "../include/Accelerometer.h"

Accelerometer::Accelerometer(uint8_t address)
  : i2cAddress(address) {}

void Accelerometer::initialize() {
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x00);  // Device ID register
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)i2cAddress, (uint8_t)1, (bool)true);

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

void Accelerometer::setupMotionInterrupt() {
  Serial.println("[DEBUG] Setting up motion interrupt...");

  // 1. Enter standby mode first
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x2D);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);

  // 2. Set DATA_FORMAT to full resolution, ±2g
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x31);
  Wire.write(0x08);
  Wire.endTransmission();
  delay(10);

  // 3. Set BW_RATE to 100 Hz output data rate
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x2C);
  Wire.write(0x0A);  // 0x0A = 100Hz output data rate
  Wire.endTransmission();
  delay(10);

  // 4. Set ACT_INACT_CTL - Enable Activity detection on X, Y, Z axes
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x27);  // ACT_INACT_CTL
  Wire.write(0x40);  // Only ACT_Z enable, DC-coupled
  Wire.endTransmission();
  delay(10);

  // 5. Set THRESH_ACT to moderate threshold
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x24);
  Wire.write(0x15);  // Moderate threshold
  Wire.endTransmission();
  delay(10);

  // 6. Map ACTIVITY interrupt to INT1 pin
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x2F);
  Wire.write(0x00);  // INT1
  Wire.endTransmission();
  delay(10);

  // 7. Enable ACTIVITY interrupt
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x2E);
  Wire.write(0x10);  // Enable only Activity interrupt
  Wire.endTransmission();
  delay(10);

  // 8. Enter Measurement mode
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x2D);
  Wire.write(0x08);  // Set Measure bit
  Wire.endTransmission();
  delay(10);

  // 9. Clear interrupt source register
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x30);  // INT_SOURCE
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)i2cAddress, (uint8_t)1, (bool)true);
  if (Wire.available()) {
    Wire.read();
  }

  Serial.println("[DEBUG] Accelerometer motion interrupt fully configured.");
}

void Accelerometer::clearInterrupt() {
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x30);  // INT_SOURCE register
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)i2cAddress, (uint8_t)1, (bool)true);
  if (Wire.available()) {
    Wire.read();  // Read to clear
    Serial.println("[DEBUG] Interrupt cleared.");
  } else {
    Serial.println("[ERROR] Failed to clear interrupt.");
  }
}

void Accelerometer::printInterruptSource() {
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x30);  // INT_SOURCE register
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)i2cAddress, (uint8_t)1, (bool)true);

  if (Wire.available()) {
    uint8_t intSource = Wire.read();

    Serial.print("[INT_SOURCE] Raw: 0x");
    Serial.println(intSource, HEX);

    if (intSource & 0x10) {  // Bit 4: Activity
      Serial.println("[DEBUG] Activity detected!");
    }
    if (intSource & 0x20) {  // Bit 5: Inactivity
      Serial.println("[DEBUG] Inactivity detected.");
    }
    if (intSource == 0x00) {
      Serial.println("[DEBUG] No interrupt flags set.");
    }
  } else {
    Serial.println("[ERROR] Failed to read INT_SOURCE.");
  }
}
