#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <Wire.h>

class Accelerometer {
public:
  Accelerometer(uint8_t address, float tiltThreshold);

  void initialize();
  void saveReferenceOrientation();
  bool detectTilt();

  int16_t refX, refY, refZ;  // Reference orientation values

private:
  uint8_t i2cAddress;        // I2C address for the accelerometer
  float tiltAngleThreshold;  // Tilt threshold angle (degrees)
};

#endif
