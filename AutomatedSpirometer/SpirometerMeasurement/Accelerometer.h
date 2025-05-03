#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <Wire.h>

class Accelerometer {
public:
  Accelerometer(uint8_t address, float tiltThreshold);

  void initialize();
  void saveReferenceOrientation();
  bool detectTilt();
  void setupMotionInterrupt();
  void printInterruptSource();
  void clearInterrupt();

  //int16_t refX, refY, refZ;  // Reference orientation values

private:
  uint8_t i2cAddress;        // I2C address for the accelerometer
  int16_t refX, refY, refZ;  // Reference orientation values
  float tiltAngleThreshold;  // Tilt threshold angle (degrees)
};

#endif
