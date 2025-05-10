#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <Wire.h>

class Accelerometer {
public:
  Accelerometer(uint8_t address);

  void initialize();
  void setupMotionInterrupt();
  void printInterruptSource();
  void clearInterrupt();

private:
  uint8_t i2cAddress;        // I2C address for the accelerometer
};

#endif
