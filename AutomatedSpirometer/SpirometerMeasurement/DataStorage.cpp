#include "DataStorage.h"
#include <EEPROM.h>  // Include EEPROM library for persistent storage

// Define EEPROM address for storage
const int CURRENT_HOUR_MEASUREMENTS_ADDR = 0;

void DataStorage::saveCurrentHourMeasurements(int measurements) {
  EEPROM.write(CURRENT_HOUR_MEASUREMENTS_ADDR, measurements);
}

int DataStorage::loadCurrentHourMeasurements() {
  return EEPROM.read(CURRENT_HOUR_MEASUREMENTS_ADDR);
}
