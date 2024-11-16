#include "DataStorage.h"
#include <EEPROM.h>  // Include EEPROM library for persistent storage

// Define EEPROM addresses for storage
const int CURRENT_HOUR_MEASUREMENTS_ADDR = 0;
const int PREVIOUS_HOUR_MEASUREMENTS_ADDR = 1;

void DataStorage::saveCurrentHourMeasurements(int measurements) {
    EEPROM.write(CURRENT_HOUR_MEASUREMENTS_ADDR, measurements);  // Save current hour to EEPROM
}

int DataStorage::loadCurrentHourMeasurements() {
    return EEPROM.read(CURRENT_HOUR_MEASUREMENTS_ADDR);          // Read current hour from EEPROM
}

void DataStorage::savePreviousHourMeasurements(int measurements) {
    EEPROM.write(PREVIOUS_HOUR_MEASUREMENTS_ADDR, measurements); // Save previous hour to EEPROM
}

int DataStorage::loadPreviousHourMeasurements() {
    return EEPROM.read(PREVIOUS_HOUR_MEASUREMENTS_ADDR);         // Read previous hour from EEPROM
}
