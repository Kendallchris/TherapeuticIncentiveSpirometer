#include "DataLogger.h"
#include "DataStorage.h"

DataLogger::DataLogger() {
  // Load data from EEPROM, handling uninitialized values
  currentHourMeasurements = DataStorage::loadCurrentHourMeasurements();
  previousHourMeasurements = DataStorage::loadPreviousHourMeasurements();

  if (currentHourMeasurements == 255) {
    currentHourMeasurements = 0;
    DataStorage::saveCurrentHourMeasurements(currentHourMeasurements);
  }

  if (previousHourMeasurements == 255) {
    previousHourMeasurements = 0;
    DataStorage::savePreviousHourMeasurements(previousHourMeasurements);
  }
}

void DataLogger::incrementMeasurement() {
  currentHourMeasurements++;                                          // Increment without limit
  DataStorage::saveCurrentHourMeasurements(currentHourMeasurements);  // Save updated count to storage
}

int DataLogger::getCurrentHourMeasurements(bool displayFull) {
  // Return the full count if displayFull is true, else limit to 10
  return displayFull ? currentHourMeasurements : (currentHourMeasurements > 10 ? 10 : currentHourMeasurements);
}

int DataLogger::getPreviousHourMeasurements() {
  return previousHourMeasurements;
}

void DataLogger::resetData() {
  currentHourMeasurements = 0;
  DataStorage::saveCurrentHourMeasurements(currentHourMeasurements);
}
