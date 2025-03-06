#include "DataLogger.h"
#include "DataStorage.h"
#include <Arduino.h>  // for Serial if needed

DataLogger::DataLogger() {
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
  // Ensure we never exceed 10 stored measurements
  if (currentHourMeasurements < 10) {
    currentHourMeasurements++;
    DataStorage::saveCurrentHourMeasurements(currentHourMeasurements);
    Serial.print("[DEBUG] Measurements incremented to: ");
    Serial.println(currentHourMeasurements);
  } else {
    Serial.println("[DEBUG] Measurements already at 10; not incrementing further.");
  }
}

int DataLogger::getCurrentHourMeasurements(bool displayFull) {
  // Return the full count if displayFull is true, else limit to 10
  return displayFull ? currentHourMeasurements
                     : (currentHourMeasurements > 10 ? 10 : currentHourMeasurements);
}

int DataLogger::getPreviousHourMeasurements() {
  return previousHourMeasurements;
}

void DataLogger::resetData() {
  currentHourMeasurements = 0;
  DataStorage::saveCurrentHourMeasurements(currentHourMeasurements);
}
