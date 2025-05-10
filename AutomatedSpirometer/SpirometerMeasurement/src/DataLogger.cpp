#include "../include/DataLogger.h"
#include "../include/DataStorage.h"
#include <Arduino.h>  // for Serial

DataLogger::DataLogger() {
  currentMeasurements = DataStorage::loadCurrentHourMeasurements();

  if (currentMeasurements == 255) {
    currentMeasurements = 0;
    DataStorage::saveCurrentHourMeasurements(currentMeasurements);
  }
}

void DataLogger::incrementMeasurement() {
  // Ensure we never exceed 10 measurements
  if (currentMeasurements < 10) {
    currentMeasurements++;
    DataStorage::saveCurrentHourMeasurements(currentMeasurements);
    Serial.print("[DEBUG] Measurements incremented to: ");
    Serial.println(currentMeasurements);
  } else {
    Serial.println("[DEBUG] Measurements already at 10; not incrementing further.");
  }
}

int DataLogger::getCurrentMeasurements() {
  return currentMeasurements;
}

void DataLogger::resetData() {
  currentMeasurements = 0;
  DataStorage::saveCurrentHourMeasurements(currentMeasurements);
}
