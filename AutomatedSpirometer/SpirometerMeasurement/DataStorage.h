#ifndef DATASTORAGE_H
#define DATASTORAGE_H

class DataStorage {
public:
  static void saveCurrentHourMeasurements(int measurements);   // Save current hour measurement count
  static int loadCurrentHourMeasurements();                    // Load current hour measurement count
  static void savePreviousHourMeasurements(int measurements);  // Save previous hour measurement count
  static int loadPreviousHourMeasurements();                   // Load previous hour measurement count
};

#endif
