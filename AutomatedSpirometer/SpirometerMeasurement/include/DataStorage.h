#ifndef DATASTORAGE_H
#define DATASTORAGE_H

class DataStorage {
public:
  static void saveCurrentHourMeasurements(int measurements);   // Save current measurement count
  static int loadCurrentHourMeasurements();                    // Load current measurement count
};

#endif
