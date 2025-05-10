#ifndef DATALOGGER_H
#define DATALOGGER_H

class DataLogger {
public:
  DataLogger();                      // Constructor
  void incrementMeasurement();        // Increment measurement count
  int getCurrentMeasurements();       // Get the current measurement count
  void resetData();                   // Reset current measurements

private:
  int currentMeasurements;            // Stores measurement count
};

#endif
