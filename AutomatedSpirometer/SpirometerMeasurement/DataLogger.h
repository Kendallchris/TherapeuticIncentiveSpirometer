#ifndef DATALOGGER_H
#define DATALOGGER_H

class DataLogger {
public:
    DataLogger();                           // Constructor
    void incrementMeasurement();            // Increment current hour measurement count
    int getCurrentHourMeasurements(bool displayFull = false); // Get the count for the current hour, optionally displaying full count
    int getPreviousHourMeasurements();      // Get the count for the previous hour
    void resetHourlyData();                 // Reset current hour data and move it to previous hour

private:
    int currentHourMeasurements;            // Stores current hour measurement count
    int previousHourMeasurements;           // Stores previous hour measurement count
};

#endif
