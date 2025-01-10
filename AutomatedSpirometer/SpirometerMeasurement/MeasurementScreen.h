#ifndef MEASUREMENTSCREEN_H
#define MEASUREMENTSCREEN_H

#include <TFT_eSPI.h>  // Include the TFT library

class MeasurementScreen {
public:
    MeasurementScreen(TFT_eSPI &display);    // Constructor that takes the display instance
    void showWaitingWithCountdown();         // Show "Waiting for object..." screen with countdown
    void showSuccess();                      // Show "SUCCESS!" screen
    void showNoObject();                     // Show "No object detected." screen

private:
    TFT_eSPI &tft;                           // Reference to the display instance
};

#endif
