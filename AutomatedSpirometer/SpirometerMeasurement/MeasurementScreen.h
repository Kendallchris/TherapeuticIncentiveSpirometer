#ifndef MEASUREMENTSCREEN_H
#define MEASUREMENTSCREEN_H

#include <TFT_eSPI.h>  // Include the TFT library
#include <lvgl.h>

class MeasurementScreen {
public:
    MeasurementScreen(TFT_eSPI &display);    // Constructor that takes the display instance
    void showWaitingWithCountdown();         // Show "Waiting for object..." screen with countdown
    void updateCountdown();
    void showSuccess();                      // Show "SUCCESS!" screen
    void showNoObject();                     // Show "No object detected." screen

private:
    TFT_eSPI &tft;                           // Reference to the display instance
    lv_obj_t *countdown_label;               // Label for the countdown
    unsigned long countdown_start;           // Start time for the countdown
    bool countdown_active;                   // Flag for active countdown
    int countdown_duration = 10;             // Duration of the countdown in seconds
};

#endif
