#ifndef HOMESCREEN_H
#define HOMESCREEN_H

#include <TFT_eSPI.h>  // Include the TFT library

class HomeScreen {
public:
  HomeScreen(TFT_eSPI &display);  // Constructor that takes the display instance
  void show();

private:
  TFT_eSPI &tft;  // Reference to the display instance
};

#endif
