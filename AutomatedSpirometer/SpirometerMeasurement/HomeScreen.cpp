#include "HomeScreen.h"

HomeScreen::HomeScreen(TFT_eSPI &display) : tft(display) {
    // Initialization of the display instance if needed
}

void HomeScreen::show(int currentHour, int previousHour) {
    tft.fillScreen(TFT_BLACK);                      // Clear the screen
    tft.setTextColor(TFT_WHITE, TFT_BLACK);         // Set text color
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.println("Press Green button");              // Display instruction message
    tft.setCursor(10, 40);
    tft.println("to begin measurement");

    // Display current hour measurements count
    tft.setCursor(10, 80);
    tft.print("Current Hour: ");
    tft.print(currentHour);
    tft.print("/10");

    // Display previous hour measurements count
    tft.setCursor(10, 120);
    tft.print("Previous Hour: ");
    tft.print(previousHour);
    tft.print("/10");
}
