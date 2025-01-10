#include "MeasurementScreen.h"

MeasurementScreen::MeasurementScreen(TFT_eSPI &display) : tft(display) {
    // Initialization if needed
}

void MeasurementScreen::showWaitingWithCountdown() {
    unsigned long startTime = millis();
    int countdownDuration = 10000; // 10 seconds
    int secondsLeft = 10;

    // Draw static elements once
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.println("Waiting for object...");
    tft.setCursor(10, 40);
    tft.println("Press green button to cancel");

    while (millis() - startTime < countdownDuration) {
        int newSecondsLeft = 10 - (millis() - startTime) / 1000;
        if (newSecondsLeft != secondsLeft) {
            secondsLeft = newSecondsLeft;
            tft.fillRect(10, 70, 220, 30, TFT_BLACK);  // Clear just the countdown area
            tft.setCursor(10, 70);
            tft.print("Beginning measurement in ");
            tft.print(secondsLeft);
            tft.print(" seconds");
        }
    }
}

void MeasurementScreen::showSuccess() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.println("SUCCESS!");
    delay(2000);  // Hold the message for 2 seconds
}

void MeasurementScreen::showNoObject() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.println("No object detected yet.");
}
