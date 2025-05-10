#include "../include/ReminderSystem.h"
#include "../include/HomeScreen.h"
#include "../include/Effects.h"
#include <Arduino.h>

extern void turnOnDisplay();
extern void performFullWake();
extern void resetAllScreenFlags();

bool ReminderSystem::reminderTriggered = false;

ReminderSystem::ReminderSystem(int buttonPin, TFT_eSPI &display, bool &sleepState, DataLogger &logger)
  : buttonPin(buttonPin),
    isAsleep(sleepState),
    tft(display),
    dataLogger(logger),
    reminderScreen(display, logger) {
  pinMode(buttonPin, INPUT_PULLUP);
}

void ReminderSystem::checkReminder() {
  /* This function now does exactly one job:
     - wait until flashing is done, then show ReminderScreen. */

  if (reminderTriggered && !Effects::isScreenFlashing() && !pendingReminderScreen) {
    Serial.println("[REM] Flash finished, scheduling ReminderScreen…");
    reminderTriggered = false;
    pendingReminderScreen = true;
    flashDoneAt = millis();
    return;
  }

  if (pendingReminderScreen && (millis() - flashDoneAt >= 200)) {
    if (isAsleep) {
      turnOnDisplay();
      isAsleep = false;
    }

    resetAllScreenFlags();
    ReminderScreen::isActive = true;
    reminderScreen.show();

    pendingReminderScreen = false;
    Serial.println("[REM] ReminderScreen displayed.");
  }
}

void ReminderSystem::triggerReminder() {
  Serial.println("Reminder triggered: Time to take a measurement!");

  if (isAsleep) {
    performFullWake();
  }

  reminderTriggered = true;

  // --- Start flashing first ---
  Effects::startVibration(3, 1000, 500);
  Effects::startToneSequence({
    {880, 200}, {988, 200}, {1047, 250}, {0, 200}, {1047, 100}, {1319, 150}
  });
  Effects::startScreenFlash(3, 1000, 500);

  // Wait until flashing is complete to show the ReminderScreen
}

void ReminderSystem::dismissReminder() {
  Serial.println("Reminder dismissed via hardware button.");
  ReminderScreen::isActive = false;
  reminderScreen.dismissReminder();
}

void ReminderSystem::prepareForSleep() {
  reminderScreen.prepare(); // prep the reminder screen to take some load off CPU during reminder after wakeup
}
