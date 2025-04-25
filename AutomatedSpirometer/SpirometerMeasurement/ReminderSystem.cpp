#include "ReminderSystem.h"
#include "Effects.h"
#include <Arduino.h>  // for Serial

extern void turnOnDisplay();
extern void wakeUp();
extern void reminderTone();
extern void stopTone();

ReminderSystem::ReminderSystem(int motorPin, int backlightPin, int buttonPin, TFT_eSPI &display, bool &sleepState, DataLogger &logger)
  : vibrationMotorPin(motorPin), screenBacklightPin(backlightPin), buttonPin(buttonPin),
    isAsleep(sleepState), lastActivityTime(millis()), tft(display), dataLogger(logger), reminderScreen(display, logger) {
  pinMode(vibrationMotorPin, OUTPUT);
  pinMode(screenBacklightPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  digitalWrite(vibrationMotorPin, LOW);
}

extern void resetAllScreenFlags();

void ReminderSystem::resetTimer() {
  lastActivityTime = millis();
}

void ReminderSystem::checkReminder() {
  if (ReminderScreen::isActive && digitalRead(buttonPin) == LOW) {
    Serial.println("Dismissing reminder via hardware button...");
    dismissReminder();
    delay(300);
    return;
  }

  // Only show ReminderScreen if flashing is done *AND* reminder was actually triggered
  if (reminderTriggered && !Effects::isScreenFlashing()) {
    Serial.println("Flashing complete. Now showing reminder screen.");

    reminderTriggered = false;  // ✅ Reset flag after using it

    resetAllScreenFlags();
    ReminderScreen::isActive = true;
    reminderScreen.show();
    lv_scr_load(lv_scr_act());
    lv_refr_now(NULL);

    turnOnDisplay();
  }

  // Timer check
  if (millis() - lastActivityTime >= reminderInterval && !Effects::isScreenFlashing()) {
    triggerReminder();
    resetTimer();
  }
}

void ReminderSystem::triggerReminder() {
  Serial.println("Reminder triggered: Time to take a measurement!");

  if (isAsleep) {
    wakeUp();
  }

  reminderTriggered = true;

  // --- Start flashing first ---
  reminderTone();
  Effects::startVibration(3, 1000, 500);
  Effects::startScreenFlash(3, 1000, 500);

  // Wait until flashing is complete to show the ReminderScreen
}



void ReminderSystem::dismissReminder() {
  Serial.println("Reminder dismissed via hardware button.");
  ReminderScreen::isActive = false;
  reminderScreen.dismissReminder();
}

void ReminderSystem::activateReminderAlert(int flashes, int vibrationDuration) {
  Serial.println("Activating reminder alert...");

  reminderTone();
  Effects::startVibration(flashes, vibrationDuration, 500);
  Effects::startScreenFlash(flashes, vibrationDuration, 500);

  Serial.println("Reminder alert sequence activated.");
  turnOnDisplay();
  lv_scr_load(lv_scr_act());
  lv_refr_now(NULL);
}
