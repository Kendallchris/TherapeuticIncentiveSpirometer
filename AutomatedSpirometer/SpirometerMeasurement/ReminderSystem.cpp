#include "ReminderSystem.h"
#include <Arduino.h>  // for Serial

extern void turnOnDisplay();
extern void wakeUp();

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

  if (millis() - lastActivityTime >= reminderInterval) {
    triggerReminder();
    resetTimer();
  }
}

void ReminderSystem::triggerReminder() {
  Serial.println("Reminder triggered: Time to take a measurement!");

  if (isAsleep) {
    wakeUp();
  }

  // Example: 3 "flashes" each 1000ms on, 500ms off
  activateReminderAlert(3, 1000);  // We'll modify the function so the motor is on for 1s and off for 0.5s

  Serial.println("Displaying reminder screen now...");
  Serial.println("[DEBUG] Setting all other screen flags to false before showing reminder...");
  resetAllScreenFlags();
  ReminderScreen::isActive = true;

  reminderScreen.show();
}

void ReminderSystem::dismissReminder() {
  Serial.println("Reminder dismissed via hardware button.");
  ReminderScreen::isActive = false;
  reminderScreen.dismissReminder();
}

void ReminderSystem::activateReminderAlert(int flashes, int vibrationDuration) {
  Serial.println("Activating reminder alert...");

  for (int i = 0; i < flashes; i++) {
    digitalWrite(screenBacklightPin, LOW);
    digitalWrite(vibrationMotorPin, HIGH);
    delay(vibrationDuration);

    digitalWrite(screenBacklightPin, HIGH);
    digitalWrite(vibrationMotorPin, LOW);
    delay(500);  // Off time to let motor spin down (adjust as desired)
  }

  Serial.println("Reminder alert completed.");
  turnOnDisplay();
  lv_scr_load(lv_scr_act());
  lv_refr_now(NULL);
}
