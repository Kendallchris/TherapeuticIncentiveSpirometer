#include "ReminderSystem.h"
#include "Effects.h"
#include <Arduino.h>  // for Serial

extern void turnOnDisplay();
extern void wakeUp();
extern void resetAllScreenFlags();

ReminderSystem::ReminderSystem(int buttonPin, TFT_eSPI &display, bool &sleepState, DataLogger &logger)
  : buttonPin(buttonPin),
    isAsleep(sleepState),
    lastReminderTime(millis()),  // <--- use millis instead of now()
    tft(display),
    dataLogger(logger),
    reminderScreen(display, logger) {
  pinMode(buttonPin, INPUT_PULLUP);
}

void ReminderSystem::resetTimer() {
  lastReminderTime = millis();  // <--- use millis()
}

void ReminderSystem::checkReminder() {
  static bool pendingReminderScreen = false;
  static unsigned long flashFinishTime = 0;

  if (ReminderScreen::isActive && digitalRead(buttonPin) == LOW) {
    Serial.println("Dismissing reminder via hardware button...");
    dismissReminder();
    delay(300);
    return;
  }

  if (reminderTriggered && !Effects::isScreenFlashing() && !pendingReminderScreen) {
    Serial.println("Flashing complete. Preparing to show reminder screen...");
    reminderTriggered = false;
    pendingReminderScreen = true;
    flashFinishTime = millis();  // mark when flash finished
    return;
  }

  if (pendingReminderScreen && (millis() - flashFinishTime >= 200)) {  // wait 200ms buffer
    Serial.println("Actually showing reminder screen now...");

    if (isAsleep) {
      turnOnDisplay();
      isAsleep = false;
    }

    resetAllScreenFlags();
    ReminderScreen::isActive = true;
    reminderScreen.show();

    pendingReminderScreen = false;  // clear pending
  }

  // 🧠 Corrected logic: Use millis instead of now()
  if (!isAsleep && (millis() - lastReminderTime >= reminderInterval) && !Effects::isScreenFlashing()) {
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
  Effects::reminderTone();
  Effects::startVibration(3, 1000, 500);
  Effects::startScreenFlash(3, 1000, 500);

  // Wait until flashing is complete to show the ReminderScreen
}

void ReminderSystem::dismissReminder() {
  Serial.println("Reminder dismissed via hardware button.");
  ReminderScreen::isActive = false;
  reminderScreen.dismissReminder();
}

void ReminderSystem::prepareForSleep(unsigned long sleepDuration) {
  sleptDuration = sleepDuration;
}

void ReminderSystem::handleTimerWake() {
  if (sleptDuration > 0) {
    lastReminderTime += sleptDuration;
    sleptDuration = 0;  // Reset after applying
  }
}
