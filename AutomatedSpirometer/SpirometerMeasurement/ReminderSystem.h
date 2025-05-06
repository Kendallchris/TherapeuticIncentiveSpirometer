#ifndef REMINDERSYSTEM_H
#define REMINDERSYSTEM_H

#include <TFT_eSPI.h>
#include "DataLogger.h"
#include "ReminderScreen.h"

class ReminderSystem {
public:
  ReminderSystem(int buttonPin, TFT_eSPI &display, bool &sleepState, DataLogger &logger);
  void resetTimer();
  void checkReminder();
  void dismissReminder();
  unsigned long getLastReminderTime() const {
    return lastReminderTime;
  }
  unsigned long getReminderInterval() const {
    return reminderInterval;
  }
  void triggerReminder();
  void prepareForSleep(unsigned long sleepDuration);
  void handleTimerWake();

private:
  int buttonPin;  // Store button pin reference
  bool &isAsleep;
  bool reminderTriggered = false;
  unsigned long lastReminderTime;
  const unsigned long reminderInterval = 600000UL;  // 10 minutes in milliseconds
  // const unsigned long reminderInterval = 70000UL;  // 70 seconds in milliseconds for testig purposes
  unsigned long sleptDuration = 0;

  TFT_eSPI &tft;
  DataLogger &dataLogger;
  ReminderScreen reminderScreen;
};

#endif