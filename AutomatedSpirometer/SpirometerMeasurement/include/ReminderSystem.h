#ifndef REMINDERSYSTEM_H
#define REMINDERSYSTEM_H

#include <TFT_eSPI.h>
#include "DataLogger.h"
#include "ReminderScreen.h"

class ReminderSystem {
public:
  ReminderSystem(int buttonPin, TFT_eSPI &display, bool &sleepState, DataLogger &logger);
  void checkReminder();
  void dismissReminder();
  unsigned long getReminderInterval() const {
    return reminderInterval;
  }
  void triggerReminder();
  void prepareForSleep();
  static bool reminderTriggered;

private:
  int buttonPin;  // Store button pin reference
  bool &isAsleep;
  const unsigned long reminderInterval = 600000UL;  // 10 minutes in milliseconds
  unsigned long sleptDuration = 0;
  bool pendingReminderScreen = false;
  unsigned long flashDoneAt = 0;

  TFT_eSPI &tft;
  DataLogger &dataLogger;
  ReminderScreen reminderScreen;
};

#endif