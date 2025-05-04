#ifndef REMINDERSYSTEM_H
#define REMINDERSYSTEM_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "ReminderScreen.h"
#include "DataLogger.h"
#include "HomeScreen.h"
#include <TimeLib.h>

class ReminderSystem {
public:
  ReminderSystem(int buttonPin, TFT_eSPI &display, bool &sleepState, DataLogger &logger);
  void resetTimer();
  void checkReminder();
  void dismissReminder();
  time_t getLastReminderTime() const { return lastReminderTime; }
  unsigned long getReminderInterval() const { return reminderInterval; }
  void triggerReminder();

private:
  int buttonPin;  // Store button pin reference
  bool &isAsleep;
  bool reminderTriggered = false;
  time_t lastReminderTime;
  const time_t reminderInterval = 600;

  TFT_eSPI &tft;
  DataLogger &dataLogger;
  ReminderScreen reminderScreen;
};

#endif
