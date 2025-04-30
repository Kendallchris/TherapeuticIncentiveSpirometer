#ifndef REMINDERSYSTEM_H
#define REMINDERSYSTEM_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "ReminderScreen.h"
#include "DataLogger.h"
#include "HomeScreen.h"

class ReminderSystem {
public:
  ReminderSystem(int buttonPin, TFT_eSPI &display, bool &sleepState, DataLogger &logger);
  void resetTimer();
  void checkReminder();
  void dismissReminder();

private:
  int buttonPin;  // Store button pin reference
  bool &isAsleep;
  bool reminderTriggered = false;
  unsigned long lastActivityTime;
  const unsigned long reminderInterval = 600000;

  TFT_eSPI &tft;
  DataLogger &dataLogger;
  ReminderScreen reminderScreen;

  void triggerReminder();
};

#endif
