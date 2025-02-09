#ifndef REMINDERSCREEN_H
#define REMINDERSCREEN_H

#include <TFT_eSPI.h>
#include <lvgl.h>
#include "DataLogger.h"
#include "HomeScreen.h"

class ReminderScreen {
public:
  ReminderScreen(TFT_eSPI &display, DataLogger &logger);
  void show();
  void dismissReminder();
  static bool isActive;
  static void dismissEventHandler(lv_event_t *e);

private:
  TFT_eSPI &tft;
  DataLogger &dataLogger;
  lv_obj_t *screen;
};

#endif
