#ifndef REMINDERSCREEN_H
#define REMINDERSCREEN_H

#include "DataLogger.h"
#include <TFT_eSPI.h>
#include <lvgl.h>

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
