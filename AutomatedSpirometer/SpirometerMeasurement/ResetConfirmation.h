#ifndef RESETCONFIRMATION_H
#define RESETCONFIRMATION_H

#include <TFT_eSPI.h>
#include <lvgl.h>
#include "DataLogger.h"
#include "HomeScreen.h"

class ResetConfirmation {
public:
  ResetConfirmation(TFT_eSPI &display, DataLogger &logger);
  void show();
  void confirmReset();  // Changed from static to instance methods
  void cancelReset();
  static bool isActive;

private:
  TFT_eSPI &tft;
  DataLogger &dataLogger;
  lv_obj_t *confirmation_label;
};

#endif
