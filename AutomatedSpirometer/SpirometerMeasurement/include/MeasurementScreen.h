#ifndef MEASUREMENTSCREEN_H
#define MEASUREMENTSCREEN_H

#include <TFT_eSPI.h>
#include <lvgl.h>
#include "HomeScreen.h"

class MeasurementScreen {
public:
  MeasurementScreen(TFT_eSPI &display, bool &awaitingDetection, bool &showingSuccess, HomeScreen &homeScreen);
  void showWaitingWithCountdown();
  void updateCountdown();
  void beginMeasurementPhase();
  void showSuccess(int successfulMeasurements, int percentageComplete);
  void showNoObject();
  void clearSuccessState();

  // The new function for an unrecorded measurement beyond 10
  void showUnrecordedSuccess();
  static void unrecordedOkEventHandler(lv_event_t *e);
  void dismissUnrecorded();

  bool isCountdownActive() const {
    return countdown_active;
  }

private:
  TFT_eSPI &tft;
  bool &awaitingObjectDetection;
  bool &showingSuccess;
  HomeScreen &homeScreen;
  lv_obj_t *countdown_label;
  unsigned long countdown_start;
  bool countdown_active;
  int countdown_duration = 10;

  static void returnToHomeEventHandler(lv_event_t *e);
};

#endif
