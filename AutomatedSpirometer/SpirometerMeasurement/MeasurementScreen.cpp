#include "MeasurementScreen.h"
#include <Arduino.h>

MeasurementScreen::MeasurementScreen(TFT_eSPI &display, bool &awaitingDetection, bool &showingSuccess, HomeScreen &homeScreen, int vibrationPin)
  : tft(display), awaitingObjectDetection(awaitingDetection), showingSuccess(showingSuccess), homeScreen(homeScreen), countdown_label(nullptr), countdown_active(false), vibrationMotorPin(vibrationPin) {
}

void MeasurementScreen::showWaitingWithCountdown() {
  lv_obj_t *screen = lv_obj_create(NULL);
  lv_obj_set_size(screen, LV_HOR_RES_MAX, LV_VER_RES_MAX);  
  lv_scr_load(screen);

  countdown_label = lv_label_create(screen);
  lv_label_set_text(countdown_label, "Beginning measurement...");
  lv_obj_align(countdown_label, LV_ALIGN_CENTER, 0, -20);

  countdown_duration = 5;
  countdown_active = true;
  countdown_start = millis();
  awaitingObjectDetection = false;

  lv_refr_now(NULL);
}

void MeasurementScreen::updateCountdown() {
  if (countdown_label) {
    int elapsed = (millis() - countdown_start) / 1000;
    int remaining = countdown_duration - elapsed;

    if (remaining > 0) {
      lv_label_set_text_fmt(countdown_label, "Beginning measurement in %d seconds\nPress green button to cancel", remaining);
      lv_refr_now(NULL);
    } else {
      lv_label_set_text(countdown_label, "Waiting for object...");
      countdown_label = nullptr;
      countdown_active = false;
      beginMeasurementPhase();
    }
  }
}

void MeasurementScreen::beginMeasurementPhase() {
  lv_obj_t *screen = lv_obj_create(NULL);
  lv_obj_set_size(screen, LV_HOR_RES_MAX, LV_VER_RES_MAX);
  lv_scr_load(screen);

  lv_obj_t *measurement_label = lv_label_create(screen);
  lv_label_set_text(measurement_label, "Waiting for object...\nPress green button to cancel");
  lv_obj_align(measurement_label, LV_ALIGN_CENTER, 0, -20);

  awaitingObjectDetection = true;
  lv_refr_now(NULL);
}

void MeasurementScreen::showSuccess(int successfulMeasurements, int percentageComplete) {
  Serial.println("[DEBUG] Showing success screen...");

  // Start vibration for entire success screen duration
  digitalWrite(vibrationMotorPin, HIGH);

  lv_obj_t *screen = lv_obj_create(NULL);
  lv_obj_set_size(screen, LV_HOR_RES_MAX, LV_VER_RES_MAX);
  lv_scr_load(screen);

  lv_obj_t *success_label = lv_label_create(screen);

  // Convert percentage to string ensuring it is a whole number
  char successText[64];  
  snprintf(successText, sizeof(successText), "SUCCESS!\n%d completed.\nYou are %d%% at your hourly goal!", 
           successfulMeasurements, percentageComplete);

  lv_label_set_text(success_label, successText);
  lv_obj_align(success_label, LV_ALIGN_CENTER, 0, 0);

  lv_refr_now(NULL);

  // Set success screen active
  showingSuccess = true;
  Serial.println("[DEBUG] Success screen active.");

  // Create an event listener for button presses to exit immediately
  lv_obj_t *exit_btn = lv_btn_create(screen);
  lv_obj_set_size(exit_btn, 150, 60);
  lv_obj_align(exit_btn, LV_ALIGN_BOTTOM_MID, 0, -20);
  lv_obj_add_event_cb(exit_btn, returnToHomeEventHandler, LV_EVENT_CLICKED, this);

  lv_obj_t *exit_label = lv_label_create(exit_btn);
  lv_label_set_text(exit_label, "OK");
  lv_obj_align(exit_label, LV_ALIGN_CENTER, 0, 0);

  // Style the button
  lv_obj_set_style_bg_color(exit_btn, lv_color_hex(0xE74C3C), LV_PART_MAIN);
  lv_obj_set_style_radius(exit_btn, 10, LV_PART_MAIN);
  lv_obj_set_style_text_color(exit_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
}

// Event handler for button press to return home instantly
void MeasurementScreen::returnToHomeEventHandler(lv_event_t * e) {
  Serial.println("[DEBUG] Button pressed, returning to home...");
  MeasurementScreen *screen = static_cast<MeasurementScreen *>(lv_event_get_user_data(e));
  screen->clearSuccessState();
}

// Clears the success state, resets showingSuccess flag, stops vibration, and transitions to home
void MeasurementScreen::clearSuccessState() {
  Serial.println("[DEBUG] Clearing success state and transitioning to home screen...");

  // Stop vibration when exiting success screen
  digitalWrite(vibrationMotorPin, LOW);

  showingSuccess = false;
  homeScreen.show(0);
  lv_refr_now(NULL);
}

void MeasurementScreen::showNoObject() {
  lv_obj_t *screen = lv_obj_create(NULL);
  lv_scr_load(screen);

  lv_obj_t *no_object_label = lv_label_create(screen);
  lv_label_set_text(no_object_label, "No object detected.");
  lv_obj_align(no_object_label, LV_ALIGN_CENTER, 0, 0);
}
