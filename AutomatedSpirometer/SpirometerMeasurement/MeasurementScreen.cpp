#include "MeasurementScreen.h"
#include "HomeScreen.h"
#include <Arduino.h>
#include <lvgl.h>

extern void resetAllScreenFlags();
extern void exitMeasurementMode();
extern void successTone();
extern void measurementsCompleteTone();
extern void stopTone();

MeasurementScreen::MeasurementScreen(
  TFT_eSPI &display,
  bool &awaitingDetection,
  bool &showingSuccess,
  HomeScreen &homeScreen,
  int vibrationPin)
  : tft(display),
    awaitingObjectDetection(awaitingDetection),
    showingSuccess(showingSuccess),
    homeScreen(homeScreen),
    countdown_label(nullptr),
    countdown_active(false),
    vibrationMotorPin(vibrationPin) {
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
      lv_label_set_long_mode(countdown_label, LV_LABEL_LONG_WRAP);
      lv_obj_set_width(countdown_label, 220);
      lv_label_set_text_fmt(
        countdown_label,
        "Beginning measurement in %d seconds\nPress green button to cancel",
        remaining);
      lv_obj_set_style_text_align(countdown_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
      lv_obj_align(countdown_label, LV_ALIGN_CENTER, 0, -40);
      lv_refr_now(NULL);

      // --- Decorative Cancel Button (Bottom Left) ---
      lv_obj_t *cancel_btn = lv_btn_create(lv_scr_act());
      lv_obj_set_size(cancel_btn, 100, 50);
      lv_obj_align(cancel_btn, LV_ALIGN_BOTTOM_LEFT, 10, -10);

      lv_obj_t *cancel_label = lv_label_create(cancel_btn);
      lv_label_set_text(cancel_label, "Cancel");
      lv_obj_align(cancel_label, LV_ALIGN_CENTER, 0, 0);

      lv_obj_set_style_bg_color(cancel_btn, lv_color_hex(0x4CAF50), LV_PART_MAIN);
      lv_obj_set_style_radius(cancel_btn, 10, LV_PART_MAIN);
      lv_obj_set_style_text_color(cancel_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    } else {
      lv_label_set_text(countdown_label, "Waiting for object...");
      lv_obj_set_style_text_align(countdown_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
      lv_obj_align(countdown_label, LV_ALIGN_CENTER, 0, -40);
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
  lv_label_set_long_mode(measurement_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(measurement_label, 220);
  lv_label_set_text(measurement_label, "Waiting for object...\nPress green button to cancel");
  lv_obj_set_style_text_align(measurement_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(measurement_label, LV_ALIGN_CENTER, 0, -40);

  // --- Decorative Cancel Button (Bottom Left) ---
  lv_obj_t *cancel_btn = lv_btn_create(lv_scr_act());
  lv_obj_set_size(cancel_btn, 100, 50);
  lv_obj_align(cancel_btn, LV_ALIGN_BOTTOM_LEFT, 10, -10);

  lv_obj_t *cancel_label = lv_label_create(cancel_btn);
  lv_label_set_text(cancel_label, "Cancel");
  lv_obj_align(cancel_label, LV_ALIGN_CENTER, 0, 0);

  lv_obj_set_style_bg_color(cancel_btn, lv_color_hex(0x4CAF50), LV_PART_MAIN);
  lv_obj_set_style_radius(cancel_btn, 10, LV_PART_MAIN);
  lv_obj_set_style_text_color(cancel_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN);

  awaitingObjectDetection = true;
  lv_refr_now(NULL);
}

void MeasurementScreen::showSuccess(int successfulMeasurements, int percentageComplete) {
  Serial.println("[DEBUG] Showing success screen...");

  // Start vibration for entire success screen duration
  digitalWrite(vibrationMotorPin, HIGH);

  if (successfulMeasurements == 10) {
    measurementsCompleteTone();  // 🎵 For completed session
  } else {
    successTone();  // 🎵 For regular success
  }

  lv_obj_t *screen = lv_obj_create(NULL);
  lv_obj_set_size(screen, LV_HOR_RES_MAX, LV_VER_RES_MAX);
  lv_scr_load(screen);

  // Build base message
  char successText[128];
  snprintf(successText, sizeof(successText),
           "SUCCESS!\n%d completed.\nYou are %d%% at your hourly goal!",
           successfulMeasurements, percentageComplete);

  // If it's the 10th success, add extra reminder message
  if (successfulMeasurements == 10) {
    strncat(successText,
            "\n\nGoal reached!\nPlease reset data before your next measurement to continue recording.",
            sizeof(successText) - strlen(successText) - 1);
  }

  lv_obj_t *success_label = lv_label_create(screen);
  lv_label_set_long_mode(success_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(success_label, 220);
  lv_label_set_text(success_label, successText);
  lv_obj_set_style_text_align(success_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(success_label, LV_ALIGN_CENTER, 0, 0);

  lv_refr_now(NULL);

  showingSuccess = true;
  Serial.println("[DEBUG] Success screen active.");

  // "OK" button -> return home
  lv_obj_t *exit_btn = lv_btn_create(screen);
  lv_obj_set_size(exit_btn, 150, 60);
  lv_obj_align(exit_btn, LV_ALIGN_BOTTOM_MID, 0, -20);
  lv_obj_add_event_cb(exit_btn, returnToHomeEventHandler, LV_EVENT_CLICKED, this);

  lv_obj_t *exit_label = lv_label_create(exit_btn);
  lv_label_set_text(exit_label, "OK");
  lv_obj_align(exit_label, LV_ALIGN_CENTER, 0, 0);

  lv_obj_set_style_bg_color(exit_btn, lv_color_hex(0xE74C3C), LV_PART_MAIN);
  lv_obj_set_style_radius(exit_btn, 10, LV_PART_MAIN);
  lv_obj_set_style_text_color(exit_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
}

// For "OK" button on success screen
void MeasurementScreen::returnToHomeEventHandler(lv_event_t *e) {
  Serial.println("[DEBUG] Button pressed, returning to home...");
  MeasurementScreen *screen = static_cast<MeasurementScreen *>(lv_event_get_user_data(e));
  screen->clearSuccessState();
}

void MeasurementScreen::clearSuccessState() {
  Serial.println("[DEBUG] Clearing success state and transitioning to home screen...");
  digitalWrite(vibrationMotorPin, LOW);
  stopTone();
  showingSuccess = false;

  exitMeasurementMode();
}

void MeasurementScreen::showNoObject() {
  lv_obj_t *screen = lv_obj_create(NULL);
  lv_scr_load(screen);

  lv_obj_t *no_object_label = lv_label_create(screen);
  lv_label_set_text(no_object_label, "No object detected.");
  lv_obj_align(no_object_label, LV_ALIGN_CENTER, 0, 0);
  lv_refr_now(NULL);
}

/* 
 * showUnrecordedSuccess(): 
 * Shown if they've already done 10 measurements 
 * and we won't increment the count further.
 */
void MeasurementScreen::showUnrecordedSuccess() {
  Serial.println("[DEBUG] Showing unrecorded success screen...");

  // Vibrate briefly (optional)
  digitalWrite(vibrationMotorPin, HIGH);
  delay(300);  // short buzz
  digitalWrite(vibrationMotorPin, LOW);

  lv_obj_t *screen = lv_obj_create(NULL);
  lv_obj_set_size(screen, LV_HOR_RES_MAX, LV_VER_RES_MAX);
  lv_scr_load(screen);

  lv_obj_t *info_label = lv_label_create(screen);
  lv_label_set_long_mode(info_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(info_label, 220);
  lv_label_set_text(info_label,
                    "Measurement done, but not recorded.\n"
                    "You have already met your goal of 10.\n\n"
                    "Press OK to continue.");
  lv_obj_set_style_text_align(info_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(info_label, LV_ALIGN_CENTER, 0, -20);

  lv_obj_t *ok_btn = lv_btn_create(screen);
  lv_obj_set_size(ok_btn, 150, 60);
  lv_obj_align(ok_btn, LV_ALIGN_CENTER, 0, -20);
  lv_obj_add_event_cb(ok_btn, unrecordedOkEventHandler, LV_EVENT_CLICKED, this);

  lv_obj_t *ok_label = lv_label_create(ok_btn);
  lv_label_set_text(ok_label, "OK");
  lv_obj_align(ok_label, LV_ALIGN_CENTER, 0, 0);

  lv_obj_set_style_bg_color(ok_btn, lv_color_hex(0xE74C3C), LV_PART_MAIN);
  lv_obj_set_style_radius(ok_btn, 10, LV_PART_MAIN);
  lv_obj_set_style_text_color(ok_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN);

  showingSuccess = true;  // We can reuse the same success-screen logic
  Serial.println("[DEBUG] Unrecorded success screen active.");

  lv_refr_now(NULL);
}

/* When the user presses "OK" on unrecorded success, we just go home. */
void MeasurementScreen::unrecordedOkEventHandler(lv_event_t *e) {
  Serial.println("[DEBUG] user pressed OK on 'unrecorded' screen, returning home...");
  MeasurementScreen *screen = static_cast<MeasurementScreen *>(lv_event_get_user_data(e));
  screen->dismissUnrecorded();
}

/* Actually returns to home from the unrecorded measurement screen. */
void MeasurementScreen::dismissUnrecorded() {
  // No vibration to stop (we only short-buzzed earlier).
  showingSuccess = false;

  homeScreen.show();
  lv_refr_now(NULL);
}
