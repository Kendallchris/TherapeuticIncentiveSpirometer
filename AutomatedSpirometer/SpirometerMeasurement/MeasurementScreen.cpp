#include "MeasurementScreen.h"
#include <lvgl.h>

MeasurementScreen::MeasurementScreen(TFT_eSPI &display, bool &awaitingDetection)
    : tft(display), awaitingObjectDetection(awaitingDetection), countdown_label(nullptr) {
    // Constructor initialization
}

void MeasurementScreen::showWaitingWithCountdown() {
    lv_obj_t *screen = lv_obj_create(NULL);
    lv_obj_set_size(screen, LV_HOR_RES_MAX, LV_VER_RES_MAX);  // Match screen size
    lv_scr_load(screen);

    countdown_label = lv_label_create(screen);
    lv_label_set_text(countdown_label, "Beginning measurement...");
    lv_obj_align(countdown_label, LV_ALIGN_CENTER, 0, 0);

    countdown_duration = 10;          // Set countdown duration in seconds
    countdown_active = true;          // Enable countdown state
    countdown_start = millis();       // Start countdown timer
    awaitingObjectDetection = false;  // Ensure object detection is disabled during countdown
    lv_refr_now(NULL);                // Refresh screen immediately
}

void MeasurementScreen::updateCountdown() {
    if (countdown_label) {
        int elapsed = (millis() - countdown_start) / 1000;
        int remaining = countdown_duration - elapsed;

        if (remaining > 0) {
            lv_label_set_text_fmt(countdown_label, "Beginning measurement in %d seconds\nPress green button to cancel", remaining);
            Serial.printf("Countdown: %d seconds remaining\n", remaining);
            lv_refr_now(NULL);
        } else {
            lv_label_set_text(countdown_label, "Waiting for object...");
            Serial.println("Countdown complete. Transitioning to measurement phase...");
            countdown_label = nullptr;
            countdown_active = false;

            beginMeasurementPhase();
        }
    }
}

void MeasurementScreen::beginMeasurementPhase() {
    Serial.println("Transitioning to measurement phase...");
    lv_obj_t *screen = lv_obj_create(NULL);
    lv_obj_set_size(screen, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_scr_load(screen);

    lv_obj_t *measurement_label = lv_label_create(screen);
    lv_label_set_text(measurement_label, "Waiting for object...\nPress green button to cancel");
    lv_obj_align(measurement_label, LV_ALIGN_CENTER, 0, 0);

    awaitingObjectDetection = true;
    Serial.println("Object detection enabled.");
    lv_refr_now(NULL);
}

void MeasurementScreen::showSuccess() {
    lv_obj_t *screen = lv_obj_create(NULL);
    lv_obj_set_size(screen, LV_HOR_RES_MAX, LV_VER_RES_MAX);  // Match screen size
    lv_scr_load(screen);

    lv_obj_t *success_label = lv_label_create(screen);
    lv_label_set_text(success_label, "SUCCESS!");
    lv_obj_align(success_label, LV_ALIGN_CENTER, 0, 0);

    lv_refr_now(NULL);  // Force immediate refresh
}

void MeasurementScreen::showNoObject() {
  lv_obj_t *screen = lv_obj_create(NULL);
  lv_scr_load(screen);

  lv_obj_t *no_object_label = lv_label_create(screen);
  lv_label_set_text(no_object_label, "No object detected.");
  lv_obj_align(no_object_label, LV_ALIGN_CENTER, 0, 0);
}
