#include "../include/ResetConfirmation.h"
#include "../include/HomeScreen.h"
#include "../include/UIHelpers.h"
#include <Arduino.h>

bool ResetConfirmation::isActive = false;  // Initialize the flag

ResetConfirmation::ResetConfirmation(TFT_eSPI &display, DataLogger &logger)
  : tft(display), dataLogger(logger) {}

extern void resetAllScreenFlags();

void ResetConfirmation::show() {
  isActive = true;  // Mark the screen as active

  lv_obj_clean(lv_scr_act());
  lv_obj_t *screen = lv_obj_create(NULL);
  lv_obj_set_size(screen, LV_HOR_RES_MAX, LV_VER_RES_MAX);  // Match screen size
  ui_switch_screen(screen);                                    // Load the confirmation screen

  // **Cancel Button (Green)**
  lv_obj_t *cancel_btn = lv_btn_create(screen);
  lv_obj_set_size(cancel_btn, 200, 40);
  lv_obj_align(cancel_btn, LV_ALIGN_TOP_MID, 0, 10);
  lv_obj_add_event_cb(
    cancel_btn, [](lv_event_t *e) {
      ResetConfirmation *self = static_cast<ResetConfirmation *>(lv_event_get_user_data(e));
      self->cancelReset();
    },
    LV_EVENT_CLICKED, this);
  lv_obj_t *cancel_label = lv_label_create(cancel_btn);
  lv_label_set_text(cancel_label, "Cancel");

  lv_obj_set_style_bg_color(cancel_btn, lv_color_hex(0x4CAF50), LV_PART_MAIN);
  lv_obj_set_style_radius(cancel_btn, 10, LV_PART_MAIN);
  lv_obj_set_style_text_color(cancel_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN);

  // **Confirm Button (Red)**
  lv_obj_t *confirm_btn = lv_btn_create(screen);
  lv_obj_set_size(confirm_btn, 200, 40);
  lv_obj_align(confirm_btn, LV_ALIGN_TOP_MID, 0, 60);
  lv_obj_add_event_cb(
    confirm_btn, [](lv_event_t *e) {
      ResetConfirmation *self = static_cast<ResetConfirmation *>(lv_event_get_user_data(e));
      self->confirmReset();
    },
    LV_EVENT_CLICKED, this);
  lv_obj_t *confirm_label = lv_label_create(confirm_btn);
  lv_label_set_text(confirm_label, "Reset");

  lv_obj_set_style_bg_color(confirm_btn, lv_color_hex(0xFF4C4C), LV_PART_MAIN);
  lv_obj_set_style_radius(confirm_btn, 10, LV_PART_MAIN);
  lv_obj_set_style_text_color(confirm_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN);

  // --- Message ---
  confirmation_label = lv_label_create(screen);
  lv_label_set_text(confirmation_label, "Confirm Reset?");
  lv_obj_align(confirmation_label, LV_ALIGN_CENTER, 0, -40);

  lv_refr_now(NULL);  // Force screen refresh
}

void ResetConfirmation::confirmReset() {
  dataLogger.resetData();
  isActive = false;

  Serial.println("[DEBUG] Confirm reset, resetting all flags...");
  resetAllScreenFlags();

  HomeScreen homeScreen(tft);
  homeScreen.show();
  lv_refr_now(NULL);
}

void ResetConfirmation::cancelReset() {
  isActive = false;

  Serial.println("[DEBUG] Cancel reset, resetting all flags...");
  resetAllScreenFlags();

  HomeScreen homeScreen(tft);
  homeScreen.show();
  lv_refr_now(NULL);
}