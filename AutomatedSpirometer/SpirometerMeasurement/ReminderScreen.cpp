#include "ReminderScreen.h"
#include "Effects.h"
#include "UIHelpers.h"
#include "HomeScreen.h"
#include <Arduino.h>

bool ReminderScreen::isActive = false;

ReminderScreen::ReminderScreen(TFT_eSPI &display, DataLogger &logger)
  : tft(display), dataLogger(logger), screen(nullptr) {}

extern void resetAllScreenFlags();

void ReminderScreen::prepare() {
  if (screen != nullptr) return;  // Already prepared

  screen = lv_obj_create(NULL);
  lv_obj_set_size(screen, LV_HOR_RES_MAX, LV_VER_RES_MAX);

  // --- Button ---
  lv_obj_t *dismiss_btn = lv_btn_create(screen);
  lv_obj_set_size(dismiss_btn, 200, 40);
  lv_obj_align(dismiss_btn, LV_ALIGN_TOP_MID, 0, 10);
  lv_obj_add_event_cb(dismiss_btn, ReminderScreen::dismissEventHandler, LV_EVENT_CLICKED, this);

  lv_obj_t *dismiss_label = lv_label_create(dismiss_btn);
  lv_label_set_text(dismiss_label, "Dismiss");
  lv_obj_align(dismiss_label, LV_ALIGN_CENTER, 0, 0);

  lv_obj_set_style_bg_color(dismiss_btn, lv_color_hex(0x4CAF50), LV_PART_MAIN);
  lv_obj_set_style_radius(dismiss_btn, 10, LV_PART_MAIN);
  lv_obj_set_style_text_color(dismiss_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN);

  // --- Message ---
  lv_obj_t *message_label = lv_label_create(screen);
  lv_label_set_long_mode(message_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(message_label, 220);
  lv_label_set_text(message_label, "Time to take a measurement!");
  lv_obj_set_style_text_align(message_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(message_label, LV_ALIGN_CENTER, 0, -40);
}

void ReminderScreen::show() {
  Effects::stopScreenFlash();  // stop any flashing effects first
  Effects::stopVibration();
  Effects::stopTone();

  isActive = true;
  Serial.println("Reminder screen is now active");

  if (screen == nullptr) {
    prepare();  // Fallback if preload somehow failed
  }

  ui_switch_screen(screen);   // PROPER safe load + old screen free

  lv_refr_now(NULL);  // Force immediate refresh
}

void ReminderScreen::dismissReminder() {
  Serial.println("Dismissing reminder...");

  isActive = false;

  Serial.println("Reloading Home Screen...");
  Serial.println("[DEBUG] Dismissing reminder, resetting all flags...");
  resetAllScreenFlags();

  // 🛠️ Create a NEW screen properly
  lv_obj_clean(lv_scr_act());
  lv_obj_t *blank = lv_obj_create(NULL);
  lv_obj_set_size(blank, LV_HOR_RES_MAX, LV_VER_RES_MAX);
  ui_switch_screen(blank);  // frees Reminder screen

  HomeScreen homeScreen(tft);
  homeScreen.show();

  lv_refr_now(NULL);
}

void ReminderScreen::dismissEventHandler(lv_event_t *e) {
  Serial.println("Dismiss button event handler triggered!");
  ReminderScreen *self = static_cast<ReminderScreen *>(lv_event_get_user_data(e));
  self->dismissReminder();
}