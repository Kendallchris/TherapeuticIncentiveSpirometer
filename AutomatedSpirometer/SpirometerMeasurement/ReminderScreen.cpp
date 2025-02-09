#include "ReminderScreen.h"

bool ReminderScreen::isActive = false;

ReminderScreen::ReminderScreen(TFT_eSPI &display, DataLogger &logger)
  : tft(display), dataLogger(logger), screen(nullptr) {}

void ReminderScreen::show() {
  isActive = true;
  Serial.println("Reminder screen is now active");

  screen = lv_obj_create(NULL);
  lv_scr_load(screen);

  lv_obj_t *message_label = lv_label_create(screen);
  lv_label_set_text(message_label, "Time to take a measurement!");
  lv_obj_align(message_label, LV_ALIGN_CENTER, 0, -40);

  lv_obj_t *dismiss_btn = lv_btn_create(screen);
  lv_obj_set_size(dismiss_btn, 100, 50);
  lv_obj_align(dismiss_btn, LV_ALIGN_CENTER, 0, 40);

  lv_obj_add_event_cb(dismiss_btn, ReminderScreen::dismissEventHandler, LV_EVENT_CLICKED, this);

  lv_obj_t *dismiss_label = lv_label_create(dismiss_btn);
  lv_label_set_text(dismiss_label, "Dismiss");

  lv_obj_set_style_bg_color(dismiss_btn, lv_color_hex(0x4CAF50), LV_PART_MAIN);
  lv_obj_set_style_radius(dismiss_btn, 10, LV_PART_MAIN);
  lv_obj_set_style_text_color(dismiss_btn, lv_color_hex(0xFFFFFF), LV_PART_MAIN);

  lv_refr_now(NULL);
}

void ReminderScreen::dismissReminder() {
  Serial.println("Dismissing reminder...");

  isActive = false;

  Serial.println("Reloading Home Screen...");
  HomeScreen homeScreen(tft);
  homeScreen.show(dataLogger.getCurrentHourMeasurements(true));

  lv_scr_load(lv_scr_act());  
  lv_refr_now(NULL);
}

void ReminderScreen::dismissEventHandler(lv_event_t *e) {
  Serial.println("Dismiss button event handler triggered!");
  ReminderScreen *self = static_cast<ReminderScreen *>(lv_event_get_user_data(e));
  self->dismissReminder();
}
