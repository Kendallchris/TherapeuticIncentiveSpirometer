#pragma once
#include <lvgl.h>

/* Replaces the active screen with new_scr and frees the old one safely. */
static inline void ui_switch_screen(lv_obj_t *new_scr)
{
    lv_obj_t *old = lv_scr_act();          // screen that is visible now
    lv_scr_load(new_scr);                  // show the new one first
    if (old && old != new_scr) {
        lv_obj_del_async(old);             // free the previous screen ­– async is safe inside callbacks
    }
}