/*
 * SPDX-License-Identifier: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "lvgl_ui.h"


void example_lvgl_demo_ui(lv_disp_t *disp, const char *message) {
    // Get the active screen
    lv_obj_t *scr = lv_disp_get_scr_act(disp);

    // Remove all children from the screen object
    lv_obj_clean(scr);

    // Create and configure the new label
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_label_set_text(label, message);
    lv_obj_set_width(label, disp->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 30);
}
