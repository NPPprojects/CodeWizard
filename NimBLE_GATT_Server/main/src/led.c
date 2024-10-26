/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */

#include "led.h"

void led_on(void) { gpio_set_level(BLINK_LED, 1); }

void led_off(void) { gpio_set_level(BLINK_LED,0); }

void led_init(void) {

    gpio_reset_pin(BLINK_LED);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_LED, GPIO_MODE_OUTPUT);
}
