#ifndef OLED_H
#define OLED_H

/* Includes */
#include "driver/i2c.h"
#include "esp_log.h"
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#include "esp_lcd_sh1107.h"
#else
#include "esp_lcd_panel_vendor.h"
#endif
#include "lvgl.h"
#include "esp_lvgl_port.h"
/* Configuration Macros */
#define I2C_HOST              0
#define OLED_PIXEL_CLOCK_HZ   (400 * 1000)
#define OLED_PIN_NUM_SDA      GPIO_NUM_21
#define OLED_PIN_NUM_SCL      GPIO_NUM_22
#define OLED_PIN_NUM_RST      -1
#define OLED_I2C_HW_ADDR      0x3C


#define OLED_H_RES            128
#define OLED_V_RES            64


#define OLED_CMD_BITS         8
#define OLED_PARAM_BITS       8

/* Public function declarations */
lv_disp_t * oled_init(void);

#endif // OLED_H
