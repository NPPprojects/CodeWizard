#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM      18

#define EXAMPLE_LED_NUMBERS         7
#define EXAMPLE_CHASE_SPEED_MS      30

static const char *TAG = "example";

static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

void color_transition_effect(rmt_channel_handle_t led_chan, rmt_encoder_handle_t led_encoder) {
    ESP_LOGI(TAG, "Start LED color transition effect between Purple and Blue using HSV");

    rmt_transmit_config_t tx_config = {.loop_count = 0};
    uint16_t hue_values[] = {320, 200};  // Update hue values to start from 320 (Purple) and then interpolate to 200 (Blue)
    int num_colors = sizeof(hue_values) / sizeof(hue_values[0]);

    while (1) { // Run continuously
        for (int current_color_index = 0; current_color_index < num_colors; ++current_color_index) {
            uint16_t start_hue = hue_values[current_color_index];
            uint16_t end_hue = hue_values[(current_color_index + 1) % num_colors];

            for (int step = 0; step < 100; ++step) {
                uint16_t interpolated_hue = start_hue + (end_hue - start_hue) * step / 100;

                for (int i = 0; i < EXAMPLE_LED_NUMBERS; ++i) {
                    uint32_t red, green, blue;
                    led_strip_hsv2rgb(interpolated_hue, 100, 100, &red, &green, &blue);

                    led_strip_pixels[i * 3 + 0] = red;
                    led_strip_pixels[i * 3 + 1] = green;
                    led_strip_pixels[i * 3 + 2] = blue;
                }

                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
                vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t led_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64, // increase the block size for less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    ESP_LOGI(TAG, "Install led strip encoder");
    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    while (1) {
        color_transition_effect(led_chan, led_encoder);
    }
}
