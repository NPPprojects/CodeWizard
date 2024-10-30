#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (LED strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM      18
#define BOOT_BUTTON_GPIO            0
#define EXAMPLE_LED_NUMBERS         64
#define EXAMPLE_CHASE_SPEED_MS      30 // Adjusted delay
#define BRIGHTNESS                  100

static const char *TAG = "example";

static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];
static QueueHandle_t gpio_evt_queue = NULL;

// Global handles for RMT channel and encoder
rmt_channel_handle_t led_chan = NULL;
rmt_encoder_handle_t led_encoder = NULL;

// Variables for color_transition_effect
static uint16_t hue_values_color_transition[2] = {200, 320};  // HSV hues for Purple and Blue
static int current_color_index = 0;
static int step_color_transition = 0;
static uint16_t start_hue_color_transition = 200;
static uint16_t end_hue_color_transition = 320;

// Variables for smooth_interpolation_effect
static uint32_t red_start_smooth_interpolation = 0, green_start_smooth_interpolation = 0, blue_start_smooth_interpolation = 0;
static uint16_t start_hue_smooth_interpolation = 0, end_hue_smooth_interpolation = 60;
static int step_smooth_interpolation = 0;

/* Helper function to convert HSV to RGB */
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b) {
    h %= 360;
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;
    uint32_t i = h / 60;
    uint32_t diff = h % 60;
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;
    switch (i) {
        case 0: *r = rgb_max; *g = rgb_min + rgb_adj; *b = rgb_min; break;
        case 1: *r = rgb_max - rgb_adj; *g = rgb_max; *b = rgb_min; break;
        case 2: *r = rgb_min; *g = rgb_max; *b = rgb_min + rgb_adj; break;
        case 3: *r = rgb_min; *g = rgb_max - rgb_adj; *b = rgb_max; break;
        case 4: *r = rgb_min + rgb_adj; *g = rgb_min; *b = rgb_max; break;
        default: *r = rgb_max; *g = rgb_min; *b = rgb_max - rgb_adj; break;
    }
}

/* Non-blocking color transition effect between Purple and Blue using HSV */
void color_transition_effect(void) {
    rmt_transmit_config_t tx_config = {.loop_count = 0};

    // Run a single step of the effect
    uint16_t interpolated_hue = start_hue_color_transition + (end_hue_color_transition - start_hue_color_transition) * step_color_transition / 100;

    for (int i = 0; i < EXAMPLE_LED_NUMBERS; ++i) {
        uint32_t red, green, blue;
        led_strip_hsv2rgb(interpolated_hue, 100, BRIGHTNESS, &red, &green, &blue);

        led_strip_pixels[i * 3 + 0] = red;
        led_strip_pixels[i * 3 + 1] = green;
        led_strip_pixels[i * 3 + 2] = blue;
    }

    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));

    step_color_transition++;
    if (step_color_transition >= 100) {
        step_color_transition = 0;
        current_color_index = (current_color_index + 1) % 2;
        start_hue_color_transition = hue_values_color_transition[current_color_index];
        end_hue_color_transition = hue_values_color_transition[(current_color_index + 1) % 2];
    }

    vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
}

/* Non-blocking smooth rainbow transition effect */
void smooth_interpolation_effect(void) {
    rmt_transmit_config_t tx_config = {.loop_count = 0};

    // Run a single step of the effect
    uint16_t hue = start_hue_smooth_interpolation + (end_hue_smooth_interpolation - start_hue_smooth_interpolation) * step_smooth_interpolation / 100;

    for (int i = 0; i < EXAMPLE_LED_NUMBERS; ++i) {
        uint32_t red, green, blue;
        led_strip_hsv2rgb(hue, 100, BRIGHTNESS, &red, &green, &blue);
        led_strip_pixels[i * 3 + 0] = green;
        led_strip_pixels[i * 3 + 1] = blue;
        led_strip_pixels[i * 3 + 2] = red;
    }

    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));

    step_smooth_interpolation++;
    if (step_smooth_interpolation >= 100) {
        step_smooth_interpolation = 0;
        uint16_t temp = start_hue_smooth_interpolation;
        start_hue_smooth_interpolation = end_hue_smooth_interpolation;
        end_hue_smooth_interpolation = temp + 60;
        if (end_hue_smooth_interpolation >= 360) end_hue_smooth_interpolation -= 360;
    }

    vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS / 2));
}

/* Strobe effect triggered by long button press */
void strobe_effect(void) {
    ESP_LOGI(TAG, "Start LED strobe effect");
    rmt_transmit_config_t tx_config = {.loop_count = 0};
    uint32_t red = 0, green = 0, blue = 0;
    uint16_t hue = 0, start_rgb = 0;

    while (gpio_get_level(BOOT_BUTTON_GPIO) == 0) { // Ensure button is held during the effect
        for (int i = 0; i < 3; i++) {
            for (int j = i; j < EXAMPLE_LED_NUMBERS; j += 3) {
                hue = j * 360 / EXAMPLE_LED_NUMBERS + start_rgb;
                led_strip_hsv2rgb(hue, 100, BRIGHTNESS, &red, &green, &blue);
                led_strip_pixels[j * 3 + 0] = green;
                led_strip_pixels[j * 3 + 1] = blue;
                led_strip_pixels[j * 3 + 2] = red;
            }
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
            memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
        }
        start_rgb += 60;
    }
}

/* Function to reset effect steps */
void reset_effect_steps(void) {
    // Reset step counters and any static variables in the effects
    // For color_transition_effect
    step_color_transition = 0;
    current_color_index = 0;
    start_hue_color_transition = hue_values_color_transition[0];
    end_hue_color_transition = hue_values_color_transition[1];

    // For smooth_interpolation_effect
    step_smooth_interpolation = 0;
    start_hue_smooth_interpolation = 0;
    end_hue_smooth_interpolation = 60;
}

/* GPIO ISR handler */
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    static uint32_t last_isr_time = 0;
    uint32_t current_time = xTaskGetTickCountFromISR();
    if ((current_time - last_isr_time) > pdMS_TO_TICKS(50)) { // 50 ms debounce time
        uint32_t gpio_num = (uint32_t) arg;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPriorityTaskWoken) != pdPASS) {
            // Handle queue full error if needed
        }
        last_isr_time = current_time;
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

/* Button handling task */
void button_task(void *arg) {
    uint8_t *effect_mode = (uint8_t *)arg;
    uint32_t io_num;
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Button Press RMT TX channel");
            if (gpio_get_level(BOOT_BUTTON_GPIO) == 0) {
                uint32_t press_start_time = xTaskGetTickCount();
                while (gpio_get_level(BOOT_BUTTON_GPIO) == 0) {
                    if ((xTaskGetTickCount() - press_start_time) > pdMS_TO_TICKS(1000)) {
                        strobe_effect(); // Run strobe effect
                        // After strobe effect, reset effect steps and update LEDs
                        reset_effect_steps();
                        // Call the effect function once to update LEDs
                        if (*effect_mode == 0) {
                            color_transition_effect();
                        } else if (*effect_mode == 1) {
                            smooth_interpolation_effect();
                        }
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                if ((xTaskGetTickCount() - press_start_time) <= pdMS_TO_TICKS(1000)) {
                    *effect_mode = (*effect_mode + 1) % 2; // Toggle between modes
                }
            }
        }
    }
}

void app_main(void) {
    // Configure the button GPIO
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Create a queue to handle GPIO events from ISR
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // Install ISR service and add ISR handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOOT_BUTTON_GPIO, gpio_isr_handler, (void*) BOOT_BUTTON_GPIO);

    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    ESP_LOGI(TAG, "Install LED strip encoder");
    led_strip_encoder_config_t encoder_config = {.resolution = RMT_LED_STRIP_RESOLUTION_HZ};
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    static uint8_t effect_mode = 0; // 0 = color transition, 1 = smooth interpolation

    // Create the button handling task
    xTaskCreate(button_task, "button_task", 2048, &effect_mode, 10, NULL);

    while (1) {
        if (effect_mode == 0) {
            color_transition_effect();
        } else if (effect_mode == 1) {
            smooth_interpolation_effect();
        }
        // No need to delay here if the effect functions handle it
    }
}
