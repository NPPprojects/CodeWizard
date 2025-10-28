#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "led_strip_encoder.h"
#include "riscv/encoding.h"
#include <stdbool.h>
#include <string.h>

/*
 * Joystick control mapping (active HIGH digital joystick):
 *  - LEFT: select rainbow animation.
 *  - RIGHT: select strobe animation.
 *  - DOWN/UP: slow down / speed up the active animation, or adjust solid hue.
 *  - CENTER: contributes to combos (press three times to toggle overall LED
 *    output).
 *  - SET: toggle solid-mode brightness.
 *  - RESET: fallback to solid mode using default parameters.
 */

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000
#define RMT_LED_STRIP_GPIO_NUM 10
#define EXAMPLE_LED_NUMBERS 7

/* Digital joystick pins (match the standalone joystick sample) */
#define JOY_UP_PIN GPIO_NUM_0
#define JOY_DOWN_PIN GPIO_NUM_1
#define JOY_LEFT_PIN GPIO_NUM_3
#define JOY_RIGHT_PIN GPIO_NUM_4
#define JOY_CENTER_PIN GPIO_NUM_7
#define JOY_SET_PIN GPIO_NUM_6
#define JOY_RESET_PIN GPIO_NUM_5

#define ESP_INTR_FLAG_DEFAULT 0

#define RAINBOW_MIN_INTERVAL_MS 1
#define RAINBOW_MAX_INTERVAL_MS 180
#define RAINBOW_INTERVAL_STEP_MS 10
#define RAINBOW_DEFAULT_INTERVAL_MS 60

#define STROBE_MIN_INTERVAL_MS 30
#define STROBE_MAX_INTERVAL_MS 250
#define STROBE_INTERVAL_STEP_MS 10
#define STROBE_DEFAULT_INTERVAL_MS 140

#define SOLID_HUE_STEP_DEG 15
#define SOLID_DEFAULT_HUE 270
#define SOLID_UPDATE_INTERVAL_MS 60

#define DEBOUNCE_TIME_US 30000
#define MAX_GPIO_INDEX 48
#define POWER_TOGGLE_COMBO_LENGTH 3

#define SOLID_BRIGHTNESS_MIN 10
#define SOLID_BRIGHTNESS_MAX 100
#define SOLID_BRIGHTNESS_STEP 10
#define SOLID_BRIGHTNSS_DEFAULT 60

static const char *TAG = "joy_led";

static QueueHandle_t s_gpio_evt_queue;
static int64_t s_last_event_us[MAX_GPIO_INDEX] = {0};
static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];
static const gpio_num_t s_power_toggle_combo[POWER_TOGGLE_COMBO_LENGTH] = {
    JOY_CENTER_PIN,
    JOY_CENTER_PIN,
    JOY_CENTER_PIN,
};
static uint32_t s_power_toggle_combo_index = 0;

static const char *mode_to_string[] = {
    "RAINBOW",
    "SOLID",
    "STROBE",
};

typedef enum {
  LED_MODE_RAINBOW = 0,
  LED_MODE_SOLID,
  LED_MODE_STROBE,
  LED_MODE_COUNT,
} led_mode_t;

typedef struct {
  uint32_t rainbow_interval_ms;
  uint32_t strobe_interval_ms;
  uint32_t solid_hue_deg;
  uint32_t solid_high_brightness;
} effect_config_t;

typedef struct {
  TickType_t rainbow_last_update;
  uint16_t rainbow_base_hue;
  TickType_t solid_last_update;
  TickType_t strobe_last_toggle;
  bool strobe_on;
} effect_runtime_t;

static inline uint32_t clamp_u32(uint32_t value, uint32_t min, uint32_t max) {
  if (value < min) {
    return min;
  }
  if (value > max) {
    return max;
  }
  return value;
}

static void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r,
                              uint32_t *g, uint32_t *b) {
  h %= 360;
  uint32_t rgb_max = v * 2.55f;
  uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

  uint32_t i = h / 60;
  uint32_t diff = h % 60;
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

static void send_pixels(rmt_channel_handle_t led_chan,
                        rmt_encoder_handle_t led_encoder) {
  rmt_transmit_config_t tx_config = {
      .loop_count = 0,
  };
  ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels,
                               sizeof(led_strip_pixels), &tx_config));
  ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

static void render_rainbow(const effect_config_t *cfg, effect_runtime_t *rt,
                           TickType_t now_ticks, rmt_channel_handle_t led_chan,
                           rmt_encoder_handle_t led_encoder) {
  if ((now_ticks - rt->rainbow_last_update) <
      pdMS_TO_TICKS(cfg->rainbow_interval_ms)) {
    return;
  }
  rt->rainbow_last_update = now_ticks;

  rt->rainbow_base_hue = (rt->rainbow_base_hue + 3) % 360;

  for (int i = 0; i < EXAMPLE_LED_NUMBERS; ++i) {
    uint32_t hue =
        (rt->rainbow_base_hue + (i * 360 / EXAMPLE_LED_NUMBERS)) % 360;
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    led_strip_hsv2rgb(hue, 100, cfg->solid_high_brightness, &red, &green,
                      &blue);
    led_strip_pixels[i * 3 + 0] = green;
    led_strip_pixels[i * 3 + 1] = blue;
    led_strip_pixels[i * 3 + 2] = red;
  }

  send_pixels(led_chan, led_encoder);
}

static void render_solid(const effect_config_t *cfg, effect_runtime_t *rt,
                         TickType_t now_ticks, rmt_channel_handle_t led_chan,
                         rmt_encoder_handle_t led_encoder) {
  if ((now_ticks - rt->solid_last_update) <
      pdMS_TO_TICKS(SOLID_UPDATE_INTERVAL_MS)) {
    return;
  }
  rt->solid_last_update = now_ticks;

  uint32_t red = 0;
  uint32_t green = 0;
  uint32_t blue = 0;
  uint32_t value = cfg->solid_high_brightness;
  led_strip_hsv2rgb(cfg->solid_hue_deg, 90, value, &red, &green, &blue);

  for (int i = 0; i < EXAMPLE_LED_NUMBERS; ++i) {
    led_strip_pixels[i * 3 + 0] = green;
    led_strip_pixels[i * 3 + 1] = blue;
    led_strip_pixels[i * 3 + 2] = red;
  }

  send_pixels(led_chan, led_encoder);
}

static void render_strobe(const effect_config_t *cfg, effect_runtime_t *rt,
                          TickType_t now_ticks, rmt_channel_handle_t led_chan,
                          rmt_encoder_handle_t led_encoder) {
  if ((now_ticks - rt->strobe_last_toggle) <
      pdMS_TO_TICKS(cfg->strobe_interval_ms)) {
    return;
  }

  rt->strobe_last_toggle = now_ticks;
  rt->strobe_on = !rt->strobe_on;

  uint32_t red = 0;
  uint32_t green = 0;
  uint32_t blue = 0;
  uint32_t brightness = rt->strobe_on ? cfg->solid_high_brightness : 0;
  led_strip_hsv2rgb(0, 0, brightness, &red, &green, &blue);

  for (int i = 0; i < EXAMPLE_LED_NUMBERS; ++i) {
    led_strip_pixels[i * 3 + 0] = green;
    led_strip_pixels[i * 3 + 1] = blue;
    led_strip_pixels[i * 3 + 2] = red;
  }

  send_pixels(led_chan, led_encoder);
}

static void reset_runtime_for_mode(effect_runtime_t *rt, led_mode_t mode) {
  switch (mode) {
  case LED_MODE_RAINBOW:
    rt->rainbow_last_update = 0;
    rt->rainbow_base_hue = 0;
    break;
  case LED_MODE_SOLID:
    rt->solid_last_update = 0;
    break;
  case LED_MODE_STROBE:
    rt->strobe_last_toggle = 0;
    rt->strobe_on = false;
    break;
  default:
    break;
  }
}

static void update_led_modes(led_mode_t mode, bool lights_enabled,
                             const effect_config_t *cfg, effect_runtime_t *rt,
                             rmt_channel_handle_t led_chan,
                             rmt_encoder_handle_t led_encoder) {
  static led_mode_t last_mode = LED_MODE_COUNT;
  static bool was_enabled_last = true;

  if (!lights_enabled) {
    if (was_enabled_last) {
      memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
      send_pixels(led_chan, led_encoder);
    }
    was_enabled_last = false;
    return;
  }

  if (mode != last_mode) {
    reset_runtime_for_mode(rt, mode);
    last_mode = mode;
  }

  was_enabled_last = true;
  TickType_t now_ticks = xTaskGetTickCount();

  switch (mode) {
  case LED_MODE_RAINBOW:
    render_rainbow(cfg, rt, now_ticks, led_chan, led_encoder);
    break;
  case LED_MODE_SOLID:
    render_solid(cfg, rt, now_ticks, led_chan, led_encoder);
    break;
  case LED_MODE_STROBE:
    render_strobe(cfg, rt, now_ticks, led_chan, led_encoder);
    break;
  default:
    break;
  }
}

static void IRAM_ATTR gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(s_gpio_evt_queue, &gpio_num, NULL);
}

static void configure_input(gpio_num_t pin) {
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_POSEDGE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = 1ULL << pin,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  ESP_ERROR_CHECK(gpio_isr_handler_add(pin, gpio_isr_handler, (void *)pin));
}

static bool process_power_toggle_combo(uint32_t gpio_num,
                                       bool *lights_enabled) {
  if (gpio_num == s_power_toggle_combo[s_power_toggle_combo_index]) {
    s_power_toggle_combo_index++;
    if (gpio_num == JOY_CENTER_PIN) {
      ESP_LOGI(TAG, "Center combo progress %u/%u",
               (unsigned int)s_power_toggle_combo_index,
               (unsigned int)POWER_TOGGLE_COMBO_LENGTH);
    }
    if (s_power_toggle_combo_index == POWER_TOGGLE_COMBO_LENGTH) {
      *lights_enabled = !*lights_enabled;
      ESP_LOGI(TAG, "LED output %s (combo)",
               *lights_enabled ? "enabled" : "disabled");
      s_power_toggle_combo_index = 0;
      return true;
    }
  } else {
    s_power_toggle_combo_index = (gpio_num == s_power_toggle_combo[0]) ? 1 : 0;
  }
  return false;
}

static void handle_joystick_event(uint32_t gpio_num, led_mode_t *mode,
                                  effect_config_t *cfg, effect_runtime_t *rt,
                                  bool *lights_enabled) {
  if (gpio_num >= MAX_GPIO_INDEX) {
    return;
  }

  (void)process_power_toggle_combo(gpio_num, lights_enabled);

  led_mode_t previous_mode = *mode;

  switch (gpio_num) {
  case JOY_LEFT_PIN:
    printf("\n LEFT STICK \n");
    *mode = LED_MODE_RAINBOW;
    break;
  case JOY_RIGHT_PIN:
    printf("\n RIGHT STICK \n");
    *mode = LED_MODE_STROBE;
    break;
  case JOY_RESET_PIN:
    printf("\n RESET BUTTON \n");
    cfg->solid_high_brightness =
        clamp_u32(cfg->solid_high_brightness + SOLID_BRIGHTNESS_STEP,
                  SOLID_BRIGHTNESS_MIN, SOLID_BRIGHTNESS_MAX);

    ESP_LOGI(TAG, "Solid brightness %d", cfg->solid_high_brightness);
    break;
  case JOY_CENTER_PIN:
    printf("\n CENTER PRESS (combo input)\n");
    *mode = LED_MODE_SOLID;
    break;
  case JOY_SET_PIN:
    printf("\n SET BUTTON \n");

    cfg->solid_high_brightness =
        clamp_u32(cfg->solid_high_brightness - SOLID_BRIGHTNESS_STEP,
                  SOLID_BRIGHTNESS_MIN, SOLID_BRIGHTNESS_MAX);
    ESP_LOGI(TAG, "Solid brightness %d", cfg->solid_high_brightness);
    break;
  case JOY_UP_PIN:
    printf("\n UP STICK \n");
    if (*mode == LED_MODE_SOLID) {
      cfg->solid_hue_deg = (cfg->solid_hue_deg + SOLID_HUE_STEP_DEG) % 360;
    } else if (*mode == LED_MODE_RAINBOW) {
      uint32_t next = cfg->rainbow_interval_ms;
      next = (next <= RAINBOW_MIN_INTERVAL_MS + RAINBOW_INTERVAL_STEP_MS)
                 ? RAINBOW_MIN_INTERVAL_MS
                 : next - RAINBOW_INTERVAL_STEP_MS;
      cfg->rainbow_interval_ms =
          clamp_u32(next, RAINBOW_MIN_INTERVAL_MS, RAINBOW_MAX_INTERVAL_MS);
    } else if (*mode == LED_MODE_STROBE) {
      uint32_t next = cfg->strobe_interval_ms;
      next = (next <= STROBE_MIN_INTERVAL_MS + STROBE_INTERVAL_STEP_MS)
                 ? STROBE_MIN_INTERVAL_MS
                 : next - STROBE_INTERVAL_STEP_MS;
      cfg->strobe_interval_ms =
          clamp_u32(next, STROBE_MIN_INTERVAL_MS, STROBE_MAX_INTERVAL_MS);
    }
    break;
  case JOY_DOWN_PIN:
    printf("\n DOWN STICK \n");
    if (*mode == LED_MODE_SOLID) {
      cfg->solid_hue_deg =
          (cfg->solid_hue_deg + 360 - SOLID_HUE_STEP_DEG) % 360;
    } else if (*mode == LED_MODE_RAINBOW) {
      uint32_t next = cfg->rainbow_interval_ms + RAINBOW_INTERVAL_STEP_MS;
      cfg->rainbow_interval_ms =
          clamp_u32(next, RAINBOW_MIN_INTERVAL_MS, RAINBOW_MAX_INTERVAL_MS);
    } else if (*mode == LED_MODE_STROBE) {
      uint32_t next = cfg->strobe_interval_ms + STROBE_INTERVAL_STEP_MS;
      cfg->strobe_interval_ms =
          clamp_u32(next, STROBE_MIN_INTERVAL_MS, STROBE_MAX_INTERVAL_MS);
    }
    break;

  default:
    break;
  }

  if (*mode != previous_mode) {
    ESP_LOGI(TAG, "Switched to %s mode", mode_to_string[*mode]);
    reset_runtime_for_mode(rt, *mode);
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "Create RMT TX channel");
  rmt_channel_handle_t led_chan = NULL;
  rmt_tx_channel_config_t tx_chan_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .gpio_num = RMT_LED_STRIP_GPIO_NUM,
      .mem_block_symbols = 64,
      .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
      .trans_queue_depth = 4,
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

  effect_config_t config = {
      .rainbow_interval_ms = RAINBOW_DEFAULT_INTERVAL_MS,
      .strobe_interval_ms = STROBE_DEFAULT_INTERVAL_MS,
      .solid_hue_deg = SOLID_DEFAULT_HUE,
      .solid_high_brightness = SOLID_BRIGHTNSS_DEFAULT,
  };
  effect_runtime_t runtime = {
      .rainbow_last_update = 0,
      .rainbow_base_hue = 0,
      .solid_last_update = 0,
      .strobe_last_toggle = 0,
      .strobe_on = false,
  };

  led_mode_t current_mode = LED_MODE_SOLID;
  bool lights_enabled = true;

  s_gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));

  configure_input(JOY_UP_PIN);
  configure_input(JOY_DOWN_PIN);
  configure_input(JOY_LEFT_PIN);
  configure_input(JOY_RIGHT_PIN);
  configure_input(JOY_CENTER_PIN);
  configure_input(JOY_SET_PIN);
  configure_input(JOY_RESET_PIN);

  ESP_LOGI(TAG, "Joystick ready. COM must be tied to 3.3V.");

  while (1) {
    uint32_t io_num;
    if (xQueueReceive(s_gpio_evt_queue, &io_num, pdMS_TO_TICKS(5))) {
      if (io_num < MAX_GPIO_INDEX) {
        int64_t now = esp_timer_get_time();
        if (now - s_last_event_us[io_num] >= DEBOUNCE_TIME_US) {
          s_last_event_us[io_num] = now;
          int level = gpio_get_level((gpio_num_t)io_num);
          if (level == 1) {
            handle_joystick_event(io_num, &current_mode, &config, &runtime,
                                  &lights_enabled);
          }
        }
      }
    }

    update_led_modes(current_mode, lights_enabled, &config, &runtime, led_chan,
                     led_encoder);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
