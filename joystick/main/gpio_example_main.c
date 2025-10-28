
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdio.h>

static const char *TAG = "joystick";

// pick GPIOs that are stable for your board:
#define JOY_UP_PIN GPIO_NUM_0
#define JOY_DOWN_PIN GPIO_NUM_1
#define JOY_LEFT_PIN GPIO_NUM_3
#define JOY_RIGHT_PIN GPIO_NUM_4
#define JOY_MID_PIN GPIO_NUM_10
#define BTN_SET_PIN GPIO_NUM_6
#define BTN_RST_PIN GPIO_NUM_5

#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task(void *arg) {
  uint32_t io_num;
  // per-pin debounce timestamps
  int64_t last_event_us[32] = {0};

  while (1) {
    if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
      int64_t now = esp_timer_get_time();
      if (now - last_event_us[io_num] < 30000) {
        // ignore if within 30ms = debounce
        continue;
      }
      last_event_us[io_num] = now;

      // read the level after debounce window
      int level = gpio_get_level(io_num);
      if (level == 1) {
        // pressed
        switch (io_num) {
        case JOY_UP_PIN:
          ESP_LOGI(TAG, "UP PRESSED");
          break;
        case JOY_DOWN_PIN:
          ESP_LOGI(TAG, "DOWN PRESSED");
          break;
        case JOY_LEFT_PIN:
          ESP_LOGI(TAG, "LEFT PRESSED");
          break;
        case JOY_RIGHT_PIN:
          ESP_LOGI(TAG, "RIGHT PRESSED");
          break;
        case JOY_MID_PIN:
          ESP_LOGI(TAG, "CENTER PRESSED");
          break;
        case BTN_SET_PIN:
          ESP_LOGI(TAG, "SET PRESSED");
          break;
        case BTN_RST_PIN:
          ESP_LOGI(TAG, "RESET PRESSED");
          break;
        default:
          break;
        }
      }
    }
  }
}

static void configure_input(gpio_num_t pin) {
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_POSEDGE, // LOW -> HIGH, i.e. button press now
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = 1ULL << pin,
      .pull_up_en = GPIO_PULLUP_DISABLE,    // <- IMPORTANT CHANGE
      .pull_down_en = GPIO_PULLDOWN_ENABLE, // <- IMPORTANT CHANGE
  };
  gpio_config(&io_conf);

  gpio_isr_handler_add(pin, gpio_isr_handler, (void *)pin);
}

void app_main(void) {
  ESP_LOGI(TAG, "Initializing joystick (active HIGH mode)...");

  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  xTaskCreate(gpio_task, "gpio_task", 4096, NULL, 10, NULL);

  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

  configure_input(JOY_UP_PIN);
  configure_input(JOY_DOWN_PIN);
  configure_input(JOY_LEFT_PIN);
  configure_input(JOY_RIGHT_PIN);
  configure_input(JOY_MID_PIN);
  configure_input(BTN_SET_PIN);
  configure_input(BTN_RST_PIN);

  ESP_LOGI(TAG, "Joystick ready. COM must be tied to 3.3V.");
}
