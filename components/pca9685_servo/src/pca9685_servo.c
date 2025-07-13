//
// Created by painnick on 25. 7. 13.
//

#include "pca9685_servo.h"

static const char *TAG = "PCA9685SRV";

#define TIMER_PERIOD_NS (20 * 1000) // 20ms

uint16_t SG90_CUR_FREQ = SG90_MIN_FREQ;
uint16_t SG90_STEP = 15;
bool SG90_FORWARD = true;
uint16_t SG90_WAIT_COUNT = 0;

const pca9685_dev_t *pca9685_for_servo;
esp_timer_handle_t periodic_timer;

static void pca9685servo_periodic_timer_callback(void *arg);

void pca9685servo_init(const pca9685_dev_t *pca9685) {
  pca9685_for_servo = pca9685;

  ESP_LOGI(TAG, "Initializing hardware timer...");
  const esp_timer_create_args_t periodic_timer_args = {
    .callback = &pca9685servo_periodic_timer_callback,
    /* name is optional, but may help identify the timer when debugging */
    .name = "pca9685servo_periodic"
  };

  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  /* The timer has been created but is not running yet */

  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, TIMER_PERIOD_NS));
}

void pca9685servo_close() {
  /* Clean up and finish the example */
  ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
  ESP_ERROR_CHECK(esp_timer_delete(periodic_timer));
}

static void pca9685servo_periodic_timer_callback(void *arg) {
  if (SG90_WAIT_COUNT > 0) {
    SG90_WAIT_COUNT--;
    return;
  }

  if (SG90_FORWARD) {
    SG90_CUR_FREQ += SG90_STEP;
    if (SG90_CUR_FREQ > SG90_MAX_FREQ + (SG90_STEP - 1)) {
      SG90_WAIT_COUNT = 50; // 20ms x 50 = 1,000ms;

      SG90_FORWARD = false;
      SG90_CUR_FREQ -= SG90_STEP * 2;
    }
  } else {
    SG90_CUR_FREQ -= SG90_STEP;
    if (SG90_CUR_FREQ < SG90_MIN_FREQ - (SG90_STEP - 1)) {
      SG90_WAIT_COUNT = 50; // 20ms x 50 = 1,000ms;

      SG90_FORWARD = true;
      SG90_CUR_FREQ += SG90_STEP * 2;
    }
  }

  // const int64_t time_since_boot = esp_timer_get_time();
  // ESP_LOGI(TAG, "Periodic timer called, time since boot: %lld ms - val %d", time_since_boot / 1000, SG90_CUR_FREQ);

  pca9685_i2c_led_pwm_set2(*pca9685_for_servo, SERVO_OUTPUT_PIN_1, SG90_CUR_FREQ, 0);
}
