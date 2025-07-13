//
// Created by painnick on 25. 7. 13.
//

#include "../include/pca9685_servo_manager.h"

static auto TAG = "PCA9685SRVMNG";

#define TIMER_PERIOD_NS (20 * 1000) // 20ms

const pca9685_dev_t *pca9685_for_servo;
esp_timer_handle_t pca9685servo_periodic_timer;

PCA9685Servo *servos[16] = {nullptr};

static void pca9685servo_periodic_timer_callback(void *arg);

void pca9685servo_init(const pca9685_dev_t *pca9685) {
  pca9685_for_servo = pca9685;

  ESP_LOGI(TAG, "Initializing hardware timer...");
  constexpr esp_timer_create_args_t periodic_timer_args = {
    .callback = &pca9685servo_periodic_timer_callback,
    /* name is optional, but may help identify the timer when debugging */
    .name = "pca9685servo_periodic"
  };

  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pca9685servo_periodic_timer));
  /* The timer has been created but is not running yet */

  ESP_ERROR_CHECK(esp_timer_start_periodic(pca9685servo_periodic_timer, TIMER_PERIOD_NS));
}

void pca9685servo_close() {
  /* Clean up and finish the example */
  ESP_ERROR_CHECK(esp_timer_stop(pca9685servo_periodic_timer));
  ESP_ERROR_CHECK(esp_timer_delete(pca9685servo_periodic_timer));
}

void pca9685servo_set_servo(const int idx, PCA9685Servo *servo) {
  servos[idx] = servo;
}

static void pca9685servo_periodic_timer_callback(void *arg) {
  // const int64_t time_since_boot = esp_timer_get_time();
  // ESP_LOGI(TAG, "Periodic timer called, time since boot: %lld ms - val %d", time_since_boot / 1000, SG90_CUR_FREQ);

  for (int i = 0; i < 16; i++) {
    if (servos[i] == nullptr) {
      ESP_LOGV(TAG, "Servo[%d] Isn't set.", i);
    } else {
      servos[i]->update(pca9685_for_servo, i);
    }
  }
}
