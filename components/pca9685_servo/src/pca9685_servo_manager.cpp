//
// Created by painnick on 25. 7. 13.
//

#include "pca9685_servo_manager.h"

#include "esp_timer.h"

static auto TAG = "PCA9685SRVMNG";

#define TIMER_PERIOD_NS (20 * 1000) // 20ms

i2c_dev_t *dev;
esp_timer_handle_t servo_periodic_timer;

PCA9685Servo *servos[16] = {nullptr};

static void servo_periodic_timer_callback(void *arg);

void pca9685servo_init(i2c_dev_t *pca9685) {
  dev = pca9685;

  ESP_LOGI(TAG, "Initializing hardware timer...");
  constexpr esp_timer_create_args_t periodic_timer_args = {
    .callback = &servo_periodic_timer_callback,
    /* name is optional, but may help identify the timer when debugging */
    .name = "pca9685servo_periodic"
  };

  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &servo_periodic_timer));
  /* The timer has been created but is not running yet */

  ESP_ERROR_CHECK(esp_timer_start_periodic(servo_periodic_timer, TIMER_PERIOD_NS));
}

void pca9685servo_close() {
  /* Clean up and finish the example */
  ESP_ERROR_CHECK(esp_timer_stop(servo_periodic_timer));
  ESP_ERROR_CHECK(esp_timer_delete(servo_periodic_timer));
}

void pca9685servo_set_servo(const int idx, PCA9685Servo *servo) {
  servos[idx] = servo;
  pca9685_set_pwm_value(dev, idx, servo->position());
}

PCA9685Servo* pca9685servo_get_servo(const int idx) {
  return servos[idx];
}

static void servo_periodic_timer_callback(void *arg) {
  // const int64_t time_since_boot = esp_timer_get_time();
  // ESP_LOGI(TAG, "Periodic timer called, time since boot: %lld ms - val %d", time_since_boot / 1000, SG90_CUR_FREQ);

  for (int i = 0; i < 16; i++) {
    if (servos[i] == nullptr) {
      ESP_LOGV(TAG, "Servo[%d] Isn't set.", i);
    } else {
      servos[i]->update(dev, i);
    }
  }
}
