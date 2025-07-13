//
// Created by painnick on 25. 7. 13.
//

#include "pca9685_servo.h"

static const char *TAG = "PCA9685SRV";

#define TIMER_PERIOD_NS (20 * 1000) // 20ms

const pca9685_dev_t *pca9685_for_servo;
esp_timer_handle_t periodic_timer;

pca9685_servo_t PCA9685Servo[16] = {};

static void pca9685servo_periodic_timer_callback(void *arg);

void pca9685servo_init(const pca9685_dev_t *pca9685) {
  pca9685_for_servo = pca9685;

  ESP_LOGI(TAG, "Initializing hardware timer...");
  constexpr esp_timer_create_args_t periodic_timer_args = {
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

void pca9685servo_set_servo(const int idx, const pca9685_servo_t servo) {
  PCA9685Servo[idx] = servo;

  if (PCA9685Servo[idx].pos > PCA9685Servo[idx].max_val) {
    ESP_LOGW(TAG, "Servo[%d] position(%d) is set to max(%d)", idx, PCA9685Servo[idx].pos, PCA9685Servo[idx].max_val);
    PCA9685Servo[idx].pos = PCA9685Servo[idx].max_val;
  }
  if (PCA9685Servo[idx].pos < PCA9685Servo[idx].min_val) {
    ESP_LOGW(TAG, "Servo[%d] position(%d) is set to min(%d)", idx, PCA9685Servo[idx].pos, PCA9685Servo[idx].min_val);
    PCA9685Servo[idx].pos = PCA9685Servo[idx].min_val;
  }

  if (PCA9685Servo[idx].target > PCA9685Servo[idx].max_val) {
    ESP_LOGW(TAG, "Servo[%d] target(%d) is set to max(%d)", idx, PCA9685Servo[idx].target, PCA9685Servo[idx].max_val);
    PCA9685Servo[idx].target = PCA9685Servo[idx].max_val;
  }
  if (PCA9685Servo[idx].target < PCA9685Servo[idx].min_val) {
    ESP_LOGW(TAG, "Servo[%d] target(%d) is set to min(%d)", idx, PCA9685Servo[idx].target, PCA9685Servo[idx].min_val);
    PCA9685Servo[idx].target = PCA9685Servo[idx].min_val;
  }

  if (PCA9685Servo[idx].step == 0) {
    ESP_LOGV(TAG, "Servo[%d] is frozen.", idx);
  } else if (PCA9685Servo[idx].step > 0) {
    if (PCA9685Servo[idx].pos > PCA9685Servo[idx].target) {
      ESP_LOGW(TAG, "Servo[%d] position(%d) is set to %d", idx, PCA9685Servo[idx].pos, PCA9685Servo[idx].target);
      PCA9685Servo[idx].pos = PCA9685Servo[idx].target;
    }
  } else {
    if (PCA9685Servo[idx].pos < PCA9685Servo[idx].target) {
      ESP_LOGW(TAG, "Servo[%d] position(%d) is set to %d", idx, PCA9685Servo[idx].pos, PCA9685Servo[idx].target);
      PCA9685Servo[idx].pos = PCA9685Servo[idx].target;
    }
  }
}

void pca9685servo_set_target(const int idx, const uint16_t target) {
  PCA9685Servo[idx].target = target;
  if (PCA9685Servo[idx].target > PCA9685Servo[idx].max_val) {
    ESP_LOGW(TAG, "Servo[%d] target(%d) is set to max(%d)", idx, PCA9685Servo[idx].target, PCA9685Servo[idx].max_val);
    PCA9685Servo[idx].target = PCA9685Servo[idx].max_val;
  }
  if (PCA9685Servo[idx].target < PCA9685Servo[idx].min_val) {
    ESP_LOGW(TAG, "Servo[%d] target(%d) is set to min(%d)", idx, PCA9685Servo[idx].target, PCA9685Servo[idx].min_val);
    PCA9685Servo[idx].target = PCA9685Servo[idx].min_val;
  }
}

void pca9685servo_set_position(const int idx, const uint16_t pos) {
  PCA9685Servo[idx].pos = pos;
  if (PCA9685Servo[idx].pos > PCA9685Servo[idx].max_val) {
    ESP_LOGW(TAG, "Servo[%d] position(%d) is set to max(%d)", idx, PCA9685Servo[idx].pos, PCA9685Servo[idx].max_val);
    PCA9685Servo[idx].pos = PCA9685Servo[idx].max_val;
  }
  if (PCA9685Servo[idx].pos < PCA9685Servo[idx].min_val) {
    ESP_LOGW(TAG, "Servo[%d] position(%d) is set to min(%d)", idx, PCA9685Servo[idx].pos, PCA9685Servo[idx].min_val);
    PCA9685Servo[idx].pos = PCA9685Servo[idx].min_val;
  }

  if (PCA9685Servo[idx].step == 0) {
    ESP_LOGV(TAG, "Servo[%d] is frozen.", idx);
  } else if (PCA9685Servo[idx].step > 0) {
    if (PCA9685Servo[idx].pos > PCA9685Servo[idx].target) {
      ESP_LOGW(TAG, "Servo[%d] position(%d) is set to %d", idx, PCA9685Servo[idx].pos, PCA9685Servo[idx].target);
      PCA9685Servo[idx].pos = PCA9685Servo[idx].target;
    }
  } else {
    if (PCA9685Servo[idx].pos < PCA9685Servo[idx].target) {
      ESP_LOGW(TAG, "Servo[%d] position(%d) is set to %d", idx, PCA9685Servo[idx].pos, PCA9685Servo[idx].target);
      PCA9685Servo[idx].pos = PCA9685Servo[idx].target;
    }
  }

  PCA9685Servo[idx].step = 0;
  PCA9685Servo[idx].target = PCA9685Servo[idx].pos;
  pca9685_i2c_led_pwm_set2(*pca9685_for_servo, idx, PCA9685Servo[idx].pos, 0);
}

static void pca9685servo_periodic_timer_callback(void *arg) {
  // const int64_t time_since_boot = esp_timer_get_time();
  // ESP_LOGI(TAG, "Periodic timer called, time since boot: %lld ms - val %d", time_since_boot / 1000, SG90_CUR_FREQ);

  for (int i = 0; i < 16; i++) {
    if (PCA9685Servo[i].max_val == 0) {
      // ESP_LOGV(TAG, "Servo[%d] Isn't set.", i);
    } else {
      if (PCA9685Servo[i].step == 0) {
        // ESP_LOGV(TAG, "Servo[%d] is frozen.", i);
      } else if (PCA9685Servo[i].step > 0) {
        if (PCA9685Servo[i].pos < PCA9685Servo[i].target && PCA9685Servo[i].pos < PCA9685Servo[i].max_val) {
          PCA9685Servo[i].pos = (PCA9685Servo[i].pos + PCA9685Servo[i].step) > PCA9685Servo[i].target
                                  ? PCA9685Servo[i].target
                                  : (PCA9685Servo[i].pos + PCA9685Servo[i].step);
          pca9685_i2c_led_pwm_set2(*pca9685_for_servo, i, PCA9685Servo[i].pos, 0);
          // ESP_LOGV(TAG, "Servo[%d] [FWD] Move to %d.", i, PCA9685Servo[i].pos);
        } else {
          // ESP_LOGV(TAG, "Servo[%d] [FWD] Reached %d.", i, PCA9685Servo[i].target);
        }
      } else {
        if (PCA9685Servo[i].pos > PCA9685Servo[i].target && PCA9685Servo[i].pos > PCA9685Servo[i].min_val) {
          PCA9685Servo[i].pos = (PCA9685Servo[i].pos + PCA9685Servo[i].step) < PCA9685Servo[i].target
                                  ? PCA9685Servo[i].target
                                  : (PCA9685Servo[i].pos + PCA9685Servo[i].step);
          pca9685_i2c_led_pwm_set2(*pca9685_for_servo, i, PCA9685Servo[i].pos, 0);
          // ESP_LOGV(TAG, "Servo[%d] [BAC] Move to %d.", i, PCA9685Servo[i].pos);
        } else {
          // ESP_LOGV(TAG, "Servo[%d] [BAC] Reached %d.", i, PCA9685Servo[i].target);
        }
      }
    }
  }
}
