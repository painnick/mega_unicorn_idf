#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

/*  PCA9685 components  */
#include "pca9685_i2c.h"
#include "pca9685_i2c_hal.h"

static const char *TAG = "main";

#define USE_I2C_ADDRESS         I2C_DEFAULT_ADDRESS

#define TIMER_DIVIDER         80         // 타이머 분주기 (80MHz / 80 = 1MHz → 1 tick = 1μs)
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // 1,000,000
#define TIMER_INTERVAL_SEC    (1.0)      // 인터럽트 간격 (초)
#define TIMER_GROUP           TIMER_GROUP_0
#define TIMER_INDEX           TIMER_0

pca9685_dev_t pca9685_1;

#define SG90_MIN_FREQ 100
#define SG90_MAX_FREQ 500

uint16_t SG90_CUR_FREQ = SG90_MIN_FREQ;
uint16_t SG90_STEP = 15;
bool SG90_FORWARD = true;
uint16_t SG90_WAIT_COUNT = 0;

static void periodic_timer_callback(void *arg) {
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

  pca9685_i2c_led_pwm_set2(pca9685_1, SERVO_OUTPUT_PIN_1, SG90_CUR_FREQ, 0);
}

void app_main(void) {
  esp_err_t err = ESP_OK;

  pca9685_i2c_hal_init();

  err += pca9685_i2c_reset(); /* Perform all device reset */
  ESP_LOGI(TAG, "Device reset: %s", err == PCA9685_OK ? "Successful" : "Failed");

  /* Register i2c device*/
  pca9685_i2c_register(&pca9685_1,
                       I2C_ADDRESS_PCA9685,
                       I2C_ALL_CALL_ADDRESS_PCA9685,
                       I2C_SUB_ADDRESS_1_PCA9685,
                       I2C_SUB_ADDRESS_2_PCA9685,
                       I2C_SUB_ADDRESS_3_PCA9685); /* Use default values */

#if USE_I2C_ADDRESS == I2C_DEFAULT_ADDRESS

  //Already in default I2C address

#elif USE_I2C_ADDRESS == I2C_SUB_ADDRESS_1

    err += pca9685_i2c_sub_addr_resp(pca9685_1, PCA9685_SUB_ADDR_1, PCA9685_ADDR_RESPOND); /* Enable subaddres 1 response*/
    ESP_LOGI(TAG, "Enable subaddress 1: %s", err == PCA9685_OK ? "Successful" : "Failed");
    pca9685_1.i2c_addr =  pca9685_1.sub_addr_1; /* Assign i2c_addr to subaddress 1 */

#elif USE_I2C_ADDRESS == I2C_SUB_ADDRESS_2

    err += pca9685_i2c_sub_addr_resp(pca9685_1, PCA9685_SUB_ADDR_2, PCA9685_ADDR_RESPOND); /* Enable subaddres 2 response*/
    ESP_LOGI(TAG, "Enable subaddress 2: %s", err == PCA9685_OK ? "Successful" : "Failed");
    pca9685_1.i2c_addr =  pca9685_1.sub_addr_2; /* Assign i2c_addr to subaddress 2 */

#elif USE_I2C_ADDRESS == I2C_SUB_ADDRESS_3

    err += pca9685_i2c_sub_addr_resp(pca9685_1, PCA9685_SUB_ADDR_3, PCA9685_ADDR_RESPOND); /* Enable subaddres 3 response*/
    ESP_LOGI(TAG, "Enable subaddress 3: %s", err == PCA9685_OK ? "Successful" : "Failed");
    pca9685_1.i2c_addr =  pca9685_1.sub_addr_3; /* Assign i2c_addr to subaddress 3 */

#elif USE_I2C_ADDRESS == I2C_ALLCALL_ADDRESS

    /* All call address response is already enabled by default */
    pca9685_1.i2c_addr =  pca9685_1.allcall_addr;

#endif

  ESP_LOGI(TAG, "i2c_addr: 0x%02x", pca9685_1.i2c_addr);

  err += pca9685_i2c_write_pre_scale(pca9685_1, SERVO_PWM_FREQ, PCA9685_OSC_CLK);
  /* Setting frequency to 50 Hz (200ms) */
  ESP_LOGI(TAG, "Frequency Setting: %s", err == PCA9685_OK ? "Successful" : "Failed");

  pca9685_i2c_sleep_mode(pca9685_1, PCA9685_MODE_NORMAL);
  pca9685_i2c_autoincrement(pca9685_1, PCA9685_AUTOINCR_ON); /* Register increment every read/write */

  if (err == ESP_OK) {
    ESP_LOGI(TAG, "PCA9685 initialization successful");
    // pca9685_i2c_led_pwm_set2(pca9685_1, SERVO_OUTPUT_PIN_1, 307, 0);
    // pca9685_i2c_hal_ms_delay(1000 * 10);

    ESP_LOGI(TAG, "Initializing hardware timer...");
    const esp_timer_create_args_t periodic_timer_args = {
      .callback = &periodic_timer_callback,
      /* name is optional, but may help identify the timer when debugging */
      .name = "periodic"
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    /* The timer has been created but is not running yet */

    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 20 * 1000));

    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* Clean up and finish the example */
    ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
    ESP_ERROR_CHECK(esp_timer_delete(periodic_timer));
  } else {
    ESP_LOGE(TAG, "PCA9685 initialization failed!");
  }
}
