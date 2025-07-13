#include <cstdio>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

/*  PCA9685 components  */
#include <format>

#include "pca9685_i2c.h"
#include "pca9685_i2c_hal.h"

#include "pca9685_servo_manager.h"

static const char *TAG = "main";

#define USE_I2C_ADDRESS         I2C_DEFAULT_ADDRESS


pca9685_dev_t pca9685_1;


extern "C" void app_main(void) {
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

    pca9685servo_init(&pca9685_1);

    for (int idx = 0; idx < 16; idx++) {
      const auto sg90 = new PCA9685Servo(std::format("Pin{}", idx));
      sg90->target(500);
      sg90->onReached([](PCA9685Servo *pca9685_servo, const int16_t step) {
        const auto abs_step = (esp_random() % 5) + 1;
        if (step > 0) {
          pca9685_servo->target(100);
          pca9685_servo->step(abs_step * -1);
        } else {
          pca9685_servo->target(500);
          pca9685_servo->step(abs_step);
        }
      });

      pca9685servo_set_servo(idx, sg90);
    }

    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    pca9685servo_close();
  } else {
    ESP_LOGE(TAG, "PCA9685 initialization failed!");
  }
}
