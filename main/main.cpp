#include <cstdio>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include <cstring>
#include <format>

#include "pca9685.h"
#include "pca9685_servo_manager.h"

static auto TAG = "main";

i2c_dev_t pca9685_dev;


extern "C" void app_main(void) {
  ESP_ERROR_CHECK(i2cdev_init());

  std::memset(&pca9685_dev, 0, sizeof(i2c_dev_t));

  ESP_ERROR_CHECK(
    pca9685_init_desc(&pca9685_dev, CONFIG_PCA9685_I2C_ADDR, i2c_port_t::I2C_NUM_0,
      static_cast<gpio_num_t> (CONFIG_PCA9685_I2C_MASTER_SDA),
      static_cast<gpio_num_t> (CONFIG_PCA9685_I2C_MASTER_SCL)));
  ESP_ERROR_CHECK(pca9685_init(&pca9685_dev));

  ESP_ERROR_CHECK(pca9685_restart(&pca9685_dev));

  pca9685servo_init(&pca9685_dev);

  for (int idx = 0; idx < 16; idx++) {
    const auto sg90 = new PCA9685Servo(std::format("Pin{}", idx));
    sg90->step(0);

    pca9685servo_set_servo(idx, sg90);
  }

  vTaskDelay(pdMS_TO_TICKS(1000 * 5)); // 5,000ms

  for (int idx = 0; idx < 16; idx++) {
    const auto sg90 = pca9685servo_get_servo(idx);
    sg90->target(500,
                 [](PCA9685Servo *pca9685_servo, const int16_t step) {
                   const auto abs_step = 1;
                   if (step > 0) {
                     pca9685_servo->target(100);
                     pca9685_servo->step(abs_step * -1);
                   } else {
                     pca9685_servo->target(500);
                     pca9685_servo->step(abs_step);
                   }
                 });
    sg90->step(1);
  }

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  pca9685servo_close();
}
