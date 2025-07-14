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
#include "servo_task_manager.h"

static auto TAG = "main";

i2c_dev_t pca9685_dev;


extern "C" void app_main(void) {
  initServoTaskManager();

  ESP_ERROR_CHECK(i2cdev_init());

  std::memset(&pca9685_dev, 0, sizeof(i2c_dev_t));

  ESP_ERROR_CHECK(
    pca9685_init_desc(&pca9685_dev, CONFIG_PCA9685_I2C_ADDR, i2c_port_t::I2C_NUM_0,
      static_cast<gpio_num_t> (CONFIG_PCA9685_I2C_MASTER_SDA),
      static_cast<gpio_num_t> (CONFIG_PCA9685_I2C_MASTER_SCL)));
  ESP_ERROR_CHECK(pca9685_init(&pca9685_dev));

  ESP_ERROR_CHECK(pca9685_restart(&pca9685_dev));

  pca9685servo_init(&pca9685_dev);

  const auto servo3_Leg1 = new PCA9685Servo(std::format("Leg{}", 1));
  pca9685servo_set_servo(3, servo3_Leg1);

  const auto servo1_Foot2 = new PCA9685Servo(std::format("Foot{}", 2));
  pca9685servo_set_servo(1, servo1_Foot2);



  vTaskDelay(pdMS_TO_TICKS(1000 * 5)); // 5,000ms

  // Leg1
  addServoTask(servo3_Leg1,
               0,
               1,
               500);

  addServoTask(servo3_Leg1,
               2000,
               -3,
               100);

  addServoTask(servo3_Leg1,
               7000,
               1,
               500);

  // Foot2
  addServoTask(servo1_Foot2,
               0,
               2,
               500);

  addServoTask(servo1_Foot2,
               2000,
               -1,
               100);

  addServoTask(servo1_Foot2,
               7000,
               3,
               500);

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  pca9685servo_close();
}
