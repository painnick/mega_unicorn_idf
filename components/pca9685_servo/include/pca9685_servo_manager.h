//
// Created by painnick on 25. 7. 13.
//

#ifndef PCA9685_SERVO_MANAGER_H
#define PCA9685_SERVO_MANAGER_H


#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pca9685.h"

#include "pca9685_servo.h"

void pca9685servo_init(i2c_dev_t *pca9685);

void pca9685servo_close();

void pca9685servo_set_servo(int idx, PCA9685Servo *servo);

PCA9685Servo* pca9685servo_get_servo(int idx);

#endif //PCA9685_SERVO_MANAGER_H
