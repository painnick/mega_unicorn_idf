//
// Created by painnick on 25. 7. 13.
//

#ifndef PCA9685_SERVO_H
#define PCA9685_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*  PCA9685 components  */
#include "pca9685_i2c.h"
#include "pca9685_i2c_hal.h"

#define TIMER_DIVIDER         80         // 타이머 분주기 (80MHz / 80 = 1MHz → 1 tick = 1μs)
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // 1,000,000
#define TIMER_INTERVAL_SEC    (1.0)      // 인터럽트 간격 (초)
#define TIMER_GROUP           TIMER_GROUP_0
#define TIMER_INDEX           TIMER_0


#define SG90_MIN_FREQ 100
#define SG90_MAX_FREQ 500


void pca9685servo_init(const pca9685_dev_t *pca9685);

void pca9685servo_close();

#ifdef __cplusplus
}
#endif

#endif //PCA9685_SERVO_H
