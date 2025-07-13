//
// Created by painnick on 25. 7. 13.
//

#ifndef PCA9685_SERVO_H
#define PCA9685_SERVO_H

#include <string>
#include "esp_log.h"

#include "pca9685_i2c.h"

class PCA9685Servo {
  public:
    const char *tag;

    uint16_t _pos = 0;
    uint16_t _min_val = 100;
    uint16_t _max_val = 500;
    uint16_t _step = 5;
    uint16_t _target = 100;

    explicit PCA9685Servo(
      const char *tag_,
      uint16_t pos_ = 100,
      uint16_t min_val_ = 100,
      uint16_t max_val_ = 500,
      uint16_t step_ = 5,
      uint16_t target_ = 100);

    void target(uint16_t target_);

    [[nodiscard]] uint16_t target() const { return _target; }

    void position(uint16_t pos_);

    [[nodiscard]] uint16_t position() const { return _pos; }

    void step(uint16_t step_);

    [[nodiscard]] uint16_t step() const { return _step; }

    void update(const pca9685_dev_t *pca9685_for_servo, uint8_t idx);
};

#endif //PCA9685_SERVO_H
