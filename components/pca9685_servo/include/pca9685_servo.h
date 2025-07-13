//
// Created by painnick on 25. 7. 13.
//

#ifndef PCA9685_SERVO_H
#define PCA9685_SERVO_H

#include <functional>
#include <string>
#include "esp_log.h"

#include "pca9685_i2c.h"

class PCA9685Servo;
typedef std::function<void(PCA9685Servo *servo, int16_t step)> OnReached;

class PCA9685Servo {
  protected:
    std::string _tag;
    uint16_t _pos = 0;
    uint16_t _min_val = 100;
    uint16_t _max_val = 500;
    int16_t _step = 5;
    uint16_t _target = 100;

    OnReached _onReached;

  public:
    explicit PCA9685Servo(
      const std::string &tag_,
      uint16_t pos_ = 100,
      uint16_t min_val_ = 100,
      uint16_t max_val_ = 500,
      int16_t step_ = 5,
      uint16_t target_ = 100);

    uint16_t minValue() const { return _min_val; };
    uint16_t maxValue() const { return _max_val; };

    void target(uint16_t target_);

    [[nodiscard]] uint16_t target() const { return _target; }

    void position(uint16_t pos_);

    [[nodiscard]] uint16_t position() const { return _pos; }

    void step(int16_t step_);

    [[nodiscard]] int16_t step() const { return _step; }

    void update(const pca9685_dev_t *pca9685_for_servo, uint8_t idx);

    void onReached(const OnReached &onReached_) { _onReached = onReached_; };
};

#endif //PCA9685_SERVO_H
