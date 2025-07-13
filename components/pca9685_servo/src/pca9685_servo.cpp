//
// Created by painnick on 25. 7. 13.
//

#include "pca9685_servo.h"

static auto TAG = "PCA9685SRV";

PCA9685Servo::PCA9685Servo(
  const char *tag_,
  const uint16_t pos_,
  const uint16_t min_val_,
  const uint16_t max_val_,
  const uint16_t step_,
  const uint16_t target_) : tag(tag_),
                            _pos(pos_),
                            _min_val(min_val_),
                            _max_val(max_val_),
                            _step(step_),
                            _target(target_) {
  if (_target > _max_val) {
    ESP_LOGW(TAG, "Servo[%s] target(%d) is set to max(%d).", tag, _target, _max_val);
    _target = _max_val;
  }
  if (_target < _min_val) {
    ESP_LOGW(TAG, "Servo[%s] target(%d) is set to min(%d).", tag, _target, _min_val);
    _target = _min_val;
  }

  if (_pos > _max_val) {
    ESP_LOGW(TAG, "Servo[%s] position(%d) is set to max(%d).", tag, _pos, _max_val);
    _pos = _max_val;
  }
  if (_pos < _min_val) {
    ESP_LOGW(TAG, "Servo[%s] position(%d) is set to min(%d).", tag, _pos, _min_val);
    _pos = _min_val;
  }

  if (_step == 0) {
    ESP_LOGV(TAG, "Servo[%s] is frozen.", tag);
  } else if (_step > 0) {
    if (_pos > _target) {
      ESP_LOGW(TAG, "Servo[%s] position(%d) is set to target(%d).", tag, _pos, _target);
      _pos = _target;
    }
  } else {
    if (_pos < _target) {
      ESP_LOGW(TAG, "Servo[%s] position(%d) is set to target(%d).", tag, _pos, _target);
      _pos = _target;
    }
  }
}

void PCA9685Servo::target(const uint16_t target_) {
  _target = target_;

  if (_target > _max_val) {
    ESP_LOGW(TAG, "Servo[%s] target(%d) is set to max(%d).", tag, target_, _max_val);
    _target = _max_val;
  }
  if (_target < _min_val) {
    ESP_LOGW(TAG, "Servo[%s] target(%d) is set to min(%d).", tag, target_, _min_val);
    _target = _min_val;
  }
}

void PCA9685Servo::position(const uint16_t pos_) {
  _pos = pos_;

  if (_pos > _max_val) {
    ESP_LOGW(TAG, "Servo[%s] position(%d) is set to max(%d).", tag, _pos, _max_val);
    _pos = _max_val;
  }
  if (_pos < _min_val) {
    ESP_LOGW(TAG, "Servo[%s] position(%d) is set to min(%d).", tag, _pos, _min_val);
    _pos = _min_val;
  }

  _target = _pos;
  _step = 0;
  ESP_LOGW(TAG, "Servo[%s] is set to(%d). And is frozen.", tag, _pos);
}

void PCA9685Servo::step(const uint16_t step_) {
  _step = step_;
}

void PCA9685Servo::update(const pca9685_dev_t *pca9685_for_servo, uint8_t idx) {
  if (_step == 0) {
    ESP_LOGV(TAG, "Servo[%s] is frozen.", tag);
  } else if (_step > 0) {
    if (_pos < _target && _pos < _max_val) {
      _pos = (_pos + _step) > _target ? _target : (_pos + _step);
      pca9685_i2c_led_pwm_set2(*pca9685_for_servo, idx, _pos, 0);
      ESP_LOGV(TAG, "Servo[%s] [FWD] Move to %d.", tag, _pos);
    } else {
      ESP_LOGV(TAG, "Servo[%s] [FWD] Reached %d.", tag, _target);
    }
  } else {
    if (_pos > _target && _pos > _min_val) {
      _pos = (_pos + _step) < _target ? _target : (_pos + _step);
      pca9685_i2c_led_pwm_set2(*pca9685_for_servo, idx, _pos, 0);
      ESP_LOGV(TAG, "Servo[%s] [BAC] Move to %d.", tag, _pos);
    } else {
      ESP_LOGV(TAG, "Servo[%s] [BAC] Reached %d.", tag, _target);
    }
  }
}
