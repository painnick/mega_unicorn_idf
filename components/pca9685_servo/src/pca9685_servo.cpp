//
// Created by painnick on 25. 7. 13.
//

#include "pca9685_servo.h"

static auto TAG = "PCA9685SRV";

PCA9685Servo::PCA9685Servo(
  const std::string &tag_,
  const uint16_t pos_,
  const uint16_t min_val_,
  const uint16_t max_val_,
  const int16_t step_,
  const uint16_t target_) : _tag(tag_),
                            _pos(pos_),
                            _min_val(min_val_),
                            _max_val(max_val_),
                            _step(step_),
                            _target(target_) {
  if (_target > _max_val) {
    ESP_LOGW(TAG, "Servo[%s] target(%d) is set to max(%d).", _tag.c_str(), _target, _max_val);
    _target = _max_val;
  }
  if (_target < _min_val) {
    ESP_LOGW(TAG, "Servo[%s] target(%d) is set to min(%d).", _tag.c_str(), _target, _min_val);
    _target = _min_val;
  }

  if (_pos > _max_val) {
    ESP_LOGW(TAG, "Servo[%s] position(%d) is set to max(%d).", _tag.c_str(), _pos, _max_val);
    _pos = _max_val;
  }
  if (_pos < _min_val) {
    ESP_LOGW(TAG, "Servo[%s] position(%d) is set to min(%d).", _tag.c_str(), _pos, _min_val);
    _pos = _min_val;
  }

  if (_step == 0) {
    ESP_LOGV(TAG, "Servo[%s] is frozen.", _tag.c_str());
  } else if (_step > 0) {
    if (_pos > _target) {
      ESP_LOGW(TAG, "Servo[%s] position(%d) is set to target(%d).", _tag.c_str(), _pos, _target);
      _pos = _target;
    }
  } else {
    if (_pos < _target) {
      ESP_LOGW(TAG, "Servo[%s] position(%d) is set to target(%d).", _tag.c_str(), _pos, _target);
      _pos = _target;
    }
  }
}

void PCA9685Servo::target(const uint16_t target_) {
  _target = target_;

  if (_target > _max_val) {
    ESP_LOGW(TAG, "Servo[%s] target(%d) is set to max(%d).", _tag.c_str(), target_, _max_val);
    _target = _max_val;
  }
  if (_target < _min_val) {
    ESP_LOGW(TAG, "Servo[%s] target(%d) is set to min(%d).", _tag.c_str(), target_, _min_val);
    _target = _min_val;
  }
}

void PCA9685Servo::target(const uint16_t target_, const OnReached &onReached_) {
  target(target_);
  _onReached = onReached_;
}

void PCA9685Servo::position(const uint16_t pos_) {
  _pos = pos_;

  if (_pos > _max_val) {
    ESP_LOGW(TAG, "Servo[%s] position(%d) is set to max(%d).", _tag.c_str(), _pos, _max_val);
    _pos = _max_val;
  }
  if (_pos < _min_val) {
    ESP_LOGW(TAG, "Servo[%s] position(%d) is set to min(%d).", _tag.c_str(), _pos, _min_val);
    _pos = _min_val;
  }

  _target = _pos;
  _step = 0;
  ESP_LOGW(TAG, "Servo[%s] is set to(%d). And is frozen.", _tag.c_str(), _pos);
}

void PCA9685Servo::step(const int16_t step_) {
  _step = step_;
  ESP_LOGI(TAG, "Servo[%s] Set step %d.", _tag.c_str(), _step);
}

void PCA9685Servo::update(i2c_dev_t *dev, const uint8_t idx) {
  if (_step == 0) {
    ESP_LOGV(TAG, "Servo[%s] is frozen.", _tag.c_str());
  } else if (_step > 0) {
    if (_pos < _target && _pos < _max_val) {
      _pos = (_pos + _step) > _target ? _target : (_pos + _step);
      pca9685_set_pwm_value(dev, idx, _pos);
      ESP_LOGD(TAG, "Servo[%s] [FWD] Move to %d.", _tag.c_str(), _pos);
    } else {
      auto last_step = _step;
      _step = 0;
      ESP_LOGD(TAG, "Servo[%s] [FWD] Reached %d.", _tag.c_str(), _target);
      _onReached(this, last_step);
      _onReached = [](PCA9685Servo *pca9685_servo, short i) {
      };
    }
  } else {
    if (_pos > _target && _pos > _min_val) {
      _pos = (_pos + _step) < _target ? _target : (_pos + _step);
      pca9685_set_pwm_value(dev, idx, _pos);
      ESP_LOGD(TAG, "Servo[%s] [BACK] Move to %d.", _tag.c_str(), _pos);
    } else {
      auto last_step = _step;
      _step = 0;
      ESP_LOGD(TAG, "Servo[%s] [BACK] Reached %d.", _tag.c_str(), _target);
      _onReached(this, last_step);
      _onReached = [](PCA9685Servo *pca9685_servo, short i) {
      };
    }
  }
}
