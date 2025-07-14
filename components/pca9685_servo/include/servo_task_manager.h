#ifndef SERVO_TASK_MANAGER_H
#define SERVO_TASK_MANAGER_H

#include "pca9685_servo.h"


#include <functional>
#include <utility>

class ServoTask {
  public:
    int64_t run_ms;
    PCA9685Servo *servo;
    int16_t step;
    uint16_t target;
    OnReached onReached;

    ServoTask(const int64_t run_ms_,
              PCA9685Servo *servo_,
              const int16_t step_,
              const uint16_t target_,
              OnReached onReached_ = DoNothingOnReached)
      : run_ms(run_ms_),
        servo(servo_), step(step_),
        target(target_),
        onReached(std::move(onReached_)) {
    }

    bool operator<(const ServoTask s) const {
      return this->run_ms > s.run_ms;
    }
};

void initServoTaskManager();

void addServoTask(PCA9685Servo *servo_,
                  int64_t after_ms_,
                  int16_t step_,
                  uint16_t target_,
                  const OnReached &onReached_ = DoNothingOnReached);

#endif //SERVO_TASK_MANAGER_H
