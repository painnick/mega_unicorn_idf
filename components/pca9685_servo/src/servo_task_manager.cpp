#include "servo_task_manager.h"

#include "esp_timer.h"

#include <queue>

#define TIMER_PERIOD_NS (20 * 1000) // 20ms

static auto TAG = "STSKMNG";

std::priority_queue<ServoTask> servo_task_queue;

esp_timer_handle_t servo_tasks_periodic_timer;

static void servo_tasks_periodic_timer_callback(void *arg);

void initServoTaskManager() {
  constexpr esp_timer_create_args_t periodic_timer_args = {
    .callback = &servo_tasks_periodic_timer_callback,
    /* name is optional, but may help identify the timer when debugging */
    .name = "servo_task_periodic"
  };

  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &servo_tasks_periodic_timer));
  /* The timer has been created but is not running yet */

  ESP_ERROR_CHECK(esp_timer_start_periodic(servo_tasks_periodic_timer, TIMER_PERIOD_NS));
}

void addServoTask(PCA9685Servo *servo_,
                  const int64_t after_ms_,
                  const int16_t step_,
                  const uint16_t target_,
                  const OnReached &onReached_) {
  const int64_t target_ms = (esp_timer_get_time() / 1000) + after_ms_;

  const ServoTask task(
    target_ms,
    servo_,
    step_,
    target_,
    onReached_);

  servo_task_queue.push(task);
}

static void servo_tasks_periodic_timer_callback(void *arg) {
  const int64_t time_since_boot_ms = esp_timer_get_time() / 1000;
  while (!servo_task_queue.empty() && servo_task_queue.top().run_ms < time_since_boot_ms) {
    const ServoTask task = servo_task_queue.top();

    const auto servo = task.servo;
    servo->target(task.target);
    servo->step(task.step);
    servo->onReached(task.onReached);

    servo_task_queue.pop();
  }
}
