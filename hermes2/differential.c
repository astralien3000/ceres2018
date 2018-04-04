#include "differential.h"

int differential_init(differential_t * diff, motor_t * motor_left, motor_t * motor_right) {
  diff->motor_left = motor_left;
  diff->motor_right = motor_right;
  diff->speed = 0;
  diff->angular = 0;
}

static inline _update(differential_t * diff) {
  motor_set(diff->motor_left, (int)(diff->speed - diff->angular));
  motor_set(diff->motor_right, (int)(diff->speed + diff->angular));
}

void differential_set_speed(differential_t * diff, float cmd) {
  diff->speed = cmd;
  _update(diff);
}

void differential_set_angular(differential_t * diff, float cmd) {
  diff->angular = cmd;
  _update(diff);
}
