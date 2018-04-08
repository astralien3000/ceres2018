#include "differential.h"

int differential_init(differential_t * diff, const differential_cfg_t * config) {
  if(!config->left_motor_set || !config->right_motor_set) {
    return -1;
  }

  diff->config = *config;
  diff->linear = 0;
  diff->angular = 0;

  return 0;
}

static inline _update(differential_t * diff) {
  diff->config.left_motor_set(diff->config.left_motor, (int)(diff->linear + diff->angular));
  diff->config.right_motor_set(diff->config.right_motor, (int)(diff->linear - diff->angular));
}

void differential_set_linear(differential_t * diff, float cmd) {
  diff->linear = cmd;
  _update(diff);
}

void differential_set_angular(differential_t * diff, float cmd) {
  diff->angular = cmd;
  _update(diff);
}
