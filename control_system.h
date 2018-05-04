#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include <stdint.h>

#include "scheduler.h"

typedef float (*control_system_eval_t)(void*, float);
typedef float (*control_system_read_t)(void*);
typedef void (*control_system_set_t)(void*, float);

typedef struct {
  uint32_t freq;

  void * feedback_filter;
  control_system_eval_t feedback_filter_eval;

  void * command_filter;
  control_system_eval_t command_filter_eval;

  void * error_filter;
  control_system_eval_t error_filter_eval;

  void * sensor;
  control_system_read_t sensor_read;

  void * actuator;
  control_system_set_t actuator_set;
} control_system_cfg_t;

typedef struct {
  control_system_cfg_t config;
  float cmd;
} control_system_t;

int control_system_init(control_system_t * cs, scheduler_t * sched, const control_system_cfg_t * config);

void control_system_set(control_system_t * cs, float cmd);

#endif//CONTROL_SYSTEM_H
