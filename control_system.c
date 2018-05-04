#include "control_system.h"

#define ENABLE_DEBUG 0
#include "debug.h"

static void _update(void * arg) {
  control_system_t * cs = (control_system_t*) arg;

  float feedback = cs->config.sensor_read(cs->config.sensor);
  float command = cs->cmd;

  if(cs->config.feedback_filter_eval) {
    feedback = cs->config.feedback_filter_eval(cs->config.feedback_filter, feedback);
  }

  if(cs->config.command_filter_eval) {
    command = cs->config.command_filter_eval(cs->config.command_filter, command);
  }

  float error = feedback - command;

  if(cs->config.error_filter_eval) {
    error = cs->config.error_filter_eval(cs->config.error_filter, error);
  }

  DEBUG("error : %f\n", error);
  cs->config.actuator_set(cs->config.actuator, error);
}

int control_system_init(control_system_t * cs, scheduler_t * sched, const control_system_cfg_t * config) {
  cs->cmd = 0;
  cs->config = *config;

  scheduler_add_task(sched, config->freq, _update, cs);
}

void control_system_set(control_system_t * cs, float cmd) {
  cs->cmd = cmd;
}
