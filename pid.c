#include "pid.h"


#define ENABLE_DEBUG 0
#include "debug.h"

int pid_init(pid_filter_t * pid, const pid_cfg_t * config) {
  pid->config = *config;
  pid->last = 0;
  pid->sum = 0;
  pid->sum_coeff = 3.0/4.0;

  return 0;
}

void pid_reset(pid_filter_t * pid) {
  pid->last = 0;
  pid->sum = 0;
}


float pid_eval(pid_filter_t * pid, float val) {
  const float diff = (val - pid->last) * pid->config.freq;
  const float sum = pid->sum + (val / pid->config.freq);


  const float ret =
      pid->config.kp * val +
      pid->config.ki * sum +
      pid->config.kd * diff;

  pid->sum = sum * pid->sum_coeff;
  pid->last = val;

  DEBUG("pid : %f %f\n", val, ret);

  return ret;
}
