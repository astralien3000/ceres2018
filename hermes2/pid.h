#ifndef PID_H
#define PID_H

#include "stdint.h"

typedef struct {
  float kp;
  float ki;
  float kd;
  uint32_t freq;
} pid_cfg_t;

typedef struct {
  pid_cfg_t config;
  float last;
  float sum;
} pid_filter_t;

int pid_init(pid_filter_t * pid, const pid_cfg_t * config);

void pid_reset(pid_filter_t * pid);

float pid_eval(pid_filter_t * pid, float val);

#endif//PID_H
