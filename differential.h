#ifndef DIFFERENTIAL_H
#define DIFFERENTIAL_H

#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*differential_set_t)(void*,float);

typedef struct {
  void * left_motor;
  differential_set_t left_motor_set;
  void * right_motor;
  differential_set_t right_motor_set;
} differential_cfg_t;

typedef struct {
  differential_cfg_t config;
  float linear;
  float angular;
} differential_t;

int differential_init(differential_t * diff, const differential_cfg_t * config);

void differential_set_linear(differential_t * diff, float cmd);

void differential_set_angular(differential_t * diff, float cmd);

#ifdef __cplusplus
}
#endif

#endif//DIFFERENTIAL_H
