#ifndef DIFFERENTIAL_H
#define DIFFERENTIAL_H

#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  motor_t * motor_left;
  motor_t * motor_right;
  float speed;
  float angular;
} differential_t;

int differential_init(differential_t * diff, motor_t * motor_left, motor_t * motor_right);

void differential_set_speed(differential_t * diff, float cmd);

void differential_set_angular(differential_t * diff, float cmd);

#ifdef __cplusplus
}
#endif

#endif//DIFFERENTIAL_H
