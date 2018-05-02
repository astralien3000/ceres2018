#ifndef SECURE_MOTOR_H
#define SECURE_MOTOR_H

#include "encoder.h"
#include "motor.h"
#include "scheduler.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float duration;
  float freq;
  float min_cmd;
  float max_speed;
} secure_motor_cfg_t;

typedef struct {
  secure_motor_cfg_t config;
  motor_t * motor;
  encoder_t * encoder;
  float counter;
  float cmd;
} secure_motor_t;

int secure_motor_init(secure_motor_t * smot, scheduler_t * sched, motor_t * motor, encoder_t * encoder, const secure_motor_cfg_t * config);

void secure_motor_set_locked(secure_motor_t * odo, bool lock);

bool secure_motor_is_locked(secure_motor_t * odo);

void secure_motor_set(secure_motor_t * smot, float cmd);

#ifdef __cplusplus
}
#endif

#endif//SECURE_MOTOR_H
