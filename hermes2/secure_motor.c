#include "secure_motor.h"

#include <math.h>

#define ENABLE_DEBUG 0
#include <debug.h>

static bool _locked = false;

static void _update(void * arg) {
  secure_motor_t * smot = (secure_motor_t*)arg;
  const float spe = -encoder_read_speed(smot->encoder);

  if(smot->cmd * spe < 0) {
    smot->counter += 1.0/smot->config.freq;
  }
  else {
    if(fabs(smot->cmd) > smot->config.min_cmd) {
      if(fabs(spe) < smot->config.max_speed) {
        smot->counter += 1.0/smot->config.freq;
      }
      else {
        smot->counter = 0;
      }
    }
    else {
      smot->counter = 0;
    }
  }

  DEBUG("SECURE %f %f %f\n", smot->cmd, spe, smot->counter);

  if(smot->counter >= smot->config.duration) {
    _locked = true;
  }
}

int secure_motor_init(secure_motor_t * smot, scheduler_t * sched, motor_t * motor, encoder_t * encoder, const secure_motor_cfg_t * config) {
  smot->config = *config;
  smot->motor = motor;
  smot->encoder = encoder;
  smot->counter = 0;
  smot->cmd = 0;

  scheduler_add_task(sched, config->freq, _update, smot);
}

void secure_motor_set_locked(bool lock) {
  _locked = lock;
}

bool secure_motor_is_locked(void) {
  return _locked;
}

void secure_motor_set(secure_motor_t * smot, float cmd) {
  if(_locked) {
    motor_set(smot->motor, 0);
    smot->cmd = 0;
  }
  else {
    motor_set(smot->motor, cmd);
    smot->cmd = cmd;
  }
}
