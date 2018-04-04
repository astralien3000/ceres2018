#ifndef ENCODER_H
#define ENCODER_H

#include "periph/qdec.h"

#include "scheduler.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  qdec_t dev;
  qdec_mode_t mode;
  uint32_t ppr;
  float radius;
  bool invert;
  uint32_t freq;
} encoder_cfg_t;

typedef struct {
  qdec_t dev;
  float dist_coef;
  float speed_coef;
  int32_t last;
  float speed;
} encoder_t;

int encoder_init(encoder_t * enc, scheduler_t * sched, const encoder_cfg_t * config);

void encoder_reset(encoder_t * enc);

float encoder_read_distance(encoder_t * enc);

float encoder_read_speed(encoder_t * enc);

#ifdef __cplusplus
}
#endif

#endif//ENCODER_H
