#ifndef ENCODER_H
#define ENCODER_H

#include "periph/qdec.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  qdec_t dev;
  qdec_mode_t mode;
  uint32_t ppr;
  float radius;
} encoder_cfg_t;

typedef struct {
  int value;
  uint32_t date;
} encoder_measure_t;

typedef struct {
  qdec_t dev;
  float coef;
} encoder_t;

int encoder_init(encoder_t * enc, const encoder_cfg_t * config);

void encoder_reset(encoder_t * enc);

float encoder_read(encoder_t * enc);

#ifdef __cplusplus
}
#endif

#endif//ENCODER_H
