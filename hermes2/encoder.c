#include "encoder.h"

#include <periph/qdec.h>

#define __BSD_VISIBLE 1
#include <math.h>

int encoder_init(encoder_t * enc, const encoder_cfg_t * config) {
  qdec_init(config->dev, config->mode, NULL, NULL);

  enc->dev = config->dev;
  enc->coef = (M_PI * 2 * config->radius) / config->ppr;

  return 0;
}

void encoder_reset(encoder_t * enc) {
  qdec_read_and_reset(enc->dev);
}

float encoder_read(encoder_t * enc) {
  return enc->coef * qdec_read(enc->dev);
}
