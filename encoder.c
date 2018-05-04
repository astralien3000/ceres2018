#include "encoder.h"

#include <periph/qdec.h>

#define __BSD_VISIBLE 1
#include <math.h>

static void _update(void * arg) {
  encoder_t * enc = (encoder_t *)arg;
  const int32_t cur = qdec_read(enc->dev);
  enc->speed = (cur - enc->last) * enc->speed_coef;
  enc->last = cur;
}

int encoder_init(encoder_t * enc, scheduler_t *sched, const encoder_cfg_t * config) {
  qdec_init(config->dev, config->mode, NULL, NULL);

  enc->dev = config->dev;
  enc->dist_coef = (M_PI * 2 * config->radius) / config->ppr;
  enc->dist_coef *= (config->invert ? -1 : 1);
  enc->speed_coef = config->freq * enc->dist_coef;
  enc->last = 0;

  scheduler_add_task(sched, config->freq, _update, enc);

  return 0;
}

void encoder_reset(encoder_t * enc) {
  enc->last = 0;
  qdec_read_and_reset(enc->dev);
}

float encoder_read_distance(encoder_t * enc) {
  return enc->dist_coef * qdec_read(enc->dev);
}

float encoder_read_speed(encoder_t * enc) {
  return enc->speed;
}
