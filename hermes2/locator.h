#ifndef LOCATOR_H
#define LOCATOR_H

#include "odometer.h"
#include "scheduler.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t freq;
} locator_cfg_t;

typedef struct {
  locator_cfg_t config;
  odometer_t * odo;
  float x;
  float y;
} locator_t;

int locator_init(locator_t * loc, scheduler_t * sched, odometer_t * odo, const locator_cfg_t * config);

void locator_reset(locator_t * loc, float x, float y, float angle);

float locator_read_x(locator_t * loc);
float locator_read_y(locator_t * loc);
float locator_read_angle(locator_t * loc);

#ifdef __cplusplus
}
#endif

#endif//LOCATOR_H
