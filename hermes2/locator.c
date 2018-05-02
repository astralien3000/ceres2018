#include "locator.h"

#define __BSD_VISIBLE 1
#include <math.h>

void _update(void * arg) {
  locator_t * loc = (locator_t *)arg;
  const float angle = odometer_read_angle(loc->odo);
  const float speed = odometer_read_linear_speed(loc->odo) / loc->config.freq;

  loc->x += cosf(angle) * speed;
  loc->y += sinf(angle) * speed;
}

int locator_init(locator_t * loc, scheduler_t * sched, odometer_t * odo, const locator_cfg_t * config) {
  loc->config = *config;
  loc->odo = odo;

  loc->x = 0;
  loc->y = 0;

  scheduler_add_task(sched, config->freq, _update, loc);

  return 0;
}

void locator_reset_pos(locator_t * loc, float x, float y) {
  loc->x = x;
  loc->y = y;
}

void locator_reset(locator_t * loc, float x, float y, float angle) {
  odometer_reset(loc->odo, 0, angle);
  locator_reset_pos(loc, x, y);
}

float locator_read_x(locator_t * loc) {
  return loc->x;
}

float locator_read_y(locator_t * loc) {
  return loc->y;
}

float locator_read_angle(locator_t * loc) {
  return odometer_read_angle(loc->odo);
}
