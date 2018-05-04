#ifndef ODOMETER_H
#define ODOMETER_H

#include "encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float wheels_distance;
} odometer_cfg_t;

typedef struct {
  odometer_cfg_t config;
  encoder_t * encoder_left;
  encoder_t * encoder_right;
  float distance_offset;
  float angle_offset;
} odometer_t;

int odometer_init(odometer_t * odo, encoder_t * encoder_left, encoder_t * encoder_right, const odometer_cfg_t * config);

void odometer_reset(odometer_t * odo, float dist, float angle);

float odometer_read_distance(odometer_t * odo);

float odometer_read_linear_speed(odometer_t * odo);

float odometer_read_angle(odometer_t * odo);

float odometer_read_angular_speed(odometer_t * odo);

#ifdef __cplusplus
}
#endif

#endif//ODOMETER_H
