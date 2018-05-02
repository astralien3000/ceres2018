#ifndef POSITIONING_ACTION_H
#define POSITIONING_ACTION_H

#include "trajectory_manager.h"

typedef enum {
  START,
  RUN,
  STOP,
  FAIL,
  FINISH,
} action_state_t;

typedef struct {
  float x;
  float y;
} xy_pos_t;

typedef struct {
  xy_pos_t pos;
  xy_pos_t dir;
} positioning_action_cfg_t;

typedef struct {
  positioning_action_cfg_t config;
  action_state_t state;
  int internal;
} positioning_action_t;

int positioning_action_init(positioning_action_t * act, const positioning_action_cfg_t * config);

void positioning_action_update(positioning_action_t * act);

#endif//POSITIONING_ACTION_H
