#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include "locator.h"
#include "scheduler.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*trajectory_manager_set_t)(void*, float);

typedef struct {
  locator_t * loc;

  void * linear_speed;
  trajectory_manager_set_t linear_speed_set;

  void * angle;
  trajectory_manager_set_t angle_set;

  uint32_t freq;
  float speed;
} trajectory_manager_cfg_t;

typedef struct {
  trajectory_manager_cfg_t config;
  float cmd_x;
  float cmd_y;
  bool arrived;
} trajectory_manager_t;

int trajectory_manager_init(trajectory_manager_t * traj, scheduler_t * sched, const trajectory_manager_cfg_t * config);

void trajectory_manager_goto(trajectory_manager_t * traj, float x, float y);

bool trajectory_manager_is_arrived(trajectory_manager_t * traj);

#ifdef __cplusplus
}
#endif

#endif//TRAJECTORY_MANAGER_H
