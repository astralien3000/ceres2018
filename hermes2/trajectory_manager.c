#include "trajectory_manager.h"

#define __BSD_VISIBLE 1
#include <math.h>

#define ENABLE_DEBUG 0
#include <debug.h>

static void _update(void * arg) {
  trajectory_manager_t * traj = (trajectory_manager_t*)arg;

  float dx = traj->cmd_x - locator_read_x(traj->config.loc);
  float dy = traj->cmd_y - locator_read_y(traj->config.loc);

  if(fabs(dx) < 0.5 && fabs(dy) < 0.5) {
    traj->config.linear_speed_set(traj->config.linear_speed, 0);
    return;
  }

  float cmd_a = atan2f(dy, dx);
  float a = locator_read_angle(traj->config.loc);
  float amod = fmod(a, 2 * M_PI);
  if(amod < -M_PI) amod += 2 * M_PI;
  if(amod > M_PI)  amod -= 2 * M_PI;
  float adiv = a - amod;
  float speed_mul = traj->config.speed;
  DEBUG("(%f;%f)=>(%f;%f;%f)\n",a,amod, cmd_a-M_PI, cmd_a, cmd_a+M_PI);

  if(amod < cmd_a) {
    if(fabs(cmd_a-amod) > fabs(cmd_a - M_PI - amod)) {
      cmd_a -= M_PI;
      speed_mul *= -1;
    }
  }
  else {
    if(fabs(cmd_a-amod) > fabs(cmd_a + M_PI - amod)) {
      cmd_a += M_PI;
      speed_mul *= -1;
    }
  }
  DEBUG("%f\n", cmd_a);

  if(fabs(cmd_a-amod) > 4*M_PI/180.0) {
    speed_mul = 0;
  }

  traj->config.angle_set(traj->config.angle, cmd_a+adiv);
  traj->config.linear_speed_set(traj->config.linear_speed, speed_mul);
}

int trajectory_manager_init(trajectory_manager_t * traj, scheduler_t * sched, const trajectory_manager_cfg_t * config) {
  traj->config = *config;

  scheduler_add_task(sched, config->freq, _update, traj);
}

void trajectory_manager_goto(trajectory_manager_t * traj, float x, float y) {
  traj->cmd_x = x;
  traj->cmd_y = y;
}
