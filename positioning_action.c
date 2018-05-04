#include "positioning_action.h"

#include "dev.h"

#define ENABLE_DEBUG 1
#include <debug.h>

#define ROBOT_BORDER_LENGTH (18.2/2.0)

int positioning_action_init(positioning_action_t * act, const positioning_action_cfg_t * config) {
  act->config = *config;
  act->internal = 0;
  act->state = START;
}

void positioning_action_update(positioning_action_t * act) {
  if(act->state == START) {
    trajectory_manager_goto(&traj, act->config.pos.x, act->config.pos.y);
    if(trajectory_manager_is_arrived(&traj)) {
      act->state = RUN;
      act->internal = 0;
    }
  }
  else if(act->state == RUN) {
    if(act->internal == 0) {
      if(act->config.dir.x == 0) {
        act->internal = 2;
      }
      trajectory_manager_goto(&traj, act->config.pos.x + act->config.dir.x * 2, act->config.pos.y);
      if(secure_motor_is_locked()) {
        act->internal = 1;
        locator_reset_pos(&loc, act->config.pos.x + act->config.dir.x, locator_read_y(&loc));
      }
    }
    else if(act->internal == 1) {
      trajectory_manager_goto(&traj, act->config.pos.x, act->config.pos.y);
      if(trajectory_manager_is_arrived(&traj)) {
        act->internal = 2;
      }
      else if(secure_motor_is_locked()) {
        secure_motor_set_locked(false);
      }
    }
    else if(act->internal == 2) {
      if(act->config.dir.y == 0) {
        act->internal = 4;
      }
      trajectory_manager_goto(&traj, act->config.pos.x, act->config.pos.y + act->config.dir.y * 2);
      if(secure_motor_is_locked()) {
        act->internal = 3;
        locator_reset_pos(&loc, locator_read_x(&loc), act->config.pos.y + act->config.dir.y);
      }
    }
    else if(act->internal == 3) {
      trajectory_manager_goto(&traj, act->config.pos.x, act->config.pos.y);
      if(trajectory_manager_is_arrived(&traj)) {
        act->internal = 4;
      }
      else if(secure_motor_is_locked()) {
        secure_motor_set_locked(false);
      }
    }
  }
}
