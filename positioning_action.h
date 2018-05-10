#ifndef POSITIONING_ACTION_H
#define POSITIONING_ACTION_H

#include "control_layer_3.hpp"

class PositioningAction {
public:
  static constexpr float ROBOT_BORDER_LENGTH = 18.2/2.0;

  typedef enum {
    START,
    RUN,
    STOP,
    FAIL,
    FINISH,
  } State;

  typedef struct {
    float x;
    float y;
  } Pos;

  typedef struct {
    Pos pos;
    Pos dir;
  } Config;

  Config config;
private:
  State state;
  int internal;

public:
  State getState() { return state; }

public:
  int init(void) {
    internal = 0;
    state = START;
    return 0;
  }

  void update(void) {
    if(state == START) {
      ControlLayer3::instance().traj.gotoXYA(config.pos.x, config.pos.y, 0);
      if(ControlLayer3::instance().traj.isArrived()) {
        state = RUN;
        internal = 0;
        ControlLayer2::instance().speed.disableDetection();
      }
    }
    else if(state == RUN) {
      if(internal == 0) {
        if(config.dir.x == 0) {
          internal = 2;
        }
        ControlLayer3::instance().traj.gotoXYA(config.pos.x + config.dir.x * 2, config.pos.y, 0);
        if(SecureMotor::locked()) {
          internal = 1;
          ControlLayer3::instance().loc.reset(config.pos.x + config.dir.x, ControlLayer3::instance().loc.getY(), 0);
        }
      }
      else if(internal == 1) {
        ControlLayer3::instance().traj.gotoXYA(config.pos.x, config.pos.y, M_PI/2);
        if(ControlLayer3::instance().traj.isArrived()) {
          internal = 2;
        }
        else if(SecureMotor::locked()) {
          SecureMotor::locked() = false;
        }
      }
      else if(internal == 2) {
        if(config.dir.y == 0) {
          internal = 4;
        }
        ControlLayer3::instance().traj.gotoXYA(config.pos.x, config.pos.y + config.dir.y * 2, M_PI/2);
        if(SecureMotor::locked()) {
          internal = 3;
          ControlLayer3::instance().loc.resetPos(ControlLayer3::instance().loc.getX(), config.pos.y + config.dir.y);
        }
      }
      else if(internal == 3) {
        ControlLayer3::instance().traj.gotoXYA(config.pos.x, config.pos.y, M_PI/2);
        if(ControlLayer3::instance().traj.isArrived()) {
          //internal = 4;
          state = FINISH;
          ControlLayer2::instance().speed.enableDetection();
        }
        else if(SecureMotor::locked()) {
          SecureMotor::locked() = false;
        }
      }
    }
  }

};

#endif//POSITIONING_ACTION_H
