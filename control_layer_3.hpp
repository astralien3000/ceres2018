#ifndef CONTROL_LAYER_3_HPP
#define CONTROL_LAYER_3_HPP

#include "control_layer_2.hpp"

#include "locator.hpp"
#include "trajectory_manager.h"

class ControlLayer3 : public Singleton<ControlLayer3> {

public:
  Locator loc;
  TrajectoryManager traj;

  int init(void) {
    loc.config.freq = 100;
    loc.odo = & ControlLayer2::instance().odo;
    loc.init();

    traj.config.freq = 10;
    traj.config.speed = 20;
    traj.loc = & loc;
    traj.setAngle(& ControlLayer2::instance().angle);
    traj.setLinearSpeed(& ControlLayer2::instance().speed);
    traj.init();

    return 0;
  }
};

#endif // CONTROL_LAYER_3_HPP
