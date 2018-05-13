#ifndef CONTROL_LAYER_1_HPP
#define CONTROL_LAYER_1_HPP

#include "control_layer_0.hpp"

#include "odometer.hpp"
#include "pid.hpp"
#include "control_system.h"

class ControlLayer1 : public Singleton<ControlLayer1> {

  static constexpr float MOT_CS_FREQ = 400;

public:
  PIDFilter pid_l;
  PIDFilter pid_r;

  ControlSystem cmot_l;
  ControlSystem cmot_r;

  int init(void) {
    pid_l.config.kp = 0.04;
    pid_l.config.ki = 0.00;
    pid_l.config.kd = 0.00;
    pid_l.config.freq = MOT_CS_FREQ;
    pid_l.init();

    pid_r.config.kp = 0.04;
    pid_r.config.ki = 0.00;
    pid_r.config.kd = 0.00;
    pid_r.config.freq = MOT_CS_FREQ;
    pid_r.init();

    cmot_l.config.freq = MOT_CS_FREQ;
    cmot_l.setActuator(&ControlLayer0::instance().smot_l);
    cmot_l.setSensor(&ControlLayer0::instance().enc_l.getSpeedSensor());
    cmot_l.setErrorFilter(&pid_l);
    cmot_l.init();

    cmot_r.config.freq = MOT_CS_FREQ;
    cmot_r.setActuator(&ControlLayer0::instance().smot_r);
    cmot_r.setSensor(&ControlLayer0::instance().enc_r.getSpeedSensor());
    cmot_r.setErrorFilter(&pid_r);
    cmot_r.init();

    return 0;
  }
};

#endif // CONTROL_LAYER_1_HPP
