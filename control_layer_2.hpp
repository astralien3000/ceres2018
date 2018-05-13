#ifndef CONTROL_LAYER_2_HPP
#define CONTROL_LAYER_2_HPP

#include "control_layer_1.hpp"

#include "odometer.hpp"
#include "differential.hpp"

#include "secure_linear.hpp"

class ControlLayer2 : public Singleton<ControlLayer2> {

  static constexpr float ROBOT_CS_FREQ = 100;

public:
  Odometer odo;
  Differential diff;

  PIDFilter pid_a;
  PIDFilter pid_d;

  SecureLinear speed;

  ControlSystem angle;
  ControlSystem dist;

  int init(void) {
    odo.encoder_left = &ControlLayer0::instance().enc_l;
    odo.encoder_right = &ControlLayer0::instance().enc_r;
    odo.config.wheels_distance = 11.16; //11
    odo.init();

    diff.config.max_angular = 20;
    diff.config.max_linear = 15;
    diff.setLeft(& ControlLayer1::instance().cmot_l);
    diff.setRight(& ControlLayer1::instance().cmot_r);
    diff.init();

    speed.config.freq = 10;
    speed.config.gp2_limit = 400;
    speed.config.max = 20;
    speed.diff = & diff;
    speed.init();

    pid_a.config.kp = 25;
    pid_a.config.ki = 0;
    pid_a.config.kd = 0;
    pid_a.config.freq = ROBOT_CS_FREQ;
    pid_a.init();

    angle.config.freq = ROBOT_CS_FREQ;
    angle.setSensor(& odo.getAngleSensor());
    angle.setActuator(& diff.getAngularActuator());
    angle.setErrorFilter(&pid_a);
    angle.init();

    pid_d.config.kp = 10;
    pid_d.config.ki = 0;
    pid_d.config.kd = 0;
    pid_d.config.freq = ROBOT_CS_FREQ;
    pid_d.init();

    dist.config.freq = ROBOT_CS_FREQ;
    dist.setSensor(& odo.getDistanceSensor());
    dist.setActuator(&speed);
    dist.setErrorFilter(&pid_d);
    dist.init();

    return 0;
  }
};

#endif // CONTROL_LAYER_2_HPP
