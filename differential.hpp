#ifndef DIFFERENTIAL_H
#define DIFFERENTIAL_H

#include "motor.hpp"

class Differential {
public:
  struct Config {
    float max_angular;
    float max_linear;
  };

  Config config;

private:
  typedef void (*set_t)(void*,float);

  void * left_motor;
  set_t left_motor_set;
  void * right_motor;
  set_t right_motor_set;

  float linear;
  float angular;

public:
  template<class Motor>
  void setLeft(Motor * dev) {
    left_motor = (void*)dev;
    left_motor_set = [](void * arg, float val) { ((Motor*)arg)->set(val); };
  }

  template<class Motor>
  void setRight(Motor * dev) {
    right_motor = (void*)dev;
    right_motor_set = [](void * arg, float val) { ((Motor*)arg)->set(val); };
  }

  int init(void) {
    if(!left_motor_set || !right_motor_set) {
      return -1;
    }

    linear = 0;
    angular = 0;

    return 0;
  }

  void update(void) {
    left_motor_set(left_motor, (int)(linear - angular));
    right_motor_set(right_motor, (int)(linear + angular));
  }

  void setLinear(float cmd) {
    if(fabs(cmd) < config.max_linear) {
      linear = cmd;
    }
    else if(cmd > 0) {
      linear = config.max_linear;
    }
    else {
      linear = -config.max_linear;
    }
    update();
  }

  void setAngular(float cmd) {
    if(fabs(cmd) < config.max_angular) {
      angular = cmd;
    }
    else if(cmd > 0) {
      angular = config.max_angular;
    }
    else {
      angular = -config.max_angular;
    }
    update();
  }

  class Linear;
  Linear& getLinearActuator(void) {
    return *(Linear*)this;
  }

  class Angular;
  Angular& getAngularActuator(void) {
    return *(Angular*)this;
  }
};

struct Differential::Linear : public Differential {
  void set(float val) {
    Differential::setLinear(val);
  }
};

struct Differential::Angular : public Differential {
  void set(float val) {
    Differential::setAngular(val);
  }
};

#endif//DIFFERENTIAL_H
