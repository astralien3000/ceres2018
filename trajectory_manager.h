#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include "locator.hpp"
#include "scheduler.hpp"

#include <math.h>

class TrajectoryManager {
private:
  typedef void (*set_t)(void*, float);

public:
  struct Config {
    uint32_t freq;
    float speed;
  };

  Config config;

  Locator * loc;

private:
  void * linear_speed;
  set_t linear_speed_set;

  void * angle;
  set_t angle_set;

  float cmd_x;
  float cmd_y;
  bool arrived;

public:
  template<class LinearSpeed>
  void setLinearSpeed(LinearSpeed * dev) {
    linear_speed = (void*)dev;
    linear_speed_set = [](void * arg, float val) { ((LinearSpeed*)arg)->set(val); };
  }

  template<class Angle>
  void setAngle(Angle * dev) {
    angle = (void*)dev;
    angle_set = [](void * arg, float val) { ((Angle*)arg)->set(val); };
  }

  void update(void) {
    float dx = cmd_x - loc->getX();
    float dy = cmd_y - loc->getY();

    arrived = fabs(dx) < 0.5 && fabs(dy) < 0.5;
    if(arrived) {
      linear_speed_set(linear_speed, 0);
      return;
    }

    float cmd_a = atan2f(dy, dx);
    float a = loc->getAngle();
    float amod = fmod(a, 2 * M_PI);
    if(amod < -M_PI) amod += 2 * M_PI;
    if(amod > M_PI)  amod -= 2 * M_PI;
    float adiv = a - amod;
    float speed_mul = config.speed;
    //DEBUG("(%f;%f)=>(%f;%f;%f)\n",a,amod, cmd_a-M_PI, cmd_a, cmd_a+M_PI);

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
    //DEBUG("%f\n", cmd_a);

    if(fabs(cmd_a-amod) > 4*M_PI/180.0) {
      speed_mul = 0;
    }

    angle_set(angle, cmd_a+adiv);
    linear_speed_set(linear_speed, speed_mul);
  }

  int init() {
    cmd_x = loc->getX();
    cmd_y = loc->getY();
    arrived = false;

    Scheduler::instance().add(config.freq, this);

    return 0;
  }

  void gotoXY(float x, float y) {
    cmd_x = x;
    cmd_y = y;
  }

  bool isArrived() {
    return arrived;
  }
};

#endif//TRAJECTORY_MANAGER_H
