#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include "locator.hpp"
#include "scheduler.hpp"

#include <math.h>

class TrajectoryManager {
private:
  typedef void (*set_t)(void*, float);

public:
  enum Way {
    FORWARD, BACKWARD, NONE
  };

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
  float cmd_a;
  bool enable_cmd_a;
  bool arrived;
  Way way;

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
      way = NONE;
      linear_speed_set(linear_speed, 0);
      cmd_x = loc->getX();
      cmd_y = loc->getY();
      if(enable_cmd_a) {
        angle_set(angle, cmd_a);
      }
      return;
    }

    float cmd_a = atan2f(dy, dx);
    float a = loc->getAngle();
    float amod = fmod(a, 2 * M_PI);
    if(amod < -M_PI) amod += 2 * M_PI;
    if(amod > M_PI)  amod -= 2 * M_PI;
    float adiv = a - amod;
    float speed_mul = config.speed;
    way = FORWARD;
    //DEBUG("(%f;%f)=>(%f;%f;%f)\n",a,amod, cmd_a-M_PI, cmd_a, cmd_a+M_PI);

    if(amod < cmd_a) {
      if(fabs(cmd_a-amod) > fabs(cmd_a - M_PI - amod)) {
        cmd_a -= M_PI;
        speed_mul *= -1;
        way = BACKWARD;
      }
    }
    else {
      if(fabs(cmd_a-amod) > fabs(cmd_a + M_PI - amod)) {
        cmd_a += M_PI;
        speed_mul *= -1;
        way = BACKWARD;
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
    enable_cmd_a = false;

    Scheduler::instance().add(config.freq, this);

    return 0;
  }

  void gotoXY(float x, float y) {
    cmd_x = x;
    cmd_y = y;
    enable_cmd_a = false;
    arrived = false;
  }

  void gotoXYA(float x, float y, float a) {
    cmd_x = x;
    cmd_y = y;
    cmd_a = a;
    enable_cmd_a = true;
    arrived = false;
  }

  bool isArrived() {
    return arrived;
  }

  Way getWay() {
    return way;
  }
};

#endif//TRAJECTORY_MANAGER_H
