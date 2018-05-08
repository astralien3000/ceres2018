#ifndef LOCATOR_H
#define LOCATOR_H

#include "odometer.hpp"
#include "scheduler.hpp"

class Locator {

public:
  struct Config {
    uint32_t freq;
  };

  Config config;
  Odometer * odo;

private:
  float _x;
  float _y;

public:
  void update(void) {
    const float angle = odo->getAngle();
    const float speed = odo->getLinearSpeed() / config.freq;

    _x += cosf(angle) * speed;
    _y += sinf(angle) * speed;
  }

  int init(void) {
    _x = 0;
    _y = 0;

    Scheduler::instance().add(config.freq, this);

    return 0;
  }

  void resetPos(float x, float y) {
    _x = x;
    _y = y;
  }

  void reset(float x, float y, float angle) {
    odo->reset(0, angle);
    resetPos(x, y);
  }

  float getX() {
    return _x;
  }

  float getY() {
    return _y;
  }

  float getAngle() {
    return odo->getAngle();
  }

};

#endif//LOCATOR_H
