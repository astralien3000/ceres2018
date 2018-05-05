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
  float x;
  float y;

public:
  void update(void) {
    const float angle = odo->getAngle();
    const float speed = odo->getLinearSpeed() / config.freq;

    x += cosf(angle) * speed;
    y += sinf(angle) * speed;
  }

  int init(void) {
    x = 0;
    y = 0;

    Scheduler::instance().add(config.freq, this);

    return 0;
  }

  void resetPos(float x, float y) {
    x = x;
    y = y;
  }

  void reset(float x, float y, float angle) {
    odo->reset(0, angle);
    resetPos(x, y);
  }

  float getX() {
    return x;
  }

  float getY() {
    return y;
  }

  float getAngle() {
    return odo->getAngle();
  }

};

#endif//LOCATOR_H
