#ifndef SECURE_LINEAR_H
#define SECURE_LINEAR_H

#include "differential.hpp"
#include "gp2.hpp"
#include "scheduler.hpp"

class SecureLinear {
public:
  struct Config {
    uint32_t freq;
    uint32_t gp2_limit;
  };

  Config config;

  Differential * diff;

private:
  bool _detected_front;
  bool _detected_back;

public:
  void update(void) {
    if(GP2::get(GP2::FRONT_LEFT) > config.gp2_limit || GP2::get(GP2::FRONT_RIGHT) > config.gp2_limit) {
      _detected_front = true;
      Serial.println("LOCKED");
    }
    else {
      _detected_front = false;
    }

    if(GP2::get(GP2::BACK_LEFT) > config.gp2_limit || GP2::get(GP2::BACK_RIGHT) > config.gp2_limit) {
      _detected_back = true;
      Serial.println("LOCKED");
    }
    else {
      _detected_back = false;
    }
  }

public:
  int init(void) {
    _detected_front = false;
    _detected_front = false;

    Scheduler::instance().add(config.freq, this);

    return 0;
  }

  void set(float cmd) {
    if(cmd < 0 && _detected_back) {
      diff->setLinear(0);
    }
    else if(cmd > 0 && _detected_front) {
      diff->setLinear(0);
    }
    else {
      diff->setLinear(cmd);
    }
  }
};

#endif//SECURE_LINEAR_H
