#ifndef ENCODER_H
#define ENCODER_H

#include <math.h>

#include "quad_decode.hpp"

#include "scheduler.hpp"

class Encoder {
private:
  static inline QuadDecode<1>& qdec1(void) {
    static QuadDecode<1> ret;
    return ret;
  }

  static inline QuadDecode<2>& qdec2(void) {
    static QuadDecode<2> ret;
    return ret;
  }

public:
  struct Config {
    uint8_t qdec;
    uint32_t ppr;
    float radius;
    bool invert;
    uint32_t freq;
  };

  Config config;

private:
  float dist_coef;
  float speed_coef;
  int32_t last;
  float speed;

public:
  void update(void) {
    int32_t cur = 0;
    if(config.qdec == 1) {
      cur = qdec1().calcPosn();
    }
    else if(config.qdec == 2) {
      cur = qdec2().calcPosn();
    }

    speed = (cur - last) * speed_coef;
    last = cur;
  }

  int init(void) {
    if(config.qdec == 1) {
      qdec1().setup();
      qdec1().start();
    }
    else if(config.qdec == 2) {
      qdec2().setup();
      qdec2().start();
    }
    else {
      return -1;
    }

    dist_coef = (M_PI * 2 * config.radius) / config.ppr;
    dist_coef *= (config.invert ? -1 : 1);
    speed_coef = config.freq * dist_coef;
    last = 0;

    Scheduler::instance().add(config.freq, this);

    return 0;
  }

  void reset(void) {
    last = 0;
    if(config.qdec == 1) {
      qdec1().zeroFTM();
    }
    else if(config.qdec == 2) {
      qdec2().zeroFTM();
    }
  }

  float getDistance(void) {
    if(config.qdec == 1) {
      return dist_coef * qdec1().calcPosn();
    }
    else if(config.qdec == 2) {
      return dist_coef * qdec2().calcPosn();
    }
    else {
      return 0;
    }
  }

  float getSpeed(void) {
    return speed;
  }

public:
  struct Distance;
  Distance& getDistanceSensor(void) {
    return *(Distance*)this;
  }

  struct Speed;
  Speed& getSpeedSensor(void) {
    return *(Speed*)this;
  }
};

struct Encoder::Distance : public Encoder {
  float get(void) {
    return Encoder::getDistance();
  }
};

struct Encoder::Speed : public Encoder {
  float get(void) {
    return Encoder::getSpeed();
  }
};
#endif//ENCODER_H
