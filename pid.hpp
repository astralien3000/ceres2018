#ifndef PID_H
#define PID_H

#include <stdint.h>

class PIDFilter {
public:
  struct Config {
    float kp;
    float ki;
    float kd;
    uint32_t freq;
  };

  Config config;

private:
  float _last;
  float _sum;
  float _sum_coeff;

public:
  int init(void) {
    _last = 0;
    _sum = 0;
    _sum_coeff = 3.0/4.0;

    return 0;
  }

  void reset(void) {
    _last = 0;
    _sum = 0;
  }

  float eval(float in) {
    const float diff = (in - _last) * config.freq;
    const float sum = _sum + (in / config.freq);


    const float out =
        config.kp * in +
        config.ki * sum +
        config.kd * diff;

    _sum = sum * _sum_coeff;
    _last = in;

    //DEBUG("pid : %f %f\n", in, ret);

    return out;
  }
};

#endif//PID_H
