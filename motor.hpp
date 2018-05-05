#ifndef MOTOR_H
#define MOTOR_H

#include <math.h>

#include <wiring.h>

class Motor {
public:
  static constexpr float FREQ = 30000.f;
  static constexpr uint32_t RESOLUTION = 16;
  static constexpr uint32_t MAX = 1 << RESOLUTION;

public:
  struct Config {
    uint8_t pwm_pin;
    uint8_t pos_pin;
    uint8_t neg_pin;
    uint32_t max;
  };

  Config config;

public:
  Motor(void) {}

public:
  void init(void) {
    pinMode(config.pwm_pin, OUTPUT);
    analogWriteFrequency(config.pwm_pin, FREQ);
    analogWriteResolution(RESOLUTION);
    pinMode(config.pos_pin, OUTPUT);
    pinMode(config.neg_pin, OUTPUT);
  }

  void set(float cmd) {
    int acmd = fabs(cmd) * config.max;

    if((uint32_t)acmd > config.max) {
      acmd = config.max;
    }

    analogWrite(config.pwm_pin, acmd);

    digitalWrite(cmd > 0 ? config.pos_pin : config.neg_pin, HIGH);
    digitalWrite(cmd > 0 ? config.neg_pin : config.pos_pin, LOW);
  }
};

#endif//MOTOR_H
