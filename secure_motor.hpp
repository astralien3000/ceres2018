#ifndef SECURE_MOTOR_H
#define SECURE_MOTOR_H

#include "encoder.hpp"
#include "motor.hpp"
#include "scheduler.hpp"

class SecureMotor {
public:
  static bool& locked(void) {
    static bool ret = false;
    return ret;
  }

public:
  struct Config {
    float duration;
    float freq;
    float min_cmd;
    float max_speed;
  };

  Config config;

  Motor * motor;
  Encoder * encoder;

private:
    float _counter;
    float _cmd;

public:
    void update(void) {
      const float spe = - encoder->getSpeed();

      if(_cmd * spe < 0) {
        _counter += 1.0/config.freq;
      }
      else {
        if(fabs(_cmd) > config.min_cmd) {
          if(fabs(spe) < config.max_speed) {
            _counter += 1.0/config.freq;
          }
          else {
            _counter = 0;
          }
        }
        else {
          _counter = 0;
        }
      }

      /*
      Serial.print(_cmd);
      Serial.print(" ");
      Serial.print(spe);
      Serial.print(" ");
      Serial.print(_counter);
      Serial.println("");
      */

      if(_counter >= config.duration) {
        locked() = true;
      }
    }

public:
    int init(void) {
      _counter = 0;
      _cmd = 0;

      Scheduler::instance().add(config.freq, this);

      return 0;
    }

    void set(float cmd) {
      if(locked()) {
        motor->set(0);
        cmd = 0;
      }
      else {
        motor->set(cmd);
        _cmd = cmd;
      }
    }
};

#endif//SECURE_MOTOR_H
