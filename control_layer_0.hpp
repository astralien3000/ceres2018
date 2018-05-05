#ifndef CONTROL_LAYER_0_H
#define CONTROL_LAYER_0_H

#include "singleton.hpp"

#include "motor.hpp"
#include "encoder.hpp"

#include "secure_motor.hpp"

class ControlLayer0 : public Singleton<ControlLayer0> {

public:
  static constexpr uint32_t MOT_MAX = Motor::MAX / 4;
  static constexpr uint32_t ENC_FREQ = 400;
  static constexpr uint32_t SMOT_FREQ = 20;

  Motor mot_l;
  Motor mot_r;

  Encoder enc_l;
  Encoder enc_r;

  SecureMotor smot_l;
  SecureMotor smot_r;

  int init(void) {
    mot_l.config.max = MOT_MAX;
    mot_l.config.pwm_pin = 22;
    mot_l.config.pos_pin = 24;
    mot_l.config.neg_pin = 33;
    mot_l.init();

    mot_r.config.max = MOT_MAX;
    mot_r.config.pwm_pin = 23;
    mot_r.config.pos_pin = 31;
    mot_r.config.neg_pin = 26;
    mot_r.init();

    enc_l.config.qdec = 1;
    enc_l.config.freq = ENC_FREQ;
    enc_l.config.invert = false;
    enc_l.config.ppr = 4096;
    enc_l.config.radius = 3;
    enc_l.init();

    enc_r.config.qdec = 2;
    enc_r.config.freq = ENC_FREQ;
    enc_r.config.invert = true;
    enc_r.config.ppr = 4096;
    enc_r.config.radius = 3;
    enc_r.init();

    smot_l.motor = &mot_l;
    smot_l.encoder = &enc_l;
    smot_l.config.duration = 0.5;
    smot_l.config.freq = SMOT_FREQ;
    smot_l.config.max_speed = 1;
    smot_l.config.min_cmd = 0.2;
    smot_l.init();

    smot_r.motor = &mot_r;
    smot_r.encoder = &enc_r;
    smot_r.config.duration = 0.5;
    smot_r.config.freq = SMOT_FREQ;
    smot_r.config.max_speed = 1;
    smot_r.config.min_cmd = 0.2;
    smot_r.init();

    return 0;
  }

};

#endif//CONTROL_LAYER_0_H
