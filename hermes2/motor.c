#include "motor.h"

#include <math.h>

int motor_init(motor_t * motor, const motor_pwm_dev_cfg_t * dev_config, const motor_cfg_t * config) {
  if(dev_config->res < config->pwm.max) {
    return -1;
  }

  motor->dev = dev_config->dev;
  motor->config = *config;

  pwm_init(dev_config->dev, PWM_LEFT, dev_config->freq, dev_config->res);

  gpio_init(config->dir.pos_pin, GPIO_OUT);
  gpio_init(config->dir.neg_pin, GPIO_OUT);

  return 0;
}

void motor_set_int(motor_t * motor, int cmd) {
  int acmd = abs(cmd);

  if(acmd > motor->config.pwm.max) {
    acmd = motor->config.pwm.max;
  }

  pwm_set(motor->dev, motor->config.pwm.chan, acmd);

  gpio_set(cmd > 0 ? motor->config.dir.pos_pin : motor->config.dir.neg_pin);
  gpio_clear(cmd > 0 ? motor->config.dir.neg_pin : motor->config.dir.pos_pin);
}

void motor_set(motor_t * motor, float cmd) {
  motor_set_int(motor, (int)cmd);
}
