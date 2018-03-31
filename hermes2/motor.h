#ifndef MOTOR_H
#define MOTOR_H

#include <periph/pwm.h>
#include <periph/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  pwm_t dev;
  uint32_t freq;
  uint16_t res;
} motor_pwm_dev_cfg_t;

typedef struct {
  uint8_t chan;
  uint16_t max;
} motor_pwm_cfg_t;

typedef struct {
  gpio_t pos_pin;
  gpio_t neg_pin;
} motor_dir_cfg_t;

typedef struct {
  motor_pwm_cfg_t pwm;
  motor_dir_cfg_t dir;
} motor_cfg_t;

typedef struct {
  pwm_t dev;
  motor_cfg_t config;
} motor_t;

int motor_init(motor_t * motor, const motor_pwm_dev_cfg_t * dev_config, const motor_cfg_t * config);

void motor_set(motor_t * motor, int cmd);

#ifdef __cplusplus
}
#endif

#endif//MOTOR_H
