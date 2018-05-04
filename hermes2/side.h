#ifndef SIDE_H
#define SIDE_H

#include <periph/gpio.h>

#define SIDE_PIN GPIO_PIN(PORT_D, 7)

typedef enum {
  GREEN, ORANGE
} side_t;

int side_init(void) {
  gpio_init(SIDE_PIN, GPIO_IN);
}

side_t side_read(void) {
  if(gpio_read(SIDE_PIN)) {
    return GREEN;
  }
  return ORANGE;
}

#endif//SIDE_H
