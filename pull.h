#ifndef PULL_H
#define PULL_H

#include <periph/gpio.h>

#define PULL_PIN GPIO_PIN(PORT_D, 0)

int pull_init(void) {
  gpio_init(PULL_PIN, GPIO_IN);
}

bool pull_is_present(void) {
  return !gpio_read(PULL_PIN);
}

#endif//PULL_H
