#ifndef PULL_H
#define PULL_H

#include <Arduino.h>

struct Pull {
  static int init(void) {
    pinMode(2, INPUT);
    return 0;
  }

  static bool isPresent(void) {
    return !digitalRead(2);
  }
};

#endif//PULL_H
