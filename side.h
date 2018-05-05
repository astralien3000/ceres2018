#ifndef SIDE_H
#define SIDE_H

#include <Arduino.h>

struct Side {

  enum Color {
    GREEN, ORANGE
  };

  static int init(void) {
    pinMode(5, INPUT);
  }

  static Color get(void) {
    if(digitalRead(5)) {
      return GREEN;
    }
    return ORANGE;
  }

};

#endif//SIDE_H
