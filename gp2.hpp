#ifndef GP2_HPP
#define GP2_HPP

#include <Arduino.h>

class GP2 {
public:
  enum Id {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT,

    MAX_GP2
  };

private:
  static uint8_t* pins(void) {
    static uint8_t pins[MAX_GP2] = {
      A19, A18, A16, A7
    };
    return pins;
  }

public:
  static int init(void) {
    for(int i = 0 ; i < MAX_GP2 ; i++) {
      pinMode(pins()[i], INPUT);
    }
    return 0;
  }

  static uint32_t get(Id id) {
    return analogRead(pins()[id]);
  }
};

#endif//GP2_HPP
