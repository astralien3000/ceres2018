#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>
#include <stdlib.h>

#include <IntervalTimer.h>

#include "singleton.hpp"

#include <Arduino.h>

class Scheduler : public Singleton<Scheduler> {
public:
  typedef void (*callback_t)(void *arg);

  struct Task {
    uint32_t period;
    callback_t cb;
    void * arg;
    uint32_t counter;
  };

private:
  static constexpr size_t MAX_TASKS = 16;

private:
  Task tasks[MAX_TASKS];
  uint32_t period;
  IntervalTimer timer;

  friend class Singleton<Scheduler>;
  Scheduler(void) {
    period = 1000000;
    timer.begin(_update, period);
  }

  static void _update(void) {
    for(size_t i = 0 ; i < MAX_TASKS ; i++) {
      if(instance().tasks[i].cb) {
        instance().tasks[i].counter++;
        const uint32_t wait_duration = instance().tasks[i].counter * instance().period;

        if(wait_duration >= instance().tasks[i].period) {
          //Serial.println((uint32_t)instance().tasks[i].period);
          instance().tasks[i].cb(instance().tasks[i].arg);
          instance().tasks[i].counter = 0;
        }
      }
    }
    //Serial.println("end");
  }

  static uint32_t _gcd(uint32_t a, uint32_t b) {
    while (b != 0) {
      a %= b;
      a ^= b;
      b ^= a;
      a ^= b;
    }
    return a;
  }

public:
  int add(uint32_t freq, callback_t cb, void * arg) {
    for(size_t i = 0 ; i < MAX_TASKS ; i++) {
      if(!tasks[i].cb) {
        tasks[i].arg = arg;
        tasks[i].period = 1000000 / freq;
        tasks[i].counter = 0;
        period = _gcd(period, tasks[i].period);
        timer.update(period);
        tasks[i].cb = cb;
        return i;
      }
    }

    return -1;
  }

  template<class Updatable>
  int add(uint32_t freq, Updatable* obj) {
    return add(freq, [](void* arg) { ((Updatable*)arg)->update(); }, (void*)obj);
  }
};

#endif//SCHEDULER_H
