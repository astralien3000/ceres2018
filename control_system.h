#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include <stdint.h>

#include "scheduler.hpp"

#include "Arduino.h"

class ControlSystem {
private:
  typedef float (*eval_t)(void*, float);
  typedef float (*read_t)(void*);
  typedef void (*set_t)(void*, float);

public:
  struct Config {
    uint32_t freq;
  };

  Config config;

private:
  void * feedback_filter;
  eval_t feedback_filter_eval;

  void * setpoint_filter;
  eval_t setpoint_filter_eval;

  void * error_filter;
  eval_t error_filter_eval;

  void * sensor;
  read_t sensor_get;

  void * actuator;
  set_t actuator_set;

  float cmd;

public:
  ControlSystem(void) {
    feedback_filter_eval = 0;
    error_filter_eval = 0;
    setpoint_filter_eval = 0;
    sensor_get = 0;
    actuator_set = 0;

    feedback_filter = 0;
    error_filter = 0;
    setpoint_filter = 0;
    sensor = 0;
    actuator = 0;
  }

  template<class Actuator>
  void setActuator(Actuator * dev) {
    actuator = (void*)dev;
    actuator_set = [](void * arg, float val) { ((Actuator*)arg)->set(val); };
  }

  template<class Sensor>
  void setSensor(Sensor * dev) {
    sensor = (void*)dev;
    sensor_get = [](void * arg) { return ((Sensor*)arg)->get(); };
  }

  template<class Filter>
  void setSetpointFilter(Filter * dev) {
    setpoint_filter = (void*)dev;
    setpoint_filter_eval = [](void * arg, float val) { return ((Filter*)arg)->eval(val); };
  }

  template<class Filter>
  void setFeedbackFilter(Filter * dev) {
    feedback_filter = (void*)dev;
    feedback_filter_eval = [](void * arg, float val) { return ((Filter*)arg)->eval(val); };
  }

  template<class Filter>
  void setErrorFilter(Filter * dev) {
    error_filter = (void*)dev;
    error_filter_eval = [](void * arg, float val) { return ((Filter*)arg)->eval(val); };
  }

  void update(void) {
    if(sensor_get && actuator_set) {
      float feedback = sensor_get(sensor);
      float command = cmd;

      if(feedback_filter_eval) {
        feedback = feedback_filter_eval(feedback_filter, feedback);
      }

      if(setpoint_filter_eval) {
        command = setpoint_filter_eval(setpoint_filter, command);
      }

      float error = feedback - command;

      if(error_filter_eval) {
        error = error_filter_eval(error_filter, error);
      }

      actuator_set(actuator, error);
    }
  }

  int init(void) {
    cmd = 0;

    Scheduler::instance().add(config.freq, this);

    return 0;
  }

  void set(float val) {
    cmd = val;
  }
};

#endif//CONTROL_SYSTEM_H
