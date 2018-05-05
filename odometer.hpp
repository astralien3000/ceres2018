#ifndef ODOMETER_H
#define ODOMETER_H

#include "encoder.hpp"

class Odometer {
public:
  struct Config {
    float wheels_distance;
  };

  Config config;

  Encoder * encoder_left;
  Encoder * encoder_right;

private:
  float distance_offset;
  float angle_offset;

public:
  int init(void) {
    distance_offset = 0;
    angle_offset = 0;

    return 0;
  }

  void reset(float dist, float angle) {
    encoder_left->reset();
    encoder_right->reset();

    const float dist_left = encoder_left->getDistance();
    const float dist_right = encoder_right->getDistance();
    distance_offset = dist - (dist_right + dist_left) / 2;
    angle_offset  = angle - (dist_right - dist_left) / config.wheels_distance;
  }

  float getDistance(void) {
    const float dist_left = encoder_left->getDistance();
    const float dist_right = encoder_right->getDistance();
    return distance_offset + (dist_right + dist_left) / 2;
  }

  float getLinearSpeed(void) {
    const float speed_left = encoder_left->getSpeed();
    const float speed_right = encoder_right->getSpeed();
    return (speed_right + speed_left) / 2;
  }

  float getAngle(void) {
    const float dist_left = encoder_left->getDistance();
    const float dist_right = encoder_right->getDistance();
    return angle_offset + (dist_right - dist_left) / config.wheels_distance;
  }

  float getAngularSpeed(void) {
    const float speed_left = encoder_left->getSpeed();
    const float speed_right = encoder_right->getSpeed();
    return (speed_right - speed_left) / config.wheels_distance;
  }

  class Distance;
  Distance& getDistanceSensor(void) {
    return *(Distance*)this;
  }

  class LinearSpeed;
  LinearSpeed& getLinearSpeedSensor(void) {
    return *(LinearSpeed*)this;
  }

  class Angle;
  Angle& getAngleSensor(void) {
    return *(Angle*)this;
  }

  class AngularSpeed;
  AngularSpeed& getAngularSpeedSensor(void) {
    return *(AngularSpeed*)this;
  }
};

struct Odometer::Distance : public Odometer {
  float get(void) {
    return Odometer::getDistance();
  }
};

struct Odometer::LinearSpeed : public Odometer {
  float get(void) {
    return Odometer::getLinearSpeed();
  }
};

struct Odometer::Angle : public Odometer {
  float get(void) {
    return Odometer::getAngle();
  }
};

struct Odometer::AngularSpeed : public Odometer {
  float get(void) {
    return Odometer::getAngularSpeed();
  }
};


#endif//ODOMETER_H
