#include "arm.hpp"

#include <feetech/sc.hpp>
#include <Arduino.h>

#include "scheduler.hpp"

struct SC :
    public Aversive::Feetech::SC<SC>,
    public Singleton<SC> {
  SC(void)
    : Aversive::Feetech::SC<SC>(*this)
  {
    Serial1.begin(1000000);
    Serial1.setTimeout(100);
  }

  size_t write(const char * data, size_t size) {
    //Serial.write(data, size);
    return Serial1.write(data, size);
  }

  size_t read(char * data, size_t size) {
    return Serial1.readBytes(data, size);
  }
};

#define ARM_SERVO_NUMOF (5)

#define SERVO_DELTA (10)

typedef enum {
  RETRACTED       = 0,
  DEPLOYED_MIDDLE = 1,
  DEPLOYED_DOWN   = 2,
  BEE             = 3,
  PANNEL          = 4,

  MAX_POSES
} pose_t;

static uint8_t _left_ids[ARM_SERVO_NUMOF] = {
  10, 11, 12, 13, 14
};

static uint8_t _right_ids[ARM_SERVO_NUMOF] = {
  20, 21, 22, 23, 24
};

static uint16_t _left_poses[MAX_POSES][ARM_SERVO_NUMOF] = {
  {150,  50, 540, 200, 100}, // RETRACTED
  {512, 512, 600, 270, 130}, // DEPLOYED_MIDDLE
  {512, 512, 100, 700, 130}, // DEPLOYED_DOWN
  {512, 900, 500, 400, 512}, // BEE
  {100, 900, 500, 400, 512}, // PANNEL
};

static uint16_t _right_poses[MAX_POSES][ARM_SERVO_NUMOF] = {
  {100,  50, 540, 200, 100}, // RETRACTED
  {512, 512, 600, 270, 150}, // DEPLOYED_MIDDLE
  {512, 512, 100, 700, 150}, // DEPLOYED_DOWN
  {512, 900, 500, 400, 512}, // BEE
  {100, 900, 370, 400, 370}, // PANNEL
};

static float _left_angles[ARM_SERVO_NUMOF] = { 0,0,0,0,0 };
static float _right_angles[ARM_SERVO_NUMOF] = { 0,0,0,0,0 };

static float _no_angles[ARM_SERVO_NUMOF] = { 0,0,0,0,0 };

void _servo_enable(uint8_t id) {
  //Serial.println("enable");
  SC::instance().enableTorque(id);
  //feetech_write8(&dev, SCS15_TORQUE_ENABLE, 1);
  //feetech_write16(&dev, SCS15_GOAL_TIME, 512);
}

void _servo_set_angle(uint8_t id, uint16_t angle) {
  //Serial.println("setAngle");
  SC::instance().enableTorque(id);
  SC::instance().setPosition(id, angle);
  //feetech_write8(&dev, SCS15_TORQUE_ENABLE, 1);
  //feetech_write16(&dev, SCS15_GOAL_TIME, 512);
  //feetech_write16(&dev, SCS15_GOAL_POSITION, angle);
}

void _arm_enable(uint8_t ids[ARM_SERVO_NUMOF]) {
  for(size_t i = 0 ; i < ARM_SERVO_NUMOF ; i++) {
    _servo_enable(ids[i]);
  }
}

uint16_t add_angle(uint16_t pose, float angle) {
  const int p = pose;
  const int a = angle * 1024 / 3.1415;

  const int ret = p+a;

  if(ret < 0) return 0;
  if(ret > 1024) return 1024;

  return ret;
}

void _arm_set_pose(uint8_t ids[ARM_SERVO_NUMOF], uint16_t pose[ARM_SERVO_NUMOF], float angle[ARM_SERVO_NUMOF]) {
  for(size_t i = 0 ; i < ARM_SERVO_NUMOF ; i++) {
    _servo_set_angle(ids[i], add_angle(pose[i], angle[i]));
  }
}

Arm::Arm(void) {}

void Arm::Left::deploy(void) {
  _arm_set_pose(_left_ids, _left_poses[DEPLOYED_DOWN], _left_angles);
}

void Arm::Left::safe(void) {
  _arm_set_pose(_left_ids, _left_poses[DEPLOYED_MIDDLE], _left_angles);
}

void Arm::Left::retract(void) {
  _arm_set_pose(_left_ids, _left_poses[RETRACTED], _no_angles);
}

void Arm::Left::bee(void) {
  _arm_set_pose(_left_ids, _left_poses[BEE], _no_angles);
}

void Arm::Right::deploy(void) {
  _arm_set_pose(_right_ids, _right_poses[DEPLOYED_DOWN], _right_angles);
}

void Arm::Right::safe(void) {
  _arm_set_pose(_right_ids, _right_poses[DEPLOYED_MIDDLE], _right_angles);
}

void Arm::Right::retract(void) {
  _arm_set_pose(_right_ids, _right_poses[RETRACTED], _no_angles);
}

void Arm::Right::bee(void) {
  _arm_set_pose(_right_ids, _right_poses[BEE], _no_angles);
}

void Arm::Right::pannel(void) {
  _arm_set_pose(_right_ids, _right_poses[PANNEL], _no_angles);
}

void Arm::Left::setAngles(float a1, float a2) {
  _left_angles[0] = a1;
  _left_angles[3] = a2;
}

void Arm::Right::setAngles(float a1, float a2) {
  _right_angles[0] = a1;
  _right_angles[3] = a2;
}
