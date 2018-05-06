#include "arm.hpp"

#include <feetech/sc.hpp>
#include <Arduino.h>

#include "scheduler.hpp"

struct SC :
    public Aversive::Feetech::SC<SC>,
    public Singleton<SC> {
  SC(void) : Aversive::Feetech::SC<SC>(instance()) {}

  size_t write(const char * data, size_t size) {
    return Serial1.write(data, size);
  }

  size_t read(char * data, size_t size) {
    return 0;
  }
};

#define ARM_SERVO_NUMOF (5)

#define SERVO_DELTA (10)

typedef enum {
  RETRACTED       = 0,
  DEPLOYED_MIDDLE = 1,
  DEPLOYED_DOWN   = 2,
} pose_t;

typedef enum {
  STATE_OFF,
  STATE_INIT,
  STATE_RETRACTED,
  STATE_DEPLOYED,
  STATE_MOVING_RETRACT_1,
  STATE_MOVING_RETRACT_2,
  STATE_MOVING_DEPLOY_1,
  STATE_MOVING_DEPLOY_2,

  STATE_ERROR
} state_t;

typedef enum {
  EVENT_INIT,
  EVENT_DEPLOY,
  EVENT_RETRACT,
  EVENT_DONE,

  EVENT_EMPTY
} event_t;

static uint8_t _left_ids[ARM_SERVO_NUMOF] = {
  10, 11, 12, 13, 14
};

static uint8_t _right_ids[ARM_SERVO_NUMOF] = {
  20, 21, 22, 23, 24
};

static uint16_t _left_poses[3][ARM_SERVO_NUMOF] = {
  {170,  80, 540, 400, 200}, // RETRACTED
  {512, 512, 630, 270, 130}, // DEPLOYED_MIDDLE
  {512, 512, 100, 700, 130}, // DEPLOYED_DOWN
};

static uint16_t _right_poses[3][ARM_SERVO_NUMOF] = {
  {140, 140, 540, 330, 180}, // RETRACTED
  {512, 512, 630, 270, 150}, // DEPLOYED_MIDDLE
  {512, 512, 100, 700, 150}, // DEPLOYED_DOWN
};

static float _left_angles[ARM_SERVO_NUMOF] = { 0,0,0,0,0 };
static float _right_angles[ARM_SERVO_NUMOF] = { 0,0,0,0,0 };

static float _no_angles[ARM_SERVO_NUMOF] = { 0,0,0,0,0 };

static state_t _left_state = STATE_OFF;
static event_t _left_event = EVENT_EMPTY;

static state_t _right_state = STATE_OFF;
static event_t _right_event = EVENT_EMPTY;

void _servo_enable(uint8_t id) {
  SC::instance().enableTorque(id);
  //feetech_write8(&dev, SCS15_TORQUE_ENABLE, 1);
  //feetech_write16(&dev, SCS15_GOAL_TIME, 512);
}

void _servo_set_angle(uint8_t id, uint16_t angle) {
  SC::instance().enableTorque(id);
  SC::instance().setPosition(id, angle);
  //feetech_write8(&dev, SCS15_TORQUE_ENABLE, 1);
  //feetech_write16(&dev, SCS15_GOAL_TIME, 512);
  //feetech_write16(&dev, SCS15_GOAL_POSITION, angle);
}

bool _servo_check_angle(uint8_t id, uint16_t angle) {
  //uint16_t res = 0xFFFF;
  //feetech_read16(&dev, SCS15_PRESENT_POSITION, &res);

  return true;
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

bool _arm_check_pose(uint8_t ids[ARM_SERVO_NUMOF], uint16_t pose[ARM_SERVO_NUMOF], float angle[ARM_SERVO_NUMOF]) {
  for(size_t i = 0 ; i < ARM_SERVO_NUMOF ; i++) {
    if(!_servo_check_angle(ids[i], add_angle(pose[i], angle[i]))) {
      return false;
    }
  }
  return true;
}

static inline event_t _update_event(uint8_t ids[ARM_SERVO_NUMOF], uint16_t poses[3][ARM_SERVO_NUMOF], float angle[ARM_SERVO_NUMOF], state_t state) {
  if(state == STATE_OFF) {
  }
  else if(state == STATE_INIT) {
    return EVENT_DONE;
  }
  else if(state == STATE_MOVING_DEPLOY_1) {
    if(_arm_check_pose(ids, poses[DEPLOYED_MIDDLE], _no_angles)) {
      return EVENT_DONE;
    }
  }
  else if(state == STATE_MOVING_DEPLOY_2) {
    if(_arm_check_pose(ids, poses[DEPLOYED_DOWN], angle)) {
      return EVENT_DONE;
    }
  }
  else if(state == STATE_MOVING_RETRACT_1) {
    if(_arm_check_pose(ids, poses[DEPLOYED_MIDDLE], _no_angles)) {
      return EVENT_DONE;
    }
  }
  else if(state == STATE_MOVING_RETRACT_2) {
    if(_arm_check_pose(ids, poses[RETRACTED], _no_angles)) {
      return EVENT_DONE;
    }
  }

  return EVENT_EMPTY;
}

static inline state_t _update_state(uint8_t ids[ARM_SERVO_NUMOF], uint16_t poses[3][ARM_SERVO_NUMOF], float angle[ARM_SERVO_NUMOF], state_t state, event_t event) {
  if(state == STATE_OFF) {
    if(event == EVENT_INIT) {
      _arm_enable(ids);
      return STATE_INIT;
    }
  }
  else if(state == STATE_INIT) {
    if(event == EVENT_DONE) {
      _arm_set_pose(ids, poses[DEPLOYED_MIDDLE], _no_angles);
      return STATE_MOVING_RETRACT_1;
    }
  }
  else if(state == STATE_RETRACTED) {
    if(event == EVENT_DEPLOY) {
      _arm_set_pose(ids, poses[DEPLOYED_MIDDLE], _no_angles);
      return STATE_MOVING_DEPLOY_1;
    }
  }
  else if(state == STATE_DEPLOYED) {
    if(event == EVENT_RETRACT) {
      _arm_set_pose(ids, poses[DEPLOYED_MIDDLE], _no_angles);
      return STATE_MOVING_RETRACT_1;
    }
    if(event == EVENT_DEPLOY) {
      _arm_set_pose(ids, poses[DEPLOYED_DOWN], angle);
      return STATE_DEPLOYED;
    }
  }
  else if(state == STATE_MOVING_DEPLOY_1) {
    if(event == EVENT_DONE) {
      _arm_set_pose(ids, poses[DEPLOYED_DOWN], angle);
      return STATE_MOVING_DEPLOY_2;
    }
  }
  else if(state == STATE_MOVING_DEPLOY_2) {
    if(event == EVENT_DONE) {
      return STATE_DEPLOYED;
    }
  }
  else if(state == STATE_MOVING_RETRACT_1) {
    if(event == EVENT_DONE) {
      _arm_set_pose(ids, poses[RETRACTED], _no_angles);
      return STATE_MOVING_RETRACT_2;
    }
  }
  else if(state == STATE_MOVING_RETRACT_2) {
    if(event == EVENT_DONE) {
      return STATE_RETRACTED;
    }
  }

  return state;
}

static inline Arm::ArmState _arm_state(state_t state) {
  switch(state) {
    case STATE_DEPLOYED:
      return Arm::ARM_DEPLOYED;
    case STATE_RETRACTED:
      return Arm::ARM_RETRACTED;
    case STATE_INIT:
    case STATE_MOVING_DEPLOY_1:
    case STATE_MOVING_DEPLOY_2:
    case STATE_MOVING_RETRACT_1:
    case STATE_MOVING_RETRACT_2:
      return Arm::ARM_MOVING;
    case STATE_OFF:
      return Arm::ARM_DISABLED;
    default:
      return Arm::ARM_ERROR;
  }
  return Arm::ARM_ERROR;
}

void Arm::update(void) {
  if(Arm::instance().state() == Service::RUN) {
    // LEFT
    if(_left_event == EVENT_EMPTY) {
      _left_event = _update_event(_left_ids, _left_poses, _left_angles, _left_state);
    }

    _left_state = _update_state(_left_ids, _left_poses, _left_angles, _left_state, _left_event);
    _left_event = EVENT_EMPTY;

    // RIGHT
    if(_right_event == EVENT_EMPTY) {
      _right_event = _update_event(_right_ids, _right_poses, _right_angles, _right_state);
    }

    _right_state = _update_state(_right_ids, _right_poses, _right_angles, _right_state, _right_event);
    _right_event = EVENT_EMPTY;
  }
}

Arm::Arm(void) {
  _left_event = EVENT_INIT;
  _right_event = EVENT_INIT;

  Scheduler::instance().add(2, this);
}

void Arm::Left::deploy(void) {
  _left_event = EVENT_DEPLOY;
}

void Arm::Left::retract(void) {
  _left_event = EVENT_RETRACT;
}

Arm::ArmState Arm::Left::state(void) {
  return _arm_state(_left_state);
}

void Arm::Right::deploy(void) {
  _right_event = EVENT_DEPLOY;
}

void Arm::Right::retract(void) {
  _right_event = EVENT_RETRACT;
}

Arm::ArmState Arm::Right::state(void) {
  return _arm_state(_right_state);
}

void Arm::Left::setAngles(float a1, float a2) {
  _left_angles[0] = a1;
  _left_angles[3] = a2;
}

void Arm::Right::setAngles(float a1, float a2) {
  _right_angles[0] = a1;
  _right_angles[3] = a2;
}

