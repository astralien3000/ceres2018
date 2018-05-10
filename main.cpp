#include "control_layer_3.hpp"
#include "pull.h"
#include "side.h"

#include "positioning_action.h"

#include <feetech/sc.hpp>

#include "gp2.hpp"

#include "arm.hpp"
#include <feetech/sc.hpp>

//constexpr int GP2_LIMIT = 500;
constexpr int GP2_LIMIT = 300;

Side::Color match_side = Side::ORANGE;

inline void gotoXYSymOrange(float x, float y) {
  if(match_side == Side::ORANGE) {
    Serial.println("ORANGE XY");
    ControlLayer3::instance().traj.gotoXY(x, y);
  }
  else {
    Serial.println("GREEN XY");
    ControlLayer3::instance().traj.gotoXY(-x, y);
  }
  while(!ControlLayer3::instance().traj.isArrived());
}

inline void gotoXYSymOrangeSkate(float x, float y) {
  if(match_side == Side::ORANGE) {
    Serial.println("ORANGE XY SKATE");
    ControlLayer3::instance().traj.gotoXY(x, y);
  }
  else {
    Serial.println("GREEN XY SKATE");
    ControlLayer3::instance().traj.gotoXY(-x, y);
  }
  while(!ControlLayer3::instance().traj.isArrived() && !SecureMotor::locked());
  SecureMotor::locked() = false;
}

inline void gotoXYSymOrangeSkateTimeout(float x, float y) {
  if(match_side == Side::ORANGE) {
    Serial.println("ORANGE XY SKATE TIMEOUT");
    ControlLayer3::instance().traj.gotoXY(x, y);
  }
  else {
    Serial.println("GREEN XY SKATE TIMEOUT");
    ControlLayer3::instance().traj.gotoXY(-x, y);
  }

  int counter = 0;
  while(!ControlLayer3::instance().traj.isArrived() && !SecureMotor::locked() && counter++ < 500) {
    delay(10);
  }
  SecureMotor::locked() = false;
}

inline void gotoXYSymOrangeUnsecure(float x, float y) {
  if(match_side == Side::ORANGE) {
    Serial.println("ORANGE XY UNSECURE");
    ControlLayer3::instance().traj.gotoXY(x, y);
  }
  else {
    Serial.println("GREEN XY UNSECURE");
    ControlLayer3::instance().traj.gotoXY(-x, y);
  }
  while(!ControlLayer3::instance().traj.isArrived()) {
    SecureMotor::locked() = false;
  }
  SecureMotor::locked() = false;
}

inline void gotoAngleSide(float angle_orange, float angle_green) {
  float x = ControlLayer3::instance().loc.getX();
  float y = ControlLayer3::instance().loc.getY();
  if(match_side == Side::ORANGE) {
    Serial.println("ORANGE ANGLE");
    const float angle = angle_orange;
    ControlLayer3::instance().traj.gotoXYA(x, y, angle);
    while(fabs(ControlLayer3::instance().loc.getAngle() - angle) > 0.1);
  }
  else {
    Serial.println("GREEN ANGLE");
    const float angle = angle_green;
    ControlLayer3::instance().traj.gotoXYA(x, y, angle);
    while(fabs(ControlLayer3::instance().loc.getAngle() - angle) > 0.1);
  }
}

struct MatchLimit {
  int counter = 0;
  bool started = false;

  void update(void) {
    if(started) {
      if(++counter > 900) {
        SecureMotor::locked() = true;
      }
    }
  }
};

int main(void) {
  Serial.begin(115200);
  ControlLayer0::instance().init();
  ControlLayer1::instance().init();
  ControlLayer2::instance().init();
  ControlLayer3::instance().init();

  Pull::init();
  Side::init();
  GP2::init();

  MatchLimit match;
  Scheduler::instance().add(10, &match);

  // init
  if(Side::get() == Side::ORANGE) {
    Serial.println("ORANGE INIT");
    match_side = Side::ORANGE;
    ControlLayer3::instance().loc.reset(130, -20, M_PI);
    ControlLayer3::instance().traj.gotoXYA(130, -20, M_PI);
  }
  else {
    Serial.println("GREEN INIT");
    match_side = Side::GREEN;
    ControlLayer3::instance().loc.reset(-130, -20, 0);
    ControlLayer3::instance().traj.gotoXYA(-130, -20, 0);
  }

  delay(500);

  //Arm::instance().left().safe();
  //Arm::instance().right().safe();
  //delay(500);

  Arm::instance().left().retract();
  Arm::instance().right().retract();
  delay(500);

  while(!Pull::isPresent());
  delay(500);

  if(match_side == Side::ORANGE) {
    Serial.println("ORANGE INIT POSITION");
    PositioningAction act;
    act.config.pos.x = 150 - 60;
    act.config.pos.y = -20;
    act.config.dir.x = 60 - PositioningAction::ROBOT_BORDER_LENGTH;
    act.config.dir.y = 20 - PositioningAction::ROBOT_BORDER_LENGTH;
    act.config.timeout_ms = 5000;
    act.init();
    while(act.getState() != PositioningAction::FINISH) {
      act.update();
    }
  }
  else {
    Serial.println("GREEN INIT POSITION");
    PositioningAction act;
    act.config.pos.x = - 150 + 60;
    act.config.pos.y = -20;
    act.config.dir.x = -60 + PositioningAction::ROBOT_BORDER_LENGTH;
    act.config.dir.y =  20 - PositioningAction::ROBOT_BORDER_LENGTH;
    act.config.timeout_ms = 5000;
    act.init();
    while(act.getState() != PositioningAction::FINISH) {
      act.update();
    }
  }

  ControlLayer2::instance().speed.disableDetection();
  gotoXYSymOrange(130, -20);
  ControlLayer2::instance().speed.enableDetection();

  while(Pull::isPresent());
  match.started = true;
  delay(1000);

  // pannel
  gotoXYSymOrange(187-150, -50);

  gotoAngleSide(M_PI/2, M_PI/2);

  Arm::instance().right().safe();
  delay(500);
  Arm::instance().right().pannel();
  delay(500);

  ControlLayer2::instance().speed.disableDetection();
  gotoXYSymOrangeSkateTimeout(187-150, 10);
  ControlLayer2::instance().speed.enableDetection();

  gotoXYSymOrangeUnsecure(187-150, -50);

  Arm::instance().right().safe();
  delay(500);

  Arm::instance().right().retract();
  delay(500);

  // bee
  gotoXYSymOrange(150-80,-120);

  if(match_side == Side::ORANGE) {
    Serial.println("ORANGE BEE POSITION");
    PositioningAction act;
    act.config.pos.x =  120;
    act.config.pos.y = -170;
    act.config.dir.x =  30 - PositioningAction::ROBOT_BORDER_LENGTH;
    act.config.dir.y = -30 + PositioningAction::ROBOT_BORDER_LENGTH;
    act.config.timeout_ms = 5000;
    act.init();
    while(act.getState() != PositioningAction::FINISH) {
      act.update();
    }
  }
  else {
    Serial.println("GREEN BEE POSITION");
    PositioningAction act;
    act.config.pos.x = -120;
    act.config.pos.y = -170;
    act.config.dir.x = -30 + PositioningAction::ROBOT_BORDER_LENGTH;
    act.config.dir.y = -30 + PositioningAction::ROBOT_BORDER_LENGTH;
    act.config.timeout_ms = 5000;
    act.init();
    while(act.getState() != PositioningAction::FINISH) {
      act.update();
    }
  }

  gotoXYSymOrange(130,-180);

  gotoAngleSide(M_PI/2, -M_PI/2);

  Arm::instance().right().safe();
  delay(500);

  Arm::instance().right().bee();
  delay(500);

  gotoAngleSide(-M_PI/2, M_PI/2);

  Arm::instance().right().safe();
  delay(500);

  Arm::instance().right().retract();
  delay(500);

  while(1);
}

// FTM Interrupt Service routines - on overflow and position compare
void ftm1_isr(void)
{
  Encoder::qdec1().ftm_isr();
}

void ftm2_isr(void)
{
  Encoder::qdec2().ftm_isr();
}
