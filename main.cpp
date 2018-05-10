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

int main(void) {
  Serial.begin(115200);
  ControlLayer0::instance().init();
  ControlLayer1::instance().init();
  ControlLayer2::instance().init();
  ControlLayer3::instance().init();

  Pull::init();
  Side::init();
  GP2::init();

  int i = 0;

  /*
  while(1) {
    Serial.print(i++);
    Serial.print(" ");
    Serial.print(GP2::get(GP2::FRONT_LEFT));
    Serial.print(" ");
    Serial.print(GP2::get(GP2::FRONT_RIGHT));
    Serial.print(" ");
    Serial.print(GP2::get(GP2::BACK_LEFT));
    Serial.print(" ");
    Serial.print(GP2::get(GP2::BACK_RIGHT));
    Serial.print(" ");
    Serial.println("");

    delay(100);
  }
  */

  if(Side::get() == Side::ORANGE) {
    ControlLayer3::instance().loc.reset(130, -20, M_PI);
    ControlLayer3::instance().traj.gotoXYA(130, -20, M_PI);
  }
  else {
    ControlLayer3::instance().loc.reset(-130, -20, 0);
    ControlLayer3::instance().traj.gotoXYA(-130, -20, 0);
  }

  Arm::instance().left().safe();
  Arm::instance().right().safe();
  delay(500);

  Arm::instance().left().retract();
  Arm::instance().right().retract();
  delay(500);

  while(!Pull::isPresent());
  delay(500);

  if(Side::get() == Side::ORANGE) {
    ControlLayer3::instance().traj.gotoXY(130, -25);
  }
  else {
    ControlLayer3::instance().traj.gotoXY(-130, -25);
  }

  while(Pull::isPresent());
  delay(1000);

  if(Side::get() == Side::ORANGE) {
    ControlLayer3::instance().traj.gotoXY(80, -50);
  }
  else {
    ControlLayer3::instance().traj.gotoXY(-80, -50);
  }

  while(!ControlLayer3::instance().traj.isArrived());

  if(Side::get() == Side::ORANGE) {
    ControlLayer3::instance().traj.gotoXY(80, -80);
  }
  else {
    ControlLayer3::instance().traj.gotoXY(-80, -80);
  }

  while(!ControlLayer3::instance().traj.isArrived());

  if(Side::get() == Side::ORANGE) {
    ControlLayer3::instance().traj.gotoXY(130, -180);
  }
  else {
    ControlLayer3::instance().traj.gotoXY(-130, -180);
  }

  while(!ControlLayer3::instance().traj.isArrived());

  Arm::instance().right().safe();

  if(Side::get() == Side::ORANGE) {
    const float angle = -M_PI/2;
    ControlLayer3::instance().traj.gotoXYA(130, -180, angle);
    while(fabs(ControlLayer3::instance().loc.getAngle() - angle) < 2);
  }
  else {
    const float angle = -M_PI/2;
    ControlLayer3::instance().traj.gotoXYA(-130, -180, angle);
    while(fabs(ControlLayer3::instance().loc.getAngle() - angle) < 2);
  }

  Arm::instance().right().bee();
  delay(1000);

  if(Side::get() == Side::ORANGE) {
    const float angle = -M_PI/2;
    ControlLayer3::instance().traj.gotoXYA(130, -180, angle);
    while(fabs(ControlLayer3::instance().loc.getAngle() - angle) < 2);
  }
  else {
    const float angle = 3*M_PI;
    ControlLayer3::instance().traj.gotoXYA(-130, -180, angle);
    while(fabs(ControlLayer3::instance().loc.getAngle() - angle) < 2);
  }

  PositioningAction pos_act;
  pos_act.config.pos.x = 100;
  pos_act.config.pos.y = -20;
  pos_act.config.dir.x = 50 - PositioningAction::ROBOT_BORDER_LENGTH;
  pos_act.config.dir.y = 20 - PositioningAction::ROBOT_BORDER_LENGTH;
  pos_act.init();

  //Arm::instance().left().retract();
  //Arm::instance().right().retract();

  /*
  Serial1.begin(1000000);
  while(1) {
    SC::instance().enableTorque(0xFE);
  }
  */

  /*
  while(1) {
    delay(1000);
    for(int i = 0 ; i < 30 ; i++) {
      Serial.print(i);
      Serial.print(" ");
      if(SC::instance().ping(i)) {
        Serial.println("SUCCESS");
      }
      else {
        Serial.println("ERROR");
      }
    }
  }
  */


  /*
  Arm::instance().left().safe();
  Arm::instance().right().safe();
  delay(500);
  Arm::instance().right().retract();
  Arm::instance().left().retract();
  */

  /*
  while(1) {
    delay(1000);
    Arm::instance().left().safe();
    Arm::instance().right().safe();
    delay(1000);
    Arm::instance().left().retract();
    Arm::instance().right().retract();
    delay(1000);
    Arm::instance().left().safe();
    Arm::instance().right().safe();
    delay(1000);
    Arm::instance().left().deploy();
    Arm::instance().right().deploy();
  }
  */


  while(1) {
    //pos_act.update();

    Serial.print(i++);
    Serial.print(" ");
    Serial.print(ControlLayer3::instance().loc.getX());
    Serial.print(" ");
    Serial.print(ControlLayer3::instance().loc.getY());
    Serial.print(" ");
    Serial.print(ControlLayer3::instance().loc.getAngle());
    Serial.print(" ");
    Serial.print(ControlLayer0::instance().enc_l.getDistance());
    Serial.print(" ");
    Serial.print(ControlLayer0::instance().enc_r.getDistance());
    Serial.print(" ");
    Serial.print(GP2::get(GP2::FRONT_LEFT));
    Serial.print(" ");
    Serial.println("");

    delay(100);
  }
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
