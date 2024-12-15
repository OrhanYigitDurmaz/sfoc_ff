#include <Arduino.h>

#include "SimpleFOC.h"
#include "SimpleCAN.h"   // <- this is the only include required, it should be smart enough to find the correct subclass
#include "vesc_can.h"

BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentsense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
BLDCMotor motor = BLDCMotor(7,0.1);

Commander command = Commander(Serial);
void onMotor(char* cmd){command.motor(&motor,cmd);}

CanInterface Can = CanInterface();

// https://community.simplefoc.com/t/b-g431-esc1-can-interface/2632/38
// b-g431-esc CAN pinout - Gnd, CAN L, CAN H, 5V

void setup()
{

  Serial.begin(230400);

  driver.voltage_power_supply = 16;
  driver.voltage_limit = driver.voltage_power_supply*0.9;
  driver.init();

  command.add('M',&onMotor,"motor");

  motor.linkDriver(&driver);

  motor.linkCurrentSense(&currentsense);
  currentsense.linkDriver(&driver);
  // don't skip current sense align with sfoc shield
  // currentsense.skip_align = true;
  currentsense.init();

  Can.linkCan(&CAN);
  Can.linkMotor(&motor);
  
  motor.init();
  motor.initFOC();

  Can.begin();
}

void loop()
{
  motor.move();
  motor.loopFOC();
  command.run();
  Can.run();

}