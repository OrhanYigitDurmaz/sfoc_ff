#include <Arduino.h>

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"

#include "SimpleCAN.h"  
#include "vesc_can.h"

// motor constants
constexpr int pole_pairs = 4;
constexpr float phase_resistance = 0.8;
constexpr float phase_inductance = 0.00008;
constexpr float motor_KV = NOT_SET;

constexpr float max_current = 4.0;
constexpr float current_bandwidth = 300;
constexpr unsigned int encoder_ppr = 2048;
// setup constants
constexpr uint32_t CAN_speed = 500000;
constexpr uint8_t CAN_address = 0xFF;
constexpr uint32_t CAN_termination = LOW; // LOW -> OFF, HIGH -> ON

// constants relevant to the driver hardware
constexpr float v_bus_scale = 10.2838;

int error_status = 0;

SimpleFOCDebug debug;

BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentsense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
BLDCMotor motor = BLDCMotor(pole_pairs, phase_resistance, motor_KV, phase_inductance);
STM32HWEncoder encoder = STM32HWEncoder(encoder_ppr, A_ENCODER_A, A_ENCODER_B);

Commander command = Commander(Serial);
void onMotor(char* cmd){command.motor(&motor,cmd);}

CanInterface Can = CanInterface();
// https://community.simplefoc.com/t/b-g431-esc1-can-interface/2632/38
// b-g431-esc CAN pinout - Gnd, CAN L, CAN H, 5V

int check_vbus();

void setup()
{
  pinMode(PC14, OUTPUT);
  digitalWrite(PC14, CAN_termination);

  encoder._pinA = PB_6;
  encoder._pinB = PB_7_ALT1;

  Serial.begin(230400);
  debug.enable();
  SimpleFOC_CORDIC_Config();      // initialize the CORDIC
  command.add('M',&onMotor,"motor");
  
  delay(2000);

  driver.voltage_power_supply = 16;
  driver.voltage_limit = driver.voltage_power_supply*0.9;
  motor.current_limit = max_current;

  int driver_init = driver.init();
  Serial.printf("Driver init status: %d\n", driver_init);

  encoder.init();
  Serial.printf("Encoder init status: %d\n", encoder.initialized);

  
  motor.linkSensor(&encoder);
  motor.linkDriver(&driver);
  motor.linkCurrentSense(&currentsense);
  currentsense.linkDriver(&driver);

  int cs_init = currentsense.init();
  Serial.printf("Current sense init status: %d\n", cs_init);

  Can.setCanAddr(CAN_address);
  Can.setCanSpeed(CAN_speed);
  Can.linkCan(&CAN);
  Can.linkMotor(&motor);
  
  motor.PID_current_d.P = phase_inductance*current_bandwidth*_2PI;
  motor.PID_current_d.I = motor.PID_current_d.P*phase_resistance/phase_inductance;
  motor.PID_current_d.D = 0;
  motor.PID_current_d.output_ramp = 0;
  motor.LPF_current_d.Tf = 1/(_2PI*3.0f*current_bandwidth);

  motor.PID_current_q.P = phase_inductance*current_bandwidth*_2PI;
  motor.PID_current_q.I = motor.PID_current_q.P*phase_resistance/phase_inductance;
  motor.PID_current_q.D = 0;
  motor.PID_current_q.output_ramp = 0;
  motor.LPF_current_q.Tf = 1/(_2PI*3.0f*current_bandwidth);

  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;

  int m_init = motor.init();
  Serial.printf("Motor init status: %d\n", m_init);

  int foc_init = motor.initFOC();
  Serial.printf("FOC init status: %d\n", foc_init);

  Can.begin();
}

void loop()
{
  motor.move();
  motor.loopFOC();
  command.run();

  error_status = check_vbus();
  Can.setErrorState(error_status);
  Can.run();

}

int check_vbus() {
  float v_bus = _readADCVoltageLowSide(A_VBUS,currentsense.params)*v_bus_scale;
  Can.setVBus(v_bus);
  driver.voltage_power_supply = v_bus;
  driver.voltage_limit = driver.voltage_power_supply*0.9;
  int error = 0;
  if (v_bus > 24.0f) {
    motor.target = 0;
    motor.disable();
    error = 1;
    Serial.printf("Overvoltage: Motor off\n");
  }
  return error;
}