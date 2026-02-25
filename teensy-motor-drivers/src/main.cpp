#include <Herkulex.h>
#include <HerkulexMotor.h>
#include <RPIComs.h>

uint16_t startTime = 0;
uint16_t elapsedTime = 0;
boolean testSetPosBool = false;    // when runTest is true, the moving motor will run once upon restart and when the arduino is uploaded.
boolean testQueueBool = false;
boolean testRPi = true;
boolean latency_queuing = false;

HerkulexMotor myMotor = HerkulexMotor(12, MotorModel::DRS_0601);
HerkulexMotor myMotor2 = HerkulexMotor(7, MotorModel::DRS_0602);
RPIComs rpi = RPIComs();

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}