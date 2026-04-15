// =============== move_and_read_one_motor.cpp =============== //
// provides basic example for how to move and read position from one motor. //
// move command sends data to invidual motor, not a simultaneous move       //
// read command is blocking.

#include <HerkulexMotor.h>

uint8_t motorId = 1; // assigned ID of motor
uint8_t busId = 2; // serial port motor is attached to
HerkulexMotor myMotor = HerkulexMotor(motorId, MotorModel::DRS_0601, busId); // create motor object

// table holding lookup motorRef object for motor.
MotorRef motorTable[1] = {
  myMotor.getMotorRef()
};

void setup()  
{
  Serial.begin(9600);    // Open serial communications
  delay(2000);           //a delay to have time for serial monitor opening

  SerialBusManager::createBus(busId);
  SerialBusManager::startAllBuses(BAUD_RATE::SPEED_115K);

  // provides STAT info for motor
  SerialBusManager::infoAllMotors(motorTable, 1);
}

void loop(){
  Serial.println("Move Angle: -50.0 degrees");
  myMotor.setPos(-50.0);
  delay(2000);

  Serial.print("Get servo Angle:");
  Serial.println(myMotor.getPos());
  delay(2000);

  Serial.println("Move Angle: 50.0 degrees");
  myMotor.setPos(50.0);
  delay(2000);

  Serial.print("Get servo Angle:");
  Serial.println(myMotor.getPos());
  delay(2000);
}