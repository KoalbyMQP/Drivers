#include <Arduino.h>
#include <HerkulexMotor.h>
#include <SerialBusManager.h>

// Bus ID correlates to a serial ID, see corresponding Teensy pins below
constexpr uint8_t MOTOR_ID = 1;
constexpr uint8_t BUS_ID = 1;

/*
The bus ID is used to select the correct serial port for communication with the motor
Bus IDs (teensy-motor-driver/lib/Herkulex/Herkulex.cpp):
#define HSerial1     1 		// Write in Serial 1 port Teensy 4.1 - Pin 00(rx) - 01(tx) 
#define HSerial2     2   	// Write in Serial 2 port Teensy 4.1 - Pin 07(rx) - 08(tx) 
#define HSerial3     3   	// Write in Serial 3 port Teensy 4.1 - Pin 15(rx) - 14(tx)
#define HSerial4     4 		// Write in Serial 4 port Teensy 4.1 - Pin 16(rx) - 17(tx) 
#define HSerial5     5   	// Write in Serial 5 port Teensy 4.1 - Pin 21(rx) - 20(tx) 
#define HSerial6     6   	// Write in Serial 6 port Teensy 4.1 - Pin 25(rx) - 24(tx)
#define HSerial7     7 		// Write in Serial 7 port Teensy 4.1 - Pin 28(rx) - 29(tx)
#define HSerial8     8 		// Write in Serial 8 port Teensy 4.1 - Pin 34(rx) - 35(tx)
*/
HerkulexMotor motor(MOTOR_ID, MotorModel::DRS_0601, BUS_ID);

void setup() {
  Serial.begin(1000000);
  delay(2000);

  Serial.println("Starting isolated Herkulex motor test");

  SerialBusManager::createBus(BUS_ID);
  SerialBusManager::startAllBuses(BAUD_RATE::SPEED_667K);
  SerialBusManager::initAllMotors();

  Serial.println("Motor bus initialized");
}

// Use Herkulex Motor library functions to actuate motors
void loop() {
  Serial.println("Moving to -30 degrees");
  motor.setPos(-30.0f);
  delay(2000);

  Serial.print("Position: ");
  Serial.println(motor.getPos());

  Serial.println("Moving to 30 degrees");
  motor.setPos(30.0f);
  delay(2000);

  Serial.print("Position: ");
  Serial.println(motor.getPos());

  delay(2000);
}
