#include <Arduino.h>
#include <HerkulexMotor.h>
#include <SerialBusManager.h>

// Bus ID correlates to a serial ID, see corresponding Teensy pins below
constexpr uint8_t DEFAULT_MOTOR_ID = 1;
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

uint8_t activeMotorId = DEFAULT_MOTOR_ID;
HerkulexMotor* motor = nullptr;
bool motionEnabled = false;

bool findMotorId(uint8_t busId, uint8_t& foundId) {
  for (uint8_t id = 1; id <= 253; ++id) {
    uint16_t model = 0;
    if (SerialBusManager::getBus(busId).getModel(id, &model)) {
      foundId = id;
      Serial.print("Found servo at ID ");
      Serial.print(id);
      Serial.print(" model=0x");
      Serial.println(model, HEX);
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(1000000);
  delay(2000);

  Serial.println("BOOT OK: turn power on");
  delay(15000);
  Serial.println("Creating bus...");
  SerialBusManager::createBus(BUS_ID);

  Serial.println("Starting bus...");
  SerialBusManager::startAllBuses(BAUD_RATE::SPEED_667K);

  Serial.println("Scanning for Herkulex IDs...");
  if (findMotorId(BUS_ID, activeMotorId)) {
    Serial.print("Using motor ID ");
    Serial.println(activeMotorId);

    motor = new HerkulexMotor(activeMotorId, MotorModel::DRS_0601, BUS_ID);

    Serial.println("Initializing motors...");
    SerialBusManager::initAllMotors();

    Serial.println("Enabling torque...");
    SerialBusManager::torqueOnAllMotors();

    Serial.println("Motor bus initialized and torque enabled");
    Serial.println("Type 'start' to enable motion or 'stop' to pause");
  } else {
    Serial.println("No motor found on this bus. Check wiring, GND, ID, and baud.");
    Serial.println("Motion will remain disabled until a valid servo is found.");
  }
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equalsIgnoreCase("start")) {
      motionEnabled = true;
      Serial.println("Motion enabled");
    } else if (command.equalsIgnoreCase("stop")) {
      motionEnabled = false;
      Serial.println("Motion disabled");
    }
  }

  if (!motionEnabled || motor == nullptr) {
    delay(20);
    return;
  }

  Serial.println("Moving to -30 degrees");
  motor->setPos(-30.0f);
  delay(2000);

  Serial.print("Position: ");
  Serial.println(motor->getPos());

  Serial.println("Moving to 30 degrees");
  motor->setPos(30.0f);
  delay(2000);

  Serial.print("Position: ");
  Serial.println(motor->getPos());

  delay(2000);
}
