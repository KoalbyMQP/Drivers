#include <HerkulexMotor.h>
#include <RPIComs.h>
#include <SerialBusManager.h>
#include <IMU.h>
#include <debug.h>
#include <ToolChanger.h>

// start at serial 2 because the raspberry pi is connected through serial 1
enum SERIAL_BUS {
    BUS_CHEST = 3,
    BUS_L_ARM = 4,
    BUS_R_ARM = 5,
};

const uint8_t MOTOR_COUNT = 10;

// robot state machine for the swapping sequence
enum ROBOT_STATE {
  IDLE,
  STATUS_STATE,
  ATTACH_STATE,
  DEPOSIT_STATE,
  ELEVATOR_STATE,
};

ROBOT_STATE robotState = IDLE;

uint16_t motorPositionsRaw[MOTOR_COUNT] = {0};
float motorPositions[MOTOR_COUNT] = {0};

MotorRef motorRefs[MOTOR_COUNT];

HerkulexMotor motors[MOTOR_COUNT] = {
  HerkulexMotor(7, MotorModel::DRS_0601, SERIAL_BUS::BUS_CHEST), // 
  HerkulexMotor(11, MotorModel::DRS_0601, SERIAL_BUS::BUS_CHEST), // 
  HerkulexMotor(6, MotorModel::DRS_0602, SERIAL_BUS::BUS_L_ARM),
  HerkulexMotor(9, MotorModel::DRS_0601, SERIAL_BUS::BUS_L_ARM),
  HerkulexMotor(14, MotorModel::DRS_0201, SERIAL_BUS::BUS_L_ARM),
  HerkulexMotor(19, MotorModel::DRS_0201, SERIAL_BUS::BUS_L_ARM),
  HerkulexMotor(219, MotorModel::DRS_0602, SERIAL_BUS::BUS_R_ARM),
  HerkulexMotor(8, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_ARM),
  HerkulexMotor(17, MotorModel::DRS_0201, SERIAL_BUS::BUS_R_ARM),
  HerkulexMotor(26, MotorModel::DRS_0201, SERIAL_BUS::BUS_R_ARM),
};

// status pin is analog
ToolChanger TCR(A13, 36, 600, 400, 1600); // status pin, servo pin, attach pos, lock pos, deposit pos
ToolChanger TCL(A12, 37, 800, 600, 1800);


void setup(){
  delay(2000); // a delay to have time for serial monitor opening on platformio after uploading
  Serial.begin(9600); // Open serial communications with computer
  Serial.println("Serial initialized for debugging output");

  // intiailize serial communications with esp32
  Serial2.begin(115200);
  Serial.println("Serial2 initialized for SwappingStation communication");

  delay(2000);

  // initialize all serial buses
  SerialBusManager::createBus(SERIAL_BUS::BUS_CHEST);
  SerialBusManager::createBus(SERIAL_BUS::BUS_L_ARM);
  SerialBusManager::createBus(SERIAL_BUS::BUS_R_ARM);
  
  SerialBusManager::startAllBuses(115200);
  SerialBusManager::initAllMotors();

  delay(2000);

  // set all motors to blue to indicate they are initialized and ready
  for (int i = 0; i < MOTOR_COUNT; i++){
    motors[i].setLed(LED_STATE::LED_BLUE);
  }

  // MotorRef table — used by the parallelized position read.
  // Each entry is {busId, servoId}. Order here determines order in rawPositions[], can mix and match serial buses
  // Add or remove entries to match the motors needed
  for (int i = 0; i < MOTOR_COUNT; i++) motorRefs[i] = motors[i].getMotorRef();
  for (int i = 0; i < MOTOR_COUNT; i++) motors[i].setPos(0.0);
  delay(2000);

  // initialize the toolchangers
  TCR.initialize();
  TCL.initialize();

  // wait until communication with the esp32 is established by waiting until it receives
  // the message HC to establish communication that homing has been completed
  while (true) {
    if (Serial2.available()) {
      String message = Serial2.readStringUntil('\n');
      message.trim();  // Remove whitespace and carriage returns
      Serial.print("Received message from ESP32: ");
      Serial.println(message);
      if (message == "HC") {
        Serial.println("Homing completed.");
        break;
      }
    }
  }
}

void loop(){
  // state machine for the robot 
  switch (robotState) {
    case IDLE:
      // read input from the serial user interface to determine what state to switch to
      if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();  // Remove whitespace and carriage returns
        Serial.print("Received input: ");
        Serial.println(input);
        if (input == "status") {
          robotState = STATUS_STATE;
        } else if (input == "attach") {
          robotState = ATTACH_STATE;
        } else if (input == "deposit") {
          robotState = DEPOSIT_STATE;
        } else if (input == "elevator") {
          robotState = ELEVATOR_STATE;
        } else {
          Serial.println("Invalid input. Please enter 'status', 'attach', 'deposit', or 'elevator'.");
        }
      }
      break;
    case STATUS_STATE:
      // read the tool status and print it
      int tcrStatus = TCR.getToolStatus();
      int tclStatus = TCL.getToolStatus();
      Serial.print("TCR Status: ");
      Serial.print(tcrStatus);
      Serial.print(" | TCL Status: ");
      Serial.println(tclStatus);
      robotState = IDLE; // go back to idle after printing status
      break;
    case ATTACH_STATE:
      motors[0].queueMove(20.0);
      motors[1].queueMove(0.0);
      motors[2].queueMove(-60.0);
      motors[3].queueMove(0.0);
      motors[4].queueMove(-100.0);
      SerialBusManager::actionAll(4000);
      delay(3000);

      motors[0].queueMove(10.0);
      motors[1].queueMove(0.0);
      motors[2].queueMove(-60.0);
      motors[3].queueMove(0.0);
      motors[4].queueMove(-100.0);
      SerialBusManager::actionAll(4000);
      delay(3000);

      motors[0].queueMove(-60.0);
      motors[1].queueMove(0.0);
      motors[2].queueMove(0.0);
      motors[3].queueMove(0.0);
      motors[4].queueMove(-10.0);
      SerialBusManager::actionAll(4000);
      delay(3000);

      TCR.attachTool();
      TCL.attachTool();

      if(TCR.getToolStatus() == 1 || TCL.getToolStatus() == 1){
        Serial.println("Tools attached successfully!");
        TCR.lockTool();
        TCL.lockTool();
      } else {
        Serial.println("Tool attachment failed. Please check the tool changers.");
      }

      motors[0].queueMove(10.0);
      motors[1].queueMove(0.0);
      motors[2].queueMove(-60.0);
      motors[3].queueMove(0.0);
      motors[4].queueMove(-100.0);
      SerialBusManager::actionAll(4000);
      delay(3000);

      motors[0].queueMove(20.0);
      motors[1].queueMove(0.0);
      motors[2].queueMove(-60.0);
      motors[3].queueMove(0.0);
      motors[4].queueMove(-100.0);
      SerialBusManager::actionAll(4000);
      delay(3000);

      motors[0].queueMove(0.0);
      motors[1].queueMove(0.0);
      motors[2].queueMove(0.0);
      motors[3].queueMove(0.0);
      motors[4].queueMove(0.0);
      SerialBusManager::actionAll(4000);
      delay(3000);

      TCR.attachTool();
      TCL.attachTool();

      // go to the idle state after attaching
      robotState = IDLE;
      break;
    case DEPOSIT_STATE:
      motors[0].queueMove(20.0);
      motors[1].queueMove(0.0);
      motors[2].queueMove(-60.0);
      motors[3].queueMove(0.0);
      motors[4].queueMove(-100.0);
      SerialBusManager::actionAll(4000);
      delay(3000);

      motors[0].queueMove(10.0);
      motors[1].queueMove(0.0);
      motors[2].queueMove(-60.0);
      motors[3].queueMove(0.0);
      motors[4].queueMove(-100.0);
      SerialBusManager::actionAll(4000);
      delay(3000);

      motors[0].queueMove(-60.0);
      motors[1].queueMove(0.0);
      motors[2].queueMove(0.0);
      motors[3].queueMove(0.0);
      motors[4].queueMove(-10.0);
      SerialBusManager::actionAll(4000);
      delay(3000);

      TCR.depositTool();
      TCL.depositTool();

      motors[0].queueMove(10.0);
      motors[1].queueMove(0.0);
      motors[2].queueMove(-60.0);
      motors[3].queueMove(0.0);
      motors[4].queueMove(-100.0);
      SerialBusManager::actionAll(4000);
      delay(3000);

      motors[0].queueMove(20.0);
      motors[1].queueMove(0.0);
      motors[2].queueMove(-60.0);
      motors[3].queueMove(0.0);
      motors[4].queueMove(-100.0);
      SerialBusManager::actionAll(4000);
      delay(3000);

      motors[0].queueMove(0.0);
      motors[1].queueMove(0.0);
      motors[2].queueMove(0.0);
      motors[3].queueMove(0.0);
      motors[4].queueMove(0.0);
      SerialBusManager::actionAll(4000);
      delay(3000);

      TCR.attachTool();
      TCL.attachTool();

      // go to the idle state after depositing
      robotState = IDLE;

      break;
    case ELEVATOR_STATE:
      // move the elevator up and down
      robotState = IDLE;
      break;
  }
}

    // loop through every motor to test to twitch them
    
    // motors[0].queueMove(40.0);
    // motors[1].queueMove(40.0);
    // motors[2].queueMove(40.0);
    // motors[3].queueMove(40.0);
    // SerialBusManager::actionAll(4000);
    // delay(3000);

    // motors[0].queueMove(0.0);
    // motors[1].queueMove(0.0);
    // motors[2].queueMove(0.0);
    // motors[3].queueMove(0.0);
    // SerialBusManager::actionAll(4000);
    // delay(3000);


    // Serial.println("Running swap");

    // motors[0].queueMove(20.0);
    // motors[1].queueMove(0.0);
    // motors[2].queueMove(-60.0);
    // motors[3].queueMove(0.0);
    // motors[4].queueMove(-100.0);
    // SerialBusManager::actionAll(4000);
    // delay(3000);

    // motors[0].queueMove(10.0);
    // motors[1].queueMove(0.0);
    // motors[2].queueMove(-60.0);
    // motors[3].queueMove(0.0);
    // motors[4].queueMove(-100.0);
    // SerialBusManager::actionAll(4000);
    // delay(3000);

    // motors[0].queueMove(-60.0);
    // motors[1].queueMove(0.0);
    // motors[2].queueMove(0.0);
    // motors[3].queueMove(0.0);
    // motors[4].queueMove(-10.0);
    // SerialBusManager::actionAll(4000);
    // delay(3000);

    // // setServoPulse(33, 1300);  // eject

    // motors[0].queueMove(10.0);
    // motors[1].queueMove(0.0);
    // motors[2].queueMove(-60.0);

    // motors[3].queueMove(0.0);
    // motors[4].queueMove(-100.0);
    // SerialBusManager::actionAll(4000);
    // delay(3000);

    // motors[0].queueMove(20.0);
    // motors[1].queueMove(0.0);
    // motors[2].queueMove(-60.0);
    // motors[3].queueMove(0.0);
    // motors[4].queueMove(-100.0);
    // SerialBusManager::actionAll(4000);
    // delay(3000);

    // motors[0].queueMove(0.0);
    // motors[1].queueMove(0.0);
    // motors[2].queueMove(0.0);
    // motors[3].queueMove(0.0);
    // motors[4].queueMove(0.0);
    // SerialBusManager::actionAll(4000);
    // delay(3000);


