#include <Wire.h>
#include <HerkulexMotor.h>
#include <RPIComs.h>
#include <SerialBusManager.h>
#include <IMU.h>
#include <debug.h>
#include <ToolChanger.h>
#include <MAX30105.h>
#include <heartRate.h>

// the serial bus IDs for the different groups of motors, used for initialization and in the MotorRef table
enum SERIAL_BUS {
    BUS_CHEST = 3,
    BUS_L_ARM = 4,
    BUS_R_ARM = 5,
};

// define the number of motors in the system, used for array sizes and loops. Adjust as needed.
const uint8_t MOTOR_COUNT = 10;

// global variables for the robot state machine
bool leftArm = false;
bool rightArm = false;

// variables for oximeter readings
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE]; // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
bool bufferFull = false; // Whether the rates array has been filled at least once

float beatsPerMinute;
float beatAvg;

// robot state machine for the swapping sequence
enum ROBOT_STATE {
  IDLE,
  RUN_TO,
  END_EFFECTOR_STATUS,
  OXIMETER,
};

// initialize in IDLE state
ROBOT_STATE robotState = IDLE;

// arrays to hold motor positions, updated by the position read state machine, used for the position write state machine
uint16_t motorPositionsRaw[MOTOR_COUNT] = {0};
float motorPositions[MOTOR_COUNT] = {0};

// MotorRef table for parallelized position reads
MotorRef motorRefs[MOTOR_COUNT];

// motor intilization: {id, model, serial bus}
HerkulexMotor motors[MOTOR_COUNT] = {
  HerkulexMotor(7, MotorModel::DRS_0601, SERIAL_BUS::BUS_CHEST), // left shoulder 
  HerkulexMotor(11, MotorModel::DRS_0601, SERIAL_BUS::BUS_CHEST), // right shoulder
  HerkulexMotor(6, MotorModel::DRS_0602, SERIAL_BUS::BUS_L_ARM),
  HerkulexMotor(9, MotorModel::DRS_0601, SERIAL_BUS::BUS_L_ARM),
  HerkulexMotor(14, MotorModel::DRS_0201, SERIAL_BUS::BUS_L_ARM),
  HerkulexMotor(19, MotorModel::DRS_0201, SERIAL_BUS::BUS_L_ARM),
  HerkulexMotor(219, MotorModel::DRS_0602, SERIAL_BUS::BUS_R_ARM),
  HerkulexMotor(8, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_ARM),
  HerkulexMotor(17, MotorModel::DRS_0201, SERIAL_BUS::BUS_R_ARM),
  HerkulexMotor(26, MotorModel::DRS_0201, SERIAL_BUS::BUS_R_ARM),
};

// 
//
//

// status pin is analog
ToolChanger TCR(A13, 24, 600, 400, 1600); // status pin, servo pin, attach pos, lock pos, deposit pos
ToolChanger TCL(A12, 25, 850, 600, 1800);

// MAX30105 sensor for oximeter readings
MAX30105 particleSensor;

void setup(){
  Serial.begin(9600); // Open serial communications with computer
  Serial.println("Teensy Initializing...");

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
  for (int i = 0; i < MOTOR_COUNT; i++) motors[i].queueMove(0.0);
  SerialBusManager::actionAll(10000); // execute the queued moves with a 1 second timeout
  delay(2000);

  // initialize the toolchangers
  TCR.initialize();
  TCL.initialize();

  Serial.println("Setup complete");
}

void loop(){
  // state machine to read serial from python for the motor positions and toolchanger commands, and execute the swapping sequence
  static String lastCommand = "";
  
  switch (robotState){
    case IDLE:
      // wait for command from python to start the swapping sequence, then transition to RUNNING_POSITIONS
      if (Serial.available() > 0){
        String command = Serial.readStringUntil('\n');
        command.trim();
        lastCommand = command;
        
        if (command.startsWith("DL")){
          TCL.depositTool();
          Serial.println("M|COMPLETE");
        }
        else if (command.startsWith("DR")){
          TCR.depositTool();
          Serial.println("M|COMPLETE");
        }
        else if (command.startsWith("A") && !command.startsWith("AL") && !command.startsWith("AR")){
          // Generic attach command - attach both sides
          TCL.attachTool();
          TCR.attachTool();
          Serial.println("M|COMPLETE");
        }
        else if (command.startsWith("LL")){
          TCL.lockTool();
          Serial.println("M|COMPLETE");
        }
        else if (command.startsWith("LR")){
          TCR.lockTool();
          Serial.println("M|COMPLETE");
        }
        else if (command.startsWith("M")){
          robotState = RUN_TO;
        }
        else if (command.startsWith("OX")){
          robotState = OXIMETER;
        }
        else if (command.startsWith("E")){
          robotState = END_EFFECTOR_STATUS;
        }
      }
      break;

    case RUN_TO: {
      // Parse command format: M0|50|10|40|-9999|1000
      // Each position corresponds to a motor, -9999 means no movement
      // Last value is the timing in milliseconds
      
      // Remove 'M' prefix and parse the pipe-separated values
      String data = lastCommand.substring(1);
      float positions[MOTOR_COUNT];
      uint16_t timing = 0;
      
      // Parse positions and timing
      int lastIndex = 0;
      int motorIndex = 0;
      for (int i = 0; i <= data.length() && motorIndex <= MOTOR_COUNT; i++) {
        if (data[i] == '|' || i == data.length()) {
          String value = data.substring(lastIndex, i);
          if (motorIndex < MOTOR_COUNT) {
            positions[motorIndex] = value.toFloat();
          } else {
            // Last value is timing
            timing = value.toInt();
          }
          motorIndex++;
          lastIndex = i + 1;
        }
      }
      
      // Queue moves for all motors
      for (int i = 0; i < MOTOR_COUNT; i++) {
        if (positions[i] != -9999) {
          motors[i].queueMove(positions[i]);
        }
      }
      
      // Execute all moves with the specified timing
      SerialBusManager::actionAll(timing);
      delay(timing + 100); // Wait for moves to complete, add small buffer
      Serial.println("M|COMPLETE");
      
      // Return to IDLE
      robotState = IDLE;
      break;
    }

    case END_EFFECTOR_STATUS:
      // print the end effector status to serial for python to read, format: E|TCR_status|TCL_status
      Serial.print("E|");
      Serial.print(TCR.getToolStatus());
      Serial.print("|");
      Serial.println(TCL.getToolStatus());
      robotState = IDLE;
      break;

    case OXIMETER:
      // Reset tracking variables
      rateSpot = 0;
      lastBeat = 0;
      beatsPerMinute = 0;
      beatAvg = 0;
      
      // Initialize rates array
      for (byte i = 0; i < RATE_SIZE; i++) {
        rates[i] = 0;
      }

      if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("OX|ERROR");
        robotState = IDLE;
        break;
      }
      particleSensor.setup(); // Configure sensor with default settings
      delay(1000); // Wait for sensor to stabilize

      particleSensor.setPulseAmplitudeRed(0x0A); // Set Red LED to low brightness
      particleSensor.setPulseAmplitudeGreen(0x0A); // Set Green LED to low brightness
      long irValue = particleSensor.getIR();
      
      Serial.println("Waiting for finger on sensor...");
      while (irValue < 50000) { // Wait for a finger to be placed on the sensor
        irValue = particleSensor.getIR();
        delay(100);
      }
      Serial.println("Finger detected, reading oximeter...");
      
      // read the average for the time passed through the command from serial after OX
      // example OX|10000, where 10000 is the time in milliseconds to read for
      long startTime = millis();
      long readDuration = lastCommand.substring(3).toInt();
      while (millis() - startTime < readDuration) {
        irValue = particleSensor.getIR();
        if (checkForBeat(irValue)) {
          long currentTime = millis();
          long beatInterval = currentTime - lastBeat;
          lastBeat = currentTime;

          beatsPerMinute = 60 / (beatInterval / 1000.0);
          if (beatsPerMinute < 255 && beatsPerMinute > 20) {
            rates[rateSpot] = (byte)beatsPerMinute;
            rateSpot++;
            rateSpot %= RATE_SIZE;

            beatAvg = 0;
            for (byte i = 0; i < RATE_SIZE; i++) {
              beatAvg += rates[i];
            }
            beatAvg /= RATE_SIZE;
          }
        }
        delay(100);
      }
      // print the average heart rate to serial for python to read, format: OX|avg_bpm
      Serial.print("OX|");
      Serial.println(beatAvg);
      robotState = IDLE;
      break;
  }
}