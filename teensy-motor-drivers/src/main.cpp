#include <Wire.h>
#include <HerkulexMotor.h>
#include <RPIComs.h>
#include <SerialBusManager.h>
#include <IMU.h>
#include <debug.h>
#include <ToolChanger.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <spo2_algorithm.h>

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

// Oximeter state machine variables
bool oximeterMeasuring = false;
unsigned long oximeterStartTime;
unsigned long oximeterDuration = 5000; // Default 5 seconds in milliseconds
const int OXIMETER_SAMPLE_RATE = 25; // 25 samples per second
const int OXIMETER_BUFFER_SIZE = 100; // 4 seconds of data at 25 Hz
uint32_t irBuffer[OXIMETER_BUFFER_SIZE];
uint32_t redBuffer[OXIMETER_BUFFER_SIZE];
int32_t bufferIndex = 0;
int32_t spo2 = 0;
int8_t spo2Valid = 0;
int32_t heartRate = 0;
int8_t heartRateValid = 0;
float oximeterAvgBPM = 0;
int32_t oximeterAvgSPO2 = 0;

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
ToolChanger TCR(A13, 24, 600, 200, 1600); // status pin, servo pin, attach pos, lock pos, deposit pos
ToolChanger TCL(A12, 25, 850, 400, 1800);

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
          // Parse OX command format: OX|duration_ms
          int pipeIndex = command.indexOf('|');
          if (pipeIndex != -1) {
            String durationStr = command.substring(pipeIndex + 1);
            oximeterDuration = durationStr.toInt();
            if (oximeterDuration <= 0) oximeterDuration = 5000; // Default to 5 seconds if invalid
          } else {
            oximeterDuration = 5000; // Default to 5 seconds if no parameter
          }
          robotState = OXIMETER;
        }
        else if (command.startsWith("E")){
          robotState = END_EFFECTOR_STATUS;
        }
      }
      break;

    case RUN_TO: {
      // Parse command format: M0|50|10|40|-9999|1000
      String data = lastCommand.substring(1);
      float positions[MOTOR_COUNT];
      uint16_t timing = 0;
      
      // Parse positions and timing
      int lastIndex = 0;
      int motorIndex = 0;
      for (unsigned int i = 0; i <= data.length() && motorIndex <= MOTOR_COUNT; i++) {
        if (i == data.length() || data[i] == '|') {
          String value = data.substring(lastIndex, i);
          if (motorIndex < MOTOR_COUNT) {
            positions[motorIndex] = value.toFloat();
          } else {
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

    case OXIMETER: {
      long irValue = particleSensor.getIR();
      
      // Wait for finger detection
      if (irValue > 50000) {
        if (!oximeterMeasuring) {
          // Start new measurement
          oximeterMeasuring = true;
          oximeterStartTime = millis();
          bufferIndex = 0;
          oximeterAvgBPM = 0;
          oximeterAvgSPO2 = 0;
          // Reset all buffers
          for (int i = 0; i < OXIMETER_BUFFER_SIZE; i++) {
            irBuffer[i] = 0;
            redBuffer[i] = 0;
          }
          // Initialize sensor readings
          spo2 = 0;
          spo2Valid = 0;
          heartRate = 0;
          heartRateValid = 0;
          Serial.println("OX|RECORDING_STARTED");
        }
        
        if (oximeterMeasuring && bufferIndex < OXIMETER_BUFFER_SIZE) {
          // Collect samples for the specified duration
          if (millis() - oximeterStartTime < oximeterDuration) {
            // Get IR and Red LED readings
            uint32_t red = particleSensor.getRed();
            
            irBuffer[bufferIndex] = irValue;
            redBuffer[bufferIndex] = red;
            bufferIndex++;
          } else {
            // 5 seconds elapsed, process the data
            if (bufferIndex > 0) {
              // Calculate heart rate and SpO2
              maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferIndex, redBuffer, &spo2, &spo2Valid, &heartRate, &heartRateValid);
              
              oximeterAvgBPM = (float)heartRate;
              oximeterAvgSPO2 = spo2;
              
              // Output recording finished and results
              Serial.println("OX|RECORDING_FINISHED");
              Serial.print("OX|");
              Serial.print(oximeterAvgBPM);
              Serial.print("|");
              Serial.println(oximeterAvgSPO2);
            } else {
              Serial.println("OX|ERROR|No samples collected");
            }
            
            // Reset for next measurement
            oximeterMeasuring = false;
            bufferIndex = 0;
            robotState = IDLE;
          }
        }
      } else {
        // No finger detected
        if (oximeterMeasuring) {
          Serial.println("OX|ERROR|Finger removed");
          oximeterMeasuring = false;
          bufferIndex = 0;
        }
        Serial.println("OX|WAITING_FOR_FINGER");
        delay(500); // Avoid flooding serial output
      }
      break;
    }
  }
}