#include <HerkulexMotor.h>
#include <RPIComs.h>
#include <SerialBusManager.h>
#include <IMU.h>
#include <debug.h>
#include <motorDefs.h>

void waitForEnter() {
  Serial.println("Press Any key to continue... (Which key is the Any key?)");
  while (Serial.available() == 0) {}
  while (Serial.available() > 0) Serial.read();
}

enum STATE {
  READING_FROM_RPI,
  SENDING_TO_RPI,
  SETTING_MOTOR_POS,
  READING_ROBOT_STATE,
  IDLE,
  STOP,
};

uint8_t robotState = IDLE;


uint32_t loopElapsedMicros;
uint32_t readStartTime;
uint32_t lastPacketTime;

float motorPositions[MOTOR_COUNT] = {0};
float RPIMotorInputs[MOTOR_COUNT] = {0};
uint16_t motorPositionsRaw[MOTOR_COUNT] = {0};
char imuPacket[32]; 

RPIComs rpi = RPIComs();
IMU imu1;  // uses sensorID=55 and address=0x28 automatically

void moveToZeroPositions() {

  HerkulexMotor::motorRefQueueMove(allMotors, motorPositions, TOTAL_COUNT);
  SerialBusManager::actionAll(2000);
  delay(2000);

  HerkulexMotor::motorRefQueueMove(allMotors[0],  8.0);
  HerkulexMotor::motorRefQueueMove(allMotors[5], -8.0);
  SerialBusManager::actionAll(2000);
  delay(2000);
  

}

void jog(int n) {
  Serial.println(n);
  HerkulexMotor::motorRefQueueMove(allMotors[n], 15);
  SerialBusManager::actionAll(1000);
  delay(1000);
  HerkulexMotor::motorRefQueueMove(allMotors[n], 0);
  SerialBusManager::actionAll(1000);
  delay(1000);
}

// moves motor n to a, then b, then back to 0, logging each step
void sweepMotor(int n, const char* jointName, const char* aName, float a, const char* bName, float b) {
  Serial.print("[legTest] ");
  Serial.print(jointName);
  Serial.print(" (allMotors[");
  Serial.print(n);
  Serial.print("], bus ");
  Serial.print(allMotors[n].busId);
  Serial.print(", servo ");
  Serial.print(allMotors[n].servoId);
  Serial.println(")");

  Serial.print("  -> ");
  Serial.print(aName);
  Serial.print(": ");
  Serial.println(a);
  HerkulexMotor::motorRefQueueMove(allMotors[n], a);
  SerialBusManager::actionAll(1000);
  delay(1000);

  Serial.print("  -> ");
  Serial.print(bName);
  Serial.print(": ");
  Serial.println(b);
  HerkulexMotor::motorRefQueueMove(allMotors[n], b);
  SerialBusManager::actionAll(1000);
  delay(1000);

  Serial.println("  -> back to 0");
  HerkulexMotor::motorRefQueueMove(allMotors[n], 0);
  SerialBusManager::actionAll(1000);
  delay(1000);
}

// exercises each leg joint one at a time: hip fwd/back, hip twist in/out, knee fwd/back, ankle fwd/back
// relies on motorDefs order: leg index +1 = hip twist, +2 = hip (thigh), +3 = knee, +4 = ankle
// left leg starts at allMotors[0], right leg at allMotors[5]
// sign convention taken from walkingGait(): left forward is negative, right forward is positive
void testLegJoints() {
  const float HIP_DEG   = 15.0;
  const float TWIST_DEG = 10.0;
  const float KNEE_DEG  = 15.0;
  const float ANKLE_DEG = 10.0;

  const int legStart[2]   = {0, 5};
  const char* legName[2]  = {"LEFT", "RIGHT"};
  const float fwdSign[2]  = {-1.0, 1.0};

  Serial.println("[legTest] ===== starting leg joint test =====");

  for (int leg = 0; leg < 2; leg++) {
    int base = legStart[leg];
    float s = fwdSign[leg];

    Serial.print("[legTest] ----- ");
    Serial.print(legName[leg]);
    Serial.println(" LEG -----");

    sweepMotor(base + 2, "hip forward/back", "forward", s * HIP_DEG,   "back", -s * HIP_DEG);
    sweepMotor(base + 1, "hip twist in/out", "in",      s * TWIST_DEG, "out",  -s * TWIST_DEG);
    sweepMotor(base + 3, "knee forward/back", "forward", s * KNEE_DEG,  "back", -s * KNEE_DEG);
    sweepMotor(base + 4, "ankle forward/back", "forward", s * ANKLE_DEG, "back", -s * ANKLE_DEG);

    Serial.print("[legTest] ");
    Serial.print(legName[leg]);
    Serial.println(" LEG done");
  }

  Serial.println("[legTest] ===== leg joint test complete =====");
}

void setup(){
  delay(2000);                  // a delay to have time for serial monitor opening on platformio after uploading
  Serial.begin(1000000);        // open serial communications with computer

  Serial.println("Beginning... ");
  delay(2000);

  if (!imu1.begin()) {
      Serial.println("ERROR: IMU not detected. Check wiring!");
      while (1);
  }

  // create the big all motors motordef
  createAvaMotorDef();

  // initialize all serial buses
  SerialBusManager::createBus(SERIAL_BUS::BUS_L_LEG);
  SerialBusManager::createBus(SERIAL_BUS::BUS_R_LEG);
  SerialBusManager::createBus(SERIAL_BUS::BUS_CHEST);
  SerialBusManager::createBus(SERIAL_BUS::BUS_L_ARM);
  SerialBusManager::createBus(SERIAL_BUS::BUS_R_ARM);
  SerialBusManager::startAllBuses(BAUD_RATE::SPEED_115K);
  delay(1000);
  SerialBusManager::initAllMotors();
  delay(1000);
  SerialBusManager::torqueOnAllMotors();
  delay(2000);

  //moveToZeroPositions();
  
  SerialBusManager::infoAllMotors(allMotors, TOTAL_COUNT);

  SerialBusManager::requestAllPositions(allMotors, motorPositionsRaw, MOTOR_COUNT);

  while(!SerialBusManager::isDoneCollecting()){
    SerialBusManager::tick(allMotors, motorPositionsRaw, MOTOR_COUNT);
  }

  for (int i = 0; i < MOTOR_COUNT; i++){
    motorPositions[i] = HerkulexMotor::motorRefRawToDegs(allMotors[i], motorPositionsRaw[i]);          
    Serial.print("Bus ID: ");
    Serial.print(allMotors[i].busId);
    Serial.print("    Motor ID: ");
    Serial.print(allMotors[i].servoId);
    Serial.print(" Position (deg): ");
    Serial.println(motorPositions[i]);
  }

  // Serial.println("Waiting for any key... ");
  // waitForEnter();
  
  while (true) {
    while (Serial.available() == 0) {}
    int n = Serial.parseInt();
    while (Serial.available() > 0) Serial.read();  // clear leftover newline
    jog(n);
  }
}



// void walkingGait() {

//   // left front
//   HerkulexMotor::motorRefQueueMove(allMotors[2],  -40.0);   // left thigh lift
//   HerkulexMotor::motorRefQueueMove(allMotors[3],  -20.0);   // left knee
//   HerkulexMotor::motorRefQueueMove(allMotors[4],  -10.0);   // left ankle
//   HerkulexMotor::motorRefQueueMove(allMotors[11],  30.0);   // left pec
//   HerkulexMotor::motorRefQueueMove(allMotors[16],  7.0);   // shoulder in
//   HerkulexMotor::motorRefQueueMove(allMotors[17], -20.0);   // bicep pos curl


//   // right back
//   HerkulexMotor::motorRefQueueMove(allMotors[7], -10.0);   // right thigh lift
//   HerkulexMotor::motorRefQueueMove(allMotors[8],   0.0);   // right knee
//   HerkulexMotor::motorRefQueueMove(allMotors[9], -10.0);   // right ankle
//   HerkulexMotor::motorRefQueueMove(allMotors[10], 20.0);   // right pec
//   HerkulexMotor::motorRefQueueMove(allMotors[21], 7.0);   // shoulder out
//   HerkulexMotor::motorRefQueueMove(allMotors[22],  0.0);   // no curl
//   SerialBusManager::actionAll(2000);
//   delay(2000);

//   // left back
//   HerkulexMotor::motorRefQueueMove(allMotors[3],    0.0);   // left knee
//   HerkulexMotor::motorRefQueueMove(allMotors[2],   30.0);   // left thigh lift
//   HerkulexMotor::motorRefQueueMove(allMotors[4],   -0.0);   // left ankle
//   HerkulexMotor::motorRefQueueMove(allMotors[11], -20.0);   // left pec
//   HerkulexMotor::motorRefQueueMove(allMotors[16], -7.0);   // shoulder out
//   HerkulexMotor::motorRefQueueMove(allMotors[17],  0.0);    //  no curl

//   // right front
//   HerkulexMotor::motorRefQueueMove(allMotors[7],   40.0);   // right thigh lift
//   HerkulexMotor::motorRefQueueMove(allMotors[8],   20.0);   // right knee
//   HerkulexMotor::motorRefQueueMove(allMotors[9],   10.0);   // right ankle
//   HerkulexMotor::motorRefQueueMove(allMotors[10], -30.0);   // right pec
//   HerkulexMotor::motorRefQueueMove(allMotors[21], -7.0);   // shoulder out
//   HerkulexMotor::motorRefQueueMove(allMotors[22], -10.0);   // curl in
//   SerialBusManager::actionAll(2000);
//   delay(2000);
// }

// void hardcodedStanding() {
//   // move knees first — positions in allMotors: 3, 8
//   waitForEnter();
//   HerkulexMotor::motorRefQueueMove(allMotors[3], -20.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[8], 20.0);
//   waitForEnter();
//   HerkulexMotor::motorRefQueueMove(allMotors[1], 0.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[2], 0.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[6], 0.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[7], 0.0);
//   SerialBusManager::actionAll(2000);
//   delay(2000);

//   // pelvis — positions in allMotors: 0, 5
//   waitForEnter();
//   HerkulexMotor::motorRefQueueMove(allMotors[0], 0.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[5], 0.0);
//   SerialBusManager::actionAll(2000);
//   delay(2000);
//   waitForEnter();

//   // chest — positions in allMotors: 10, 11, 12, 13, 14
//   waitForEnter();
//   HerkulexMotor::motorRefQueueMove(allMotors[10], 0.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[11], 0.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[12], 0.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[13], 0.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[14], 0.0);
//   SerialBusManager::actionAll(2000);
//   delay(2000);
//   waitForEnter();

//   // arms/neck — positions in allMotors: 15-22
//   waitForEnter();
//   HerkulexMotor::motorRefQueueMove(allMotors[15], 0.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[16], 0.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[17], 0.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[18], 0.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[19], 0.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[20], 0.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[21], 0.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[22], 0.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[23], 0.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[24], 0.0);
//   SerialBusManager::actionAll(2000);
//   delay(2000);

//   // then do the rest of the legs — positions in allMotors: 1, 2, 4, 6, 7, 9
//   waitForEnter();
//   HerkulexMotor::motorRefQueueMove(allMotors[1], 5.7);
//   HerkulexMotor::motorRefQueueMove(allMotors[2], -9.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[4], 12.0);

//   HerkulexMotor::motorRefQueueMove(allMotors[6], -5.7);
//   HerkulexMotor::motorRefQueueMove(allMotors[7], 9.0);
//   HerkulexMotor::motorRefQueueMove(allMotors[9], -12.0);
//   SerialBusManager::actionAll(2000);
//   delay(2000);

//   // pelvis — positions in allMotors: 0, 5
//   waitForEnter();
//   HerkulexMotor::motorRefQueueMove(allMotors[0], 6.0);
//   SerialBusManager::actionAll(2000);
//   delay(2000);
//   waitForEnter();
//   HerkulexMotor::motorRefQueueMove(allMotors[5], -6.0);
//   SerialBusManager::actionAll(2000);
//   delay(2000);

  // // chest — positions in allMotors: 10, 11, 12, 13, 14
  // waitForEnter();
  // HerkulexMotor::motorRefQueueMove(allMotors[10], 0.0);
  // SerialBusManager::actionAll(2000);
  // delay(2000);
  // waitForEnter();
  // HerkulexMotor::motorRefQueueMove(allMotors[11], 0.0);
  // SerialBusManager::actionAll(2000);
  // delay(2000);
  // waitForEnter();
  // HerkulexMotor::motorRefQueueMove(allMotors[12], 0.0);
  // SerialBusManager::actionAll(2000);
  // delay(2000);
  // waitForEnter();
  // HerkulexMotor::motorRefQueueMove(allMotors[13], 0.0);
  // SerialBusManager::actionAll(2000);
  // delay(2000);
  // waitForEnter();
  // HerkulexMotor::motorRefQueueMove(allMotors[14], -10.0);
  // SerialBusManager::actionAll(2000);
  // delay(2000);

  // // arms/neck — positions in allMotors: 15-22 (23, 24 removed — lost motors on left arm)
  // waitForEnter();
  // HerkulexMotor::motorRefQueueMove(allMotors[15], 0.0);
  // SerialBusManager::actionAll(2000);
  // delay(2000);
  // waitForEnter();
  // // everything here up to bad
  // // HerkulexMotor::motorRefQueueMove(allMotors[16], 0.0);
  // // SerialBusManager::actionAll(2000);
  // // delay(2000);
  // // waitForEnter();
  // HerkulexMotor::motorRefQueueMove(allMotors[17], 0.0);
  // SerialBusManager::actionAll(2000);
  // delay(2000);
  // waitForEnter();
  // HerkulexMotor::motorRefQueueMove(allMotors[18], 0.0);
  // SerialBusManager::actionAll(2000);
  // delay(2000);
  // waitForEnter();
  // HerkulexMotor::motorRefQueueMove(allMotors[19], 0.0);
  // SerialBusManager::actionAll(2000);
  // delay(2000);
  // waitForEnter();
  // HerkulexMotor::motorRefQueueMove(allMotors[20], 0.0);
  // SerialBusManager::actionAll(2000);
  // delay(2000);
  // waitForEnter();

  // myMotor.setPos(0.0);
  // HerkulexMotor::motorRefQueueMove(allMotors[21], 0.0);
  // SerialBusManager::actionAll(2000);
  // delay(2000);
  // waitForEnter();
  // HerkulexMotor::motorRefQueueMove(allMotors[22], 0.0);
  // SerialBusManager::actionAll(2000);
  // delay(2000);
  // waitForEnter();

  // myMotor.setPos(0);
  // delay(2000);
  // waitForEnter();
// }


void loop(){
  // Serial.println("Loop running");
  // Serial.flush();
  int rpi_status = rpi.uartRead(); // continuously read from the pi
  // Serial.print("RPI status: ");
  // Serial.println(rpi_status);
  // Serial.flush();
  // if (millis() - lastPacketTime > 500000) { // if it's been more than 5 seconds since we received a packet, go to idle state
  //   Serial.println("No packet received for a while, stopping...");
  //   delay(5);
  //   robotState = STOP;
  // }

  switch (robotState){
    case(READING_FROM_RPI):
    {
      Serial.println("State: READING_FROM_RPI");
      if (rpi_status == -1){
        break;
      }
      Serial.println("Reading from RPI...!!!!");
      // If a packet arrived, handle it
      const uint8_t* pkt = rpi.getPacket();
      if (pkt != nullptr) {
         Serial.print("Packet received from RPI: ");
        Serial.println(pkt[0]);
        lastPacketTime = millis();

        // copy packet to avoid buffer overwrite
        int16_t buffer[NUM_INT16];        
        memcpy(buffer, pkt, PACKET_SIZE);

        int16_t flag = buffer[0];
        Serial.print("Flag byte: ");
        Serial.println(flag);
        Serial.flush();

        for (int i = 0; i < MOTOR_COUNT; i++) {
            RPIMotorInputs[i] = buffer[1 + i] / 100.0f;
        }
        // done receiving packet now, we set position
        if (flag == START_BYTE) { // 1 is start
          // moveToZeroPositions();  // uncomment for later implementation
          robotState = READING_FROM_RPI;
          break;
        } else if (flag == STOP_BYTE) { // -1 is stop
          robotState = STOP;
          break;
        } else { 
          robotState = SETTING_MOTOR_POS;
        }
      }

      break;
    }
    case(SETTING_MOTOR_POS):
    {
      if (rpi_status == -1){
        robotState = STOP;
        break;
      }
      if (true) { //only for testing
        for (int i = 0; i < MOTOR_COUNT; i++) {
            Serial.print("Received from RPI - Motor ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(RPIMotorInputs[i]);
        }
        robotState = READING_ROBOT_STATE;
        break;
      }
      // // queue all motors in a loop
      // // ensure this lines up with the correct motors
      HerkulexMotor::motorRefQueueMove(allMotors, RPIMotorInputs, MOTOR_COUNT);
      SerialBusManager::actionAll(10);

      robotState = READING_ROBOT_STATE;
      SerialBusManager::requestAllPositions(allMotors, motorPositionsRaw, MOTOR_COUNT);
      Serial.println("Requesting position and switching to READIING_ROBOT_STATE");
      imu1.requestUpdate();
      readStartTime = micros();
      break;
    }
    case(READING_ROBOT_STATE):
    {
      if (true) { //only for testing
        char* test_packet = (char*) malloc(PACKET_SIZE);
        memset(test_packet, 0, PACKET_SIZE);
        for (int i = 0; i < MOTOR_COUNT; i++) {
            motorPositions[i] = 0;
          }
          rpi.enqueueTXPacket(test_packet);
          rpi.uartSend();
          robotState = READING_FROM_RPI;
          free(test_packet);
          break;
      }
      SerialBusManager::tick(allMotors, motorPositionsRaw, MOTOR_COUNT);
      imu1.collectUpdate();
      if (SerialBusManager::isDoneCollecting() && imu1.isDoneCollecting()){
        // put data togehter into one packet 
        // send packet to RPI
        loopElapsedMicros = micros() - readStartTime;
        Serial.print("Elapsed time: ");
        Serial.print(loopElapsedMicros);
        Serial.println(" microseconds");

        // //for (int i = 0; i < MOTOR_COUNT; i++){
        //   // allmotors is an array of motor refs, as opposed to holding motor objects like motors did before
        //   motorPositions[i] = HerkulexMotor::motorRefRawToDegs(allMotors[i], motorPositionsRaw[i]);          
        //   Serial.print("Bus ID: ");
        //   Serial.print(allMotors[i].busId);
        //   Serial.print("    Motor ID: ");
        //   Serial.print(allMotors[i].servoId);
        //   Serial.print(" Position (deg): ");
        //   Serial.println(motorPositions[i]);
        // }

        imu1.formatPacket(imuPacket, sizeof(imuPacket));

        // Serial.print("IMU: ");
        // Serial.println(imuPacket);
        // Serial.print("IMU cal_sys: ");
        // Serial.println(imu1.getCalSys());

        //not yet implemented: compile all observations into full_packet
        // full_packet = (char*) malloc(PACKET_SIZE);
        // memcpy(full_packet, motorPositions, MOTOR_COUNT * sizeof(float));
        // rpi.enqueueTXPacket(full_packet);
        // rpi.uartSend();

        robotState = READING_FROM_RPI;
      };
      break;
    }
    case(IDLE):
    {
      for(int i = 0; i < 5; i++){
        walkingGait();
      }
      Serial.println("Press any key to walk again");
      waitForEnter();
      break;
    }
    case(STOP):
    {
      delay(1000);
    }
  }
}



