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

  moveToZeroPositions();
  
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

  Serial.println("Waiting for any key... ");
  waitForEnter();
}


void walkingGait() {

  // left front
  HerkulexMotor::motorRefQueueMove(allMotors[2],  -40.0);   // left thigh lift
  HerkulexMotor::motorRefQueueMove(allMotors[3],  -20.0);   // left knee
  HerkulexMotor::motorRefQueueMove(allMotors[4],  -10.0);   // left ankle
  HerkulexMotor::motorRefQueueMove(allMotors[11],  30.0);   // left pec
  HerkulexMotor::motorRefQueueMove(allMotors[16],  7.0);   // shoulder in
  HerkulexMotor::motorRefQueueMove(allMotors[17], -20.0);   // bicep pos curl


  // right back
  HerkulexMotor::motorRefQueueMove(allMotors[7], -10.0);   // right thigh lift
  HerkulexMotor::motorRefQueueMove(allMotors[8],   0.0);   // right knee
  HerkulexMotor::motorRefQueueMove(allMotors[9], -10.0);   // right ankle
  HerkulexMotor::motorRefQueueMove(allMotors[10], 20.0);   // right pec
  HerkulexMotor::motorRefQueueMove(allMotors[21], 7.0);   // shoulder out
  HerkulexMotor::motorRefQueueMove(allMotors[22],  0.0);   // no curl
  SerialBusManager::actionAll(2000);
  delay(2000);

  // left back
  HerkulexMotor::motorRefQueueMove(allMotors[3],    0.0);   // left knee
  HerkulexMotor::motorRefQueueMove(allMotors[2],   30.0);   // left thigh lift
  HerkulexMotor::motorRefQueueMove(allMotors[4],   -0.0);   // left ankle
  HerkulexMotor::motorRefQueueMove(allMotors[11], -20.0);   // left pec
  HerkulexMotor::motorRefQueueMove(allMotors[16], -7.0);   // shoulder out
  HerkulexMotor::motorRefQueueMove(allMotors[17],  0.0);    //  no curl

  // right front
  HerkulexMotor::motorRefQueueMove(allMotors[7],   40.0);   // right thigh lift
  HerkulexMotor::motorRefQueueMove(allMotors[8],   20.0);   // right knee
  HerkulexMotor::motorRefQueueMove(allMotors[9],   10.0);   // right ankle
  HerkulexMotor::motorRefQueueMove(allMotors[10], -30.0);   // right pec
  HerkulexMotor::motorRefQueueMove(allMotors[21], -7.0);   // shoulder out
  HerkulexMotor::motorRefQueueMove(allMotors[22], -10.0);   // curl in
  SerialBusManager::actionAll(2000);
  delay(2000);
}





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



