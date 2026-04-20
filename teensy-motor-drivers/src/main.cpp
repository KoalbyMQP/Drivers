#include <HerkulexMotor.h>
#include <RPIComs.h>
#include <SerialBusManager.h>
#include <IMU.h>
#include <debug.h>
#include <motorDefs.h>

enum STATE {
  READING_FROM_RPI,
  SETTING_MOTOR_POS,
  READING_ROBOT_STATE,
  IDLE,
  STOP,
  START,
};

uint8_t robotState = SETTING_MOTOR_POS;

uint32_t elapsedMicros;
uint32_t readStartTime;
elapsedMillis imuTimer; 

const uint8_t MOTOR_COUNT = TOTAL_COUNT;
const uint8_t PACKET_SIZE = 192;   // this depends on the number of motors used, HOW???


uint16_t motorPositionsRaw[MOTOR_COUNT] = {0};
float motorPositions[MOTOR_COUNT] = {0};
float RPIMotorInputs[MOTOR_COUNT] = {0};

HerkulexMotor myMotor = HerkulexMotor(3, DRS_0601, BUS_R_LEG);


RPIComs rpi = RPIComs();
IMU imu1;  // uses sensorID=55 and address=0x28 automatically

void setup(){
  delay(2000);                  // a delay to have time for serial monitor opening on platformio after uploading
  Serial.begin(9600);           // open serial communications with computer

  Serial.println("Beginning... ");

  Serial8.begin(1000000); // begin serial communication with the raspberry pi, this has been changed to Serial 8 instead of 1
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
  
  
  // SerialBusManager::torqueOnAllMotors();

  // The following snippet is to test the wiring by looking for all the motor ids on each bus
  // Serial.println();
  // Serial.println("Scanning BUS_L_LEG...");
  // find_all_motors_on_bus(SerialBusManager::getBus(BUS_L_LEG));

  // Serial.println();
  // Serial.println("Scanning BUS_R_LEG..");
  // find_all_motors_on_bus(SerialBusManager::getBus(BUS_R_LEG));

  // Serial.println();
  // Serial.println("Scanning BUS_CHEST..");
  // find_all_motors_on_bus(SerialBusManager::getBus(BUS_CHEST));

  // Serial.println();
  // Serial.println("Scanning BUS_L_ARM...");
  // find_all_motors_on_bus(SerialBusManager::getBus(BUS_L_ARM));

  // Serial.println();
  // Serial.println("Scanning BUS_R_ARM..");
  // find_all_motors_on_bus(SerialBusManager::getBus(BUS_R_ARM));

  // SerialBusManager::infoAllMotors(allMotors, TOTAL_COUNT);
}


void loop(){
  switch (robotState){
    case(READING_FROM_RPI):
    {
      if (rpi.uartRead() == -1){
        robotState = STOP;
        break;
      }

      // If a packet arrived, handle it
      const char* pkt = rpi.getPacket();
      if (pkt != nullptr) {

        // copy packet to avoid buffer overwrite
        char buffer[PACKET_SIZE];        
        strncpy(buffer, pkt, sizeof(buffer));
        buffer[sizeof(buffer)-1] = '\0';


        char* token = strtok(buffer, ",");
        for (int i = 0; i < MOTOR_COUNT && token != nullptr; i++) {
            RPIMotorInputs[i] = atof(token);
            token = strtok(nullptr, ",");
        }
        // done receiving packet now, we set position
        robotState = SETTING_MOTOR_POS;
      }

      break;
    }
    case(SETTING_MOTOR_POS):
    {
      if (rpi.uartRead() == -1){
        robotState = STOP;
        break;
      }
      // // queue all motors in a loop
      // // does this line up with the correct motors?
      // // for (int i = 0; i < MOTOR_COUNT; i++) {
      // //   if (count % 2){
      // //     motors[i].queueMove(dummyMotorInputs[i]);
      // //   } else {
      // //     motors[i].queueMove(dummyMotorInputsTwo[i]);
      // //   }
      // // }
      // SerialBusManager::actionAll(10);

      robotState = READING_ROBOT_STATE;
      SerialBusManager::requestAllPositions(allMotors, motorPositionsRaw, MOTOR_COUNT);
      Serial.println("Requesting position and switching to READIING_ROBOT_STATE");
      readStartTime = micros();
      // //imu1.requestRead();
      
      break;
    }
    case(READING_ROBOT_STATE):
    {
      SerialBusManager::tick(allMotors, motorPositionsRaw, MOTOR_COUNT);
      // imu1.tick(imuReadBuffer, imuReadBufferSize); // imuReadBufferSize should be a const, it's defined somewhere in the IMU stack
      if (SerialBusManager::isDoneCollecting()){ // && imu1.doneCollecting()){
        // put data togehter into one packet
        // send packet to RPI
        elapsedMicros = micros() - readStartTime;
        Serial.print("Elapsed time: ");
        Serial.print(elapsedMicros);
        Serial.println(" microseconds");

        for (int i = 0; i < 5; i++){
          // allmotors is an array of motor refs, as opposed to holding motor objects like motors did before
          motorPositions[i] = HerkulexMotor::motorRefRawToDegs(allMotors[i], motorPositionsRaw[i]);          
          Serial.print("Bus ID: ");
          Serial.print(allMotors[i].busId);
          Serial.print("    Motor ID: ");
          Serial.print(allMotors[i].servoId);
          Serial.print(" Position (deg): ");
          Serial.println(motorPositions[i]);
        }
        delay(4000);
        robotState = SETTING_MOTOR_POS;
      };
      break;
    }
    case(IDLE):
    {
      delay(1000);
      break;
    }
    case(STOP):
    {
      delay(1000);
      break;
    }
  }
}



// THIS IS RELIANT ON MOTORDEFS. IF YOU CHANGE MOTORDEFS, YOU MUST UPDATE THIS
void moveToZeroPositions(){

  // move knees first  ids: 3, 8 on respective buses
  
  // then do the rest of the legs ids: 1, 2, 4, 6, 7, 9 on respective buses


  // pelvis ids: 0, 5 on respective buses

  // then do chest ids:  10, 11, 12, 13, 14 on respective buses

  // then do arms/neck ids: 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25 on respective buses

}