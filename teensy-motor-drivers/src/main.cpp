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
  IDLE // allows us to not run code in main
};

uint8_t robotState = IDLE;

uint32_t elapsedMicros;
uint32_t startTime;
elapsedMillis imuTimer; 

const uint8_t MOTOR_COUNT = BUS_L_LEG_COUNT;
const uint8_t PACKET_SIZE = 192;   // this depends on the number of motors used, HOW???


uint16_t motorPositionsRaw[MOTOR_COUNT] = {0};
float motorPositions[MOTOR_COUNT] = {0};
float RPIMotorInputs[MOTOR_COUNT] = {0};

RPIComs rpi = RPIComs();
IMU imu1;  // uses sensorID=55 and address=0x28 automatically

void setup(){
  delay(2000);                  // a delay to have time for serial monitor opening on platformio after uploading
  Serial.begin(9600);           // open serial communications with computer

  Serial.println("Beginning... ");

  Serial8.begin(9600); // begin serial communication with the raspberry pi, this has been changed to Serial 8 instead of 1
  delay(2000);

  if (!imu1.begin()) {
      Serial.println("ERROR: IMU not detected. Check wiring!");
      while (1);
  }

  // create the big all motors motordef
  createAvaMotorDef();

  // initialize all serial buses
  SerialBusManager::createBus(SERIAL_BUS::BUS_L_LEG);
  SerialBusManager::startAllBuses(BAUD_RATE::SPEED_115K);
  
  delay(2000);
  
  Serial.println();
  Serial.println("Scanning BUS_L_LEG...");
  find_all_motors_on_bus(SerialBusManager::getBus(BUS_L_LEG));
  
  // SerialBusManager::infoAllMotors(rightLegMotors, BUS_R_LEG_COUNT);
  // SerialBusManager::initAllMotors();
}


void loop(){
  switch (robotState){
    case(READING_FROM_RPI):
    {
      // rpi.uartRead();
      // // If a packet arrived, handle it
      // const char* pkt = rpi.getPacket();
      // if (pkt != nullptr) {

      //   // copy packet to avoid buffer overwrite
      //   char buffer[PACKET_SIZE];        
      //   strncpy(buffer, pkt, sizeof(buffer));
      //   buffer[sizeof(buffer)-1] = '\0';


      //   char* token = strtok(buffer, ",");
      //   for (int i = 0; i < MOTOR_COUNT && token != nullptr; i++) {
      //       RPIMotorInputs[i] = atof(token);
      //       token = strtok(nullptr, ",");
      //   }
      //   // done receiving packet now, we set position
      //   robotState = SETTING_MOTOR_POS;
      // }
      break;
    }
    case(SETTING_MOTOR_POS):
    {
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

      // robotState = READING_ROBOT_STATE;
      // // SerialBusManager::requestAllPositions(allGroups, motorPositionsRaw, MOTOR_COUNT);
      // startTime = micros();
      // //imu1.requestRead();
      
      break;
    }
    case(READING_ROBOT_STATE):
    {
      // // SerialBusManager::tick(motorRefs, motorPositionsRaw, MOTOR_COUNT);
      // // imu1.tick(imuReadBuffer, imuReadBufferSize); // imuReadBufferSize should be a const, it's defined somewhere in the IMU stack

      // if (SerialBusManager::isDoneCollecting()){ // && imu1.doneCollecting()){
      //   // put data togehter into one packet
      //   // send packet to RPI
      //   elapsedMicros = micros() - startTime;
      //   Serial.print("Elapsed time: ");
      //   Serial.print(elapsedMicros);
      //   Serial.println(" microseconds");
      //   for (int i = 0; i < MOTOR_COUNT; i++){
      //     motorPositions[i] = motors[i].rawToDegs(motorPositionsRaw[i]);
      //     Serial.println(motorPositions[i]);
      //   }
      //   delay(4000);
      //   count++;
      //   robotState = SETTING_MOTOR_POS;
      // };
      // break;
    }
    case(IDLE):
    {
      delay(1000);
      break;
    }
  }
}



void moveToZeroPositions(){

  // move knees first

  // then do the rest of the legs

  // then do chest

  // then do arms

}