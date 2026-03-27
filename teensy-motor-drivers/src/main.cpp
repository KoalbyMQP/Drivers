#include <HerkulexMotor.h>
#include <RPIComs.h>
#include <SerialBusManager.h>
#include <debug.h>

// start at serial 2 because the raspberry pi is connected through serial 1
enum SERIAL_BUS {
    BUS_L_LEG = 2,
    BUS_R_LEG = 3,
    BUS_CHEST = 4,
    BUS_L_ARM = 5,
    BUS_R_ARM = 6,
    EXTRA_1 = 7,
    EXTRA_2 = 8
};

enum STATE {
  READING_FROM_RPI,
  SETTING_MOTOR_POS,
  READING_ROBOT_STATE,
};

uint8_t robotState = SETTING_MOTOR_POS;


const uint8_t MOTOR_COUNT = 3;
const uint8_t PACKET_SIZE = 192;   // this depends on the number of motors used, HOW???

float RPIMotorInputs[MOTOR_COUNT] = {0};
float dummyMotorInputs[MOTOR_COUNT] = {40.0, 40.0, 40.0};
float dummyMotorInputsTwo[MOTOR_COUNT] = {-40.0, -40.0, -40.0};
int count = 0 ;
uint16_t motorPositionsRaw[MOTOR_COUNT] = {0};
float motorPositions[MOTOR_COUNT] = {0};

MotorRef motorRefs[MOTOR_COUNT];
HerkulexMotor motors[MOTOR_COUNT] = {
  HerkulexMotor(12, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG),
  HerkulexMotor(5, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG),
  HerkulexMotor(1, MotorModel::DRS_0601, SERIAL_BUS::BUS_L_LEG),
  
};



RPIComs rpi = RPIComs();


void setup(){
  delay(2000);  // a delay to have time for serial monitor opening on platformio after uploading
  Serial.begin(9600);    // Open serial communications with computer
  Serial.println("Begin");

  Serial1.begin(9600); //begin serial communication with the raspberry pi
  delay(2000);

  // initialize all serial buses
  SerialBusManager::createBus(SERIAL_BUS::BUS_L_LEG); // begin serial communications with motor, these are on Serial 2
  SerialBusManager::createBus(SERIAL_BUS::BUS_R_LEG);
  SerialBusManager::startAllBuses(115200);
  SerialBusManager::initAllMotors();

  // // MotorRef table — used by the parallelized position read.
  // // Each entry is {busId, servoId}. Order here determines order in rawPositions[], can mix and match serial buses
  // // Add or remove entries to match the motors needed
  for (int i = 0; i < MOTOR_COUNT; i++) motorRefs[i] = motors[i].getMotorRef();
  for (int i = 0; i < MOTOR_COUNT; i++) motors[i].setPos(-20.0);
  delay(2000);

}


void loop(){
  switch (robotState){
    case(READING_FROM_RPI):
    {
      rpi.uartRead();
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
      // queue all motors in a loop
      // does this line up with the correct motors?
      for (int i = 0; i < MOTOR_COUNT; i++) {
        if (count % 2){
          motors[i].queueMove(dummyMotorInputs[i]);
        } else {
          motors[i].queueMove(dummyMotorInputsTwo[i]);
        }
      }
      SerialBusManager::actionAll(10);

      robotState = READING_ROBOT_STATE;
      SerialBusManager::requestAllPositions(motorRefs, motorPositionsRaw, MOTOR_COUNT);
      //imu1.requestRead();
      
      break;
    }
    case(READING_ROBOT_STATE):
    {
      count++;
      delay(2000);
      robotState = SETTING_MOTOR_POS;
      // SerialBusManager::tick(motorRefs, motorPositionsRaw, MOTOR_COUNT);
      //imu1.tick(imuReadBuffer, imuReadBufferSize); // imuReadBufferSize should be a const, it's defined somewhere in the IMU stack

      // if (SerialBusManager::isDoneCollecting()){ // && imu1.doneCollecting()){
      //   // put data togehter into one packet
      //   // send packet to RPI
      //   for (int i = 0; i < MOTOR_COUNT; i++){
      //     motorPositions[i] = motors[i].rawToDegs(motorPositionsRaw[i]);
      //     Serial.println(motorPositions[i]);
      //   }
      //   robotState = SETTING_MOTOR_POS;
      // };
      break;
    }
  }
}
