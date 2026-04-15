#include <HerkulexMotor.h>

enum motorsState{
    READING_POS,
    SETTING_POS,
};

uint8_t state = SETTING_POS;

const uint8_t numMotors = 2;
HerkulexMotor motors[numMotors];
MotorRef motorRefs[numMotors];

uint16_t motorPositionsRaw[numMotors];
float motorPositions[numMotors];

uint8_t busIdA = 2; // serial port myMotorOne is attached to
uint8_t busIdB = 4; // serial port myMotorTwo is attached to

void setup()
{
  Serial.begin(9600);    // Open serial communications
  delay(2000);           // a delay to have time for serial monitor opening

  SerialBusManager::createBus(busIdA);
  SerialBusManager::createBus(busIdB);
  SerialBusManager::startAllBuses(BAUD_RATE::SPEED_115K);

  motors[1] = HerkulexMotor(12, MotorModel::DRS_0601, busIdA);
  motors[2] = HerkulexMotor(15, MotorModel::DRS_0601, busIdB);

  for (int i = 0; i < numMotors; i++) motorRefs[i] = motors[i].getMotorRef();

  // provides STAT info for motor
  SerialBusManager::infoAllMotors(motorRefs, numMotors);
}

void loop(){
    switch(state){
        case(SETTING_POS):
        {
            for (int i = 0; i < numMotors; i++) motors[i].queueMove(50.0);
            SerialBusManager::actionAll(100);
            delay(100); // small delay for motors to get to position

            state = READING_POS;
            SerialBusManager::requestAllPositions(motorRefs, motorPositionsRaw, numMotors);
            break;
        }
        case(READING_POS):
        {
            SerialBusManager::tick(motorRefs, motorPositionsRaw, numMotors);
            if (SerialBusManager::isDoneCollecting()){
                for (int i = 0; i < numMotors; i++){
                motorPositions[i] = motors[i].rawToDegs(motorPositionsRaw[i]);
                Serial.println(motorPositions[i]);
                }
                state = SETTING_POS;
                delay(5000);
            }
            break;
        }
    }
}