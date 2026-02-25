#include <Herkulex.h>
#include <HerkulexMotor.h>
#include <RPIComs.h>

uint16_t startTime = 0;
uint16_t elapsedTime = 0;
boolean testSetPosBool = false;    // when runTest is true, the moving motor will run once upon restart and when the arduino is uploaded.
boolean testQueueBool = false;
boolean testRPi = true;
boolean latency_queuing = false;

HerkulexMotor myMotor = HerkulexMotor(12, MotorModel::DRS_0601);
HerkulexMotor myMotor2 = HerkulexMotor(7, MotorModel::DRS_0602);
RPIComs rpi = RPIComs();


void setup(){
  delay(2000);  // a delay to have time for serial monitor opening on platformio after uploading
  Serial.begin(9600);    // Open serial communications with computer
  Serial.println("Begin");

  Serial3.begin(9600); //begin serial communication with the raspberry pic

  HerkulexMotor::initSerialPorts(115200); // begin serial communications with motor

  myMotor.reboot();
  myMotor2.reboot();

  delay(500);

  HerkulexMotor::initialize(); // initialize all motors

  // set the motor positions to 0 to initialize
  myMotor.setPos(0.0);
  myMotor2.setPos(0.0);
  delay(500);
}


void testingRPi(){
    rpi.uartRead();
    if (latency_queuing){
      startTime = micros();
    }
    // If a packet arrived, handle it
    const char* pkt = rpi.getPacket();
    if (pkt != nullptr) {
      if (latency_queuing) {
        elapsedTime = micros() - startTime;
        Serial.print("It took ");
        Serial.print(elapsedTime);
        Serial.println(" microseconds between putting the message in the queue (right after uartRead) and reading it");
      }
      Serial.print("Received: ");
      Serial.println(pkt);
    }

}

void loop(){
  // this loops
  while(testRPi){
    testingRPi();
    testRPi = true;
  }
}