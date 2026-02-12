#include <Herkulex.h>
#include <HerkulexMotor.h>
int n=12; //motor ID - verify your ID !!!!
int is0601 = true;

uint16_t startTime = 0;
uint16_t elapsedTime = 0;
boolean runTest = true;

HerkulexMotor myMotor = HerkulexMotor(12, MotorModel::DRS_0601);

void setup(){
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Serial.println("Begin");
  Herkulex.beginSerial1(115200); //open serial port 1
  Herkulex.reboot(n); //reboot first motor
  delay(500);
  Herkulex.initialize(); //initialize motors
  delay(200);


}

void loop(){
  // this if statement makes sure that this only runs once at the beginning
  if (runTest) {
    startTime = micros();
    myMotor.setPos(0.0);
    elapsedTime = micros() - startTime;

    Serial.print("time to send moveOne cmd: ");
    Serial.print(elapsedTime);
    Serial.println(" microseconds");

    delay(1200);
    
    // reading the time needed for the angle of the motor to get back
    startTime = micros();
    myMotor.getPos();
    elapsedTime = micros() - startTime;

    Serial.print("time to read position: ");
    Serial.print(elapsedTime);
    Serial.println(" microseconds");

    myMotor.setPos(90);
    delay(1200);
  }
  runTest = false;
}