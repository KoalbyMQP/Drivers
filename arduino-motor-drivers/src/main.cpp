#include <Herkulex.h>
#include <HerkulexMotor.h>
int n=5; //motor ID - verify your ID !!!!

uint16_t startTime = 0;
uint16_t elapsedTime = 0;
boolean runTest = true;

HerkulexMotor myMotor = HerkulexMotor(5, MotorModel::DRS_0601);

void setup(){
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(9600);    // Open serial communications
  Serial.println("Begin");
  Herkulex.beginSerial1(115200); //open serial port 1
  Herkulex.reboot(n); //reboot first motor
  delay(500);
  Herkulex.initialize(); //initialize motors
  delay(200);


}

void loop(){
  while(runTest){
  // this if statement makes sure that this only runs once at the beginning
    startTime = micros();
    myMotor.setPos(0.0);
    elapsedTime = micros() - startTime;

    Serial.print("time to send moveOne cmd: ");
    Serial.print(elapsedTime);
    Serial.println(" microseconds");

    delay(300);

    startTime = micros();
    myMotor.setPos(20.0);
    elapsedTime = micros() - startTime;

    Serial.print("time to send moveOne cmd: ");
    Serial.print(elapsedTime);
    Serial.println(" microseconds");

    delay(300);

    startTime = micros();
    myMotor.setPos(50.0);
    elapsedTime = micros() - startTime;

    Serial.print("time to send moveOne cmd: ");
    Serial.print(elapsedTime);
    Serial.println(" microseconds");

    runTest = false;
  }
  
}