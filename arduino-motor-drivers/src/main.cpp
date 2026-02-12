#include <Herkulex.h>
int n=12; //motor ID - verify your ID !!!!
int is0601 = true;

uint16_t startTime = 0;
uint16_t elapsedTime = 0;
boolean runTest = true;

void setup()
{
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
    Herkulex.moveOne(n, 100, 300, LED_BLUE, true);
    elapsedTime = micros() - startTime;

    Serial.print("time to send moveOne cmd: ");
    Serial.print(elapsedTime);
    Serial.println(" microseconds");

    delay(1200);
    
    // reading the time needed for the angle of the motor to get back
    startTime = micros();
    Herkulex.getAngle(n, is0601);
    elapsedTime = micros() - startTime;

    Serial.print("time to read position: ");
    Serial.print(elapsedTime);
    Serial.println(" microseconds");

    Herkulex.moveOne(n, 1000, 300, LED_BLUE, true);
    delay(1200);
  }
  runTest = false;
}