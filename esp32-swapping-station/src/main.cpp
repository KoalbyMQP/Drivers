#include <Arduino.h>
#include "DRV8825.h"

// limit switch connection
#define L_LIMIT_SWITCH 18 
#define R_LIMIT_SWITCH 25

// DRV8825 stepper motor driver connections
#define L_SLEEP_PIN 16
#define L_RESET_PIN 17
#define L_STEP_PIN 27
#define L_DIR_PIN 26

#define R_SLEEP_PIN 22
#define R_RESET_PIN 23
#define R_STEP_PIN 21
#define R_DIR_PIN 19

enum SWAPPING_STATE {
  IDLE,
  MOVING_TO_POSITION,
};

enum TOOL_POSITIONS {
  H = 0,
  P1 = 100,
  P2 = 9680,
};

char motor; // variable to store which motor to move (L or R)
int targetPos; // variable to store target position for motor movement

SWAPPING_STATE swappingState = IDLE;

DRV8825 leftMotor(L_SLEEP_PIN, L_RESET_PIN, L_STEP_PIN, L_DIR_PIN, L_LIMIT_SWITCH);
DRV8825 rightMotor(R_SLEEP_PIN, R_RESET_PIN, R_STEP_PIN, R_DIR_PIN, R_LIMIT_SWITCH);

void setup() {
  leftMotor.initialize();
  rightMotor.initialize();

  delay(2000); // wait for motors to initialize

  // begin serial communication with teensy
  Serial.begin(115200);
  Serial.println("Begin");

  delay(1000);

  // print out limit switch s
  Serial.print("Left Limit Switch State: ");
  Serial.println(digitalRead(L_LIMIT_SWITCH));
  Serial.print("Right Limit Switch State: ");
  Serial.println(digitalRead(R_LIMIT_SWITCH));


  // home motor at startup
  leftMotor.homeMotor();
  rightMotor.homeMotor();
  Serial.println("HC");
}

void loop() {
  // read serial input from teensy in a state machine
  switch (swappingState) {
    case IDLE:
      if (Serial.available() > 0) {
        // read the target position and target station from input
        // example input: "RP2" to move right motor to the second position according to tool positions
        String input = Serial.readStringUntil('\n');
        if (input.length() >= 3) {
          motor = input.charAt(0);
          char posChar = input.charAt(2);
          if (posChar == '1') targetPos = TOOL_POSITIONS::P1;
          else if (posChar == '2') targetPos = TOOL_POSITIONS::P2;
          else targetPos = TOOL_POSITIONS::H; // default to home for any other position
          swappingState = MOVING_TO_POSITION; // move to next state to execute movement
        }
      }
      break;

    case MOVING_TO_POSITION:
      // move the specified motor to the target position
      if (motor == 'L') {
        leftMotor.runToPosition(targetPos);
      }
      else if (motor == 'R') {
        rightMotor.runToPosition(targetPos);
      }
      swappingState = IDLE; // return to idle state after moving
      break;
  }
}
