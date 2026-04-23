// #include <Arduino.h>
// #include <BluetoothSerial.h>

// BluetoothSerial SerialBT;

// // Status LED connection
// #define STATUS_LED 23

// // Limit switch connection
// #define LIMIT_SWITCH_PIN 27

// // DRV8825 stepper motor driver connections
// #define SLEEP_PIN 26
// #define RESET_PIN 25
// #define STEP_PIN 17
// #define DIR_PIN 18

// #define STEP_RANGE 10200  // Maximum steps for full range
// // Steps per inch
// // 
// // Steps per mm
// // 

// int currentPosition = 0;  // Track current position of the motor

// void homeMotor() {
//   SerialBT.println("Starting homing sequence...");

//   // Move motor backward until limit switch is pressed
//   digitalWrite(DIR_PIN, HIGH);  // Set direction to backward
//   delay(10);  // Allow direction to settle

//   digitalWrite(STATUS_LED, HIGH); // Turn on status LED to indicate homing in progress

//   while (digitalRead(LIMIT_SWITCH_PIN) == LOW) { // Assuming LOW means not pressed
//     digitalWrite(STEP_PIN, HIGH);
//     delayMicroseconds(1000);  // Speed of homing
//     digitalWrite(STEP_PIN, LOW);
//     delayMicroseconds(1000);
//   }
  
//   SerialBT.println("Homing complete.");

//   digitalWrite(STATUS_LED, LOW); // Turn off status LED
  
//   currentPosition = 0;  // Reset position
// }

// void moveToPosition(int position) {
//   int stepsToMove = position - currentPosition;
  
//   if (stepsToMove > 0) {
//     moveForward(stepsToMove);
//   } else if (stepsToMove < 0) {
//     moveBackward(-stepsToMove);
//   }
  
//   currentPosition = position;
// }

// void moveForward(int steps) {
//   SerialBT.print("Moving forward ");
//   SerialBT.print(steps);
//   SerialBT.println(" steps");
  
//   digitalWrite(DIR_PIN, LOW);  // Set direction to forward
//   delay(10);

//   digitalWrite(STATUS_LED, HIGH); // Turn on status LED to indicate homing in progress
  
//   for (int i = 0; i < steps; i++) {
//     digitalWrite(STEP_PIN, HIGH);
//     delayMicroseconds(1000);
//     digitalWrite(STEP_PIN, LOW);
//     delayMicroseconds(1000);
//   }
  
//   SerialBT.println("Forward motion complete.");

//   digitalWrite(STATUS_LED, LOW); // Turn off status LED

// }

// void moveBackward(int steps) {
//   SerialBT.print("Moving backward ");
//   SerialBT.print(steps);
//   SerialBT.println(" steps");
  
//   digitalWrite(DIR_PIN, HIGH);  // Set direction to backward
//   delay(10);

//   digitalWrite(STATUS_LED, HIGH); // Turn on status LED to indicate homing in progress
  
//   for (int i = 0; i < steps; i++) {
//     digitalWrite(STEP_PIN, HIGH);
//     delayMicroseconds(1000);
//     digitalWrite(STEP_PIN, LOW);
//     delayMicroseconds(1000);
//   }
  
//   SerialBT.println("Backward motion complete.");

//   digitalWrite(STATUS_LED, LOW); // Turn off status LED
// }

// void stopMotor() {
//   SerialBT.println("Motor stopped.");
//   digitalWrite(STATUS_LED, LOW);
// }

// void setup() {
//   // Configure LED pin
//   pinMode(STATUS_LED, OUTPUT);

//   // Configure limit switch pin
//   pinMode(LIMIT_SWITCH_PIN, INPUT);

//   // Configure DRV8825 pins
//   pinMode(SLEEP_PIN, OUTPUT);
//   pinMode(RESET_PIN, OUTPUT);
//   pinMode(STEP_PIN, OUTPUT);
//   pinMode(DIR_PIN, OUTPUT);
  
//   // Wake up the driver
//   digitalWrite(SLEEP_PIN, HIGH);
//   digitalWrite(RESET_PIN, HIGH);
  
//   Serial.begin(115200);

//   if (!SerialBT.begin("ESP32_SWAPPING_STATION")) {
//     Serial.println("Bluetooth failed to start");
//   } else {
//     Serial.println("Bluetooth ready");
//   }
// }

// void loop() {
//   // Check if data is available from Bluetooth
//   if (SerialBT.available()) {
//     String command = SerialBT.readStringUntil('\n');
//     command.trim();  // Remove whitespace
    
//     SerialBT.print("Received command: ");
//     SerialBT.println(command);
    
//     // Parse command and control motor motion
//     if (command.startsWith("H")) {
//       // H - Home the motor
//       homeMotor();
//     }
//     else if (command.startsWith("F")) {
//       // F<steps> - Move forward
//       int steps = command.substring(1).toInt();
//       if (steps > 0) {
//         moveForward(steps);
//       } else {
//         SerialBT.println("Invalid number of steps for forward motion");
//       }
//     }
//     else if (command.startsWith("B")) {
//       // B<steps> - Move backward
//       int steps = command.substring(1).toInt();
//       if (steps > 0) {
//         moveBackward(steps);
//       } else {
//         SerialBT.println("Invalid number of steps for backward motion");
//       }
//     }
//     else if(command.startsWith("M")) {
//       // M<position> - Move to absolute position
//       int position = command.substring(1).toInt();
//       if (position >= 0 && position <= STEP_RANGE) {
//         moveToPosition(position);
//       } else {
//         SerialBT.println("Invalid position. Must be between 0 and " + String(STEP_RANGE));
//       }
//     }
//     else if (command == "STOP") {
//       // STOP - Stop the motor
//       stopMotor();
//     }
//     else {
//       // Invalid command
//       SerialBT.println("Invalid command. Available commands:");
//       SerialBT.println("  H - Home the motor");
//       SerialBT.println("  F<steps> - Move forward (e.g., F100)");
//       SerialBT.println("  B<steps> - Move backward (e.g., B50)");
//       SerialBT.println("  M<position> - Move to absolute position (e.g., M100)");
//       SerialBT.println("  STOP - Stop the motor");
//     }
//   }
// }
