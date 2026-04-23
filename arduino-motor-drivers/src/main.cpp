#include "HerkulexMotor.h"
#include "Config.h"
#include "Herkulex.h"

// Function prototype
void printJoints();
void startFromZero();
void makeMove(char hand, char file);
void goToHomePosition(HerkulexMotor* arm[], float gripOpen);

// Global Variables
bool isHoldingPiece = false;

// Define the motors
// Right Arm (Serial Bus 1)
HerkulexMotor shoulderR(shoulderspin_right, DRS_0601, 1);
HerkulexMotor bicepR(biceplift_right, DRS_0602, 1);
HerkulexMotor elbowR(elbow_right, DRS_0601, 1);
HerkulexMotor wristR(wristspin_right, DRS_0201, 1);
HerkulexMotor handR(handcurl_right, DRS_0201, 1);
HerkulexMotor gripperR(gripper_right, DRS_0201, 1);

// Left Arm (Serial Bus 2)
HerkulexMotor shoulderL(shoulderspin_left, DRS_0601, 2);
HerkulexMotor bicepL(biceplift_left, DRS_0602, 2);
HerkulexMotor elbowL(elbow_left, DRS_0601, 2);
HerkulexMotor wristL(wristspin_left, DRS_0201, 2);
HerkulexMotor handL(handcurl_left, DRS_0201, 2);
HerkulexMotor gripperL(gripper_left, DRS_0201, 2);

HerkulexMotor* allMotors[] = {
    &shoulderR, &bicepR, &elbowR, &wristR, &handR, &gripperR,
    &shoulderL, &bicepL, &elbowL, &wristL, &handL, &gripperL
};

HerkulexMotor* rightArmMotors[] = { &shoulderR, &bicepR, &elbowR, &wristR, &handR, &gripperR };
HerkulexMotor* leftArmMotors[] = { &shoulderL, &bicepL, &elbowL, &wristL, &handL, &gripperL };

void setup() {

    Serial.begin(9600);
    while (!Serial); // Wait for Serial Monitor to open
    Serial.println("System Starting...");

    // 1. Initialize Serial Ports at 115200 baud (standard for Herkulex on Mega)
    HerkulexMotor::initSerialPorts(115200); 
        
    // 2. Clear errors and enable torque for all motors on all active buses
    HerkulexMotor::initialize();

    // shoulderR.queueMove(0.0);
    // bicepR.queueMove(0.0);
    // elbowR.queueMove(0.0);
    // wristR.queueMove(0.0);
    // handR.queueMove(0.0);
    // gripperR.queueMove(0.0);
        
    // shoulderL.queueMove(0.0);
    // bicepL.queueMove(0.0);
    // elbowL.queueMove(0.0);
    // wristL.queueMove(0.0);
    // handL.queueMove(0.0);
    // gripperL.queueMove(0.0);

    // HerkulexMotor::actionMoves(1000, allMotors, 12);

    // delay(1000);
    
    // Serial.println("Set up Finished");

    delay(500);

    printJoints();

    }

void loop() {

    startFromZero();

    delay(2000);

    // makeMove('L', 'A');

    delay(2000);

    makeMove('L', 'B');

    // goToHomePosition(leftArmMotors, ROpen);
    
    while (true) {
        // Stop here forever
    } 
}

void printJoints(){
  Serial.println("Reading joint positions...");
  Serial.println("============================");

  // Right Arm
  Serial.print("shoulderR: "); Serial.println(shoulderR.getPos());
  Serial.print("bicepR:    "); Serial.println(bicepR.getPos());
  Serial.print("elbowR:    "); Serial.println(elbowR.getPos());
  Serial.print("wristR:    "); Serial.println(wristR.getPos());
  Serial.print("handR:     "); Serial.println(handR.getPos());
  Serial.print("gripperR:  "); Serial.println(gripperR.getPos());

  Serial.println("----------------------------");

  // Left Arm
  Serial.print("shoulderL: "); Serial.println(shoulderL.getPos());
  Serial.print("bicepL:    "); Serial.println(bicepL.getPos());
  Serial.print("elbowL:    "); Serial.println(elbowL.getPos());
  Serial.print("wristL:    "); Serial.println(wristL.getPos());
  Serial.print("handL:     "); Serial.println(handL.getPos());
  Serial.print("gripperL:  "); Serial.println(gripperL.getPos());

  Serial.println("============================");
  Serial.println("Done.");
}

void startFromZero() {
  shoulderR.queueMove(-2.28);
  bicepR.queueMove(151.62);
  elbowR.queueMove(1.14);
  wristR.queueMove(5.52);
  handR.queueMove(6.17);
  gripperR.queueMove(-2.60);
    
  shoulderL.queueMove(2.28);
  bicepL.queueMove(-151.62);
  elbowL.queueMove(-1.14);
  wristL.queueMove(-5.52);
  handL.queueMove(-6.17);
  gripperL.queueMove(2.60);

  HerkulexMotor::actionMoves(1000, allMotors, 12);

  delay(1000);

  shoulderR.queueMove(89.16);
  bicepR.queueMove(151.79);
  elbowR.queueMove(0.65);
  wristR.queueMove(5.20);
  handR.queueMove(-2.27);
  gripperR.queueMove(-2.60);

  shoulderL.queueMove(-89.16);
  bicepL.queueMove(-151.79);
  elbowL.queueMove(-0.65);
  wristL.queueMove(-5.20);
  handL.queueMove(2.27);
  gripperL.queueMove(2.60);

  HerkulexMotor::actionMoves(1000, allMotors, 12);

  delay(1000);

  shoulderR.queueMove(106.44);
  bicepR.queueMove(52.95);
  elbowR.queueMove(-54.93);
  wristR.queueMove(-5.52);
  handR.queueMove(51.68);
  gripperR.queueMove(-2.60);
  
  shoulderL.queueMove(-103.34);
  bicepL.queueMove(-52.45);
  elbowL.queueMove(44.50);
  wristL.queueMove(12.67);
  handL.queueMove(-39.00);
  gripperL.queueMove(2.60);

  HerkulexMotor::actionMoves(1000, allMotors, 12);

  delay(1000);
}

// Helpers in main.cpp

void applyPosition(const float angles[5], HerkulexMotor* arm[], float gripperAngle) {
    arm[0]->queueMove(angles[0]); // shoulder
    arm[1]->queueMove(angles[1]); // bicep
    arm[2]->queueMove(angles[2]); // elbow
    arm[3]->queueMove(angles[3]); // wrist
    arm[4]->queueMove(angles[4]); // hand
    arm[5]->queueMove(gripperAngle);
    HerkulexMotor::actionMoves(1000, arm, 6);
    delay(1000);
}

int columnIndex(char file, char hand) {
    if (hand == 'R') {
        for (int i = 0; i < NUM_COLUMNS; i++) {
            if (RIGHT_COLUMN_KEYS[i] == file) return i;
        }
    } else if (hand == 'L') {
        for (int i = 0; i < NUM_COLUMNS; i++) {
            if (LEFT_COLUMN_KEYS[i] == file) return i;
        }
    }
    return -1;
}

void pickUpFromColumn(HerkulexMotor* arm[], const ColumnPositions& col, float gripOpen, float gripClosed) {
    applyPosition(col.above.angles, arm, gripOpen);   // Move above
    printJoints();
    applyPosition(col.pick.angles,  arm, gripOpen);   // Lower
    printJoints();
    applyPosition(col.pick.angles,  arm, gripClosed); // Grip
    applyPosition(col.above.angles, arm, gripClosed); // Raise
}

void placeDownAtColumn(HerkulexMotor* arm[], const ColumnPositions& col, float gripOpen, float gripClosed) {
    applyPosition(col.above.angles, arm, gripClosed); // Move above with grip
    printJoints();
    applyPosition(col.pick.angles,  arm, gripClosed); // Lower
    printJoints();
    applyPosition(col.pick.angles,  arm, gripOpen);   // Release
    applyPosition(col.above.angles, arm, gripOpen);   // Raise
}

void makeMove(char hand, char file) {
    int col = columnIndex(file, hand);
    if (col == -1) {
        Serial.println("Unknown file!");
        return;
    }

    if (hand == 'R') {
        if (!isHoldingPiece) {
            pickUpFromColumn(rightArmMotors, RIGHT_COLUMNS[col], ROpen, RClosed);
            isHoldingPiece = true;
        } else {
            placeDownAtColumn(rightArmMotors, RIGHT_COLUMNS[col], ROpen, RClosed);
            isHoldingPiece = false;
        }
    } else if (hand == 'L') {
        if (!isHoldingPiece) {
            pickUpFromColumn(leftArmMotors, LEFT_COLUMNS[col], LOpen, LClosed);
            isHoldingPiece = true;
        } else {
            placeDownAtColumn(leftArmMotors, LEFT_COLUMNS[col], LOpen, LClosed);
            isHoldingPiece = false;
        }
    }
}

void goToHomePosition(HerkulexMotor* arm[], float gripOpen) {
    if (arm == rightArmMotors) {
        shoulderR.queueMove(106.44);
        bicepR.queueMove(52.95);
        elbowR.queueMove(-54.93);
        wristR.queueMove(-5.52);
        handR.queueMove(51.68);
        gripperR.queueMove(-2.60);
        
        HerkulexMotor::actionMoves(1000, rightArmMotors, 6);

    } else if (arm == leftArmMotors) {
        shoulderL.queueMove(-103.34);
        bicepL.queueMove(-52.45);
        elbowL.queueMove(44.50);
        wristL.queueMove(12.67);
        handL.queueMove(-39.00);
        gripperL.queueMove(2.60);

        HerkulexMotor::actionMoves(1000, leftArmMotors, 6);
    }

    delay(1000);
}