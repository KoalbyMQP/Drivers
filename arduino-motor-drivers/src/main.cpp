#include <Herkulex.h>
#include <HerkulexMotor.h>
#include <RPIComs.h>
#include <ToolChanger.h>

#define NUM_MOTORS 2

uint16_t startTime = 0;
uint16_t elapsedTime = 0;
boolean testSetPosBool = false;    // when runTest is true, the moving motor will run once upon restart and when the arduino is uploaded.
boolean testQueueBool = false;
boolean testRPi = false;
boolean testToolSwap = true;

HerkulexMotor motors[] = {
    // HerkulexMotor(15, MotorModel::DRS_0201),  // shoulderspin_left
    // HerkulexMotor(16, MotorModel::DRS_0201),  // shoulderspin_right
    // HerkulexMotor(17, MotorModel::DRS_0201),  // bicep_left
    // HerkulexMotor(18, MotorModel::DRS_0201),  // bicep_right
    // HerkulexMotor(19, MotorModel::DRS_0201),  // elbow_left
    // HerkulexMotor(20, MotorModel::DRS_0201),  // elbow_right
    HerkulexMotor(26, MotorModel::DRS_0201),  // wristspin_left
    // HerkulexMotor(22, MotorModel::DRS_0201),  // wristspin_right
    HerkulexMotor(4, MotorModel::DRS_0201),  // handcurl_left
    // HerkulexMotor(24, MotorModel::DRS_0201),  // handcurl_right
    // HerkulexMotor(25, MotorModel::DRS_0201),  // gripper_left
    // HerkulexMotor(26, MotorModel::DRS_0201),  // gripper_right
};

RPIComs rpi = RPIComs();
ToolChanger tcr = ToolChanger(0, 13);

void setup(){
  delay(2000);  // a delay to have time for serial monitor opening on platformio after uploading
  Serial.begin(9600);    // Open serial communications with computer
  Serial.println("Begin");

  HerkulexMotor::initSerialPorts(115200); // begin serial communications with motor

  for (HerkulexMotor& m : motors){
    m.reboot(); // reboot each motor to initialize
  }

  delay(500);

  HerkulexMotor::initialize(); // initialize all motors
  tcr.initialize();

  // set the motor positions to 0 to initialize
  for (HerkulexMotor& m : motors){
    m.setPos(0.0);
  }
  delay(500);
}


void testingRPi(){
    rpi.uartRead();

  // If a packet arrived, handle it
    const char* pkt = rpi.getPacket();
    if (pkt != nullptr) {
      Serial.print("Received: ");
      Serial.println(pkt);
    }

}

void handlePacket(const char* pkt) {
    char buf[512];
    strncpy(buf, pkt, sizeof(buf));

    uint32_t time = 0;
    int8_t tclCmd = 255;
    int8_t tcrCmd = 255;
    float positions[NUM_MOTORS];
    bool queued[NUM_MOTORS];

    for (int i = 0; i < NUM_MOTORS; i++) {
        positions[i] = 0.0;
        queued[i]    = false;
    }

    char *token = strtok(buf, ",");
    while (token != nullptr) {
        char *colon = strchr(token, ':');
        if (colon != nullptr) {
            *colon      = '\0';
            char *key   = token;
            char *value = colon + 1;

            if (strcmp(key, "t") == 0) {
                time = atol(value);
            } else if (strcmp(key, "tcl") == 0) {
                tclCmd = atoi(value);
            } else if (strcmp(key, "tcr") == 0) {
                tcrCmd = atoi(value);
            } else if (key[0] == 'm') {
                int motorID = atoi(key + 1);
                if (motorID >= 0 && motorID < NUM_MOTORS) {
                    positions[motorID] = atof(value);
                    queued[motorID]    = true;
                }
            }
        }
        token = strtok(nullptr, ",");
    }

    bool anyQueued = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (queued[i]) {
            motors[i].queueMove(positions[i]);
            anyQueued = true;
        }
    }

    if (anyQueued) {
        Herkulex.actionMoves(time / 11.2);
    }

    if (tcrCmd != 255) {
        if (tcrCmd == 0) {
            tcr.dockTool();
        } else if (tcrCmd == 1) {
            tcr.lockTool();
        } else if (tcrCmd == 2) {
            tcr.ejectTool();
        }
    }
    if (tclCmd != 255) {
        // handle tool changer left commands when we have them
    }
}

void sendStatus() {
    char buf[1024];
    int  pos = 0;

    // arm status
    // pos += snprintf(buf + pos, sizeof(buf) - pos, "arm:%d,",
    //     // _armStatus
    // );

    // tool changer right - id only
    pos += snprintf(buf + pos, sizeof(buf) - pos, "tcr:%d,",
        tcr.getToolStatus()
    );

    // motor positions - only motors that exist
    for (int i = 0; i < NUM_MOTORS; i++) {
      float currentPos = motors[i].getPos(); // read motor position using HerkulexMotor getPos
      pos += snprintf(buf + pos, sizeof(buf) - pos, "%d:%.2f,",
          i,
          currentPos
      );
    }

    // error and heartbeat
    // pos += snprintf(buf + pos, sizeof(buf) - pos, "err:%d,hb:%d\n",
    //     // _errorCode,
    //     // _heartbeat++
    // );

    Serial.print(buf);
}

void testToolChanger(){
    tcr.dockTool();
    delay(2000);
    tcr.lockTool();
    delay(2000);
    tcr.ejectTool();
}

void loop() {

    testToolChanger();
    // rpi.uartRead();

    // const char* pkt = rpi.getPacket();
    // if (pkt != nullptr) {
    //     // during testing - print what arrived
    //     Serial.print("Received: ");
    //     Serial.println(pkt);

    //     // then handle it
    //     handlePacket(pkt);
    // }

    // // send status every 10ms
    // static uint32_t lastSend = 0;
    // if (millis() - lastSend >= 10) {
    //     sendStatus();
    //     lastSend = millis();
    // }
}

// m1:30,m2:45,tcr:1,