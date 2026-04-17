#include <Arduino.h>
#include "RPIComs.h"
#include "Queue.h"
// For use with Serial 0
// changing it to serial 3 so that serial 0 can be used by the usb 0 to monitor

//Different buffers
char rxBuf[RPIComs::RX_BUF_SIZE];
char pktBuf[RPIComs::RX_BUF_SIZE];
char txBuf[RPIComs::RX_BUF_SIZE];
bool testing_serial = false;
uint16_t startTimeRPI = 0;
uint16_t elapsedTimeRPI = 0;



uint16_t rxPos = 0;
uint16_t txPos = 0;


/*
    Following function returns an integer representing a command code:
    |  -1   |   Terminate / End Loop    |
    |   0   |   No Command from Pi Read |
    |   1   |   Begin startup           |
*/
int RPIComs::uartRead(){
    while(Serial1.available() > 0){
        // should we update 0 with the expected packet size? if it will be constant... of course when we know what it is
        // Set temp char to the packets with .read

        char c = (char)Serial1.read();

        // -1 Case
        if (rxPos == 0 && (unsigned char)c == 0xFF) {

            while (Serial1.available()) {
                if (Serial1.read() == '\n') break;
            }

            return -1;
        } else if(rxPos == 0 && (unsigned char)c == 0x01){
            while (Serial1.available()) {
                if (Serial1.read() == '\n') break;
            }
            return 1;
        }

        if (c == '\n') {
            rxBuf[rxPos] = '\0';
            rxPos = 0;

            if (testing_serial) {
                startTimeRPI = micros();
            }

            _rxPacketQueue.enqueue(rxBuf);
            continue;
        }


        // Set next character in the packet buffer. If the buffer has reached the size limit, stop overflow by just making rxPos = 0 and reset
        if(rxPos < RX_BUF_SIZE - 1) {
            rxBuf[rxPos++] = c;
        } else {
            rxPos = 0;
            //handle overflow
            while (Serial1.available()) {
                if (Serial1.read() == '\n') break;
            }
        }

    }
    return 0;
}

const char* RPIComs::getPacket(){
    // Successful dequeue will return true, which means there was a packet to recieve. Else, return a nullptr
    if(!_rxPacketQueue.dequeue(pktBuf, sizeof(pktBuf))){
        return nullptr;
    } else {
        if (testing_serial){
            elapsedTimeRPI = micros() - startTimeRPI;
            Serial.print("it took ");
            Serial.print(elapsedTimeRPI);
            Serial.println(" microseconds between enqueing the message and sending it back");
        }
        return pktBuf;
    }
}

void RPIComs::enqueueTXPacket(const char* pkt){
    _txPacketQueue.enqueue(pkt);

}

// Send uart packet to pi
void RPIComs::uartSend(){
    if(_txPacketQueue.dequeue(txBuf, sizeof(txBuf))){
        // Serial1.write((byte*)txBuf, sizeof(txBuf));
        Serial1.println(txBuf);
    }
}