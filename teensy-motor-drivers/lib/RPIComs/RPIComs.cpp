#include <Arduino.h>
#include "RPIComs.h"
#include "Queue.h"
// For use with Serial 0
// changing it to serial 3 so that serial 0 can be used by the usb 0 to monitor
#define RPI_SERIAL Serial

//Different buffers
static uint8_t rxBuf[PACKET_SIZE];
uint8_t pktBuf[PACKET_SIZE];
uint8_t outBuf[PACKET_SIZE];
char txBuf[RPIComs::TX_BUF_SIZE];
bool testing_serial = false;
uint32_t startTimeRPI = 0;
uint32_t elapsedTimeRPI = 0;

uint16_t rxPos = 0;
uint16_t txPos = 0;

static bool receiving = false;
static bool packetReady = false;

/*
    Following function returns an integer representing a command code:
    |   -1   |   No Command from Pi Read |
    |   1   |   Command Received        |
*/
int RPIComs::uartRead(){
    while(RPI_SERIAL.available() > 0){
        // should we update 0 with the expected packet size? if it will be constant... of course when we know what it is
        // Set temp char to the packets with .read

        // Serial.print("In loop rxPos: ");
        // Serial.println(rxPos);
        // Serial.flush();

        uint8_t byte  = RPI_SERIAL.read();

        // wait for start byte to start receiving
        if (!receiving) {
            // Serial.print("Received byte: ");
            // Serial.println(byte, HEX);
            // Serial.flush();

            if (byte == 0xAA) {
                receiving = true;
                rxPos = 0; // reset position for new packet
            }
            continue; // skip to next byte
        }


        // store byte in buffer and increment position
        // Serial.print("Received byte: ");
        // Serial.println(byte, HEX);
        // Serial.flush();
        rxBuf[rxPos++] = byte;

        if (rxPos == PACKET_SIZE) {
            receiving = false;
            rxPos = 0; // reset for next packet

            memcpy(pktBuf, rxBuf, PACKET_SIZE); // copy to packet buffer for processing
            packetReady = true;
            // Serial.println("Packet received and ready to process");
            // Serial.println("First byte of packet (flag): ");
            // Serial.flush();
            for(int j = 0; j < PACKET_SIZE; j++){
                // Serial.print(pktBuf[j], HEX);
                // Serial.print(" ");
                // Serial.flush();
            }
            //Serial.println(pktBuf[0], HEX);
            // Serial.flush();
            return 1; // packet received
        }
        if (rxPos > PACKET_SIZE) {
            // packet too large, reset
            receiving = false;
            rxPos = 0;
        }

    }
    // Serial.println("No packet received");
    // Serial.flush();
    return -1;;
}

const uint8_t* RPIComs::getPacket(){
    // Successful dequeue will return true, which means there was a packet to recieve. Else, return a nullptr
    if(!packetReady){
        return nullptr;
    } else {
        if (testing_serial){
            elapsedTimeRPI = micros() - startTimeRPI;
            // Serial.print("it took ");
            // Serial.print(elapsedTimeRPI);
            // Serial.println(" microseconds between enqueing the message and sending it back");
        }
        packetReady = false; // reset for next packet
        memcpy(outBuf, pktBuf, PACKET_SIZE); // set output buffer to the received packet
        return outBuf;
    }
}

void RPIComs::enqueueTXPacket(const char* pkt){
    _txPacketQueue.enqueue(pkt);

}

// Send uart packet to pi
void RPIComs::uartSend(){
    if(_txPacketQueue.dequeue(txBuf, sizeof(txBuf))){
        RPI_SERIAL.write((byte*)txBuf, sizeof(txBuf));
        // RPI_SERIAL.println(txBuf);
    }
}