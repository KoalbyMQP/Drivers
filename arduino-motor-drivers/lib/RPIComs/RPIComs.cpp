#include <Arduino.h>
#include "RPIComs.h"
#include "Queue.h"
// For use with RX1 pin 19



//Different buffers
char rxBuf[RPIComs::RX_BUF_SIZE];
char pktBuf[RPIComs::RX_BUF_SIZE];

static PacketQueue<128, RPIComs::RX_BUF_SIZE> pktQueue;

uint16_t rxPos = 0;

void RPIComs::uartRead(){
    while(Serial1.available() > 0){
        // Set temp char to the packets with .read

        char c = (char)Serial1.read();

        // Check if newline character for packet completion

        if(c == '\n'){
            rxBuf[rxPos] = '\0';

            rxPos = 0;
            pktQueue.enqueue(rxBuf);
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
}

const char* RPIComs::getPacket(){
    // Successful dequeue will return true, which means there was a packet to recieve. Else, return a nullptr
    if(!pktQueue.dequeue(pktBuf, sizeof(pktBuf))){
        return nullptr;
    } else {
        return pktBuf;
    }
}