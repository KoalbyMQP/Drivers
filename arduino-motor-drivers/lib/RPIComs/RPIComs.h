#ifndef RPIComs_h
#define RPIComs_h

#include <Arduino.h>
#include "Queue.h"

class RPIComs{
    public:
        static constexpr size_t RX_BUF_SIZE   = 1024;
        static constexpr size_t MAX_PACKETS   = 8;

        RPIComs() = default;

        void uartRead();
        const char* getPacket();     // returns nullptr if none

        bool hasPacket() const { return !packetQueue.isEmpty(); }
        uint16_t queuedCount() const { return packetQueue.size(); }

        private:
        PacketQueue<MAX_PACKETS, RX_BUF_SIZE> packetQueue;

        char rxBuf[RX_BUF_SIZE] = {0};
        char pktBuf[RX_BUF_SIZE] = {0};
        uint16_t rxPos = 0;

};



#endif