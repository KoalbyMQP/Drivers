#ifndef RPIComs_h
#define RPIComs_h

#include <Arduino.h>
#include "Queue.h"


class RPIComs{
    public:
        static constexpr size_t RX_BUF_SIZE   = 256; // shorten this when we know the constant packet size?
        static constexpr size_t MAX_PACKETS   = 8;

        // RPIComs() = default;

        void uartRead();
        void uartSend();
        const char* getPacket();     // returns nullptr if none
        void enqueueTXPacket(const char* pkt);  // const because we are only reading it

        bool hasPacket() const { return !_rxPacketQueue.isEmpty(); }
        uint16_t queuedCount() const { return _rxPacketQueue.size(); }

        private:
        PacketQueue<MAX_PACKETS, RX_BUF_SIZE> _rxPacketQueue;
        PacketQueue<MAX_PACKETS, RX_BUF_SIZE> _txPacketQueue;

        char rxBuf[RX_BUF_SIZE] = {0};
        char pktBuf[RX_BUF_SIZE] = {0};
        uint16_t rxPos = 0;

};



#endif