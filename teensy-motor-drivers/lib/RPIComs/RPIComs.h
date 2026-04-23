#ifndef RPIComs_h
#define RPIComs_h

#include <Arduino.h>
#include "Queue.h"
#include "motorDefs.h"

#define MOTOR_COUNT TOTAL_COUNT
#define NUM_INT16 (1 + MOTOR_COUNT) // 1 for flag, rest for motor inputs
#define PACKET_SIZE (NUM_INT16 * sizeof(int16_t))


enum FLAG_BYTE{
    START_BYTE = 1,
    STOP_BYTE = -1,
    CONTINUTE_BYTE = 0,
};

class RPIComs{
    public:
        static constexpr size_t RX_BUF_SIZE   = 256; // shorten this when we know the constant packet size?
        static constexpr size_t MAX_PACKETS   = 8;
        static const uint16_t TX_BUF_SIZE = PACKET_SIZE;

        // RPIComs() = default;

        int uartRead();
        void uartSend();
        const uint8_t* getPacket();     // returns nullptr if none
        void enqueueTXPacket(const char* pkt);  // const because we are only reading it

        bool hasPacket() const { return !_rxPacketQueue.isEmpty(); }
        uint16_t queuedCount() const { return _rxPacketQueue.size(); }

        private:
        PacketQueue<MAX_PACKETS, RX_BUF_SIZE> _rxPacketQueue;
        PacketQueue<MAX_PACKETS, RX_BUF_SIZE> _txPacketQueue;

        uint8_t rxBuf[RX_BUF_SIZE] = {0};
        uint8_t pktBuf[RX_BUF_SIZE] = {0};
        uint16_t rxPos = 0;

};



#endif