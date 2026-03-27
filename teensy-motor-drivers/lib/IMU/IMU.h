#ifndef IMU_h
#define IMU_h

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

struct IMUData {
    float heading;
    float roll;
    float pitch;
    float qw;
    float qx;
    float qy;
    float qz;
    uint8_t cal_sys;
    uint8_t cal_gyro;
    uint8_t cal_accel;
    uint8_t cal_mag;
};

class IMU {
    public:
        IMU(int32_t sensorID = 55, uint8_t address = BNO055_ADDRESS_A);

        bool begin();

        // Phase 1: write register address to BNO055 TX buffer and send.
        // endTransmission(false) sends without a STOP condition, keeping
        // the bus held so the follow-up requestFrom can issue a restart.
        void requestUpdate();

        // Phase 2: read all bytes out of the RX buffer in one call.
        // Call after other work has been done in the loop.
        bool collectUpdate();

        IMUData getData()  const;
        int formatPacket(char* buf, size_t bufSize) const;
        bool isCalibrated() const;

        float getHeading() const { return _data.heading; }
        float getRoll()    const { return _data.roll;    }
        float getPitch()   const { return _data.pitch;   }
        
        bool isDoneCollecting() const { return !_requested; }

    private:
        Adafruit_BNO055 _bno;
        IMUData _data;
        uint8_t _address;
        bool _requested = false;

        static constexpr uint8_t READ_LEN = 18;
        static constexpr uint8_t START_REG = Adafruit_BNO055::BNO055_EULER_H_LSB_ADDR;

        void parseBuffer(uint8_t* buf);
};

#endif