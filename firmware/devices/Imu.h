// Includeguard
#ifndef IMU_H_INCLUDE
#define IMU_H_INCLUDE
#include "../communications/helpers.h"
#include "MPU9150/MPU9150.h"
#include "BNO055/Adafruit_BNO055.h"


// #define MPU9150_NO_DIFF

volatile uint8_t interruptsFired = 0;
void onInterrupt() {
    interruptsFired += 1;
}

class Imu {
public:
    Imu(uint8_t id_)
        : id(id_) {
    }

    //loop until imu is ready!
    virtual uint8_t initialize();

    //get data if available (following Serial Protocol --> 20 bytes)
    // true if data was written in given pointer-location
    virtual bool getData(char* buffer);

    // prepare fetching data (e.g. send address to bus)
    virtual void prepare() {}

    uint8_t getId() {
        return id;
    }

private:
    uint8_t id;
};

class ImuMpu9150: public Imu {
public:
    ImuMpu9150(uint8_t id_, uint8_t address = 0x69, uint8_t interruptPin = 14)
        : Imu(id_),
          mpu(address) {
        interrupt = interruptPin;
    }
    uint8_t initialize() {
        Wire.begin();

        while (!mpu.isReady()) {
            Serial.println("Imu is not ready!");
            delay(1000);
        }

        pinMode(interrupt, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(interrupt), onInterrupt, RISING);

        uint8_t err = mpu.initialiseDMP();
        if (err) {
            return 1;
        }

        mpu.setFifoReset(true);
        mpu.setDMPEnabled(true);
        return 0;
    }

    bool getData(char* buffer) {
        if (interruptsFired > 0) {
            interruptsFired -= 1;
            mpu.getInterruptStatus(); // clear the interrupt status
            if (mpu.getFifoCount() >= 48) {
                mpu.getFifoBuffer(fifo,  48);
                motionAppsBufferToPacket(fifo, buffer);
                return true;
            }
        }
        return false;
    }
private:
    uint8_t id;
    MPU9150 mpu;
    char fifo[48];
    uint8_t interrupt = 0;

};

void read(TwoWire& bus, uint8_t address, uint8_t reg, byte* buffer, uint8_t length) {
    // request
    bus.beginTransmission(address);
    bus.write(reg);
    bus.endTransmission();

    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = bus.read();
    }
}

class ImuBno055: public Imu {
public:
    ImuBno055(uint8_t id_, TwoWire& bus_, bool ad0PulledLow = false)
        : Imu(id_)
        , bus(bus_)
        , address(ad0PulledLow ? 0x28 : 0x29)
        , bno(bus, -1, address) {
    }

    uint8_t initialize() {
        uint8_t err = bno.begin();
        if (!err) {
            bno.setExtCrystalUse(false);
        }
        return err;
    }

    void prepare() {
        bus.beginTransmission(address);
        bus.write(0x14);
        bus.endTransmission();
    }

    bool getData(char* buffer) {
        //   gyro 0x14..0x19 6  -> 0
        //   acc  0x28..0x2D 6  -> 20
        //   quat 0x20..0x27 8  -> 12

        // optimized version: fetch all values at once (includes eulers when fetching, but whatever)
        bus.requestFrom(address, 26);

        // gryo
        for (uint8_t i = 0; i < 6; i++) {
            buffer[i + 6] = bus.read();
        }

        // euler
        for (uint8_t i = 0; i < 6; i++) {
            bus.read(); // drop data
        }

        // quat (in different byte order, hence manually reorder)
        buffer[13] = bus.read();
        buffer[12] = bus.read();
        buffer[15] = bus.read();
        buffer[14] = bus.read();
        buffer[17] = bus.read();
        buffer[16] = bus.read();
        buffer[19] = bus.read();
        buffer[18] = bus.read();

        // accel
        for (uint8_t i = 0; i < 6; i++) {
            buffer[i] = bus.read();
        }

        return true;
    }

private:
    uint8_t address;
    Adafruit_BNO055 bno;
    TwoWire& bus;
};
#endif
