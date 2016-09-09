#ifndef _I2C_H
#define _I2C_H

#include <Arduino.h>
#include <Wire.h>

#define BUS(num) Wire

#ifndef I2CDEVICE_WIRE_BUFFER_LENGTH
#define I2CDEVICE_WIRE_BUFFER_LENGTH 32
#endif

class I2cDevice {
public:
    I2cDevice(uint8_t bus_, uint8_t address_);

    // inspect instance
    uint8_t getBus();
    uint8_t getAddress();

    // main methods
    bool writeBytes(char reg, char* data, uint16_t length);
    bool readBytes(char reg, char* data, uint16_t length);

    // helpers
    bool getBit(char reg, uint8_t bit);
    uint8_t getBits(char reg, uint8_t bit, uint8_t length);
    int8_t getByte(char reg);
    int16_t getWord(char reg);
    int16_t getWordLittleEndian(char reg);

    void setBit(char reg, uint8_t bit, bool value);
    void setBits(char reg, uint8_t bit, uint8_t length, uint8_t value);
    bool setByte(char reg, uint8_t data);

private:
    uint8_t bus;
    uint8_t address;
};

#endif
