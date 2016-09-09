#include "i2c.h"

I2cDevice::I2cDevice(uint8_t bus_, uint8_t address_)
    : bus(bus_)
    , address(address_)
    {}

uint8_t I2cDevice::getBus() {
    return bus;
}

uint8_t I2cDevice::getAddress() {
    return address;
}

bool I2cDevice::getBit(char reg, uint8_t bit) {
    return !!(getByte(reg) & (1 << bit));
}

uint8_t I2cDevice::getBits(char reg, uint8_t bit, uint8_t length) {
    uint8_t b = getByte(reg);
    uint8_t mask = ((1 << length) - 1) << (bit - length + 1);
    b &= mask;
    b >>= (bit - length + 1);
    return b;
}

void I2cDevice::setBit(char reg, uint8_t bit, bool value) {
    char byte = getByte(reg);
    if (value) byte |= 1 << bit;
    else byte &= ~(1 << bit);
    writeBytes(reg, &byte, 1);
}

void I2cDevice::setBits(char reg, uint8_t bit, uint8_t length, uint8_t value) {
    uint8_t b = getByte(reg);
    uint8_t mask = ((1 << length) - 1) << (bit - length + 1);
    value <<= (bit - length + 1);
    value &= mask;
    b &= ~(mask);
    b |= value;
    setByte(reg, b);
}

int8_t I2cDevice::getByte(char reg) {
    char data[1];
    readBytes(reg, data, 1);
    return data[0];
}

bool I2cDevice::setByte(char reg, uint8_t data) {
    return writeBytes(reg, (char*)&data, 1);
}

int16_t I2cDevice::getWord(char reg) {
    char data[2];
    readBytes(reg, data, 2);
    return (data[0] << 8) + data[1];
}

int16_t I2cDevice::getWordLittleEndian(char reg) {
    char data[2];
    readBytes(reg, data, 2);
    return (data[1] << 8) + data[0];
}

bool I2cDevice::writeBytes(char reg, char* data, uint16_t length) {
    BUS(bus).beginTransmission(address);
    BUS(bus).write((uint8_t) reg); // send address

    for (uint8_t i = 0; i < length; i++) {
        BUS(bus).write((uint8_t) data[i]);
    }
    uint8_t status = BUS(bus).endTransmission();
    return status == 0;
}

bool I2cDevice::readBytes(char reg, char* data, uint16_t length) {
    uint16_t count = 0;
    for (uint8_t k = 0; k < length; k += min(length, I2CDEVICE_WIRE_BUFFER_LENGTH)) {
        BUS(bus).beginTransmission(address);
        BUS(bus).write(reg);
        BUS(bus).endTransmission();

        BUS(bus).beginTransmission(address);
        BUS(bus).requestFrom(address, (uint8_t)min(length - k, I2CDEVICE_WIRE_BUFFER_LENGTH));

        for (; BUS(bus).available(); count++) {
            data[count] = BUS(bus).read();
        }

        BUS(bus).endTransmission();
    }
    return true;
}
