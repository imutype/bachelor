#include "EEPROM.h"

EEPROM::EEPROM(uint8_t chipSelect_, uint16_t size_, uint16_t pageSize_, TwoWire& bus_)
    : chipSelect(chipSelect_)
    , size(size_)
    , pageSize(pageSize_)
    , bus(bus_) {
}

uint16_t nextPage(uint16_t x, uint16_t size) {
    return (x + size) - (x % size);
}

byte EEPROM::read(uint16_t reg) {
    reg %= size; // wrap around end of memory

    // set the 24C256 eeprom address to 0
    bus.beginTransmission(80 | chipSelect);
    bus.write(reg >> 8);  // address low byte
    bus.write(reg & 0xFF);  // address high byte
    bus.endTransmission();

    // read 1 byte, from address 0
    bus.requestFrom(80 | chipSelect, 1);
    byte num = 0x00;
    while(bus.available()) {
        num = bus.read();
    }
    return num;
}

void EEPROM::write(uint16_t reg, byte data) {
    reg %= size; // wrap around end of memory

    // write "num" to 24C256 eeprom at address zero
    bus.beginTransmission(80 | chipSelect);
    bus.write(reg >> 8);    // address low byte
    bus.write(reg & 0xFF);    // address high byte
    bus.write(data);  // any more write starts writing
    bus.endTransmission();
}

void EEPROM::read(uint16_t reg, byte* buffer, uint16_t length) {
    reg %= size; // wrap around end of memory

    bus.beginTransmission(80 | chipSelect);
    bus.write(reg >> 8);  // address low byte
    bus.write(reg & 0xFF);  // address high byte
    bus.endTransmission();

    // read 1 byte, from address 0
    bus.requestFrom(80 | chipSelect, length);
    for(uint16_t i = 0; i < length && bus.available(); i++) {
        buffer[i] = bus.read();
    }
}

void EEPROM::write(uint16_t reg, byte* buffer, uint16_t length) {
    reg %= size; // wrap around end of memory

    uint8_t bytesWritten = 0;

    for (uint16_t offset = reg; offset < reg + length; offset = nextPage(offset, pageSize)) {
        bus.beginTransmission(80 | chipSelect);
        bus.write((offset % size) >> 8);
        bus.write((offset % size) & 0xFF);
        for (uint16_t i = offset; i < nextPage(offset, pageSize); i++) {
            bus.write(buffer[bytesWritten]);
            bytesWritten++;
        }
        bus.endTransmission(80 | chipSelect);
        delay(50);
    }
    delay(200);
}
