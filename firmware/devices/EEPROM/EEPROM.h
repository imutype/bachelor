#include <Arduino.h>
#include <Wire.h>

class EEPROM {
public:
  EEPROM(uint8_t chipSelect_ = 0, uint16_t size_ = 8192, uint16_t pageSize_ = 32, TwoWire& bus_ = Wire);
  byte read(uint16_t reg);
  void write(uint16_t reg, byte data);
  void read(uint16_t reg, byte* buffer, uint16_t length);
  void write(uint16_t reg, byte* buffer, uint16_t length);

private:
  uint8_t chipSelect;

  uint16_t size;
  uint16_t pageSize;

  TwoWire& bus;
};
