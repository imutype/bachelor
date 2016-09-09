#include <Arduino.h>
#include "../firmware/devices/EEPROM/EEPROM.h"

#include <Wire.h>
#include "wiring_private.h" // pinPeripheral() function

#define EEPROM_SIZE 2048
#define PAGE_SIZE 32
#define LED_PIN 13

#define WIRE_PIN_SDA 11
#define WIRE_PIN_SCL 13
TwoWire wire1(&sercom3, WIRE_PIN_SDA, WIRE_PIN_SCL);

#define EEPROM_WIRE wire1

EEPROM rom(0, EEPROM_SIZE, PAGE_SIZE, EEPROM_WIRE);
byte page[PAGE_SIZE];

#define FLASH(x) digitalWrite(LED_PIN, 1); delay(50); digitalWrite(LED_PIN, 0); delay(x);

#define DUMP
// #define CLEAR
#define WRITE

void setup() {
    EEPROM_WIRE.begin();
    pinPeripheral(WIRE_PIN_SDA, PIO_SERCOM_ALT);
    pinPeripheral(WIRE_PIN_SCL, PIO_SERCOM_ALT);

    Serial.begin(115200);
    while(!Serial);
    pinMode(LED_PIN, OUTPUT);
    FLASH(50); FLASH(50); FLASH(50);

#ifdef DUMP
    // dump the whole memory
    for (uint16_t i = 0; i < EEPROM_SIZE; i++) {
        Serial.write(rom.read(i));
    }
    Serial.flush();
    FLASH(50); FLASH(50); FLASH(50);
#endif

#ifdef CLEAR
    for (uint16_t i = 0; i < EEPROM_SIZE; i += PAGE_SIZE) {
        rom.write(i, page, PAGE_SIZE);
    }
    FLASH(50); FLASH(50); FLASH(50);
#endif

#ifdef WRITE
    // write into memory

    // LED on = ready to read
    digitalWrite(LED_PIN, 1);
    while (!Serial.available()) delay(10);
    digitalWrite(LED_PIN, 0);

    // LED off = reading
    uint16_t bytesWritten = 0;
    while (bytesWritten < EEPROM_SIZE) {
        for(uint8_t i = 0; i < PAGE_SIZE; i++) {
            while (!Serial.available()) {
                delay(10);
            }
            page[i] = Serial.read();
        }

        rom.write(bytesWritten, page, PAGE_SIZE);
        delay(50);
        bytesWritten += PAGE_SIZE;
    }
#endif

    // blink = done
    while (true) {
        FLASH(50);
        FLASH(50);
        FLASH(50);
        FLASH(1000);
    }
}

void loop() {
}
