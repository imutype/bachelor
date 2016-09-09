#include <Arduino.h>
#include "communications/pc.h"
#include "devices/Imu.h"

#include <Wire.h>
#include "wiring_private.h" // pinPeripheral() function

/*
 * Define I2C bus interfaces.
 */
TwoWire wire0(&sercom0, A3, A4);
TwoWire wire1(&sercom3, 11, 13);
TwoWire wire2(&sercom5, 20, 21);

/**
 *  Which I2C Bus the EEPROM for reading Wifi credentials  is connected to. Comment
 *  out to disable Wifi connection and send all data via Serial port.
 */
// #define WIFI_EEPROM_BUS wire1

// the connection to the host computer
PC pc;

// a buffer for data fetched from the IMU
char imuData[20];

// a list of the IMUs connected
uint8_t imusLength = 6;
Imu* imus[] = {
    new ImuBno055(0, wire2, false),
    new ImuBno055(1, wire2, true),
    new ImuBno055(2, wire1, false),
    new ImuBno055(3, wire1, true),
    new ImuBno055(4, wire0, false),
    new ImuBno055(5, wire0, true),
};

void setup() {
    Serial.begin(9600);
    while(!Serial);

    // wait for BNOs to boot and know their addresses
    // delay(1000);

    // boot i2c buses, and set bus speed to 400kHz
    wire0.begin();
    wire0.setClock(400000L);

    wire1.begin();
    wire1.setClock(400000L);

    wire2.begin();
    wire2.setClock(400000L);

    delay(200);

    pinPeripheral(A3, PIO_SERCOM_ALT); // SERCOM0.0 (alt)
    pinPeripheral(A4, PIO_SERCOM_ALT); // SERCOM0.1 (alt)
    pinPeripheral(11, PIO_SERCOM_ALT); // SERCOM3.0 (alt)
    pinPeripheral(13, PIO_SERCOM_ALT); // SERCOM3.1 (alt)
    pinPeripheral(20, PIO_SERCOM_ALT); // SERCOM5.0 (alt)
    pinPeripheral(21, PIO_SERCOM_ALT); // SERCOM5.1 (alt)
    delay(1000);

    #ifdef WIFI_EEPROM_BUS
        Serial.println("Connecting to Wifi");
        EEPROM rom(0, 2048, 32, WIFI_EEPROM_BUS);
        pc.setupWifi(rom);
        Serial.println("Connected");
    #endif

    pc.setImuCount(imusLength);
    pc.sendHeader();

    uint8_t err;
    for (uint8_t i = 0; i < imusLength; i++) {
        pc.sendMessage(0, "Imu %d initializing", imus[i]->getId());

        err = imus[i]->initialize();
        if (err) {
            pc.sendMessage(0, "Imu initialization error: %d", err);
            while (true);
        }

        pc.sendMessage(0, "Imu %d initialized", imus[i]->getId());
        delay(500);
    }

    #ifdef WIFI_EEPROM_BUS
        Serial.println("Initialization done, starting loop.");
    #endif
    // pc.sendMessage(0x01, "IMUs initialized, starting main loop");
}

void loop() {
    for (uint8_t i = 0; i < imusLength; i ++){
        imus[i]->prepare();
        if(imus[i]->getData(imuData)){
            pc.sendImuEvent(imus[i]->getId(), imuData);
        }
    }
}
