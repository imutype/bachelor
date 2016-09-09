#ifndef _PC_H
#define _PC_H

// #include <WProgram.h>

#include <Arduino.h>
#include <WiFi101.h>
#include <WiFiUdp.h>

#include "../devices/EEPROM/EEPROM.h"

//
// Helpers for serial communication with the host PC.
//

#define PROTOCOL_VERSION 0x03
#define TRANSMISSION_START 0xC1
#define TRANSMISSION_STOP 0x1C

#define TRANSMISSION_TYPE_MESSAGE   0x01
#define TRANSMISSION_TYPE_IMU_EVENT 0x02
#define TRANSMISSION_TYPE_KEY_EVENT 0x03

class PC {
public:
    PC();

    // loads available wifi configurations from a connected EEPROM and tries the first
    // one it finds
    void setupWifi(EEPROM& rom);

    // connects to the wifi with the specified SSID/passphrase and sends subsequent transmissions
    // via UDP to the specified address:port
    void setupWifi(char* ssid, char* passphrase, const IPAddress& ip, uint16_t port);

    void setAccelScale(uint16_t scale);
    void setGyroScale(uint16_t scale);
    void setImuCount(uint8_t count);

    void sendHeader();

    void sendMessage(uint16_t code, const char* message, ...);
    void sendImuEvent(uint8_t imuNumber, char* data);
    void sendKeyEvent(uint16_t keyCode, bool pressed);

private:
    uint8_t writeTransmissionHeader(uint8_t transmissionType);
    void sendBuffer(uint16_t length);

    char buffer[100]; // let's see how much we need

    uint16_t accelScale;
    uint16_t gyroScale;
    uint8_t imuCount;
    uint32_t now;

    bool useWifi;
    IPAddress targetIp;
    uint16_t targetPort;
    char* wifiSsid;
    char* wifiPassphrase;
    WiFiUDP wifiUdp;
};

#endif
