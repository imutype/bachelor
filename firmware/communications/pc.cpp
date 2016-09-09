#include "pc.h"

#include <WiFi101.h>
#include <WiFiUdp.h>
#include <stdio.h>
#include <stdarg.h>

PC::PC()
    : accelScale(4)
    , gyroScale(500)
    , imuCount(1) {}

uint16_t readUntil(char* input, char delimiter, char* output) {
    uint16_t i = 0;
    while (true) {
        if (input[i] == delimiter) {
            output[i] = 0;
            return i;
        }

        if (input[i] == 0) {
            return -1;
        }

        output[i] = input[i];
        i++;
    }
}

void PC::setupWifi(EEPROM& rom) {
    WiFi.setPins(8, 7, 4, 2);

    char buffer[2048];

    // read eeprom into buffer until 0 found
    Serial.print("Reading EEPROM... ");
    uint16_t bufferLength = 0;
    for (; bufferLength < 2048; ++bufferLength) {
        buffer[bufferLength] = rom.read(bufferLength);
        if (buffer[bufferLength] == 0) {
            break;
        }
    }
    ++bufferLength;
    Serial.print("read ");
    Serial.print(bufferLength);
    Serial.println(" bytes.");

    Serial.print("Booting wifi... ");
    if (WiFi.begin() == WL_NO_SHIELD) {
        Serial.println("Error: WiFi shield not present");
        while (true);
    }
    Serial.println("done.");

    // scan networks
    Serial.print("Scanning networks... ");
    uint8_t ssidCount = WiFi.scanNetworks();
    Serial.print("found ");
    Serial.print(ssidCount);
    Serial.println(" networks.");

    char ssid[64];
    char passphrase[64];
    char ip[64];
    char port[64];

    // walk through the list of networks and see if it is available
    for(uint16_t bufferOffset = 0; bufferOffset < bufferLength;) {
        bufferOffset += readUntil(buffer + bufferOffset, ':', ssid) + 1;
        bufferOffset += readUntil(buffer + bufferOffset, ':', passphrase) + 1;
        bufferOffset += readUntil(buffer + bufferOffset, ':', ip) + 1;
        bufferOffset += readUntil(buffer + bufferOffset, '\n', port) + 1;

        for (uint8_t i = 0; i < ssidCount; i++) {
            if (strcmp(WiFi.SSID(i), ssid) == 0) {
                IPAddress address;
                address.fromString(ip);

                uint16_t portNumber = String(port).toInt();

                setupWifi(ssid, passphrase, address, portNumber);
                return;
            }
        }
    }
}

void PC::setupWifi(char* ssid, char* passphrase, const IPAddress& ip, uint16_t port) {
    Serial.print("Connecting to ");
    Serial.println(ssid);

    useWifi = true;
    wifiSsid = ssid;
    wifiPassphrase = passphrase;
    targetIp = ip;
    targetPort = port;

    delay(100);
    if (WiFi.status() == WL_NO_SHIELD) {
        Serial.println("WiFi shield not present");
        while (true);
    }

    // blink LED while connecting
    pinMode(13, OUTPUT);

    Serial.print("Connecting to WiFi '");
    Serial.print(ssid);
    Serial.print("'");

    uint8_t status = WiFi.begin(ssid, passphrase);

    while (status != WL_CONNECTED) {
        Serial.print(".");
        digitalWrite(13, 1);
        delay(500);
        digitalWrite(13, 0);
        delay(500);
        status = WiFi.status();
    }

    // LED ON = connected
    digitalWrite(13, 1);

    Serial.println(" Done.");

    // todo: listen to incoming connections, keey a list of targets to talk to
    wifiUdp.begin(8080);
}

void PC::setAccelScale(uint16_t scale) {
    accelScale = scale;
}

void PC::setGyroScale(uint16_t scale) {
    gyroScale = scale;
}

void PC::setImuCount(uint8_t count) {
    imuCount = count;
}

void PC::sendHeader() {
    // magic header
    buffer[0] = 0x49;
    buffer[1] = 0xd2;
    buffer[2] = 0xc8;
    buffer[3] = 0xce;

    // version
    buffer[4] = PROTOCOL_VERSION;

    buffer[5] = imuCount;

    buffer[6] = (accelScale & 0xFF);
    buffer[7] = (accelScale >> 8);

    buffer[8] = (gyroScale & 0xFF);
    buffer[9] = (gyroScale >> 8);

    now = millis();
    memcpy(buffer + 10, &now, 4);

    for(uint8_t i = 14; i < 32; ++i) {
        buffer[i] = 0x00; // explicit zero
    }

    sendBuffer(32);
}

char messageBuffer[256];

void PC::sendMessage(uint16_t code, const char* message, ...) {
    va_list args;
    va_start(args, message);
    vsprintf(messageBuffer, message, args);

    uint8_t count = writeTransmissionHeader(TRANSMISSION_TYPE_MESSAGE);

    buffer[count++] = (code & 0xFF);
    buffer[count++] = (code >> 8);

    uint16_t length = 0;
    while (messageBuffer[length] != 0) {
        buffer[count + 2 + length] = messageBuffer[length];
        ++length;
    }

    buffer[count++] = (length & 0xFF);
    buffer[count++] = (length >> 8);
    count += length;

    buffer[count++] = TRANSMISSION_STOP;
    sendBuffer(count);

    va_end(args);
}

void PC::sendImuEvent(uint8_t imuNumber, char* data) {
    uint8_t count = writeTransmissionHeader(TRANSMISSION_TYPE_IMU_EVENT);

    buffer[count++] = imuNumber;

    memcpy(buffer + count, data, 20);
    count += 20;

    buffer[count++] = TRANSMISSION_STOP;
    sendBuffer(count);
}

void PC::sendKeyEvent(uint16_t keyCode, bool pressed) {
    uint8_t count = writeTransmissionHeader(TRANSMISSION_TYPE_KEY_EVENT);

    buffer[count++] = (keyCode & 0xFF);
    buffer[count++] = (keyCode >> 8);

    buffer[count++] = pressed ? 1 : 0;

    buffer[count++] = TRANSMISSION_STOP;
    sendBuffer(count);
}

uint8_t PC::writeTransmissionHeader(uint8_t transmissionType) {
    buffer[0] = TRANSMISSION_START;
    buffer[1] = transmissionType;
    now = millis();
    memcpy(buffer + 2, &now, 4);
    return 6;
}

void PC::sendBuffer(uint16_t length) {
    if (useWifi) {
        wifiUdp.beginPacket(targetIp, targetPort);
        wifiUdp.write(buffer, length);
        wifiUdp.endPacket();
        // Serial.print("Sent packet of length ");
        // Serial.print(length);
        // Serial.print(" to ");
        // Serial.print(targetIp);
        // Serial.print(":");
        // Serial.println(targetPort);
    } else {
        Serial.write(buffer, length);
    }
}
