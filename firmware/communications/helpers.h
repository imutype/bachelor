#ifndef _HELPERS_H
#define _HELPERS_H

void motionAppsBufferToPacket(char* buffer, char* packet) {
    // accel [X, Y, Z]
    packet[0] = buffer[34];
    packet[1] = buffer[35];
    packet[2] = buffer[38];
    packet[3] = buffer[39];
    packet[4] = buffer[42];
    packet[5] = buffer[43];

    // gyro [X, Y, Z]
    packet[6] = buffer[16];
    packet[7] = buffer[17];
    packet[8] = buffer[20];
    packet[9] = buffer[21];
    packet[10] = buffer[24];
    packet[11] = buffer[25];

    // quaternion
    // memcpy(packet + 12, buffer, 16);
    packet[12] = buffer[0];
    packet[13] = buffer[1];

    packet[14] = buffer[4];
    packet[15] = buffer[5];

    packet[16] = buffer[8];
    packet[17] = buffer[9];

    packet[18] = buffer[12];
    packet[19] = buffer[13];
}

#endif
