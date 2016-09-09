import io
import struct
import sys
import socket

HEADER = b'\x49\xd2\xc8\xce'

TRANSMISSION_START = 0xc1
TRANSMISSION_STOP  = 0x1c

TRANSMISSION_TYPE_MESSAGE   = 0x01
TRANSMISSION_TYPE_IMU_EVENT = 0x02
TRANSMISSION_TYPE_KEY_EVENT = 0x03

class EndOfStream(Exception):
    pass

class Parser:
    def __init__(self, filename, isUdp=False):
        self.filename = filename

        if isUdp:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if filename:
                parts = filename.split(':')
                if len(parts) == 2:
                    host, port = parts[0], parts[1]
                else:
                    host, port = filename, 1444
            else:
                host, port = '0.0.0.0', 1444
            self.socket.bind((host, port))
            file = self.socket.fileno()
        else:
            file = filename or sys.stdin.fileno()

        self.stream = io.open(file, "rb", buffering=50)

        self.accelScale = 4
        self.gyroScale = 500
        self.version = 3
        self.numberImu = 6

    def __iter__(self):
        try:
            while True:
                if self.stream.peek(4)[:4] == HEADER:
                    self.headHandler()
                yield self.transmissionHandler()
        except EndOfStream:
            pass

    def headHandler(self):
        if not (self.stream.read(4) == HEADER) :
            raise Exception('Invalid magic header')

        self.version = self.read(1, 'B')
        self.numberImu = self.read(1, 'B')
        self.accelScale = self.read(2, '<h')
        self.gyroScale = self.read(2, '<h')
        self.read(22)

        # print("Got header. Version {}, imu count {}, accelScale {}, gyroScale {}.".format(
        #     self.version, self.numberImu, self.accelScale, self.gyroScale))

    def read(self, count=1, fmt=None):
        result = self.stream.read(count)

        if len(result) < count:
            raise EndOfStream()

        if fmt:
            result = struct.unpack(fmt, result)[0]

        return result

    def transmissionHandler(self):
        if self.read(1, 'B') != TRANSMISSION_START:
            raise Exception('Corrupt transmission: Faulty start of transmission')

        transmissionType = self.read(1, 'B')
        timeStamp = self.read(4, '<L')

        if transmissionType == TRANSMISSION_TYPE_MESSAGE:
            code = self.read(2, '<H')
            length = self.read(2, '<H')
            message = self.stream.read(length)
            result = Message(timeStamp, code, message)
        elif transmissionType == TRANSMISSION_TYPE_IMU_EVENT:
            rawBytes = self.read(21)
            result = ImuEvent(rawBytes, self.accelScale, self.gyroScale, timeStamp)
        elif transmissionType == TRANSMISSION_TYPE_KEY_EVENT:
            keyCode = self.read(2, '<H')
            pressed = bool(self.read(1, 'B'))
            result = KeyEvent(timeStamp, keyCode, pressed)
        else:
            raise Exception('Invalid transmission type: ' + str(transmissionType))

        if not self.read(1, 'B') == TRANSMISSION_STOP:
            raise Exception('Corrupt Transmission: Faulty end of transmission')

        return result

def parseVector(bytes, scale):
    x = parseInt(bytes[0:2]) * 1.0 / (2 ** 15) * scale
    y = parseInt(bytes[2:4]) * 1.0 / (2 ** 15) * scale
    z = parseInt(bytes[4:6]) * 1.0 / (2 ** 15) * scale
    return x, y, z

def parseQuaternion(bytes):
    w = parseQuaternionValue(bytes[0:2])
    x = parseQuaternionValue(bytes[2:4])
    y = parseQuaternionValue(bytes[4:6])
    z = parseQuaternionValue(bytes[6:8])
    return w,x,y,z

class Message:
    def __init__(self, timeStamp, code, message):
        self.timeStamp = timeStamp
        self.code = code
        self.message = message

class KeyEvent:
    def __init__(self, timeStamp, keyCode, pressed):
        self.timeStamp = timeStamp
        self.keyCode = keyCode
        self.message = message

class ImuEvent:
    def __init__(self, bytes, accelScale, gyroScale, timeStamp):
        self.imuId = parseByte(bytes[0:1])
        self.accel = parseVector(bytes[1:7],accelScale)
        self.gyro = parseVector(bytes[7:13], gyroScale)
        self.quat = parseQuaternion(bytes[13:21])
        self.timeStamp = timeStamp

    def __repr__(self):
        return 'A: {}  G: {}  Q: {}'.format(self.accel, self.gyro, self.quat)

def parseByte(bytes):
    return struct.unpack('B', bytes)[0]

def parseInt(bytes, signed=True):
    return struct.unpack('<h' if signed else '<H', bytes)[0]

def parseQuaternionValue(bytes):
    return struct.unpack('>h', bytes)[0] * 1.0 / (2 ** 14)
