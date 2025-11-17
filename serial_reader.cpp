#include <iostream>
#include <windows.h>
#include <string>
#include <cstdint>

// Data structure for joystick data
struct JoystickData {
    uint16_t buttons;
    uint8_t misc;
    uint8_t dpad;
    int16_t lx, ly, rx, ry;
};

class SerialReader {
private:
    HANDLE hSerial;
    const char* portName;
    DWORD baudRate;

public:
    SerialReader(const char* port, DWORD baud = CBR_115200)
        : portName(port), baudRate(baud), hSerial(INVALID_HANDLE_VALUE) {}

    ~SerialReader() {
        close();
    }

    bool open() {
        hSerial = CreateFileA(portName, GENERIC_READ, 0, NULL, OPEN_EXISTING, 0, NULL);
        if (hSerial == INVALID_HANDLE_VALUE) {
            std::cerr << "Error opening serial port " << portName << std::endl;
            return false;
        }

        DCB dcbSerialParams = {0};
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

        if (!GetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "Error getting serial port state" << std::endl;
            close();
            return false;
        }

        dcbSerialParams.BaudRate = baudRate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;

        if (!SetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "Error setting serial port parameters" << std::endl;
            close();
            return false;
        }

        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = 50;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        timeouts.WriteTotalTimeoutConstant = 50;
        timeouts.WriteTotalTimeoutMultiplier = 10;

        if (!SetCommTimeouts(hSerial, &timeouts)) {
            std::cerr << "Error setting serial port timeouts" << std::endl;
            close();
            return false;
        }

        return true;
    }

    void close() {
        if (hSerial != INVALID_HANDLE_VALUE) {
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
        }
    }

    bool readByte(uint8_t& byte) {
        DWORD bytesRead;
        if (!ReadFile(hSerial, &byte, 1, &bytesRead, NULL)) {
            return false;
        }
        return bytesRead == 1;
    }

    bool readData(uint8_t* buffer, size_t length) {
        DWORD totalBytesRead = 0;
        while (totalBytesRead < length) {
            DWORD bytesRead;
            if (!ReadFile(hSerial, buffer + totalBytesRead, length - totalBytesRead, &bytesRead, NULL)) {
                return false;
            }
            if (bytesRead == 0) {
                return false; // Timeout
            }
            totalBytesRead += bytesRead;
        }
        return true;
    }

    bool readPacket(JoystickData& data) {
        uint8_t byte;

        // Wait for start byte (0xAA)
        while (true) {
            if (!readByte(byte)) {
                return false;
            }
            if (byte == 0xAA) {
                break;
            }
        }

        // Read length
        uint8_t length;
        if (!readByte(length)) {
            return false;
        }

        // Read payload
        uint8_t payload[14]; // H(2) + B(1) + B(1) + h(2) + h(2) + h(2) + h(2) = 12 bytes
        if (length > sizeof(payload)) {
            std::cerr << "Payload too large: " << length << " bytes" << std::endl;
            return false;
        }

        if (!readData(payload, length)) {
            return false;
        }

        // Read CRC
        uint8_t crcRecv;
        if (!readByte(crcRecv)) {
            return false;
        }

        // Calculate CRC
        uint8_t crcCalc = 0;
        crcCalc ^= 0xAA; // Start byte
        crcCalc ^= length;
        for (int i = 0; i < length; i++) {
            crcCalc ^= payload[i];
        }

        if (crcCalc != crcRecv) {
            std::cerr << "CRC mismatch! Packet dropped." << std::endl;
            return false;
        }

        // Unpack payload (HBBhhhh format)
        data.buttons = (static_cast<uint16_t>(payload[1]) << 8) | payload[0];
        data.misc = payload[2];
        data.dpad = payload[3];
        data.lx = (static_cast<int16_t>(payload[5]) << 8) | payload[4];
        data.ly = (static_cast<int16_t>(payload[7]) << 8) | payload[6];
        data.rx = (static_cast<int16_t>(payload[9]) << 8) | payload[8];
        data.ry = (static_cast<int16_t>(payload[11]) << 8) | payload[10];

        return true;
    }
};

int main() {
    SerialReader serial("COM3");

    if (!serial.open()) {
        std::cerr << "Failed to open serial port" << std::endl;
        return 1;
    }

    std::cout << "Serial port opened. Waiting for data..." << std::endl;

    JoystickData data;
    while (true) {
        if (serial.readPacket(data)) {
            std::cout << "Buttons: " << data.buttons
                      << ", Misc: " << static_cast<int>(data.misc)
                      << ", Dpad: " << static_cast<int>(data.dpad)
                      << ", LX: " << data.lx
                      << ", LY: " << data.ly
                      << ", RX: " << data.rx
                      << ", RY: " << data.ry << std::endl;
        }
    }

    return 0;
}
