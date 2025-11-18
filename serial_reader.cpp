#include <iostream>
#include <string>
#include <cstdint>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

// Data structure for joystick data
struct JoystickData {
    uint16_t buttons;
    uint8_t misc;
    uint8_t dpad;
    int16_t lx, ly, rx, ry;
};

class SerialReader {
private:
    int fd;
    const char* portName;
    int baudRate;

public:
    SerialReader(const char* port, int baud = 115200)
        : portName(port), baudRate(baud), fd(-1) {}

    ~SerialReader() {
        close();
    }

    bool open() {
        fd = ::open(portName, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            std::cerr << "Error opening serial port " << portName << ": " << strerror(errno) << std::endl;
            return false;
        }

        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) {
            std::cerr << "Error getting serial port attributes: " << strerror(errno) << std::endl;
            close();
            return false;
        }

        // Clear old settings
        tty.c_cflag &= ~PARENB;  // No parity
        tty.c_cflag &= ~CSTOPB;  // 1 stop bit
        tty.c_cflag &= ~CSIZE;   // Clear data size bits
        tty.c_cflag |= CS8;      // 8 data bits
        tty.c_cflag &= ~CRTSCTS; // No hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control

        // Disable software flow control
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);

        // Non-canonical mode
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;    // Disable echo
        tty.c_lflag &= ~ECHOE;   // Disable erasure
        tty.c_lflag &= ~ECHONL;  // Disable new-line echo
        tty.c_lflag &= ~ISIG;    // Disable signal characters

        // Disable output processing
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;   // Prevent conversion of newline to carriage return

        // Set timeouts (100ms read timeout)
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;    // 0.1 seconds

        // Set baud rate
        speed_t speed;
        switch (baudRate) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            default:
                std::cerr << "Unsupported baud rate: " << baudRate << std::endl;
                close();
                return false;
        }

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            std::cerr << "Error setting serial port attributes: " << strerror(errno) << std::endl;
            close();
            return false;
        }

        return true;
    }

    void close() {
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }

    bool readByte(uint8_t& byte) {
        ssize_t bytesRead = ::read(fd, &byte, 1);
        return bytesRead == 1;
    }

    bool readData(uint8_t* buffer, size_t length) {
        size_t totalBytesRead = 0;
        while (totalBytesRead < length) {
            ssize_t bytesRead = ::read(fd, buffer + totalBytesRead, length - totalBytesRead);
            if (bytesRead <= 0) {
                return false; // Error or timeout
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
    SerialReader serial("/dev/ttyUSB0");  // Change to /dev/ttyACM0 or other appropriate device

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
