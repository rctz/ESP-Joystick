# Serial Reader (C++)

This program is a C++ version of the Python code for reading joystick data from a serial port.

## BluePad32 Library Installation

### Arduino IDE Setup
1. Add board packages in Arduino IDE (File > Preferences > Additional Boards Manager URLs):
   - ESP32: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - Bluepad32: `https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json`

2. Install both packages via Tools > Board > Boards Manager
3. Select "ESP32 + Bluepad32 Arduino" board from Tools > Board
4. Open example: File > Examples > Bluepad32_ESP32 > Controller
5. Select `esp32_bluepad32` -> `DOIT ESP32 DEVKIT V1` 


**Compatible with ESP32 / ESP32-S3 / ESP32-C3 modules**

## Compilation (MinGW/MSYS2)
```bash
g++ -o serial_reader.exe serial_reader.cpp
```

## Compilation (Visual Studio)
```bash
cl /EHsc serial_reader.cpp
```

## Usage
1. Connect device to COM3 port (or modify code for desired port)
2. Run the program
3. The program will display joystick data received from ESP

## Features
- Serial communication support at 115200 baud rate
- Start byte verification (0xAA)
- Data integrity verification with CRC
- Binary to variable data conversion
- Joystick data display (buttons, misc, dpad, lx, ly, rx, ry)

## Data Structure
- **Buttons**: 16-bit (various buttons)
- **Misc**: 8-bit (additional information)
- **Dpad**: 8-bit (D-pad direction)
- **LX, LY**: 16-bit signed (left analog stick X, Y axes)
- **RX, RY**: 16-bit signed (right analog stick X, Y axes)