import serial
import struct

ser = serial.Serial('COM3', 115200)

while True:
    # Wait for start byte
    if ser.read(1) != b'\xAA':
        continue

    length = ser.read(1)[0]
    payload = ser.read(length)
    crc_recv = ser.read(1)[0]

    crc_calc = 0
    for b in (b'\xAA' + bytes([length]) + payload):
        crc_calc ^= b

    if crc_calc != crc_recv:
        print("CRC mismatch! Packet dropped.")
        continue

    # Unpack payload
    buttons, misc, dpad, lx, ly, rx, ry = struct.unpack("HBBhhhh", payload)
    print(buttons, misc, dpad, lx, ly, rx, ry)
