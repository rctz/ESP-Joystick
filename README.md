# Serial Reader (C++)

โปรแกรมนี้เป็น C++ version ของโค้ด Python สำหรับอ่านข้อมูลจอยสติ๊กจากพอร์ตอนุกรม

## การคอมไพล์ (MinGW/MSYS2)
```bash
g++ -o serial_reader.exe serial_reader.cpp
```

## การคอมไพล์ (Visual Studio)
```bash
cl /EHsc serial_reader.cpp
```

## การใช้งาน
1. เชื่อมต่ออุปกรณ์กับพอร์ต COM3 (หรือแก้ไขโค้ดเป็นพอร์ตที่ต้องการ)
2. รันโปรแกรม
3. โปรแกรมจะแสดงข้อมูลจอยสติ๊กที่ได้รับจาก ESP

## ฟีเจอร์
- รองรับการสื่อสารแบบ serial ที่ 115200 baud rate
- ตรวจสอบ start byte (0xAA)
- ตรวจสอบความถูกต้องของข้อมูลด้วย CRC
- แปลงข้อมูลจาก binary format เป็นตัวแปรที่ใช้งานได้
- แสดงผลข้อมูลจอยสติ๊ก (buttons, misc, dpad, lx, ly, rx, ry)

## โครงสร้างข้อมูล
- Buttons: 16-bit (ปุ่มต่างๆ)
- Misc: 8-bit (ข้อมูลเพิ่มเติม)
- Dpad: 8-bit (ทิศทาง D-pad)
- LX, LY: 16-bit signed (แกน X, Y ของ analog stick ซ้าย)
- RX, RY: 16-bit signed (แกน X, Y ของ analog stick ขวา)
