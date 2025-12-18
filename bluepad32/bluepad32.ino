#include <Bluepad32.h>

struct __attribute__((packed)) JoyPacket {
  uint8_t header; // 0xAA
  uint8_t length; // number of payload bytes
  uint16_t buttons;
  uint8_t misc;
  uint8_t dpad;
  int16_t lx;
  int16_t ly;
  int16_t rx;
  int16_t ry;
  uint8_t crc;
};

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

uint8_t computeCRC(uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
  }
  return crc;
}

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      Serial.printf("Controller connected at index=%d\n", i);
      return;
    }
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      Serial.printf("Controller disconnected from index=%d\n", i);
      return;
    }
  }
}

void processGamepad(ControllerPtr ctl) {
  JoyPacket pkt;
  pkt.header = 0xAA;
  pkt.length = 12;

  pkt.buttons = ctl->buttons();
  pkt.misc = ctl->miscButtons();
  pkt.dpad = ctl->dpad();
  pkt.lx = ctl->axisX();
  pkt.ly = ctl->axisY();
  pkt.rx = ctl->axisRX();
  pkt.ry = ctl->axisRY();

  pkt.crc = computeCRC((uint8_t *)&pkt, sizeof(pkt) - 1);

  Serial.write((uint8_t *)&pkt, sizeof(pkt));
}

void processControllers() {
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->hasData()) {
      if (ctl->isGamepad()) {
        processGamepad(ctl);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  delay(200);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
}

void loop() {

  bool dataUpdated = BP32.update();

  if (dataUpdated) {
    processControllers();
  }

  delay(5);
}