#include <Bluepad32.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

struct __attribute__((packed)) JoyPacket {
    uint8_t header;      // 0xAA
    uint8_t length;      // number of payload bytes (11)
    uint16_t buttons;
    uint8_t misc;
    uint8_t dpad;
    int16_t lx;
    int16_t ly;
    int16_t rx;
    int16_t ry;
    uint8_t crc;
};

JoyPacket joy; 

// Mutex for safe cross-core access
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

uint8_t computeCRC(uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            myControllers[i] = ctl;
            Serial.printf("Controller connected at index=%d\n", i);
            return;
        }
    }
    Serial.println("No empty slot for controller!");
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
    portENTER_CRITICAL(&mux);

    joy.header  = 0xAA;     // start byte
    joy.length  = 12;       // payload size

    joy.buttons = ctl->buttons();
    joy.misc    = ctl->miscButtons();
    joy.dpad    = ctl->dpad();

    joy.lx = ctl->axisX();
    joy.ly = ctl->axisY();
    joy.rx = ctl->axisRX();
    joy.ry = ctl->axisRY();

    portEXIT_CRITICAL(&mux);
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

void joystickTask(void *p) {
    for (;;) {
        if (BP32.update()) {
            processControllers();
        }
        vTaskDelay(1);  // non-blocking
    }
}

void serialSenderTask(void *p) {
    for (;;) {

        JoyPacket pkt;

        // Safely copy shared struct
        portENTER_CRITICAL(&mux);
        pkt = joy;
        portEXIT_CRITICAL(&mux);

        // Calculate CRC for all bytes except CRC itself
        pkt.crc = computeCRC((uint8_t*)&pkt, sizeof(pkt) - 1);

        // Send complete packet
        Serial.write((uint8_t*)&pkt, sizeof(pkt));

        vTaskDelay(50);   // ~200 Hz
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);

    Serial.printf("Bluepad32 Firmware: %s\n", BP32.firmwareVersion());

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);

    // Create tasks pinned to different cores
    xTaskCreatePinnedToCore(joystickTask, "joyReader", 4096, NULL, 2, NULL, 0); // Core 0
    xTaskCreatePinnedToCore(serialSenderTask, "serialSender", 4096, NULL, 1, NULL, 1); // Core 1
}

void loop() {
    vTaskDelay(1000);
}
