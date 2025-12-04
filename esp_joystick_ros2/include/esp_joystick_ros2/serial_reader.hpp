#ifndef ESP_JOYSTICK_ROS2__SERIAL_READER_HPP_
#define ESP_JOYSTICK_ROS2__SERIAL_READER_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace serial {
class Serial;
}

namespace esp_joystick_ros2 {

struct JoyData {
  uint16_t buttons;
  uint8_t misc;
  uint8_t dpad;
  int16_t lx;
  int16_t ly;
  int16_t rx;
  int16_t ry;

  JoyData() : buttons(0), misc(0), dpad(0), lx(0), ly(0), rx(0), ry(0) {}
};

class SerialReader {
public:
  SerialReader();
  ~SerialReader();

  bool connect(const std::string &port, int baud_rate);
  void disconnect();
  bool isConnected() const;

  bool readPacket(JoyData &data);
  JoyData getLatestData() const;

  std::string getErrorMessage() const;

  // Enable/disable CRC validation (for debugging)
  void setCrcValidationEnabled(bool enabled);
  bool isCrcValidationEnabled() const;

private:
  static constexpr uint8_t START_BYTE = 0xAA;
  static constexpr uint8_t EXPECTED_LENGTH = 12;
  static constexpr size_t PACKET_SIZE = 15; // header + length + payload + crc

  std::unique_ptr<serial::Serial> serial_;
  JoyData latest_data_;
  mutable std::mutex data_mutex_;
  mutable std::string error_message_;
  bool crc_validation_enabled_;

  uint8_t computeCRC(const uint8_t *data, size_t len) const;
  bool waitForStartByteAndReadPacket(uint8_t *packet);
  bool validatePacket(const uint8_t *packet) const;
  void parsePayload(const uint8_t *payload, JoyData &data) const;
};

} // namespace esp_joystick_ros2

#endif // ESP_JOYSTICK_ROS2__SERIAL_READER_HPP_
