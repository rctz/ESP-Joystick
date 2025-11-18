#include "esp_joystick_ros2/serial_reader.hpp"
#include <serial/serial.h>
#include <cstring>
#include <unistd.h>

namespace esp_joystick_ros2
{

SerialReader::SerialReader() : serial_(nullptr)
{
}

SerialReader::~SerialReader()
{
  disconnect();
}

bool SerialReader::connect(const std::string& port, int baud_rate)
{
  try {
    serial_ = std::make_unique<serial::Serial>(port, baud_rate, 
                                               serial::Timeout::simpleTimeout(1000));
    
    // Give some time for the connection to establish
    usleep(100000);  // 100ms
    
    if (serial_->isOpen()) {
      error_message_.clear();
      return true;
    } else {
      error_message_ = "Failed to open serial port: " + port;
      return false;
    }
  } catch (const std::exception& e) {
    error_message_ = "Serial connection error: " + std::string(e.what());
    return false;
  }
}

void SerialReader::disconnect()
{
  if (serial_ && serial_->isOpen()) {
    serial_->close();
  }
  serial_.reset();
}

bool SerialReader::isConnected() const
{
  return serial_ && serial_->isOpen();
}

bool SerialReader::readPacket(JoyData& data)
{
  if (!isConnected()) {
    error_message_ = "Serial port not connected";
    return false;
  }

  try {
    // Wait for start byte
    if (!waitForStartByte()) {
      return false;  // No valid packet start found
    }

    // Read complete packet
    uint8_t packet[PACKET_SIZE];
    if (!readPacketBytes(packet, PACKET_SIZE)) {
      error_message_ = "Failed to read complete packet";
      return false;
    }

    // Validate packet
    if (!validatePacket(packet)) {
      error_message_ = "CRC validation failed - packet dropped";
      return false;
    }

    // Parse payload
    parsePayload(packet + 2, data);  // Skip header and length bytes
    
    // Update latest data thread-safe
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_data_ = data;
    }
    
    error_message_.clear();
    return true;

  } catch (const std::exception& e) {
    error_message_ = "Error reading packet: " + std::string(e.what());
    return false;
  }
}

JoyData SerialReader::getLatestData() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return latest_data_;
}

std::string SerialReader::getErrorMessage() const
{
  return error_message_;
}

uint8_t SerialReader::computeCRC(const uint8_t* data, size_t len) const
{
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
  }
  return crc;
}

bool SerialReader::waitForStartByte()
{
  uint8_t byte;
  try {
    // Look for start byte with timeout
    while (true) {
      size_t bytes_read = serial_->read(&byte, 1);
      if (bytes_read == 0) {
        return false;  // Timeout
      }
      
      if (byte == START_BYTE) {
        return true;
      }
    }
  } catch (const std::exception& e) {
    error_message_ = "Error waiting for start byte: " + std::string(e.what());
    return false;
  }
}

bool SerialReader::readPacketBytes(uint8_t* buffer, size_t size)
{
  try {
    size_t total_read = 0;
    while (total_read < size) {
      size_t bytes_read = serial_->read(buffer + total_read, size - total_read);
      if (bytes_read == 0) {
        return false;  // Timeout
      }
      total_read += bytes_read;
    }
    return true;
  } catch (const std::exception& e) {
    error_message_ = "Error reading packet bytes: " + std::string(e.what());
    return false;
  }
}

bool SerialReader::validatePacket(const uint8_t* packet) const
{
  // Check header
  if (packet[0] != START_BYTE) {
    return false;
  }

  // Check length
  if (packet[1] != EXPECTED_LENGTH) {
    return false;
  }

  // Validate CRC
  uint8_t crc_calc = computeCRC(packet, PACKET_SIZE - 1);  // All bytes except CRC
  uint8_t crc_recv = packet[PACKET_SIZE - 1];
  
  return crc_calc == crc_recv;
}

void SerialReader::parsePayload(const uint8_t* payload, JoyData& data) const
{
  // Parse according to Arduino struct packing
  // struct payload: buttons(2), misc(1), dpad(1), lx(2), ly(2), rx(2), ry(2)
  
  // Extract buttons (uint16_t, little endian)
  data.buttons = static_cast<uint16_t>(payload[0] | (payload[1] << 8));
  
  // Extract misc and dpad
  data.misc = payload[2];
  data.dpad = payload[3];
  
  // Extract analog sticks (int16_t, little endian)
  data.lx = static_cast<int16_t>(payload[4] | (payload[5] << 8));
  data.ly = static_cast<int16_t>(payload[6] | (payload[7] << 8));
  data.rx = static_cast<int16_t>(payload[8] | (payload[9] << 8));
  data.ry = static_cast<int16_t>(payload[10] | (payload[11] << 8));
}

}  // namespace esp_joystick_ros2