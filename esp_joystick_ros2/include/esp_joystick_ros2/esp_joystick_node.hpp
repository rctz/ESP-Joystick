#ifndef ESP_JOYSTICK_ROS2__ESP_JOYSTICK_NODE_HPP_
#define ESP_JOYSTICK_ROS2__ESP_JOYSTICK_NODE_HPP_

#include <atomic>
#include <esp_joystick_interfaces/msg/joystick_info.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>
#include <thread>

#include "esp_joystick_ros2/serial_reader.hpp"

namespace esp_joystick_ros2 {

class EspJoystickNode : public rclcpp::Node {
public:
  explicit EspJoystickNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~EspJoystickNode();

private:
  // ROS2 components
  rclcpp::Publisher<esp_joystick_interfaces::msg::JoystickInfo>::SharedPtr
      joy_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;

  // Serial communication
  std::unique_ptr<SerialReader> serial_reader_;

  // Node parameters
  std::string serial_port_;
  int baud_rate_;
  double publish_rate_;
  int max_button_count_;
  int max_axis_count_;
  bool crc_validation_enabled_;

  // Thread control
  std::atomic<bool> running_;
  std::thread serial_thread_;

  // Latest joystick data
  JoyData latest_joy_data_;
  std::mutex joy_data_mutex_;

  // Methods
  void declare_parameters();
  void get_parameters();
  void initialize_serial();
  void setup_publishers_and_timers();

  // Timer callbacks
  void read_timer_callback();
  void publish_timer_callback();

  // Data conversion
  esp_joystick_interfaces::msg::JoystickInfo
  convert_to_joy_message(const JoyData &data) const;

  // Utility methods
  void serial_read_worker();
  void log_connection_status(bool connected) const;
};

} // namespace esp_joystick_ros2

#endif // ESP_JOYSTICK_ROS2__ESP_JOYSTICK_NODE_HPP_
