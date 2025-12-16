#include "esp_joystick_ros2/esp_joystick_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <chrono>
#include <memory>

namespace esp_joystick_ros2 {

EspJoystickNode::EspJoystickNode(const rclcpp::NodeOptions &options)
    : Node("esp_joystick_node", options), running_(false) {
  RCLCPP_INFO(this->get_logger(), "Initializing ESP Joystick Node...");

  declare_parameters();
  get_parameters();
  initialize_serial();
  setup_publishers();

  RCLCPP_INFO(this->get_logger(), "ESP Joystick Node initialized successfully");
}

EspJoystickNode::~EspJoystickNode() {
  running_ = false;
  if (serial_thread_.joinable()) {
    serial_thread_.join();
  }
  RCLCPP_INFO(this->get_logger(), "ESP Joystick Node destroyed");
}

void EspJoystickNode::declare_parameters() {
  this->declare_parameter("serial_port", "/dev/ttyUSB0");
  this->declare_parameter("baud_rate", 115200);
  this->declare_parameter("crc_validation_enabled", true);
}

void EspJoystickNode::get_parameters() {
  serial_port_ = this->get_parameter("serial_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  crc_validation_enabled_ =
      this->get_parameter("crc_validation_enabled").as_bool();

  RCLCPP_INFO(this->get_logger(), "Parameters:");
  RCLCPP_INFO(this->get_logger(), "  Serial Port: %s", serial_port_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Baud Rate: %d", baud_rate_);
  RCLCPP_INFO(this->get_logger(), "  CRC Validation: %s",
              crc_validation_enabled_ ? "ENABLED" : "DISABLED");
}

void EspJoystickNode::initialize_serial() {
  serial_reader_ = std::make_unique<SerialReader>();

  // Set CRC validation setting
  serial_reader_->setCrcValidationEnabled(crc_validation_enabled_);

  if (!serial_reader_->connect(serial_port_, baud_rate_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to serial port %s: %s",
                 serial_port_.c_str(),
                 serial_reader_->getErrorMessage().c_str());
    throw std::runtime_error("Serial connection failed");
  }

  log_connection_status(true);

  // Start serial reading thread
  running_ = true;
  serial_thread_ = std::thread(&EspJoystickNode::serial_read_worker, this);
}

void EspJoystickNode::setup_publishers() {
  // Using Sensor QOS
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.best_effort();
  qos.durability_volatile();

  joy_publisher_ =
      this->create_publisher<esp_joystick_interfaces::msg::JoystickInfo>("joy",
                                                                         10);
}

void EspJoystickNode::serial_read_worker() {
  JoyData temp_data;

  while (running_ && rclcpp::ok()) {
    if (serial_reader_->isConnected()) {

      // ฟังก์ชันนี้ควรเป็นแบบ Blocking หรือมี timeout สั้นๆ
      if (serial_reader_->readPacket(temp_data)) {

        // --- CRITICAL SECTION: PUBLISH IMMEDIATELY ---
        // ไม่รอ Timer แล้ว ได้ของปุ๊บ ส่งปั๊บ
        auto joy_msg = convert_to_joy_message(temp_data);

        // ใส่ Timestamp ล่าสุดเพื่อให้ ROS รู้ว่าข้อมูลสดแค่ไหน
        joy_msg.header.stamp = this->now();
        joy_msg.header.frame_id = "joy_link";

        joy_publisher_->publish(joy_msg);
        // ---------------------------------------------
      }

    } else {
      RCLCPP_WARN(this->get_logger(), "Reconnecting...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      serial_reader_->connect(serial_port_, baud_rate_);
    }
  }
}

void EspJoystickNode::read_timer_callback() {
  if (!serial_reader_->isConnected()) {
    RCLCPP_WARN(this->get_logger(), "Serial connection status: disconnected");
  }
}

esp_joystick_interfaces::msg::JoystickInfo
EspJoystickNode::convert_to_joy_message(const JoyData &data) const {
  esp_joystick_interfaces::msg::JoystickInfo joy_msg;

  // Convert 16-bit button mask to individual button states
  // Based on typical gamepad layout:
  joy_msg.y = (data.buttons >> 3) & 0x1;      // Y button (bit 3)
  joy_msg.b = (data.buttons >> 1) & 0x1;      // B button (bit 1)
  joy_msg.a = (data.buttons >> 0) & 0x1;      // A button (bit 0)
  joy_msg.x = (data.buttons >> 2) & 0x1;      // X button (bit 2)
  joy_msg.r1 = (data.buttons >> 5) & 0x1;     // R1 button (bit 5)
  joy_msg.r2 = (data.buttons >> 7) & 0x1;     // R2 button (bit 7)
  joy_msg.l1 = (data.buttons >> 4) & 0x1;     // L1 button (bit 4)
  joy_msg.l2 = (data.buttons >> 6) & 0x1;     // L2 button (bit 6)
  joy_msg.axis_l = (data.buttons >> 8) & 0x1; // L2 button (bit 6)
  joy_msg.axis_r = (data.buttons >> 9) & 0x1; // L1 button (bit 4)

  // D-pad directions
  joy_msg.up = (data.dpad >> 0) & 0x1;
  joy_msg.down = (data.dpad >> 1) & 0x1;
  joy_msg.right = (data.dpad >> 2) & 0x1;
  joy_msg.left = (data.dpad >> 3) & 0x1;

  // Misc buttons from misc byte
  joy_msg.select = (data.misc >> 1) & 0x1; // Select bit 0
  joy_msg.start = (data.misc >> 2) & 0x1;  // Start bit 1

  // Normalize analog stick values
  joy_msg.lx = static_cast<int16_t>(data.lx);
  joy_msg.ly = static_cast<int16_t>(data.ly);
  joy_msg.rx = static_cast<int16_t>(data.rx);
  joy_msg.ry = static_cast<int16_t>(data.ry);

  return joy_msg;
}

void EspJoystickNode::log_connection_status(bool connected) const {
  if (connected) {
    RCLCPP_INFO(this->get_logger(), "Connected to ESP32 on %s at %d baud",
                serial_port_.c_str(), baud_rate_);
  } else {
    RCLCPP_WARN(this->get_logger(), "Disconnected from ESP32");
  }
}

} // namespace esp_joystick_ros2

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<esp_joystick_ros2::EspJoystickNode>();

  try {
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}