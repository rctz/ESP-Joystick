#include "esp_joystick_ros2/esp_joystick_node.hpp"
#include <chrono>

namespace esp_joystick_ros2 {

EspJoystickNode::EspJoystickNode(const rclcpp::NodeOptions &options)
    : Node("esp_joystick_node", options), running_(false) {
  RCLCPP_INFO(this->get_logger(), "Initializing ESP Joystick Node...");

  // Declare and get parameters
  declare_parameters();
  get_parameters();

  // Initialize serial communication
  initialize_serial();

  // Setup ROS2 publishers and timers
  setup_publishers_and_timers();

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
  this->declare_parameter("publish_rate", 100.0); // Hz
  this->declare_parameter("max_button_count", 16);
  this->declare_parameter("max_axis_count", 8);
  this->declare_parameter("crc_validation_enabled", true);
}

void EspJoystickNode::get_parameters() {
  serial_port_ = this->get_parameter("serial_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  max_button_count_ = this->get_parameter("max_button_count").as_int();
  max_axis_count_ = this->get_parameter("max_axis_count").as_int();
  crc_validation_enabled_ =
      this->get_parameter("crc_validation_enabled").as_bool();

  RCLCPP_INFO(this->get_logger(), "Parameters:");
  RCLCPP_INFO(this->get_logger(), "  Serial Port: %s", serial_port_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Baud Rate: %d", baud_rate_);
  RCLCPP_INFO(this->get_logger(), "  Publish Rate: %.1f Hz", publish_rate_);
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

void EspJoystickNode::setup_publishers_and_timers() {
  // Create joy publisher
  joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);

  // Create timers
  auto publish_period =
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
  publish_timer_ = this->create_wall_timer(
      publish_period,
      std::bind(&EspJoystickNode::publish_timer_callback, this));

  // Check for connection status periodically
  read_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&EspJoystickNode::read_timer_callback, this));
}

void EspJoystickNode::serial_read_worker() {
  JoyData temp_data;

  while (running_ && rclcpp::ok()) {
    if (serial_reader_->isConnected()) {
      if (serial_reader_->readPacket(temp_data)) {
        // Update latest data thread-safe
        std::lock_guard<std::mutex> lock(joy_data_mutex_);
        latest_joy_data_ = temp_data;
      } else {
        // Print error message when packet reading fails
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to read packet from serial: %s",
                     serial_reader_->getErrorMessage().c_str());
      }
    } else {
      // Try to reconnect
      RCLCPP_WARN(this->get_logger(),
                  "Serial connection lost, attempting to reconnect...");
      if (serial_reader_->connect(serial_port_, baud_rate_)) {
        log_connection_status(true);
      } else {
        std::this_thread::sleep_for(std::chrono::seconds(2));
      }
    }

    // Small delay to prevent CPU spinning
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void EspJoystickNode::read_timer_callback() {
  if (!serial_reader_->isConnected()) {
    RCLCPP_WARN(this->get_logger(), "Serial connection status: disconnected");
  }
}

void EspJoystickNode::publish_timer_callback() {
  if (!serial_reader_->isConnected()) {
    return;
  }

  JoyData current_data;
  {
    std::lock_guard<std::mutex> lock(joy_data_mutex_);
    current_data = latest_joy_data_;
  }

  auto joy_msg = convert_to_joy_message(current_data);
  joy_publisher_->publish(joy_msg);
}

sensor_msgs::msg::Joy
EspJoystickNode::convert_to_joy_message(const JoyData &data) const {
  sensor_msgs::msg::Joy joy_msg;
  joy_msg.header.stamp = this->now();
  joy_msg.header.frame_id = "joystick";

  joy_msg.buttons.resize(max_button_count_, 0);
  joy_msg.axes.resize(max_axis_count_, 0.0);

  convert_buttons(data.buttons, data.misc, data.dpad, joy_msg);

  convert_axes(data, joy_msg);

  return joy_msg;
}

void EspJoystickNode::convert_buttons(uint16_t raw_buttons, uint8_t misc,
                                      uint8_t dpad,
                                      sensor_msgs::msg::Joy &joy_msg) const {
  // Convert 16-bit button mask to individual buttons
  for (int i = 0; i < 16 && i < max_button_count_; ++i) {
    joy_msg.buttons[i] = (raw_buttons >> i) & 0x1;
  }

  // Map misc buttons to additional button indices if space allows
  if (max_button_count_ > 16) {
    // Example mapping - adjust based on your specific requirements
    joy_msg.buttons[16] = (misc >> 0) & 0x1; // Misc bit 0
    if (max_button_count_ > 17) {
      joy_msg.buttons[17] = (misc >> 1) & 0x1; // Misc bit 1
    }
  }

  // Map dpad to additional buttons or axes
  // D-pad typically has 8 directions: 0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W,
  // 7=NW
  if (max_axis_count_ >= 2) {
    // Map dpad to axes (optional)
    switch (dpad) {
    case 0:
      joy_msg.axes[4] = 0.0;
      joy_msg.axes[5] = 1.0;
      break; // N
    case 1:
      joy_msg.axes[4] = 1.0;
      joy_msg.axes[5] = 1.0;
      break; // NE
    case 2:
      joy_msg.axes[4] = 1.0;
      joy_msg.axes[5] = 0.0;
      break; // E
    case 3:
      joy_msg.axes[4] = 1.0;
      joy_msg.axes[5] = -1.0;
      break; // SE
    case 4:
      joy_msg.axes[4] = 0.0;
      joy_msg.axes[5] = -1.0;
      break; // S
    case 5:
      joy_msg.axes[4] = -1.0;
      joy_msg.axes[5] = -1.0;
      break; // SW
    case 6:
      joy_msg.axes[4] = -1.0;
      joy_msg.axes[5] = 0.0;
      break; // W
    case 7:
      joy_msg.axes[4] = -1.0;
      joy_msg.axes[5] = 1.0;
      break; // NW
    default:
      joy_msg.axes[4] = 0.0;
      joy_msg.axes[5] = 0.0;
      break; // Center/Released
    }
  }
}

void EspJoystickNode::convert_axes(const JoyData &data,
                                   sensor_msgs::msg::Joy &joy_msg) const {
  // Normalize analog stick values from int16 range (-32768 to 32767) to float
  // (-1.0 to 1.0)
  const float normalization_factor = 32767.0f;

  // Left stick (axes 0 and 1)
  if (max_axis_count_ > 0) {
    joy_msg.axes[0] = static_cast<float>(data.lx) / normalization_factor;
  }
  if (max_axis_count_ > 1) {
    joy_msg.axes[1] = -static_cast<float>(data.ly) /
                      normalization_factor; // Invert Y for standard convention
  }

  // Right stick (axes 2 and 3)
  if (max_axis_count_ > 2) {
    joy_msg.axes[2] = static_cast<float>(data.rx) / normalization_factor;
  }
  if (max_axis_count_ > 3) {
    joy_msg.axes[3] = -static_cast<float>(data.ry) /
                      normalization_factor; // Invert Y for standard convention
  }
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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(esp_joystick_ros2::EspJoystickNode)
