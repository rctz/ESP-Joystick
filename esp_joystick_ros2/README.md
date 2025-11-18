# ESP Joystick ROS2 Node

A ROS2 Humble node for reading joystick data from ESP32 devices via serial communication. This node is designed to work with the Bluepad32 ESP32 firmware that sends joystick data over USB serial.

## Features

- **Robust Serial Communication**: CRC-validated packet parsing for reliable data transmission
- **Standard ROS2 Interface**: Publishes `sensor_msgs/Joy` messages compatible with standard ROS2 joystick consumers
- **Configurable Parameters**: Adjustable serial port, baud rate, and data mapping
- **Automatic Reconnection**: Handles connection loss and attempts reconnection
- **Threaded Operation**: Separate reading and publishing threads for optimal performance
- **High Frequency**: Supports up to 200Hz update rate (matching ESP32 capabilities)

## Protocol Compatibility

This node is specifically designed to work with ESP32 devices running Bluepad32 firmware that sends data using the following packet format:

```
JoyPacket Structure (15 bytes total):
┌─────────┬────────┬───────────┬──────┬──────┬─────┬─────┬─────┬─────┬─────┐
│ Header  │ Length │ Buttons   │ Misc │ Dpad │  LX  │  LY  │  RX  │  RY  │ CRC │
│ (0xAA)  │ (12)   │ (uint16)  │(u8)  │(u8)  │(i16) │(i16) │(i16) │(i16) │(u8) │
└─────────┴────────┴───────────┴──────┴──────┴─────┴─────┴─────┴─────┴─────┘
Byte:      0        1      2-3       4      5    6-7   8-9  10-11 12-13  14
```

## Dependencies

- ROS2 Humble
- `serial` package for C++ serial communication

### Install Dependencies

```bash
# Install ROS2 serial library
sudo apt update
sudo apt install ros-humble-serial
```

## Building

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws

# Clone or copy this package to src/ directory
# If copying from this project:
cp -r /path/to/ESP-Joystick/esp_joystick_ros2 src/

# Build the package
colcon build --packages-select esp_joystick_ros2

# Source the workspace
source install/setup.bash
```

## Usage

### Basic Usage

```bash
# Launch with default settings (serial port: /dev/ttyUSB0)
ros2 launch esp_joystick_ros2 esp_joystick.launch.py

# Launch with custom serial port
ros2 launch esp_joystick_ros2 esp_joystick.launch.py serial_port:=/dev/ttyACM0

# Launch with custom baud rate
ros2 launch esp_joystick_ros2 esp_joystick.launch.py baud_rate:=9600

# Launch with debug logging
ros2 launch esp_joystick_ros2 esp_joystick.launch.py log_level:=debug
```

### Using with Parameters File

```bash
ros2 launch esp_joystick_ros2 esp_joystick.launch.py
```

The node will load parameters from `config/esp_joystick_params.yaml`.

## Topics

### Published

- `/joy` (sensor_msgs/msg/Joy) - Joystick state data

#### Message Structure

```cpp
// Standard sensor_msgs/Joy message
std_msgs/Header header
uint32[] buttons      # Button states (0=pressed, 1=released)
float32[] axes        # Analog axis values (-1.0 to 1.0)
```

#### Data Mapping

**Buttons (16-bit mask to array):**
- `buttons[0]` - Button 0 (bit 0 of 16-bit mask)
- `buttons[1]` - Button 1 (bit 1 of 16-bit mask)
- ...
- `buttons[15]` - Button 15 (bit 15 of 16-bit mask)

**Axes (normalized analog values):**
- `axes[0]` - Left stick X-axis (-1.0 to 1.0)
- `axes[1]` - Left stick Y-axis (-1.0 to 1.0, inverted for standard convention)
- `axes[2]` - Right stick X-axis (-1.0 to 1.0)
- `axes[3]` - Right stick Y-axis (-1.0 to 1.0, inverted for standard convention)
- `axes[4]` - D-pad X-axis (-1.0 to 1.0, optional)
- `axes[5]` - D-pad Y-axis (-1.0 to 1.0, optional)

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | "/dev/ttyUSB0" | Serial port device path |
| `baud_rate` | int | 115200 | Serial communication baud rate |
| `publish_rate` | double | 100.0 | Joy message publishing frequency (Hz) |
| `max_button_count` | int | 16 | Number of buttons in joy message |
| `max_axis_count` | int | 8 | Number of axes in joy message |

## Troubleshooting

### Common Issues

1. **Permission Denied on Serial Port**
   ```bash
   sudo usermod -a -G dialout $USER
   # Logout and login again
   ```

2. **Wrong Serial Port**
   ```bash
   # List available serial ports
   ls /dev/tty*
   # Look for USB devices like /dev/ttyUSB0, /dev/ttyACM0
   ```

3. **No Data Received**
   - Check ESP32 is powered and connected
   - Verify baud rate matches ESP32 configuration
   - Check if ESP32 is running compatible firmware

4. **CRC Errors**
   - Check for serial interference/noise
   - Verify cable quality and length
   - Ensure stable power supply to ESP32

### Debug Mode

Launch with debug logging to see detailed information:

```bash
ros2 launch esp_joystick_ros2 esp_joystick.launch.py log_level:=debug
```

### Monitor Joy Topic

```bash
# View joy messages
ros2 topic echo /joy

# View topic info
ros2 topic info /joy

# Check publishing rate
ros2 topic hz /joy
```

## Example Integration

### Teleoperation with TurtleBot

```bash
# Terminal 1: Launch ESP joystick node
ros2 launch esp_joystick_ros2 esp_joystick.launch.py

# Terminal 2: Launch teleoperation node (example)
ros2 run teleop_twist_joy teleop_node
```

### Custom Joy Consumer

```cpp
// Example C++ joy subscriber
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoySubscriber : public rclcpp::Node
{
public:
  JoySubscriber() : Node("joy_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoySubscriber::joy_callback, this, std::placeholders::_1));
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: buttons=%zu, axes=%zu", 
                msg->buttons.size(), msg->axes.size());
    
    // Process joystick data
    if (msg->axes.size() >= 2) {
      double linear_x = msg->axes[1];   // Left stick Y
      double angular_z = msg->axes[0];  // Left stick X
      // Use these values for robot control
    }
  }
  
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};
```

## License

MIT License

## Contributing

Feel free to submit issues and enhancement requests!