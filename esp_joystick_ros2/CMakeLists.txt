cmake_minimum_required(VERSION 3.8)
project(esp_joystick_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(esp_joystick_interfaces REQUIRED)
# find_package(cxx-serial REQUIRED)

# Include directories
include_directories(include)

# Add component library
add_library(esp_joystick_component SHARED
  utils/serial_reader.cpp
)

# Add executable
add_executable(esp_joystick_node
  src/esp_joystick_node.cpp
)

# Add compile definitions
target_compile_definitions(esp_joystick_component PRIVATE HAVE_STDINT_H)
target_compile_definitions(esp_joystick_node PRIVATE HAVE_STDINT_H)

# Component dependencies
ament_target_dependencies(esp_joystick_component
  rclcpp
  rclcpp_components
  sensor_msgs
  serial
  esp_joystick_interfaces
)

# Executable dependencies
ament_target_dependencies(esp_joystick_node
  rclcpp
  rclcpp_components
  sensor_msgs
  serial
  esp_joystick_interfaces
)
target_link_libraries(esp_joystick_node
  esp_joystick_component
)

# Register the component
rclcpp_components_register_nodes(esp_joystick_component "esp_joystick_ros2::EspJoystickNode")

# Install targets
install(TARGETS
  esp_joystick_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS
  esp_joystick_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
