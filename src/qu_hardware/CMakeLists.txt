cmake_minimum_required(VERSION 3.8)
project(qu_hardware)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Arduino Driver Library
add_library(arduino_driver SHARED
  src/arduino_driver.cpp
)

# Hardware Interface Library
add_library(qu_hardware_interface SHARED
  src/qu_hardware_interface.cpp
)

target_link_libraries(qu_hardware_interface
  arduino_driver
)

target_include_directories(qu_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(qu_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)
target_include_directories(arduino_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arduino_driver
  rclcpp
)

# Install library
# Install libraries
install(
  TARGETS arduino_driver qu_hardware_interface
  EXPORT qu_hardwareTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install plugin description file
install(
  FILES qu_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Test executable
add_executable(test_arduino_driver
  test/test_arduino_driver.cpp
)

target_link_libraries(test_arduino_driver
  arduino_driver
)

install(TARGETS test_arduino_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(qu_hardwareTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
# Export plugin
pluginlib_export_plugin_description_file(hardware_interface qu_hardware_plugin.xml)
ament_package()