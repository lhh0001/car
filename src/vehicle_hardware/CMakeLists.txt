cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0077 NEW)
project(vehicle_hardware)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/serial_bridge.py
  scripts/wifi_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
