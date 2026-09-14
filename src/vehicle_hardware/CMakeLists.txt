cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0077 NEW)
project(vehicle_hardware)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/wifi_bridge.py
  scripts/odom_imu_fusion.py
  scripts/esp32_pwm_test.py
  scripts/esp32_encoder_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
