cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0077 NEW)
project(vehicle_control)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/teleop_gui.py
  scripts/graph_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
