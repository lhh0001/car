cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0077 NEW)
project(vehicle_control)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/steering_controller.py
  scripts/teleop_gui.py
  scripts/graph_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Unit tests
  add_launch_test(test/steering_controller_test.launch.py)

  # Code quality (informational only — won't block build)
  # ament_pep257 and ament_flake8 will pass even with warnings
endif()

ament_package()
