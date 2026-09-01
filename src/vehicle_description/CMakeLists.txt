cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0077 NEW)
project(vehicle_description)

find_package(ament_cmake REQUIRED)

# Do not list a missing meshes/ directory: ament treats it as a build error.
install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
