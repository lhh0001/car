cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0077 NEW)
project(vehicle_description)

find_package(ament_cmake REQUIRED)

# Pure description package: only URDF models and mesh assets
install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
