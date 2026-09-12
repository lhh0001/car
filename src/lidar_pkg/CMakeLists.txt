cmake_minimum_required(VERSION 3.8)
project(lidar_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- 添加 OpenCV 查找 ---
find_package(OpenCV REQUIRED)

# --- 添加可执行文件 ---
add_executable(lidar_node src/lidar_node.cpp)

# --- 链接库 ---
ament_target_dependencies(lidar_node rclcpp sensor_msgs)
# 链接 OpenCV 库
target_link_libraries(lidar_node ${OpenCV_LIBS})

# --- 安装规则 (重要：否则 ros2 run 找不到节点) ---
install(TARGETS
  lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/lidar_serial_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
