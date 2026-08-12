cmake_minimum_required(VERSION 3.15)
project(control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Busca de bibliotecas
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vida_interfaces REQUIRED)

# ----------------- GamepadReader -----------------
add_executable(gamepad_reader
    src/gamepad_reader.cpp
    include/gamepad_reader.h
)
target_include_directories(gamepad_reader PRIVATE include)
ament_target_dependencies(gamepad_reader rclcpp sensor_msgs std_msgs vida_interfaces)

# ----------------- Install -----------------
install(TARGETS
  gamepad_reader
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
