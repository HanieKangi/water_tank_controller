cmake_minimum_required(VERSION 3.5)
project(tank_controller)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(water_tank_interface REQUIRED)

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp water_tank_interface)

install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()