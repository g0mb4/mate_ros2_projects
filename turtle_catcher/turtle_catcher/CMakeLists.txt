cmake_minimum_required(VERSION 3.5)
project(turtle_catcher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtle_catcher_interfaces REQUIRED)

include_directories(
  include
)

add_executable(turtle1_controller src/turtle1_controller_node.cpp src/turtle1_controller.cpp)
ament_target_dependencies(turtle1_controller rclcpp turtlesim geometry_msgs turtle_catcher_interfaces)

add_executable(game_master src/game_master_node.cpp src/game_master.cpp)
ament_target_dependencies(game_master rclcpp turtlesim geometry_msgs turtle_catcher_interfaces)

install(TARGETS
  turtle1_controller
  game_master
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
