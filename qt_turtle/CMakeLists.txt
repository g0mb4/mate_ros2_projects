cmake_minimum_required(VERSION 3.5)
project(qt_turtle)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

file(GLOB HEADERS_TO_MOC
  include/${PROJECT_NAME}/ui/*.h
)

file(GLOB UI_FILES
  ui/*.ui
)

file(GLOB_RECURSE SRC_FILES
  src/*.cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

include_directories(
  include
)

qt5_wrap_cpp(PROCESSED_MOCS ${HEADERS_TO_MOC})
qt5_wrap_ui(SRC_FILES ${UI_FILES})

add_executable(${PROJECT_NAME}
  ${SRC_FILES}
  ${PROCESSED_MOCS}
)

ament_target_dependencies(${PROJECT_NAME} rclcpp turtlesim)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
