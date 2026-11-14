cmake_minimum_required(VERSION 3.5)
project(origincar_yolo)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(dnn_node REQUIRED)
find_package(hobot_cv REQUIRED)
find_package(cv_bridge REQUIRED)

set(SYS_ROOT ${CMAKE_SYSROOT})

link_directories(
  ${SYS_ROOT}/usr/lib/
  ${SYS_ROOT}/usr/lib/hbbpu/
  ${SYS_ROOT}/usr/lib/hbmedia/
  /usr/hobot/lib
)

include_directories(
  include
  ${SYS_ROOT}/usr/include
)

add_executable(${PROJECT_NAME}
  src/yolo_node.cpp
  src/parser.cpp
  src/image_utils.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  dnn_node
  sensor_msgs
  ai_msgs
  hobot_cv
  cv_bridge
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(PROGRAMS
  scripts/yolo_cone_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
