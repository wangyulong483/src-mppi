cmake_minimum_required(VERSION 3.8)
project(origincar_mission_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

include_directories(include)

add_executable(mission_manager
  src/mission_manager.cpp
)

ament_target_dependencies(mission_manager
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  nav2_msgs
)

install(TARGETS mission_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
