cmake_minimum_required(VERSION 3.5)
project(swarmie_msgs)

set(CMAKE_BUILD_TYPE "Release")
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/Obstacle.msg"
  "msg/SwarmieIMU.msg"
  "msg/StartQueuePriority.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

