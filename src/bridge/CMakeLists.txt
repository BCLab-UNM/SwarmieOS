cmake_minimum_required(VERSION 3.5)
project(bridge)

set(CMAKE_BUILD_TYPE "Release")
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(swarmie_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#find_package(bridge REQUIRED)
#catkin_package()
#include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)
include_directories(${bridge}/install)

#add_executable(sbridge src/sbridge.cpp src/pid.cpp)
#ament_target_dependencies(sbridge rclcpp bridge)

add_executable(abridge src/abridge.cpp src/usbSerial.cpp src/pid.cpp)

#ament_target_dependencies(abridge rclcpp bridge)

set_property(TARGET abridge PROPERTY ENABLE_EXPORTS 1)
#set_property(TARGET sbridge PROPERTY ENABLE_EXPORTS 1)
#target_link_libraries(bridge ${catkin_LIBRARIES})
ament_target_dependencies(abridge
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  nav_msgs
  swarmie_msgs
  )
#target_link_libraries(abridge bridge)
#[[
ament_target_dependencies(sbridge
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  nav_msgs
  swarmie_msgs
  )
#target_link_libraries(sbridge bridge)
]]

#install(TARGETS bridge
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS 
  abridge
  #sbridge
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_export_dependencies(bridge)
ament_package()

