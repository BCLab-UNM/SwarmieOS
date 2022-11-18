cmake_minimum_required(VERSION 3.0.2)
project(mobility)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  random_numbers
  tf
  rospy
  message_generation
  dynamic_reconfigure
  grid_map_ros
  grid_map_msgs
  nav_msgs
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/drive.py
)

add_message_files ( 
  FILES MoveResult.msg MoveRequest.msg HomeTransformAuthority.msg
)

add_service_files (
  FILES Core.srv Queue.srv QueueRemove.srv GetRoverNames.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs std_msgs 
  random_numbers tf rospy message_runtime
  grid_map_ros grid_map_msgs dynamic_reconfigure
  nav_msgs
)

catkin_install_python (
  PROGRAMS 
  scripts/cal_imu.py
  scripts/coordinate.py
  scripts/core.py
  scripts/home_transform.py
  scripts/imu_covariance.py
  scripts/rdb.py
  scripts/targets_block_location.py
  scripts/task.py
  scripts/teleop_keyboard.py
  scripts/test_square.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY camera_info
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
