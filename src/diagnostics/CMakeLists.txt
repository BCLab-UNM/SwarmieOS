cmake_minimum_required(VERSION 3.0.2)
project(diagnostics)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  DEPENDS
  rospy
)

catkin_install_python (
	PROGRAMS 
	scripts/diags_node.py 
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
