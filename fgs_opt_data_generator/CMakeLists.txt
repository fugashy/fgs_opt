cmake_minimum_required(VERSION 2.8.3)
project(fgs_opt_data_generator)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy
)

install(PROGRAMS
  scripts/sample
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
