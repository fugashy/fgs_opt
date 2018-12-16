cmake_minimum_required(VERSION 2.8.3)
project(fgs_opt_data_storage)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy
)

install(PROGRAMS
  scripts/generate_data_2d
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
