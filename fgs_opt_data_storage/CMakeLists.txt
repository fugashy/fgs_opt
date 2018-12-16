cmake_minimum_required(VERSION 2.8.3)
project(fgs_opt_data_storage)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  cv_bridge
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy roscpp cv_bridge
)

find_package(OpenCV 3 REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/loader.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(${PROJECT_NAME}_load_sample
  src/${PROJECT_NAME}/load_sample.cpp
)
target_link_libraries(${PROJECT_NAME}_load_sample
  ${PROJECT_NAME}
)

install(PROGRAMS
  scripts/generate_data_2d
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_load_sample
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
