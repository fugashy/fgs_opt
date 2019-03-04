cmake_minimum_required(VERSION 2.8.3)
project(fgs_ceres_playground)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  CATKIN_DEPENDS roscpp
)

find_package(Ceres REQUIRED)
find_package(OpenCV 3.4.1 EXACT REQUIRED
  PATHS /opt/opencv/3.4.1/share/OpenCV
  NO_DEFAULT_PATH
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_hello_world
  src/${PROJECT_NAME}/hello_world.cpp)
target_link_libraries(${PROJECT_NAME}_hello_world
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/bundle_adjustment_in_the_large.cpp
  src/${PROJECT_NAME}/cv_viz.cpp
  src/${PROJECT_NAME}/optimization_context_factory.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(${PROJECT_NAME}_solve
  src/${PROJECT_NAME}/solve.cpp)
target_link_libraries(${PROJECT_NAME}_solve
  ${PROJECT_NAME}
)

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_hello_world
    ${PROJECT_NAME}_solve
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
