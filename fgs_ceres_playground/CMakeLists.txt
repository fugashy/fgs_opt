cmake_minimum_required(VERSION 2.8.3)
project(fgs_ceres_playground)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  fgs_opt_data_storage
)

catkin_package(
  CATKIN_DEPENDS fgs_opt_data_storage
)

find_package(Ceres REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_hello_world
  src/${PROJECT_NAME}/hello_world.cpp)
target_link_libraries(${PROJECT_NAME}_hello_world
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_executable(${PROJECT_NAME}_fitting
  src/${PROJECT_NAME}/fitting.cpp)
target_link_libraries(${PROJECT_NAME}_fitting
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_fitting
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
