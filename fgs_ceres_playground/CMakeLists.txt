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

message(WARN ${CERES_LIBRARIES})

add_executable(${PROJECT_NAME}_curve_fitting
  src/${PROJECT_NAME}/curve_fitting.cpp)
target_link_libraries(${PROJECT_NAME}_curve_fitting
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)
install(TARGETS ${PROJECT_NAME}_curve_fitting
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
